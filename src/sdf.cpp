// sdf.cpp - s-expression parser plus SDF semantics.
#include "sdf.h"

#include <algorithm>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <vector>

#include "netlist.h"  // normalizeName

namespace sta {

namespace {

// A node in the parsed SDF tree: either an atom or a parenthesized list.
struct SExpr {
    std::string atom;
    std::vector<SExpr> list;
    bool isList = false;

    const std::string& head() const {
        static const std::string empty;
        return (isList && !list.empty() && !list[0].isList) ? list[0].atom : empty;
    }
};

class SExprParser {
public:
    explicit SExprParser(const std::string& text) : t_(text) {}

    SExpr parse() {
        skipSpace();
        SExpr root = parseOne();
        return root;
    }

private:
    void skipSpace() {
        while (i_ < t_.size() && std::isspace(static_cast<unsigned char>(t_[i_]))) ++i_;
    }

    SExpr parseOne() {
        skipSpace();
        if (i_ >= t_.size()) throw std::runtime_error("unexpected end of SDF");
        SExpr e;
        if (t_[i_] == '(') {
            e.isList = true;
            ++i_;
            while (true) {
                skipSpace();
                if (i_ >= t_.size()) throw std::runtime_error("unbalanced '(' in SDF");
                if (t_[i_] == ')') { ++i_; break; }
                e.list.push_back(parseOne());
            }
        } else if (t_[i_] == '"') {
            size_t end = t_.find('"', i_ + 1);
            e.atom = t_.substr(i_ + 1, end - i_ - 1);
            i_ = end + 1;
        } else {
            size_t start = i_;
            while (i_ < t_.size() && !std::isspace(static_cast<unsigned char>(t_[i_])) &&
                   t_[i_] != '(' && t_[i_] != ')') {
                if (t_[i_] == '\\') ++i_;  // escaped char, e.g. \[
                ++i_;
            }
            e.atom = t_.substr(start, i_ - start);
        }
        return e;
    }

    const std::string& t_;
    size_t i_ = 0;
};

// "(0.012::0.012)" -> 0.012. Uses the max value of min:typ:max; empty -> nullopt-like flag.
bool readValue(const SExpr& e, double& out) {
    if (!e.isList || e.list.empty()) return false;
    std::string v = e.list[0].atom;
    size_t p = v.rfind(':');
    if (p != std::string::npos) v = v.substr(p + 1);
    if (v.empty()) return false;
    out = std::stod(v);
    return true;
}

RiseFall readRiseFall(const SExpr& e, size_t first) {
    RiseFall rf;
    double v;
    if (first < e.list.size() && readValue(e.list[first], v)) rf.rise = v;
    if (first + 1 < e.list.size() && readValue(e.list[first + 1], v)) rf.fall = v;
    else rf.fall = rf.rise;  // single value applies to both edges
    return rf;
}

}  // namespace

// Walks the parsed tree and fills SdfData.
class SdfBuilder {
public:
    explicit SdfBuilder(SdfData& d) : d_(d) {}

    void build(const SExpr& root) {
        for (const SExpr& cell : root.list)
            if (cell.head() == "CELL") readCell(cell);
    }

private:
    static void keepMax(RiseFall& dst, const RiseFall& src) {
        dst.rise = std::max(dst.rise, src.rise);
        dst.fall = std::max(dst.fall, src.fall);
    }

    void readCell(const SExpr& cell) {
        std::string inst;
        for (const SExpr& c : cell.list) {
            if (c.head() == "INSTANCE" && c.list.size() > 1) inst = normalizeName(c.list[1].atom);
        }
        for (const SExpr& c : cell.list) {
            if (c.head() == "DELAY") walkDelay(c, inst);
            if (c.head() == "TIMINGCHECK") walkChecks(c, inst);
        }
    }

    void walkDelay(const SExpr& e, const std::string& inst) {
        for (const SExpr& c : e.list) {
            if (!c.isList) continue;
            const std::string& h = c.head();
            if (h == "INTERCONNECT") readInterconnect(c, inst);
            else if (h == "IOPATH") readIopath(c, inst);
            else walkDelay(c, inst);  // ABSOLUTE, COND wrappers
        }
    }

    void readInterconnect(const SExpr& e, const std::string& scope) {
        std::string from = normalizeName(e.list[1].atom);
        std::string to = normalizeName(e.list[2].atom);
        if (!scope.empty()) { from = scope + "/" + from; to = scope + "/" + to; }
        RiseFall rf = readRiseFall(e, 3);
        keepMax(d_.interconnects_[from + "->" + to], rf);
    }

    void readIopath(const SExpr& e, const std::string& inst) {
        CellArc arc;
        std::string in;
        if (e.list[1].isList) {  // (posedge CP)
            arc.edgeTriggered = true;
            in = e.list[1].list[1].atom;
        } else {
            in = e.list[1].atom;
        }
        std::string out = e.list[2].atom;
        // Async reset/set arcs are checked by recovery/removal, not as data paths.
        if (in == "CDN" || in == "SDN") return;
        arc.delay = readRiseFall(e, 3);
        CellArc& slot = d_.cellArcs_[inst + "/" + in + "->" + out];
        keepMax(slot.delay, arc.delay);  // multiple COND entries: keep worst case
        slot.edgeTriggered = slot.edgeTriggered || arc.edgeTriggered;
    }

    void walkChecks(const SExpr& e, const std::string& inst) {
        for (const SExpr& c : e.list) {
            if (c.head() != "SETUPHOLD" && c.head() != "SETUP") continue;
            // (SETUPHOLD (posedge D) (COND x (posedge CP)) (setup) (hold))
            const SExpr& dataSpec = c.list[1];
            const SExpr* clkSpec = &c.list[2];
            if (clkSpec->head() == "COND") clkSpec = &clkSpec->list.back();
            if (!dataSpec.isList) continue;
            std::string edge = dataSpec.list[0].atom;
            std::string dataPin = dataSpec.list[1].atom;
            std::string clkPin = clkSpec->isList ? clkSpec->list[1].atom : clkSpec->atom;
            double setup = 0.0;
            readValue(c.list[3], setup);

            SetupCheck& sc = d_.setups_[inst + "/" + dataPin];
            sc.clockPin = clkPin;
            if (edge == "posedge") {
                sc.setup.rise = sc.haveRise ? std::max(sc.setup.rise, setup) : setup;
                sc.haveRise = true;
            } else {
                sc.setup.fall = sc.haveFall ? std::max(sc.setup.fall, setup) : setup;
                sc.haveFall = true;
            }
        }
    }

    SdfData& d_;
};

SdfData SdfData::parseFile(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("cannot open SDF: " + path);
    std::stringstream ss;
    ss << in.rdbuf();
    return parseString(ss.str());
}

SdfData SdfData::parseString(const std::string& text) {
    SExprParser p(text);
    SExpr root = p.parse();
    if (root.head() != "DELAYFILE") throw std::runtime_error("not an SDF file (no DELAYFILE)");
    SdfData d;
    SdfBuilder(d).build(root);
    return d;
}

}  // namespace sta
