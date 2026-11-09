// netlist.cpp - parser for flat gate-level Verilog.
#include "netlist.h"

#include <fstream>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>

namespace sta {

std::string normalizeName(const std::string& raw) {
    std::string out;
    out.reserve(raw.size());
    for (char c : raw) {
        if (c != '\\' && c != ' ' && c != '\t' && c != '\n' && c != '\r') out += c;
    }
    return out;
}

bool Instance::isOutputPin(const std::string& pin) {
    static const std::set<std::string> kOutputs = {"Z", "ZN", "Q", "QN", "S", "CO"};
    return kOutputs.count(pin) > 0;
}

static std::string stripComments(const std::string& text) {
    std::string out;
    out.reserve(text.size());
    for (size_t i = 0; i < text.size(); ++i) {
        if (text.compare(i, 2, "//") == 0) {
            while (i < text.size() && text[i] != '\n') ++i;
            out += '\n';
        } else if (text.compare(i, 2, "/*") == 0) {
            size_t end = text.find("*/", i + 2);
            if (end == std::string::npos) break;
            i = end + 1;
        } else {
            out += text[i];
        }
    }
    return out;
}

static std::string trim(const std::string& s) {
    size_t b = s.find_first_not_of(" \t\r\n");
    if (b == std::string::npos) return "";
    size_t e = s.find_last_not_of(" \t\r\n");
    return s.substr(b, e - b + 1);
}

Netlist Netlist::parseFile(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("cannot open netlist: " + path);
    std::stringstream ss;
    ss << in.rdbuf();
    return parseString(ss.str());
}

Netlist Netlist::parseString(const std::string& text) {
    Netlist nl;
    std::string body = stripComments(text);
    std::stringstream stmts(body);
    std::string stmt;
    // Every Verilog statement we care about ends in ';'.
    while (std::getline(stmts, stmt, ';')) {
        stmt = trim(stmt);
        if (stmt.empty()) continue;
        if (stmt.rfind("endmodule", 0) == 0) stmt = trim(stmt.substr(9));
        if (stmt.empty()) continue;

        if (stmt.rfind("module ", 0) == 0) {
            std::istringstream is(stmt.substr(7));
            is >> nl.module_;
            size_t p = nl.module_.find('(');
            if (p != std::string::npos) nl.module_ = nl.module_.substr(0, p);
        } else if (stmt.rfind("input ", 0) == 0) {
            nl.addPorts(stmt.substr(6), nl.inputs_);
        } else if (stmt.rfind("output ", 0) == 0) {
            nl.addPorts(stmt.substr(7), nl.outputs_);
        } else if (stmt.rfind("wire ", 0) == 0 || stmt.rfind("assign ", 0) == 0) {
            if (stmt.rfind("assign ", 0) == 0)
                throw std::runtime_error("assign statements are not supported: " + stmt);
        } else {
            nl.addInstance(stmt);
        }
    }
    if (nl.module_.empty()) throw std::runtime_error("no module found in netlist");
    return nl;
}

// "[15:0] inp_a" -> inp_a[15] ... inp_a[0];  "clk" -> clk;  "a, b" -> a, b.
void Netlist::addPorts(const std::string& decl, std::vector<std::string>& out) {
    static const std::regex kBus(R"(^\s*\[\s*(\d+)\s*:\s*(\d+)\s*\]\s*(.*)$)");
    std::smatch m;
    int hi = -1, lo = -1;
    std::string names = decl;
    if (std::regex_match(decl, m, kBus)) {
        hi = std::stoi(m[1]);
        lo = std::stoi(m[2]);
        names = m[3];
    }
    std::stringstream ss(names);
    std::string name;
    while (std::getline(ss, name, ',')) {
        name = normalizeName(name);
        if (name.empty()) continue;
        if (hi < 0) {
            out.push_back(name);
        } else {
            int step = hi >= lo ? -1 : 1;
            for (int i = hi;; i += step) {
                out.push_back(name + "[" + std::to_string(i) + "]");
                if (i == lo) break;
            }
        }
    }
}

// "CELLTYPE inst_name ( .A(n1), .ZN(n2) )"
void Netlist::addInstance(const std::string& stmt) {
    size_t open = stmt.find('(');
    if (open == std::string::npos) return;  // not an instance
    std::string head = trim(stmt.substr(0, open));

    std::string cellType, instName;
    size_t sp = head.find_first_of(" \t\n");
    if (sp == std::string::npos) throw std::runtime_error("bad instance: " + head);
    cellType = head.substr(0, sp);
    instName = normalizeName(head.substr(sp));

    Instance inst(instName, cellType);
    static const std::regex kPin(R"(\.\s*(\w+)\s*\(\s*([^()]*?)\s*\))");
    std::string conns = stmt.substr(open);
    for (std::sregex_iterator it(conns.begin(), conns.end(), kPin), end; it != end; ++it) {
        std::string net = normalizeName((*it)[2]);
        if (!net.empty()) inst.connect((*it)[1], net);
    }
    instances_.push_back(std::move(inst));
}

}  // namespace sta
