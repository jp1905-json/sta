// timing_graph.cpp - builds net and cell arcs, plus unateness rules.
#include "timing_graph.h"

#include <queue>
#include <regex>
#include <set>
#include <stdexcept>

namespace sta {

// "OAI21D2BWP16P90LVT" -> "OAI21", "CKND2D4BWP16P90LVT" -> "CKND2", "CKND8BWP..." -> "CKN".
std::string CellLibrary::family(const std::string& cellType) {
    static const std::regex kFamily(R"(^(.*?)D\d+BWP)");
    std::smatch m;
    if (std::regex_search(cellType, m, kFamily)) return m[1];
    return cellType;
}

Sense CellLibrary::sense(const std::string& cellType, const std::string& in,
                         const std::string& out) {
    static const std::set<std::string> kNegative = {
        "INV", "CKN", "ND2", "ND3", "ND4", "NR2", "NR3", "NR4", "CKND2", "CKNR2",
        "AOI21", "AOI22", "AOI211", "OAI21", "OAI22", "OAI211", "MAOI222", "MOAI22"};
    static const std::set<std::string> kPositive = {
        "BUFF", "CKB", "AN2", "AN3", "AN4", "OR2", "OR3", "OR4", "CKAN2", "CKOR2",
        "AO21", "AO22", "OA21", "OA22"};
    const std::string f = family(cellType);

    if (kNegative.count(f)) return Sense::Negative;
    if (kPositive.count(f)) return Sense::Positive;
    // Cells with one inverted input: ZN = !(!A1 op B1). A1 is positive, B pins negative.
    if (f == "IND2" || f == "INR2" || f == "IND3" || f == "INR3")
        return in == "A1" ? Sense::Positive : Sense::Negative;
    // IOAI21: ZN = !((!A1 | !A2) & B). A pins positive, B negative.
    if (f == "IOAI21") return in[0] == 'A' ? Sense::Positive : Sense::Negative;
    // Adders: carry-out is a majority function (positive); sum is XOR (non-unate).
    if ((f == "FA1" || f == "HA1") && out == "CO") return Sense::Positive;
    // XOR/XNOR, adder sums, and anything unrecognised: assume worst case.
    return Sense::NonUnate;
}

TimingGraph::TimingGraph(const Netlist& netlist, const SdfData& sdf) : netlist_(netlist) {
    // 1. Nodes for ports and every connected instance pin.
    for (const auto& p : netlist.inputs()) addNode(p, NodeKind::InputPort, -1);
    for (const auto& p : netlist.outputs()) addNode(p, NodeKind::OutputPort, -1);

    std::unordered_map<std::string, int> netDriver;
    std::unordered_map<std::string, std::vector<int>> netLoads;
    for (const auto& p : netlist.inputs()) netDriver[p] = find(p);
    for (const auto& p : netlist.outputs()) netLoads[p].push_back(find(p));

    const auto& insts = netlist.instances();
    for (int i = 0; i < static_cast<int>(insts.size()); ++i) {
        for (const auto& [pin, net] : insts[i].connections()) {
            bool isOut = Instance::isOutputPin(pin);
            int n = addNode(insts[i].name() + "/" + pin,
                            isOut ? NodeKind::CellOutput : NodeKind::CellInput, i);
            if (isOut) {
                if (netDriver.count(net))
                    throw std::runtime_error("net " + net + " has multiple drivers");
                netDriver[net] = n;
            } else {
                netLoads[net].push_back(n);
            }
        }
    }

    // 2. Net arcs: driver -> each load, with SDF INTERCONNECT delay.
    const auto& ic = sdf.interconnects();
    for (const auto& [net, loads] : netLoads) {
        auto d = netDriver.find(net);
        if (d == netDriver.end()) continue;  // undriven / tie-off net
        for (int load : loads) {
            if (load == d->second) continue;  // port that is both driver and load
            auto it = ic.find(nodes_[d->second].name + "->" + nodes_[load].name);
            RiseFall delay;
            if (it != ic.end()) delay = it->second;
            else ++netArcsWithoutSdf_;
            addArc(d->second, load, delay, Sense::Positive, ArcKind::Net);
        }
    }

    // 3. Cell arcs from SDF IOPATH entries ("inst/in->out").
    std::unordered_map<std::string, int> instIndex;
    for (int i = 0; i < static_cast<int>(insts.size()); ++i) instIndex[insts[i].name()] = i;
    for (const auto& [key, arc] : sdf.cellArcs()) {
        size_t arrow = key.find("->");
        std::string left = key.substr(0, arrow);
        std::string outPin = key.substr(arrow + 2);
        size_t slash = left.rfind('/');
        std::string inst = left.substr(0, slash);
        std::string inPin = left.substr(slash + 1);
        int from = find(left);
        int to = find(inst + "/" + outPin);
        if (from < 0 || to < 0) continue;  // pin left unconnected in the netlist
        const std::string& cell = insts[instIndex.at(inst)].cellType();
        Sense s = arc.edgeTriggered ? Sense::RisingEdge : CellLibrary::sense(cell, inPin, outPin);
        addArc(from, to, arc.delay, s, ArcKind::Cell);
    }
}

int TimingGraph::addNode(const std::string& name, NodeKind kind, int instance) {
    auto it = index_.find(name);
    if (it != index_.end()) return it->second;
    nodes_.push_back(Node{name, kind, instance, {}, {}});
    int id = static_cast<int>(nodes_.size()) - 1;
    index_[name] = id;
    return id;
}

void TimingGraph::addArc(int from, int to, RiseFall d, Sense s, ArcKind k) {
    arcs_.push_back(Arc{from, to, d, s, k});
    int id = static_cast<int>(arcs_.size()) - 1;
    nodes_[from].fanout.push_back(id);
    nodes_[to].fanin.push_back(id);
}

int TimingGraph::find(const std::string& name) const {
    auto it = index_.find(name);
    return it == index_.end() ? -1 : it->second;
}

std::vector<int> TimingGraph::topologicalOrder() const {
    std::vector<int> indeg(nodes_.size(), 0);
    for (const auto& a : arcs_) ++indeg[a.to];
    std::queue<int> q;
    for (int i = 0; i < static_cast<int>(nodes_.size()); ++i)
        if (indeg[i] == 0) q.push(i);
    std::vector<int> order;
    order.reserve(nodes_.size());
    while (!q.empty()) {
        int n = q.front();
        q.pop();
        order.push_back(n);
        for (int a : nodes_[n].fanout)
            if (--indeg[arcs_[a].to] == 0) q.push(arcs_[a].to);
    }
    if (order.size() != nodes_.size())
        throw std::runtime_error("combinational loop detected in timing graph");
    return order;
}

}  // namespace sta
