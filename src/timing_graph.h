// timing_graph.h - pin-level timing graph built from a netlist and SDF delays.
#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "netlist.h"
#include "sdf.h"

namespace sta {

// How an output transition relates to an input transition across an arc.
enum class Sense { Positive, Negative, NonUnate, RisingEdge };

// Looks up arc unateness from the cell's logic family (INV, ND2, OAI21, ...).
class CellLibrary {
public:
    static std::string family(const std::string& cellType);
    static Sense sense(const std::string& cellType, const std::string& inPin,
                       const std::string& outPin);
};

enum class NodeKind { InputPort, OutputPort, CellInput, CellOutput };

struct Node {
    std::string name;  // "U835/A1" or "inp_a[3]"
    NodeKind kind;
    int instance = -1;  // index into Netlist::instances(), -1 for ports
    std::vector<int> fanout;  // arc indices
    std::vector<int> fanin;
};

enum class ArcKind { Net, Cell };

struct Arc {
    int from;
    int to;
    RiseFall delay;
    Sense sense;
    ArcKind kind;
};

class TimingGraph {
public:
    TimingGraph(const Netlist& netlist, const SdfData& sdf);

    const std::vector<Node>& nodes() const { return nodes_; }
    const std::vector<Arc>& arcs() const { return arcs_; }
    int find(const std::string& name) const;  // -1 if missing

    // Kahn's algorithm; throws if the graph has a combinational loop.
    std::vector<int> topologicalOrder() const;

    int netArcsWithoutSdf() const { return netArcsWithoutSdf_; }
    const Netlist& netlist() const { return netlist_; }

private:
    int addNode(const std::string& name, NodeKind kind, int instance);
    void addArc(int from, int to, RiseFall d, Sense s, ArcKind k);

    const Netlist& netlist_;
    std::vector<Node> nodes_;
    std::vector<Arc> arcs_;
    std::unordered_map<std::string, int> index_;
    int netArcsWithoutSdf_ = 0;
};

}  // namespace sta
