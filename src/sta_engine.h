// sta_engine.h - late-mode (setup) static timing analysis.
#pragma once

#include <string>
#include <vector>

#include "timing_graph.h"

namespace sta {

enum Trans { kRise = 0, kFall = 1 };

struct Constraints {
    std::string clockPort = "clk";
    double period = 1.0;        // ns
    double uncertainty = 0.0;   // ns, subtracted from every required time
    double inputDelay = 0.0;    // ns, applied to all non-clock inputs
    double outputDelay = 0.0;   // ns, applied to all outputs
};

struct PathStep {
    std::string pin;
    Trans trans;
    double incr;
    double arrival;
};

struct EndpointResult {
    std::string endpoint;      // "prod_reg[25]/D" or "prod[3]"
    std::string startpoint;    // first data pin after the clock tree
    Trans trans;
    double arrival = 0.0;
    double required = 0.0;
    double slack = 0.0;
    double captureLatency = 0.0;
    double setup = 0.0;
    std::vector<PathStep> path;
};

class StaEngine {
public:
    StaEngine(const TimingGraph& graph, const SdfData& sdf, Constraints c);

    void run();  // propagate arrivals and evaluate all endpoints

    const std::vector<EndpointResult>& endpoints() const { return results_; }  // worst first
    double wns() const;   // worst negative slack (or worst slack if all positive)
    double tns() const;   // total negative slack
    int violations() const;

    // Smallest clock period at which every register-to-register and I/O check passes.
    double minimumPeriod() const;

private:
    struct Pred { int node = -1; int trans = 0; };

    void propagate();
    void evaluate();
    std::vector<PathStep> trace(int node, int trans) const;

    const TimingGraph& g_;
    const SdfData& sdf_;
    Constraints c_;
    std::vector<double> arr_[2];
    std::vector<Pred> pred_[2];
    std::vector<EndpointResult> results_;
};

}  // namespace sta
