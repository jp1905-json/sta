// sta_engine.cpp - arrival propagation, setup checks, path tracing.
#include "sta_engine.h"

#include <algorithm>
#include <limits>

namespace sta {

namespace {
constexpr double kNegInf = -std::numeric_limits<double>::infinity();
}

StaEngine::StaEngine(const TimingGraph& graph, const SdfData& sdf, Constraints c)
    : g_(graph), sdf_(sdf), c_(std::move(c)) {}

void StaEngine::run() {
    propagate();
    evaluate();
}

void StaEngine::propagate() {
    const auto& nodes = g_.nodes();
    for (int t = 0; t < 2; ++t) {
        arr_[t].assign(nodes.size(), kNegInf);
        pred_[t].assign(nodes.size(), Pred{});
    }

    // Sources. The clock rises at t=0 at its port; the clock tree is timed like
    // any other logic, so each flop's CP arrival is its real propagated latency.
    for (const auto& p : g_.netlist().inputs()) {
        int n = g_.find(p);
        if (p == c_.clockPort) {
            arr_[kRise][n] = 0.0;
        } else {
            arr_[kRise][n] = c_.inputDelay;
            arr_[kFall][n] = c_.inputDelay;
        }
    }

    auto relax = [&](int to, int toT, double value, int from, int fromT) {
        if (value > arr_[toT][to]) {
            arr_[toT][to] = value;
            pred_[toT][to] = Pred{from, fromT};
        }
    };

    for (int n : g_.topologicalOrder()) {
        for (int a : nodes[n].fanout) {
            const Arc& arc = g_.arcs()[a];
            const double inR = arr_[kRise][n], inF = arr_[kFall][n];
            const double dR = arc.delay.rise, dF = arc.delay.fall;
            switch (arc.sense) {
                case Sense::Positive:
                    if (inR > kNegInf) relax(arc.to, kRise, inR + dR, n, kRise);
                    if (inF > kNegInf) relax(arc.to, kFall, inF + dF, n, kFall);
                    break;
                case Sense::Negative:
                    if (inF > kNegInf) relax(arc.to, kRise, inF + dR, n, kFall);
                    if (inR > kNegInf) relax(arc.to, kFall, inR + dF, n, kRise);
                    break;
                case Sense::NonUnate:
                    for (int t = 0; t < 2; ++t) {
                        double in = arr_[t][n];
                        if (in == kNegInf) continue;
                        relax(arc.to, kRise, in + dR, n, t);
                        relax(arc.to, kFall, in + dF, n, t);
                    }
                    break;
                case Sense::RisingEdge:  // flop CP -> Q, launched by the rising clock
                    if (inR > kNegInf) {
                        relax(arc.to, kRise, inR + dR, n, kRise);
                        relax(arc.to, kFall, inR + dF, n, kRise);
                    }
                    break;
            }
        }
    }
}

void StaEngine::evaluate() {
    results_.clear();
    const auto& nodes = g_.nodes();

    auto addResult = [&](int node, double requiredR, double requiredF, double capLat,
                         double setupR, double setupF) {
        EndpointResult best;
        bool have = false;
        for (int t = 0; t < 2; ++t) {
            if (arr_[t][node] == kNegInf) continue;
            double req = t == kRise ? requiredR : requiredF;
            double slack = req - arr_[t][node];
            if (!have || slack < best.slack) {
                best.endpoint = nodes[node].name;
                best.trans = static_cast<Trans>(t);
                best.arrival = arr_[t][node];
                best.required = req;
                best.slack = slack;
                best.captureLatency = capLat;
                best.setup = t == kRise ? setupR : setupF;
                have = true;
            }
        }
        if (!have) return;
        best.path = trace(node, best.trans);
        // Startpoint: first pin after the clock tree (a flop Q or a data input port).
        for (const auto& s : best.path) {
            int id = g_.find(s.pin);
            if (nodes[id].kind == NodeKind::InputPort && s.pin != c_.clockPort) { best.startpoint = s.pin; break; }
            if (nodes[id].kind == NodeKind::CellOutput && s.pin.size() > 2 &&
                s.pin.compare(s.pin.size() - 2, 2, "/Q") == 0) { best.startpoint = s.pin; break; }
        }
        results_.push_back(std::move(best));
    };

    // Register endpoints: data pins with a setup check in the SDF.
    for (const auto& [dpin, check] : sdf_.setupChecks()) {
        int d = g_.find(dpin);
        if (d < 0) continue;
        std::string inst = dpin.substr(0, dpin.rfind('/'));
        int cp = g_.find(inst + "/" + check.clockPin);
        double capLat = (cp >= 0 && arr_[kRise][cp] > kNegInf) ? arr_[kRise][cp] : 0.0;
        double base = c_.period + capLat - c_.uncertainty;
        addResult(d, base - check.setup.rise, base - check.setup.fall, capLat,
                  check.setup.rise, check.setup.fall);
    }

    // Output port endpoints, captured by an ideal clock outside the block.
    for (const auto& p : g_.netlist().outputs()) {
        int n = g_.find(p);
        double req = c_.period - c_.outputDelay - c_.uncertainty;
        addResult(n, req, req, 0.0, c_.outputDelay, c_.outputDelay);
    }

    std::sort(results_.begin(), results_.end(),
              [](const EndpointResult& a, const EndpointResult& b) { return a.slack < b.slack; });
}

std::vector<PathStep> StaEngine::trace(int node, int trans) const {
    std::vector<PathStep> rev;
    int n = node, t = trans;
    while (n >= 0) {
        Pred p = pred_[t][n];
        double incr = p.node >= 0 ? arr_[t][n] - arr_[p.trans][p.node] : arr_[t][n];
        rev.push_back(PathStep{g_.nodes()[n].name, static_cast<Trans>(t), incr, arr_[t][n]});
        n = p.node;
        t = p.trans;
    }
    return {rev.rbegin(), rev.rend()};
}

double StaEngine::wns() const { return results_.empty() ? 0.0 : results_.front().slack; }

double StaEngine::tns() const {
    double sum = 0.0;
    for (const auto& r : results_) if (r.slack < 0) sum += r.slack;
    return sum;
}

int StaEngine::violations() const {
    return static_cast<int>(std::count_if(results_.begin(), results_.end(),
                                          [](const EndpointResult& r) { return r.slack < 0; }));
}

double StaEngine::minimumPeriod() const {
    // Every required time contains exactly one +period term, so slack moves 1:1 with it.
    return c_.period - wns();
}

}  // namespace sta
