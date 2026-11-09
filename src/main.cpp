// main.cpp - command-line front end.
//
//   sta <netlist.v> <delays.sdf> [--period ns] [--uncertainty ns]
//       [--input-delay ns] [--output-delay ns] [--clock port] [--paths N]
#include <chrono>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>

#include "netlist.h"
#include "sdf.h"
#include "sta_engine.h"
#include "timing_graph.h"

using namespace sta;

static void printPath(const EndpointResult& r, int index) {
    std::printf("\nPath %d: %s setup check at %s (%s)\n", index, r.slack >= 0 ? "MET" : "VIOLATED",
                r.endpoint.c_str(), r.trans == kRise ? "rise" : "fall");
    std::printf("  Startpoint: %s\n", r.startpoint.c_str());
    std::printf("  %-34s %5s %8s %8s\n", "Pin", "Edge", "Incr", "Arrival");
    for (const auto& s : r.path)
        std::printf("  %-34s %5s %8.3f %8.3f\n", s.pin.c_str(), s.trans == kRise ? "^" : "v",
                    s.incr, s.arrival);
    std::printf("  Capture clock latency  %8.3f\n", r.captureLatency);
    std::printf("  Setup                  %8.3f\n", r.setup);
    std::printf("  Required time          %8.3f\n", r.required);
    std::printf("  Arrival time           %8.3f\n", r.arrival);
    std::printf("  Slack                  %8.3f\n", r.slack);
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::fprintf(stderr, "usage: %s <netlist.v> <delays.sdf> [--period ns] [--uncertainty ns]"
                             " [--input-delay ns] [--output-delay ns] [--clock port] [--paths N]\n",
                     argv[0]);
        return 2;
    }
    Constraints c;
    int nPaths = 3;
    for (int i = 3; i + 1 < argc; i += 2) {
        std::string k = argv[i];
        std::string v = argv[i + 1];
        if (k == "--period") c.period = std::stod(v);
        else if (k == "--uncertainty") c.uncertainty = std::stod(v);
        else if (k == "--input-delay") c.inputDelay = std::stod(v);
        else if (k == "--output-delay") c.outputDelay = std::stod(v);
        else if (k == "--clock") c.clockPort = v;
        else if (k == "--paths") nPaths = std::stoi(v);
        else { std::fprintf(stderr, "unknown option %s\n", k.c_str()); return 2; }
    }

    try {
        auto t0 = std::chrono::steady_clock::now();
        Netlist nl = Netlist::parseFile(argv[1]);
        SdfData sdf = SdfData::parseFile(argv[2]);
        TimingGraph g(nl, sdf);
        StaEngine engine(g, sdf, c);
        engine.run();
        auto ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();

        int cellArcs = 0;
        for (const auto& a : g.arcs()) if (a.kind == ArcKind::Cell) ++cellArcs;
        std::printf("Design %s: %zu instances, %zu pins/ports, %zu arcs (%d cell, %zu net)\n",
                    nl.moduleName().c_str(), nl.instances().size(), g.nodes().size(),
                    g.arcs().size(), cellArcs, g.arcs().size() - cellArcs);
        if (g.netArcsWithoutSdf() > 0)
            std::printf("Warning: %d net arcs had no SDF interconnect delay (assumed 0)\n",
                        g.netArcsWithoutSdf());
        std::printf("Clock %s period %.4f ns, uncertainty %.3f ns\n", c.clockPort.c_str(), c.period,
                    c.uncertainty);
        std::printf("Endpoints checked: %zu   WNS: %.3f ns   TNS: %.3f ns   Violations: %d\n",
                    engine.endpoints().size(), engine.wns(), engine.tns(), engine.violations());
        std::printf("Minimum clock period: %.4f ns (Fmax %.0f MHz)\n", engine.minimumPeriod(),
                    1000.0 / engine.minimumPeriod());
        std::printf("Runtime: %.1f ms\n", ms);

        for (int i = 0; i < nPaths && i < static_cast<int>(engine.endpoints().size()); ++i)
            printPath(engine.endpoints()[i], i + 1);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }
    return 0;
}
