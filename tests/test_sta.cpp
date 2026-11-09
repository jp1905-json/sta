// test_sta.cpp - unit tests with hand-computed expected values.
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>

#include "../src/netlist.h"
#include "../src/sdf.h"
#include "../src/sta_engine.h"
#include "../src/timing_graph.h"

using namespace sta;

static int failures = 0;
#define CHECK(cond)                                                              \
    do {                                                                         \
        if (!(cond)) { std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); ++failures; } \
    } while (0)
static bool near(double a, double b) { return std::fabs(a - b) < 1e-9; }

// F1 -> INV U1 -> NAND U2 -> F2.  Exercises negative unateness and a negative setup value.
static const char* kNetlist = R"(
module tiny (clk, din, sel);
   input clk;
   input din;
   input sel;
   wire q1, n1, n2;
   DFQD1BWP16P90 \F1 (.CP(clk), .D(din), .Q(q1));
   INVD1BWP16P90 U1 (.I(q1), .ZN(n1));
   ND2D1BWP16P90 U2 (.A1(n1), .A2(sel), .ZN(n2));
   DFQD1BWP16P90 \F2 (.CP(clk), .D(n2), .Q());
endmodule
)";

static const char* kSdf = R"(
(DELAYFILE (SDFVERSION "3.0") (DESIGN "tiny") (TIMESCALE 1.0 ns)
  (CELL (CELLTYPE "tiny") (INSTANCE)
    (DELAY (ABSOLUTE
      (INTERCONNECT F1/Q U1/I (0.010::0.010) (0.020::0.020)))))
  (CELL (CELLTYPE "DFQD1BWP16P90") (INSTANCE F1)
    (DELAY (ABSOLUTE (IOPATH (posedge CP) Q (0.100::0.100) (0.120::0.120)))))
  (CELL (CELLTYPE "INVD1BWP16P90") (INSTANCE U1)
    (DELAY (ABSOLUTE (IOPATH I ZN (0.030::0.030) (0.040::0.040)))))
  (CELL (CELLTYPE "ND2D1BWP16P90") (INSTANCE U2)
    (DELAY (ABSOLUTE
      (COND A2==1'b1 (IOPATH A1 ZN (0.050::0.050) (0.060::0.060)))
      (IOPATH A2 ZN (0.001::0.001) (0.001::0.001)))))
  (CELL (CELLTYPE "DFQD1BWP16P90") (INSTANCE F2)
    (TIMINGCHECK
      (SETUPHOLD (posedge D) (COND CDN_SDFCHK (posedge CP)) (0.020::0.020) (0.0::0.0))
      (SETUPHOLD (negedge D) (COND CDN_SDFCHK (posedge CP)) (-0.005::-0.005) (0.0::0.0))))
)
)";

static void testHandComputedSlack() {
    Netlist nl = Netlist::parseString(kNetlist);
    SdfData sdf = SdfData::parseString(kSdf);
    TimingGraph g(nl, sdf);
    Constraints c;
    c.period = 1.0;
    StaEngine e(g, sdf, c);
    e.run();

    // F1/Q: rise 0.100, fall 0.120. Wire adds 0.010 / 0.020 -> U1/I rise 0.110, fall 0.140.
    // INV (negative): ZN rise = 0.140 + 0.030 = 0.170, ZN fall = 0.110 + 0.040 = 0.150.
    // NAND (negative): ZN rise = 0.150 + 0.050 = 0.200, ZN fall = 0.170 + 0.060 = 0.230.
    // Rise: required 1.000 - 0.020 = 0.980 -> slack 0.780.
    // Fall: required 1.000 + 0.005 = 1.005 -> slack 0.775 (worst).
    CHECK(e.endpoints().size() == 1);
    const auto& r = e.endpoints().front();
    CHECK(r.endpoint == "F2/D");
    CHECK(r.trans == kFall);
    CHECK(near(r.arrival, 0.230));
    CHECK(near(r.required, 1.005));
    CHECK(near(e.wns(), 0.775));
    CHECK(near(e.minimumPeriod(), 0.225));
    CHECK(r.startpoint == "F1/Q");
    CHECK(e.violations() == 0);

    // At a 0.2 ns clock the same path must fail by exactly 0.025 ns.
    c.period = 0.2;
    StaEngine tight(g, sdf, c);
    tight.run();
    CHECK(near(tight.wns(), -0.025));
    CHECK(tight.violations() == 1);
}

static void testUnateness() {
    CHECK(CellLibrary::family("OAI21D2BWP16P90LVT") == "OAI21");
    CHECK(CellLibrary::family("CKND2D4BWP16P90LVT") == "CKND2");
    CHECK(CellLibrary::family("CKND8BWP16P90LVT") == "CKN");
    CHECK(CellLibrary::sense("INVD1BWP16P90LVT", "I", "ZN") == Sense::Negative);
    CHECK(CellLibrary::sense("BUFFD8BWP16P90LVT", "I", "Z") == Sense::Positive);
    CHECK(CellLibrary::sense("XOR2D1BWP16P90LVT", "A1", "Z") == Sense::NonUnate);
    CHECK(CellLibrary::sense("IND2D1BWP16P90LVT", "A1", "ZN") == Sense::Positive);
    CHECK(CellLibrary::sense("IND2D1BWP16P90LVT", "B1", "ZN") == Sense::Negative);
    CHECK(CellLibrary::sense("FA1D1BWP16P90LVT", "A", "CO") == Sense::Positive);
    CHECK(CellLibrary::sense("FA1D1BWP16P90LVT", "A", "S") == Sense::NonUnate);
}

static void testLoopDetection() {
    const char* loop = R"(
module loop (a);
   input a;
   wire x, y;
   INVD1BWP16P90 U1 (.I(y), .ZN(x));
   INVD1BWP16P90 U2 (.I(x), .ZN(y));
endmodule
)";
    const char* sdf = R"((DELAYFILE
      (CELL (CELLTYPE "INVD1BWP16P90") (INSTANCE U1) (DELAY (ABSOLUTE (IOPATH I ZN (0.01::0.01) (0.01::0.01)))))
      (CELL (CELLTYPE "INVD1BWP16P90") (INSTANCE U2) (DELAY (ABSOLUTE (IOPATH I ZN (0.01::0.01) (0.01::0.01))))))
    )";
    Netlist nl = Netlist::parseString(loop);
    SdfData s = SdfData::parseString(sdf);
    TimingGraph g(nl, s);
    bool threw = false;
    try { g.topologicalOrder(); } catch (const std::runtime_error&) { threw = true; }
    CHECK(threw);
}

static void testNameNormalization() {
    CHECK(normalizeName("\\b_reg_reg[15] ") == "b_reg_reg[15]");
    CHECK(normalizeName("b_reg_reg\\[15\\]/CP") == "b_reg_reg[15]/CP");
    Netlist nl = Netlist::parseString("module m (a, y); input [3:0] a; output y; endmodule");
    CHECK(nl.inputs().size() == 4);
    CHECK(nl.inputs().front() == "a[3]");
}

int main() {
    testHandComputedSlack();
    testUnateness();
    testLoopDetection();
    testNameNormalization();
    if (failures == 0) std::printf("All tests passed\n");
    return failures == 0 ? 0 : 1;
}
