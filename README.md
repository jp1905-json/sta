# mini-sta: a static timing analyzer in C++

A small setup-mode (late) static timing analyzer for post-route gate-level netlists.
It reads the structural Verilog netlist and SDF delay file that Cadence Innovus writes
after place-and-route, builds a pin-level timing graph, propagates rise/fall arrival
times in topological order, and reports slack, WNS/TNS, the minimum clock period, and
the worst paths.

Built and checked against my ECE 5746 Lab 3 design: a pipelined 16x16-bit multiplier
placed and routed in TSMC 16nm (1,251 cells).

## Build and run

```
make            # builds ./sta and ./test_sta
make test       # unit tests with hand-computed answers

./sta mult.apr.v mult.sdf --period 0.4548 --uncertainty 0.01 \
      --input-delay 0.01 --output-delay 0.01 --paths 3
```

## Design

| File | Class | Role |
|---|---|---|
| `src/netlist.*` | `Netlist`, `Instance` | Parses flat Verilog: ports (bus bits expanded), cell instances, pin-to-net connections, escaped names |
| `src/sdf.*` | `SdfData` | S-expression parser for SDF 3.0: INTERCONNECT, IOPATH (incl. `posedge` and `COND`), SETUPHOLD |
| `src/timing_graph.*` | `TimingGraph`, `CellLibrary` | Pin-level DAG of net arcs and cell arcs; arc unateness from the cell's logic family; Kahn topological sort with loop detection |
| `src/sta_engine.*` | `StaEngine` | Rise/fall arrival propagation, propagated clock latency, setup checks, slack, path backtrace, minimum period |
| `src/main.cpp` | | CLI and report formatting |

Key modelling choices:

- **Propagated clock.** The clock tree is timed like any other logic, so every flop gets its real clock arrival, and clock skew between launch and capture flops falls out of the analysis.
- **Unateness.** Inverting cells (INV, NAND, NOR, AOI, OAI) swap rise and fall, buffering cells keep them, and XOR/XNOR and adder sum outputs are treated as non-unate (worst of both).
- **Conditional arcs.** When the SDF gives state-dependent (`COND`) delays for one arc, the worst value is used. This is conservative.
- **Async pins.** Reset arcs (`CDN -> Q`) are excluded from data paths, as they are covered by recovery/removal checks.

## Results on the Lab 3 multiplier (SS corner, 0.72 V, -40 C, 0.4548 ns clock)

| Metric | mini-sta | Innovus `report_timing` |
|---|---|---|
| Worst setup slack | +0.097 ns | +0.105 ns |
| Worst endpoints | prod_reg[23], [22], [25], [26] | prod_reg[25], [23], [26] |
| Graph size | 4,320 pins, 5,963 arcs | |
| Runtime | ~40 ms | |

The worst path into `prod_reg[25]` ends in the same seven cells Innovus reports. The
8 ps difference comes from using the worst case of state-dependent SDF arcs where
Innovus uses the specific condition, so mini-sta is slightly pessimistic, which is the
safe direction for timing sign-off.

Implied minimum clock period: 0.358 ns (about 2.79 GHz) at this corner.

## Not implemented

Hold (early-mode) checks, multiple clocks, false/multicycle path exceptions, and
slew/load-based delay calculation from Liberty (delays come pre-computed from the SDF).
