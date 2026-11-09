CXX      ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Wpedantic
SRC      := src/netlist.cpp src/sdf.cpp src/timing_graph.cpp src/sta_engine.cpp

all: sta test_sta

sta: $(SRC) src/main.cpp src/*.h
	$(CXX) $(CXXFLAGS) -o $@ $(SRC) src/main.cpp

test_sta: $(SRC) tests/test_sta.cpp src/*.h
	$(CXX) $(CXXFLAGS) -o $@ $(SRC) tests/test_sta.cpp

test: test_sta
	./test_sta

clean:
	rm -f sta test_sta

.PHONY: all test clean
