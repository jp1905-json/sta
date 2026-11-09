// netlist.h - gate-level structural Verilog netlist model and parser.
#pragma once

#include <map>
#include <string>
#include <vector>

namespace sta {

// One placed standard cell: its library cell type and pin -> net connections.
class Instance {
public:
    Instance(std::string name, std::string cellType)
        : name_(std::move(name)), cellType_(std::move(cellType)) {}

    const std::string& name() const { return name_; }
    const std::string& cellType() const { return cellType_; }
    const std::map<std::string, std::string>& connections() const { return conns_; }

    void connect(const std::string& pin, const std::string& net) { conns_[pin] = net; }

    // Pin direction is inferred from the TSMC standard-cell naming convention,
    // since the flow does not hand us a Liberty file.
    static bool isOutputPin(const std::string& pin);

private:
    std::string name_;
    std::string cellType_;
    std::map<std::string, std::string> conns_;
};

// A flat, single-module netlist (what Innovus writes with saveNetlist).
class Netlist {
public:
    static Netlist parseFile(const std::string& path);
    static Netlist parseString(const std::string& text);

    const std::string& moduleName() const { return module_; }
    const std::vector<std::string>& inputs() const { return inputs_; }
    const std::vector<std::string>& outputs() const { return outputs_; }
    const std::vector<Instance>& instances() const { return instances_; }

private:
    void addPorts(const std::string& stmt, std::vector<std::string>& out);
    void addInstance(const std::string& stmt);

    std::string module_;
    std::vector<std::string> inputs_;   // bus ports are expanded to bits: inp_a[15] ...
    std::vector<std::string> outputs_;
    std::vector<Instance> instances_;
};

// Removes Verilog/SDF escape characters so both files name objects the same way:
// Verilog "\b_reg_reg[15] " and SDF "b_reg_reg\[15\]" both become "b_reg_reg[15]".
std::string normalizeName(const std::string& raw);

}  // namespace sta
