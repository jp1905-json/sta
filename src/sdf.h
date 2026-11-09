// sdf.h - Standard Delay Format (SDF 3.0) reader for post-route delays.
#pragma once

#include <map>
#include <string>
#include <utility>

namespace sta {

// Rise and fall values for one arc, in ns.
struct RiseFall {
    double rise = 0.0;
    double fall = 0.0;
};

// A cell delay arc: input pin -> output pin. edgeTriggered marks
// "(IOPATH (posedge CP) Q ...)" launch arcs on flops.
struct CellArc {
    RiseFall delay;
    bool edgeTriggered = false;
};

// Setup requirement on a flop data pin for a rising and a falling D.
struct SetupCheck {
    std::string clockPin;  // e.g. "CP"
    RiseFall setup;        // rise = (posedge D), fall = (negedge D)
    bool haveRise = false;  // setup values may be negative, so track presence
    bool haveFall = false;
};

class SdfData {
public:
    static SdfData parseFile(const std::string& path);
    static SdfData parseString(const std::string& text);

    // Key "driver_pin_path->load_pin_path", e.g. "U1/ZN->U2/A1".
    const std::map<std::string, RiseFall>& interconnects() const { return interconnects_; }
    // Key "instance/in->out", e.g. "U835/A1->Z".
    const std::map<std::string, CellArc>& cellArcs() const { return cellArcs_; }
    // Key "instance/D".
    const std::map<std::string, SetupCheck>& setupChecks() const { return setups_; }

private:
    friend class SdfBuilder;
    std::map<std::string, RiseFall> interconnects_;
    std::map<std::string, CellArc> cellArcs_;
    std::map<std::string, SetupCheck> setups_;
};

}  // namespace sta
