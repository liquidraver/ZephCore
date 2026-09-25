// helpers/ui/telemetry_text.cpp: the joystick UI's rendering of a repeater's telemetry.
#include "test.h"
#include "CayenneLPP.h"
#include "ui/telemetry_text.h"

#include <string>

namespace {
std::string text(CayenneLPP &l) {
    char out[256];
    telemetry_text(l.getBuffer(), l.getSize(), out, sizeof(out));
    return out;
}
}

TEST(ttext_server_order, "UNIT-TTEXT-001", "A repeater's telemetry in slice-10 order renders every field, GPS second") {
    CayenneLPP l(180);
    l.addVoltage(1, 4.22f);
    l.addGPS(1, 47.1234f, -19.5f, 91.7f);
    l.addLuminosity(1, 3);
    l.addTemperature(2, -2.5f);
    l.addRelativeHumidity(2, 45.5f);
    l.addBarometricPressure(2, 1013.2f);
    l.addAltitude(2, 120.0f);
    l.addTemperature(1, 23.3f);
    CHECK(text(l) == "batt 4.22V\ngps 47.1234,-19.5000\nlight 3\nc2 temp -2.5C\n"
                     "c2 hum 45.5%\nc2 pres 1013.2hPa\nc2 alt 120m\ntemp 23.3C");
}

TEST(ttext_power, "UNIT-TTEXT-002", "Power monitor: channel voltage is not 'batt', current is signed") {
    CayenneLPP l(180);
    l.addVoltage(3, 12.10f);
    l.addCurrent(3, -0.0125f);
    l.addPower(3, 2.0f);
    CHECK(text(l) == "c3 12.10V\nc3 cur -13mA\nc3 pwr 2W");
}

TEST(ttext_unknown, "UNIT-TTEXT-003", "A type the screen does not show is skipped, not the end; empty is 'no telemetry'") {
    CayenneLPP l(180);
    l.addGenericSensor(4, 123456.0f);
    l.addVoltage(1, 3.70f);
    CHECK(text(l) == "batt 3.70V");
    CayenneLPP e(10);
    CHECK(text(e) == "no telemetry");
}
