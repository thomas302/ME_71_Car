#include "vehicle.h"

void vehicle::Init() {
    _serial = new SoftwareSerial(RX_PIN, TX_PIN);
    _serial->begin(9600);
    pinMode(TX_PIN, OUTPUT);
    digitalWrite(TX_PIN, HIGH);

    // Match ACB_SmartCar_V2::Init()'s startup behavior: park all four motors.
    motorControl(1, 0);
    motorControl(2, 0);
    motorControl(3, 0);
    motorControl(4, 0);
}

void vehicle::motorControl(uint8_t motor, int speed) {
    if (_serial == nullptr) return;

    uint8_t dir = 0;
    uint8_t speed_value = 0;

    if (motor == 1 || motor == 2) {
        dir = speed > 0 ? 1 : 2;
        speed_value = map(abs(speed), 0, 255, 0, 100);
    }
    if (motor == 3 || motor == 4) {
        dir = speed > 0 ? 2 : 1;
        speed_value = map(abs(speed), 0, 255, 0, 100);
    }

    // Carried over from the original ACB_SmartCar_V2 driver code.
    // Unclear why 40 specifically is special-cased to a hard stop --
    // worth double-checking against the driver board's manual/firmware.
    if (speed_value == 40) speed_value = 0;

    _serial->write(motor);        // motor index
    _serial->write(dir);          // direction (1: forward, 2: reverse)
    _serial->write(speed_value);  // speed (0-100)
    _serial->write('\r');
    _serial->write('\n');
}

void vehicle::Move(int Dir, int Speed1, int Speed2) {
    // Decode the shift-register-style bitmask that ME_71_car.cpp's
    // update_motors() builds, into a signed per-motor speed, then
    // send each motor its own serial command.
    int m1 = 0, m2 = 0, m3 = 0, m4 = 0;

    if (Dir & M1_Forward)  m1 =  Speed1;
    if (Dir & M1_Backward) m1 = -Speed1;
    if (Dir & M2_Forward)  m2 =  Speed1;
    if (Dir & M2_Backward) m2 = -Speed1;
    if (Dir & M3_Forward)  m3 =  Speed2;
    if (Dir & M3_Backward) m3 = -Speed2;
    if (Dir & M4_Forward)  m4 =  Speed2;
    if (Dir & M4_Backward) m4 = -Speed2;

    motorControl(1, m1);
    motorControl(2, m2);
    motorControl(3, m3);
    motorControl(4, m4);
}