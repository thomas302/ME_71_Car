#ifndef _VEHICLE_H__
#define _VEHICLE_H__

#include <Arduino.h>         // uint8_t and friends
#include <SoftwareSerial.h>  // SoftwareSerial is a type alias on ESP32 (EspSoftwareSerial),
                              // not a class -- it can't be forward-declared, needs the real header.

// ----------------------------------------------------------------
// Serial link to the ACB SmartCar V2 motor driver board.
// Reuses GPIO 17/16 (previously STCP_PIN/EN_PIN in the shift-register
// scheme). Those pins are unused now that the 74HC595 path is gone --
// if you need them for something else, move TX_PIN/RX_PIN elsewhere.
// ----------------------------------------------------------------
#define TX_PIN 17
#define RX_PIN 16

// ----------------------------------------------------------------
// Direction-byte bit masks. ME_71_car.cpp's update_motors() builds
// dirByte out of these -- unchanged from the original header, so
// nothing in ME_71_car.cpp/.h needs to change.
// ----------------------------------------------------------------
const int M1_Forward        = 128;
const int M1_Backward       = 64;
const int M2_Forward        = 32;
const int M2_Backward       = 16;
const int M3_Forward        = 2;
const int M3_Backward       = 4;
const int M4_Forward        = 1;
const int M4_Backward       = 8;

class vehicle
{
    public:
        void Init();
        // Dir:    bitmask built from the M#_Forward / M#_Backward constants above
        // Speed1: left-side speed (applied to M1 & M2), 0-255
        // Speed2: right-side speed (applied to M3 & M4), 0-255
        void Move(int Dir, int Speed1, int Speed2);

    private:
        // speed: signed, -255..255. Sign selects direction, magnitude selects speed.
        void motorControl(uint8_t motor, int speed);

        SoftwareSerial* _serial = nullptr;
};
#endif