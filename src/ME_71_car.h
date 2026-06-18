#include <Wire.h>
#include <array>
#include <string>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>
#include "Adafruit_VL53L0X.h"
#include <Adafruit_NeoPixel.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

//Define Pins
// QA052 Car Shield – shift register (74HC595) + L293D
#define SR_SHCP  18   // shift register clock
#define SR_EN    16   // enable (active LOW)
#define SR_DATA  5    // serial data
#define SR_STCP  17   // latch

// PWM speed pins (one per motor side, or one shared — check your board)
// QA052 exposes these; adjust to match your actual wiring
#define MTR_PWM_L  25   // left side PWM
#define MTR_PWM_R  26   // right side PWM

// LEDC channels
#define MTR_L_LEDC_CH  0
#define MTR_R_LEDC_CH  1
#define MTR_PWM_FREQ   1000
#define MTR_PWM_RES    8

// Direction bitmasks for the 74HC595 (4WD, 2 motors per side)
// Bit layout per QA052: M1–M4 mapped into the byte
// Forward  = left motors bits HIGH, right motors bits HIGH (correct polarity)
// Adjust these if motors run backward
#define DIR_FORWARD   0b10100101   // all 4 wheels forward
#define DIR_BACKWARD  0b01011010   // all 4 wheels backward
#define DIR_TURN_L    0b01100110   // left reverse, right forward
#define DIR_TURN_R    0b10011001   // left forward, right reverse
#define DIR_STOP      0b00000000

constexpr int IR_PINS[3]   = {35, 36, 39};
constexpr int TOF_XSHUT[3] = {4, 16, 17};
constexpr uint8_t TOF_IDS[3] = {0x30, 0x31, 0x32};
static bool _tof_ok[3] = {false, false, false};

// LCD / task constants
constexpr int          LCD_COLS        = 16;
constexpr int          LCD_LINES       = 2;
constexpr unsigned long SCROLL_DELAY_MS = 300;
constexpr int          IR_SAMPLES      = 3;

// LCD
static hd44780_I2Cexp  _lcd;
static SemaphoreHandle_t _lcdMutex;
static TaskHandle_t      _lcdTaskHandle;

struct LineState {
    std::string   text;
    int           offset;
    unsigned long lastScroll;
};
static LineState _lineState[LCD_LINES];

// TOF
static Adafruit_VL53L0X _tof[3];

// IR
static std::array<int, 3> _irValues;
static SemaphoreHandle_t  _irMutex;
static TaskHandle_t       _irTaskHandle;

// RGB
static Adafruit_NeoPixel _rgb;

// Motors
static int  _r_speed = 0;
static int  _l_speed = 0;
static bool _r_dir   = false;
static bool _l_dir   = false;

// --- Public API (no class needed — one car, always) ---
void car_init();

// Sensors
std::array<int, 3> get_tof_dist_mm();
std::array<int, 3>   get_IR_values();

// LCD
void print_to_lcd(std::string str, int line);

// LED
void set_rgb(int r, int g, int b);
void set_led_power(int power);

// Motors
void set_r_speed(float spd);
void set_l_speed(float spd);
void set_r_dir(bool setReverse);
void set_l_dir(bool setReverse);
void stop();
