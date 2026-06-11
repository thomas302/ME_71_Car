#ifndef ME71_CAR
#define ME71_CAR
#include <Arduino.h>
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
#define MTR_R_ONE 1
#define MTR_R_TWO 2
#define MTR_L_ONE 3
#define MTR_L_TWO 4

#define RGB_PIN  40
#define I2C_SDA  21
#define I2C_SCL  22

constexpr int IR_PINS[3]   = {5, 6, 7};
constexpr int TOF_XSHUT[3] = {8, 9, 10};
constexpr int TOF_IDS[3]   = {51, 52, 53};

// LCD / task constants
constexpr int          LCD_COLS        = 16;
constexpr int          LCD_LINES       = 2;
constexpr unsigned long SCROLL_DELAY_MS = 300;
constexpr int          IR_SAMPLES      = 3;

// --- Public API (no class needed — one car, always) ---
void car_init();

// Sensors
std::array<float, 3> get_tof_dist_mm();
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

#endif
