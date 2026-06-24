#include <Wire.h>
static TwoWire Wire2(1);
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
#define I2C_SCL 22
#define I2C_SDA 21

#define SHCP_PIN            18                          
#define EN_PIN              16                            
#define DATA_PIN            5                           
#define STCP_PIN            17  

#define RGB_PIN 4

#define MTR_PWM_L  19
#define MTR_PWM_R  23 


#define LEFT_FORWARD  0b10100000
#define RIGHT_FORWARD 0b00001010
#define LEFT_REVERSE  0b01010000
#define RIGHT_REVERSE 0b00000101
#define MTRS_STOP      0b00000000


constexpr int IR_PINS[3]   = {35, 36, 39};
constexpr int TOF_XSHUT[3] = {25, 26, 27};
constexpr uint8_t TOF_IDS[3] = {0x30, 0x31, 0x32};

static bool _tof_ok[3] = {false, false, false};

// LCD / task constants
constexpr int          LCD_COLS        = 16;
constexpr int          LCD_LINES       = 2;
constexpr unsigned long SCROLL_DELAY_MS = 300;
constexpr int          IR_SAMPLES      = 3;

// LCD
static hd44780_I2Cexp     _lcd;
static SemaphoreHandle_t  _lcdMutex;
static TaskHandle_t       _lcdTaskHandle;

struct LineState {
    std::string   text;
    int           offset;
    unsigned long lastScroll;
	bool          dwelling   = false;
    unsigned long dwellStart = 0;
};
static LineState _lineState[LCD_LINES];


// TOF
static Adafruit_VL53L0X _tof[3];

// IR
static std::array<int, 3> _irValues;
static SemaphoreHandle_t  _irMutex;
static TaskHandle_t       _irTaskHandle;

// RGB
static Adafruit_NeoPixel _rgb(1, RGB_PIN, NEO_GRB + NEO_KHZ800);;

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