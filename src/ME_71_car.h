#ifndef ME71_CAR
#define ME71_CAR
#include <Arduino.h>
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

const int IR_PINS[3] = {5,6,7};
const int TOF_XSHUT[3] = {8,9,10};
const int TOF_IDS[3] = {51,52,53};

#define RGB_PIN 40

#define I2C_SDA 21
#define I2C_SCL 22

//Define LCD Constants
const int LCD_COLS = 16;
const int LCD_LINES = 2;
const unsigned long SCROLL_DELAY_MS = 300;

class Car{
    public:
        //Constructor
        Car ();
        
        //Sensors
        std::array<float, 3> get_tof_dist_mm();
        std::array<int, 3>  get_IR_values();
        
        //Peripherals
        void print_to_lcd(std::string msg, int line);
        void set_rgb(int r, int g, int b);
        void set_led_power(int power);

        //Motor
        void set_r_speed(float spd);
        void set_l_speed(float spd);

        void set_r_dir(bool setReverse);
        void set_l_dir(bool setReverse);

        void stop();

    private:
        // Motors
        int _r_speed = 0;
        int _l_speed = 0;
        bool _l_dir = false;
        bool _r_dir = false;
        void init_motors();
        void update_motors();

        // LCD
        hd44780_I2Cexp _lcd;
        SemaphoreHandle_t _lcdMutex;
        TaskHandle_t _lcdTaskHandle;
        static void lcd_task(void* pvParameters);
        void init_lcd();
        struct LineState {
            std::string text;
            int offset;
            unsigned long lastScroll;
        };
        LineState _lineState[LCD_LINES];

        // TOF
        Adafruit_VL53L0X _tof[3];
        void init_tof();

        // IR
        std::array<int, 3> _irValues;
        SemaphoreHandle_t _irMutex;
        TaskHandle_t _irTaskHandle;
        static void ir_task(void* pvParameters);
        void init_ir();

        // RGB
        Adafruit_NeoPixel _rgb;
        void init_rgb();
};
#endif
