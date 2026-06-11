#include "ME_71_car.h"
#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>

// ----------------------------------------------------------------
// Internal state (hidden from students)
// ----------------------------------------------------------------

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

// ----------------------------------------------------------------
// Forward declarations of internal helpers
// ----------------------------------------------------------------
static void init_lcd();
static void init_tof();
static void init_ir();
static void init_rgb();
static void init_motors();
static void update_motors();
static void lcd_task(void* pvParameters);
static void ir_task(void* pvParameters);

// ----------------------------------------------------------------
// Public: car_init
// ----------------------------------------------------------------
void car_init() {
    Serial.begin(115200);

    pinMode(MTR_R_ONE, OUTPUT);
    pinMode(MTR_R_TWO, OUTPUT);
    pinMode(MTR_L_ONE, OUTPUT);
    pinMode(MTR_L_TWO, OUTPUT);

    pinMode(IR_PINS[0], INPUT);
    pinMode(IR_PINS[1], INPUT);
    pinMode(IR_PINS[2], INPUT);

    pinMode(TOF_XSHUT[0], OUTPUT);
    pinMode(TOF_XSHUT[1], OUTPUT);
    pinMode(TOF_XSHUT[2], OUTPUT);

    Wire.begin(I2C_SDA, I2C_SCL);
    delay(100);

    init_lcd();
    delay(100);
    print_to_lcd("LCD Initialized", 0);

    init_tof();
    print_to_lcd("TOF Initialized", 0);
    delay(100);

    init_ir();
    print_to_lcd("IR Initialized", 0);
    delay(100);

    init_rgb();
    print_to_lcd("LED Initialized", 0);
    delay(100);

    init_motors();
    print_to_lcd("Motors Initialized", 0);
    delay(100);
}

// ----------------------------------------------------------------
// LCD
// ----------------------------------------------------------------
static void lcd_task(void* pvParameters) {
    for (;;) {
        unsigned long now = millis();
        for (int line = 0; line < LCD_LINES; line++) {
            if (xSemaphoreTake(_lcdMutex, 0)) {
                LineState& ls = _lineState[line];
                if ((int)ls.text.size() > LCD_COLS &&
                    now - ls.lastScroll >= SCROLL_DELAY_MS) {

                    ls.lastScroll = now;
                    ls.offset++;
                    if (ls.offset > (int)ls.text.size() - LCD_COLS)
                        ls.offset = 0;

                    _lcd.setCursor(0, line);
                    std::string view = ls.text.substr(ls.offset, LCD_COLS);
                    while ((int)view.size() < LCD_COLS) view += ' ';
                    _lcd.print(view.c_str());
                }
                xSemaphoreGive(_lcdMutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void init_lcd() {
    _lcd.begin(LCD_COLS, LCD_LINES);
    _lcdMutex = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(lcd_task, "lcd_task", 2048, nullptr, 1, &_lcdTaskHandle, 0);
}

void print_to_lcd(std::string str, int line) {
    if (line < 0 || line >= LCD_LINES) return;
    if (xSemaphoreTake(_lcdMutex, portMAX_DELAY)) {
        _lineState[line].text      = str;
        _lineState[line].offset    = 0;
        _lineState[line].lastScroll = millis();

        _lcd.setCursor(0, line);
        std::string view = str.substr(0, LCD_COLS);
        while ((int)view.size() < LCD_COLS) view += ' ';
        _lcd.print(view.c_str());

        xSemaphoreGive(_lcdMutex);
    }
}

// ----------------------------------------------------------------
// TOF
// ----------------------------------------------------------------
static void init_tof() {
    for (int i = 0; i < 3; i++) {
        pinMode(TOF_XSHUT[i], OUTPUT);
        digitalWrite(TOF_XSHUT[i], LOW);
    }
    delay(10);

    for (int i = 0; i < 3; i++) {
        digitalWrite(TOF_XSHUT[i], HIGH);
        delay(100);
        if (!_tof[i].begin(TOF_IDS[i], &Wire)) {
            Serial.printf("TOF %d failed\n", i);
        }
    }
}

std::array<float, 3> get_tof_dist_mm() {
    std::array<float, 3> distances;
    for (int i = 0; i < 3; i++) {
        VL53L0X_RangingMeasurementData_t measure;
        _tof[i].rangingTest(&measure, false);
        distances[i] = (measure.RangeStatus != 4) ? measure.RangeMilliMeter : -1.0f;
    }
    return distances;
}

// ----------------------------------------------------------------
// IR
// ----------------------------------------------------------------
static void ir_task(void* pvParameters) {
    std::array<int, IR_SAMPLES> samples[3] = {};
    int idx = 0;

    for (;;) {
        for (int i = 0; i < 3; i++)
            samples[i][idx] = analogRead(IR_PINS[i]);
        idx = (idx + 1) % IR_SAMPLES;

        if (xSemaphoreTake(_irMutex, 0)) {
            for (int i = 0; i < 3; i++) {
                int sum = 0;
                for (int j = 0; j < IR_SAMPLES; j++) sum += samples[i][j];
                _irValues[i] = sum / IR_SAMPLES;
            }
            xSemaphoreGive(_irMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

static void init_ir() {
    _irMutex = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(ir_task, "ir_task", 2048, nullptr, 1, &_irTaskHandle, 0);
}

std::array<int, 3> get_IR_values() {
    std::array<int, 3> result;
    if (xSemaphoreTake(_irMutex, portMAX_DELAY)) {
        result = _irValues;
        xSemaphoreGive(_irMutex);
    }
    return result;
}

// ----------------------------------------------------------------
// RGB
// ----------------------------------------------------------------
static void init_rgb() {
    _rgb = Adafruit_NeoPixel(1, RGB_PIN, NEO_GRB + NEO_KHZ800);
    _rgb.begin();
    _rgb.setBrightness(100);
    _rgb.clear();
    _rgb.show();
}

void set_rgb(int r, int g, int b) {
    _rgb.setPixelColor(0, _rgb.Color(r, g, b));
    _rgb.show();
}

void set_led_power(int power) {
    power = constrain(power, 0, 255);
    _rgb.setBrightness(power);
    _rgb.show();
}

// ----------------------------------------------------------------
// Motors
// ----------------------------------------------------------------
static void init_motors() {
    // TODO: implement once driver is known
}

static void update_motors() {
    // TODO: implement once driver is known
    // L9110S example:
    //   digitalWrite(MTR_R_ONE, _r_dir);
    //   analogWrite(MTR_R_TWO, _r_speed);
    //   digitalWrite(MTR_L_ONE, _l_dir);
    //   analogWrite(MTR_L_TWO, _l_speed);
}

void set_r_speed(float spd) {
    _r_speed = (int)(constrain(spd, 0, 100) / 100.0f * 255);
    update_motors();
}

void set_l_speed(float spd) {
    _l_speed = (int)(constrain(spd, 0, 100) / 100.0f * 255);
    update_motors();
}

void set_r_dir(bool setReverse) {
    _r_dir = setReverse;
    update_motors();
}

void set_l_dir(bool setReverse) {
    _l_dir = setReverse;
    update_motors();
}

void stop() {
    set_l_speed(0);
    set_r_speed(0);
}
