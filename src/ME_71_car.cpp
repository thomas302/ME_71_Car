#include "ME_71_car.h"
#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h> 

Car::Car(){
    Serial.begin(115200);

    // Set pin modes, declare interfaces
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
    print_to_lcd("TOF Sensors Initialized", 0);
    delay(100); 

    init_ir();
    print_to_lcd("IR Sensors Initialized", 0);
    delay(100);

    init_rgb();
    print_to_lcd("LED Initialized", 0);
    delay(100);

    init_motors();
    print_to_lcd("Motor Initialized", 0);
    delay(100);


}

void Car::init_lcd(){
    int status = _lcd.begin(LCD_COLS, LCD_LINES);
    _lcdMutex = xSemaphoreCreateMutex();

    xTaskCreatePinnedToCore(
        lcd_task,        // task function
        "lcd_task",      // name (for debugging)
        2048,            // stack size in bytes
        this,            // pass Car instance as parameter
        1,               // priority
        &_lcdTaskHandle, // handle
        0                // pin to Core 0
    );

}

void Car::lcd_task(void* pvParameters) {
    Car* car = static_cast<Car*>(pvParameters);
    
    for (;;) {
        unsigned long now = millis();
        
        for (int line = 0; line < LCD_LINES; line++) {
            if (xSemaphoreTake(car->_lcdMutex, 0)) {  // non-blocking
                LineState& ls = car->_lineState[line];
                if ((int)ls.text.size() > LCD_COLS &&
                    now - ls.lastScroll >= SCROLL_DELAY_MS) {

                    ls.lastScroll = now;
                    ls.offset++;
                    if (ls.offset > (int)ls.text.size() - LCD_COLS)
                        ls.offset = 0;

                    car->_lcd.setCursor(0, line);
                    std::string view = ls.text.substr(ls.offset, LCD_COLS);
                    while ((int)view.size() < LCD_COLS) view += ' ';
                    car->_lcd.print(view.c_str());
                }
                xSemaphoreGive(car->_lcdMutex);
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void Car::print_to_lcd(std::string str, int line) {
    if (line < 0 || line >= LCD_LINES) return;

    if (xSemaphoreTake(_lcdMutex, portMAX_DELAY)) {
        _lineState[line].text = str;
        _lineState[line].offset = 0;
        _lineState[line].lastScroll = millis();

        // Render first frame immediately
        _lcd.setCursor(0, line);
        std::string view = str.substr(0, LCD_COLS);
        while ((int)view.size() < LCD_COLS) view += ' ';
        _lcd.print(view.c_str());

        xSemaphoreGive(_lcdMutex);
    }
}

void Car::init_tof() {
    // Shut down all sensors first
    for (int i = 0; i < 3; i++) {
        pinMode(TOF_XSHUT[i], OUTPUT);
        digitalWrite(TOF_XSHUT[i], LOW);
    }
    delay(10);

    // Bring up each sensor one at a time and assign unique address
    for (int i = 0; i < 3; i++) {
        digitalWrite(TOF_XSHUT[i], HIGH);
        delay(100);

        if (!_tof[i].begin(TOF_IDS[i], &Wire)) {
            // sensor i failed to init — handle error here
            Serial.printf("TOF %d failed\n", i);
        }
    }
}

std::array<float, 3> Car::get_tof_dist_mm() {
    std::array<float, 3> distances;
    
    for (int i = 0; i < 3; i++) {
        VL53L0X_RangingMeasurementData_t measure;
        _tof[i].rangingTest(&measure, false);
        
        if (measure.RangeStatus != 4) {  // status 4 = out of range/error
            distances[i] = measure.RangeMilliMeter;
        } else {
            distances[i] = -1.0f;  // sentinel for invalid reading
        }
    }
    
    return distances;
}

void Car::init_ir(){
    _irMutex = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(ir_task, "ir_task", 2048, this, 1, &_irTaskHandle, 0);
}

void Car::ir_task(void* pvParameters) {
    Car* car = static_cast<Car*>(pvParameters);
    std::array<int, 3> samples[3] = {};
    int idx = 0;

    for (;;) {
        // Take one sample across all 3 sensors
        for (int i = 0; i < 3; i++) {
            samples[i][idx] = analogRead(IR_PINS[i]);
        }
        idx = (idx + 1) % 3;  // ring buffer

        // Compute averages and update shared state
        if (xSemaphoreTake(car->_irMutex, 0)) {
            for (int i = 0; i < 3; i++) {
                int sum = 0;
                for (int j = 0; j < 3; j++) sum += samples[i][j];
                car->_irValues[i] = sum / 3;
            }
            xSemaphoreGive(car->_irMutex);
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

std::array<int, 3> Car::get_IR_values() {
    std::array<int, 3> result;
    if (xSemaphoreTake(_irMutex, portMAX_DELAY)) {
        result = _irValues;
        xSemaphoreGive(_irMutex);
    }
    return result;
}

void Car::init_rgb() {
    _rgb.begin();
    _rgb.setBrightness(100);  // 0-255, 50 is a good default to avoid blinding
    _rgb.clear();
    _rgb.show();
}

void Car::set_rgb(int r, int g, int b) {
    _rgb.setPixelColor(0, _rgb.Color(r, g, b));
    _rgb.show();
}

void Car::set_led_power(int power){
    power = power >= 0 ? (power <= 255 ? power: 255): 0;
    _rgb.setBrightness(power);
    _rgb.show();
}

void Car::init_motors() {
    // TODO: setup once driver is known
}

void Car::set_r_speed(float spd) {
    spd = constrain(spd, 0, 100);
    _r_speed = (int)(spd / 100.0f * 255);
    update_motors();
}

void Car::set_l_speed(float spd) {
    spd = constrain(spd, 0, 100);
    _l_speed = (int)(spd / 100.0f * 255);
    update_motors();
}

void Car::set_r_dir(bool setReverse) {
    _r_dir = setReverse;
    update_motors();
}

void Car::set_l_dir(bool setReverse) {
    _l_dir = setReverse;
    update_motors();
}

void Car::stop(){
    set_l_speed(0);
    set_r_speed(0);
}

void Car::update_motors() {
    // TODO: implement once driver is known
    // L9110S example:
    //   digitalWrite(MTR_R_ONE, _r_dir);
    //   analogWrite(MTR_R_TWO, _r_speed);
    //   digitalWrite(MTR_L_ONE, _l_dir);
    //   analogWrite(MTR_L_TWO, _l_speed);
}
