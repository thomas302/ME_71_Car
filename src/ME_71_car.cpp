
#include "ME_71_car.h"
#include "vehicle.h"
Preferences prefs;

// ----------------------------------------------------------------
// Forward declarations of internal helpers
// ----------------------------------------------------------------
static void init_lcd();
static void init_tof();
static void init_ir();
static void init_rgb();
static void init_motors();

static void lcd_task(void* pvParameters);
static void ir_task(void* pvParameters);
static void sr_write(uint8_t dirByte);

vehicle vh;

Adafruit_VL53L0X create_tof_sensor() {
    Adafruit_VL53L0X tof;

    Wire2.begin(14, 13);
    // Wire2.setClock(100000);

    tof.begin(
        0x29,
        false,
        &Wire2,
        Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_ACCURACY
    );

    return tof;
}

// ----------------------------------------------------------------
// Public: car_init
// ----------------------------------------------------------------
void car_init() {
    Serial.begin(115200);

    analogSetPinAttenuation(IR_PINS[0], ADC_11db);
    analogSetPinAttenuation(IR_PINS[1], ADC_11db);
    analogSetPinAttenuation(IR_PINS[2], ADC_11db);

    pinMode(IR_PINS[0], INPUT);
    pinMode(IR_PINS[1], INPUT);
    pinMode(IR_PINS[2], INPUT);

    pinMode(TOF_XSHUT, OUTPUT);

    Wire.begin(I2C_SDA, I2C_SCL);
	Wire2.begin(14, 13);
	Wire2.setClock(100000);
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
static std::string        _pendingText[LCD_LINES];

static void lcd_task(void* pvParameters) {
    unsigned long lastUpdate[LCD_LINES] = {};
    for (;;) {
        unsigned long now = millis();
        for (int line = 0; line < LCD_LINES; line++) {
            if (xSemaphoreTake(_lcdMutex, 0)) {
                LineState& ls = _lineState[line];

                bool dwellActive = ls.dwelling && (now - ls.dwellStart < 400);

                if (!dwellActive && now - lastUpdate[line] >= 1000) {
                    if (_pendingText[line] != ls.text) {
                        ls.text       = _pendingText[line];
                        ls.offset     = 0;
                        ls.lastScroll = now;
                        ls.dwelling   = false;
                        _lcd.setCursor(0, line);
                        std::string view = ls.text.substr(0, LCD_COLS);
                        while ((int)view.size() < LCD_COLS) view += ' ';
                        _lcd.print(view.c_str());
                    }
                    lastUpdate[line] = now;
                }

                if (!dwellActive && (int)ls.text.size() > LCD_COLS &&
                    now - ls.lastScroll >= SCROLL_DELAY_MS) {
                    
                    if (ls.offset >= (int)ls.text.size() - LCD_COLS) {
                        // reached the end, start dwell
                        ls.dwelling   = true;
                        ls.dwellStart = now;
                        ls.offset     = 0;
                    } else {
                        ls.lastScroll = now;
                        ls.offset++;
                    }

                    if (!ls.dwelling) {
                        _lcd.setCursor(0, line);
                        std::string view = ls.text.substr(ls.offset, LCD_COLS);
                        while ((int)view.size() < LCD_COLS) view += ' ';
                        _lcd.print(view.c_str());
                    }
                }

                if (ls.dwelling && !dwellActive) {
                    ls.dwelling = false;
                    // render from offset 0
                    _lcd.setCursor(0, line);
                    std::string view = ls.text.substr(0, LCD_COLS);
                    while ((int)view.size() < LCD_COLS) view += ' ';
                    _lcd.print(view.c_str());
                }

                xSemaphoreGive(_lcdMutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
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
        _pendingText[line] = str;
        xSemaphoreGive(_lcdMutex);
    }
}

// ----------------------------------------------------------------
// TOF
// ----------------------------------------------------------------

int offset = 0;
static void init_tof() {
	pinMode(TOF_XSHUT, OUTPUT);
	digitalWrite(TOF_XSHUT, LOW);
    delay(50);
	int i = 0;
	digitalWrite(TOF_XSHUT, HIGH);
	delay(100);
	
	prefs.begin("TOF", false);
	offset = prefs.getShort("offset", 0);
	prefs.end();
	
	_tof_ok = _tof.begin(0x29, false, &Wire2, Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_ACCURACY);
	
	Serial.printf("TOF %d: %s\n", i, _tof_ok ? "OK" : "FAILED");
	delay(200);
}

static int last_val = -1;     // last good corrected reading, -1 until the first one

int get_tof_dist_mm(bool calibration) {
    if (!_tof_ok) return -1;

    VL53L0X_RangingMeasurementData_t measure;
    _tof.rangingTest(&measure, false);
	int raw = measure.RangeMilliMeter;
    bool ok = (measure.RangeStatus == 0) && raw > 5 && raw < 2000;

    if (calibration) {
        return ok ? raw : -1;   // raw value, no offset, no held value
    }

    if (ok) last_val = raw + offset;
    return last_val;                                // bad read -> last good value
}

// ----------------------------------------------------------------
// IR
// ----------------------------------------------------------------
static void ir_task(void* pvParameters) {
    std::array<int, IR_SAMPLES> samples[3] = {};
    int idx = 0;

    for (;;) {
        for (int i = 0; i < 3; i++) samples[i][idx] = analogRead(IR_PINS[i]);
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
int map_clip(int in, int imin = 0, int imax = 4096, int omin = 0, int omax = 1000) {
  int mapped = map(in, imin, imax, omin, omax);
  return max(min(omin, omax), min(max(omin, omax), mapped));
}

int imin[3] = {0,0,0};
static void init_ir() {
	
	prefs.begin("IR", false);
	imin[0] = prefs.getShort("L_WHITE");
	imin[1] = prefs.getShort("M_WHITE");
	imin[2] = prefs.getShort("R_WHITE");
	prefs.end();
	pinMode(35, INPUT);
	pinMode(36, INPUT);
	pinMode(39, INPUT);
	
    _irMutex = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(ir_task, "ir_task", 2048, nullptr, 1, &_irTaskHandle, 0);
}

std::array<int, 3> get_IR_values() {
    std::array<int, 3> result;
    if (xSemaphoreTake(_irMutex, portMAX_DELAY)) {
        result = _irValues;
		result[0] = map_clip(result[0], imin[0]); 
		result[1] = map_clip(result[1], imin[1]); 
		result[2] = map_clip(result[2], imin[2]); 
        xSemaphoreGive(_irMutex);
    }
    return result;
}

// ----------------------------------------------------------------
// RGB
// ----------------------------------------------------------------
static void init_rgb() {
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
    vh.Init();
}

// Write a direction byte to the shift register
static void sr_write(uint8_t dirByte) {
    digitalWrite(STCP_PIN, LOW);
    shiftOut(DATA_PIN, SHCP_PIN, MSBFIRST, dirByte);
    digitalWrite(STCP_PIN, HIGH);
}

static void update_motors() {
    // Determine direction byte from left/right dir flags
    uint8_t dirByte;

    bool stopped = (_l_speed == 0 && _r_speed == 0);
	int l_forward = M1_Forward + M2_Forward;
	int r_forward = M3_Forward + M4_Forward;
	
	int l_reverse = M1_Backward + M2_Backward;
	int r_reverse = M3_Backward + M4_Backward;
	
	
	uint8_t r_dir = (!_r_dir) ? r_forward : r_reverse;
	uint8_t l_dir = (!_l_dir) ? l_forward : l_reverse;
	
	dirByte = r_dir | l_dir;

    if (stopped) {
		_l_speed = 0;
		_r_speed = 0;
		dirByte = MTRS_STOP;
	}
	
	vh.Move(dirByte, _l_speed, _r_speed);
}

void set_r_speed(float spd) {
	spd = constrain(spd, 0, 100);
	spd = map(spd, 0, 100, 50, 100);
	if (spd < 52.5) spd = 0;
	
    _r_speed = (int)(( spd/ 100.0f) * (256 -1));
    update_motors();
}

void set_l_speed(float spd) {
	spd = constrain(spd, 0, 100);
	spd = map(spd, 0, 100, 50, 100);
	if (spd < 52.5) spd = 0;
	
	
    _l_speed = (int)((spd / 100.0f) * (256 -1));
    update_motors();
}

void set_r_reverse(bool setReverse) {
    _r_dir = setReverse;
    update_motors();
}

void set_l_reverse(bool setReverse) {
    _l_dir = setReverse;
    update_motors();
}

void stop() {
    set_l_speed(0);
    set_r_speed(0);
}


