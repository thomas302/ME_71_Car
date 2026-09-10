#include <ME_71_car.h>
#include <format>
#include <string>
using namespace std;


void setup() {
  car_init();

  set_rgb(100,100,100);
  set_led_power(20);

  delay(2000);
}

void loop() {
  auto IR_values =  get_IR_values();
  int TOF_value = get_tof_dist_mm();

  auto tof_str = format("TOF:{} IR L:{}", TOF_value, IR_values[0]);
  print_to_lcd(tof_str, 0);

  auto ir_str = format("C: {} R: {}", IR_values[1], IR_values[2]);
  print_to_lcd(ir_str, 1);

  if (IR_values[0] < 3500){
    set_rgb(250,0,0); // Red
  }
  else if (IR_values[1] < 3500){
    set_rgb(255, 191, 0); // Yellow
  }
  else if (IR_values[2] < 3500){
    set_rgb(0, 110, 42); // Green
  }
  else {
    set_rgb(50, 50, 50); // white
  }

}
