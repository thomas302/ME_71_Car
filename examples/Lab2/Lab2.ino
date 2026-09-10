#include <ME_71_car.h>
#include <format>
#include <string>
using namespace std;

void drive_forward(float spd, int time_ms) {
    set_r_dir(false);
    set_l_dir(false);
    set_r_speed(spd);
    set_l_speed(spd);
    delay(time_ms);
    stop();
}
void drive_reverse(float spd, int time_ms) {
    set_r_dir(true);
    set_l_dir(true);
    set_r_speed(spd);
    set_l_speed(spd);
    delay(time_ms);
    stop();
}
void turn_CCW(float spd, int time_ms) {
    set_r_dir(false);
    set_l_dir(true);
    set_r_speed(spd);
    set_l_speed(spd);
    delay(time_ms);
    stop();
}
void turn_CW(float spd, int time_ms) {
    set_r_dir(true);
    set_l_dir(false);
    set_r_speed(spd);
    set_l_speed(spd);
    delay(time_ms);
    stop();
}

void setup() {
  car_init();           // Initialize car
  /*
  // Lab 2 Part One
  set_r_speed(100);     // Sets the right and left speeds to 100%
  set_l_speed(100);
  set_r_dir(false);     // Sets the direction to forward
  set_l_dir(false);
  delay(2000);          // Wait 2 seconds
  stop();               // Stops the motors
  delay(2000);          // Waits 2 Seconds
  set_r_speed(50);      // Sets the right and left speeds to 50%
  set_l_speed(50);
  set_r_dir(true);      // Sets the direction to reverse
  set_l_dir(true);
  delay(2000);          // Waits 2 Seconds
  stop();               // Stops the motors
  */
  /*
  // Lab 2 Part Two
  for (int i = 0; i <= 4; i++) {
    set_r_dir(false); // Forward
    set_l_dir(true);  // Reverse
    // The car will turn counterclockwise
    set_r_speed(60);
    set_l_speed(60);
    delay(1500);  // Small delay, 150 ms, to allow the car to turn

    stop();
    delay(2000); // Wait 2 seconds before the loop restarts.
  }
  */
  /*
  // Lab 2 Part 4
  for (int i = 0; i < 4; i++) {
    drive_forward(50, 2000);  // Drive forward at 50% speed for 2 seconds
    turn_CCW(60, 1500);
    drive_forward(50, 2000);  // Drive forward at 50% speed for 2 seconds
    turn_CCW(60, 1500);
    drive_forward(50, 2000);  // Drive forward at 50% speed for 2 seconds
    turn_CCW(60, 1500);
    drive_forward(50, 2000);  // Drive forward at 50% speed for 2 seconds
    turn_CCW(60, 1500);
  }
  */
}

void loop() {
}