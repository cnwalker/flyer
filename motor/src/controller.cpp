/***
 * This code must be placed in the arduino editor to compile/flash for now
 */

#include <Servo.h>

/**
 * Servo object to control the ESC.
 */
int MIN_SPEED = 1000;
int MAX_SPEED = 2000;
int target_speed = MIN_SPEED;
int SPEED_INCREMENT = 2;
const int NUM_MOTORS = 4;
const int NUM_MOTOR_SELECT_BITS = 2;
int FORCE_MIN_SPEED_CMD = 8;
int SPEED_RANGE = MAX_SPEED - MIN_SPEED;

unsigned char motor_select_mask = 0b00000011;
unsigned char motor_speed_mask = 0b11111100;

int POWER_STATE_UPDATE_MILLIS = 500;
int POWER_STATE_CONSISTENCY_MILLIS = 50;
int MOTOR_BATTERY_HIGH_POWER_THRESHOLD = 100;
int READY_FOR_COMMANDING_THRESHOLD = 8500;

class MotorConfig {
  public:
    int pwm_pin;
    int power_pin;
    int target_speed;
    bool calibrated;
  
    MotorConfig(int in_pwm_pin, int in_power_pin, int in_target_speed, bool is_calibrated = true) {
      pwm_pin = in_pwm_pin;
      power_pin = in_power_pin;
      target_speed = in_target_speed;
      calibrated = is_calibrated;
    };

    bool ready_for_commanding() {
      /**
       * Motors need to go through a manual calibration sequence before starting at lowest and going to highest
       */
      if (!calibrated) {
        return true;
      }
 
      unsigned long current_time = millis();
      if (last_powered_by_battery == 0) {
        return false;
      }
      if (current_time - last_powered_by_battery > READY_FOR_COMMANDING_THRESHOLD) {
        return true;
      }
      return false;
    }

    void dispatch() {
      handle_battery_power();
    }

  private:
    unsigned long last_consistent_power_low_time;
    unsigned long last_power_low_time;
    unsigned long last_power_high_time;
    unsigned long last_powered_by_battery = 0;
    bool powered_by_battery;

    void handle_battery_power() {
      unsigned long current_time = millis();
      int power_reading = analogRead(power_pin);
      if (power_reading > MOTOR_BATTERY_HIGH_POWER_THRESHOLD) {
        last_power_high_time = current_time;
        if (!powered_by_battery) {
          last_powered_by_battery = current_time;
        }
        powered_by_battery = true;
      } else {
        last_power_low_time = current_time;
        powered_by_battery = false;
      }

      if (current_time - last_power_high_time > POWER_STATE_CONSISTENCY_MILLIS) {
        last_consistent_power_low_time = current_time;
      }
    }
};

Servo motors[NUM_MOTORS];
MotorConfig motor_confs[NUM_MOTORS] = {
  {3, A0, MIN_SPEED, true},
  {5, A1, MIN_SPEED, true},
  {6, A2, MIN_SPEED, true},
  {9, A3, MIN_SPEED, true}
};

void handle_speed_updates() {
  if (!Serial.available()) {
    return;
  }

  unsigned char incoming_byte = (unsigned char)Serial.read();
  if (incoming_byte == 10) {
    return -1;
  }

  Serial.print("Received byte: ");
  Serial.println(incoming_byte);

  unsigned int motor_num = (unsigned int)(incoming_byte & motor_select_mask);
  if (motor_num < 0 || motor_num > NUM_MOTORS - 1) {
    Serial.print("No motor ");
    Serial.println(motor_num);
    return;
  }
  unsigned int requested_speed = (unsigned int)((incoming_byte & motor_speed_mask) >> NUM_MOTOR_SELECT_BITS);
  Serial.print("Requested speed: ");
  Serial.println(requested_speed);
  unsigned int new_speed = MIN_SPEED;
  if (requested_speed != FORCE_MIN_SPEED_CMD) {
    // This number should be 2^(8 - NUM_MOTOR_SELECT_BITS)
    float thrust_fraction = ((float)requested_speed)/64.0;
    // TODO: Remove the multipled by 2 once commands are sent via the pi
    new_speed = MIN_SPEED + (unsigned int)((thrust_fraction * 2) * SPEED_RANGE);
  }

  motor_confs[motor_num].target_speed = new_speed;
  Serial.print("Motor ");
  Serial.print(motor_num);
  Serial.print(" speed: ");
  Serial.println(new_speed);
}

void setup() {
    /**
     * Start all motors with min speed
     */
    for (int i = 0; i < NUM_MOTORS; i++) {
      motors[i].attach(motor_confs[i].pwm_pin, MIN_SPEED, MAX_SPEED);
      motors[i].writeMicroseconds(MIN_SPEED);
    }
    Serial.begin(9600);
}

void loop() {
    delay(10);
    // Run at 100 Hz
    handle_speed_updates();
    for (int i = 0; i < NUM_MOTORS; i++) {
      motor_confs[i].dispatch();
      if (motor_confs[i].ready_for_commanding()) {
        motors[i].writeMicroseconds(motor_confs[i].target_speed);
      } else {
        motors[i].writeMicroseconds(MIN_SPEED);
      }
    }
}