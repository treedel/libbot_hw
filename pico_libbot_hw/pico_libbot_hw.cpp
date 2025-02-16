#include <stdio.h>
#include "pico/stdlib.h"
#include <RF24.h>

#include "defaultPins.h"

#include "pid.h"
#include "motor.h"
#include "encoder.h"
#include "servo.h"

#pragma pack(push, 1)
typedef struct {
    char command;
    float arg1;
    float arg2;
    float arg3;
} CustomPacket;
#pragma pack(pop)

CustomPacket packet;

const uint8_t address[6] = "00001";
RF24 radio(CE_PIN, CSN_PIN);

#define TOTAL_CHANNELS 2

// Defining pins
const uint ENC_PINS[TOTAL_CHANNELS] = {20, 21};
const uint MOTOR_PINS[TOTAL_CHANNELS][TOTAL_MOTOR_PINS] = {
  {11, 10},
  {13, 12}
};
const uint DOOR_SERVO_PIN = 22;

// For PID loop control
const float PID_LOOP_RATE = 5.0;
const float PID_LOOP_S = 1.0 / PID_LOOP_RATE;
const float PID_LOOP_MS = PID_LOOP_S * 1000;

// LPF Smoothing Factor (0 < alpha < 1)
const float LPF_ALPHA = 0.3;

Motor left_motor, right_motor;
EncoderCounter left_encoder, right_encoder;
PidControl left_pid_controller, right_pid_controller;

// LPF state variables
float left_filtered_speed = 0;
float right_filtered_speed = 0;

bool pid_control_run(__unused struct repeating_timer *t) {
  update_encoder_values(&left_encoder);
  update_encoder_values(&right_encoder);

  int32_t left_encoder_delta = get_encoder_delta(left_encoder) * PID_LOOP_RATE;
  int32_t right_encoder_delta = get_encoder_delta(right_encoder) * PID_LOOP_RATE;

  // Apply Low-Pass Filter (LPF)
  left_filtered_speed = (LPF_ALPHA * left_encoder_delta) + ((1 - LPF_ALPHA) * left_filtered_speed);
  right_filtered_speed = (LPF_ALPHA * right_encoder_delta) + ((1 - LPF_ALPHA) * right_filtered_speed);

  // Use filtered values in PID calculation
  int32_t left_control_level = calculate_pid_output(&left_pid_controller, left_filtered_speed);
  int32_t right_control_level = calculate_pid_output(&right_pid_controller, right_filtered_speed);

  // Set encoder direction
  set_encoder_motor_direction(&left_encoder, (left_control_level<0));
  set_encoder_motor_direction(&right_encoder, (right_control_level<0));

  // Use the PID output to drive the motors
  control_motor(&left_motor, left_control_level);
  control_motor(&right_motor, right_control_level);

/*   printf("Left: %d (Filtered: %.2f) : %d, Right %d (Filtered: %.2f) : %d\n",
         left_encoder_delta, left_filtered_speed, left_control_level,
         right_encoder_delta, right_filtered_speed, right_control_level); */

  return true;
}

void rf24_interface_run() {
  if (radio.available()) {
      printf("Received a packet\n");
      radio.read(&packet, sizeof(CustomPacket));
      printf("Command: %c, %.2f, %.2f, %.2f\n", packet.command,  packet.arg1, packet.arg2, packet.arg3);

      switch (packet.command) {
        case 'c':
          set_pid_target(&left_pid_controller, packet.arg1);
          set_pid_target(&right_pid_controller, packet.arg2);
          printf("OK\r\n");
          break;

        case 'e':
          printf("%d %d\r\n", get_encoder_counts(left_encoder), get_encoder_counts(right_encoder));
          break;

        case 'p':
          set_pid_constants(&left_pid_controller, packet.arg1, packet.arg2, packet.arg3);
          set_pid_constants(&right_pid_controller, packet.arg1, packet.arg2, packet.arg3);
          break;

        case 's':
          set_servo_angle(DOOR_SERVO_PIN, packet.arg1);
          printf("OK\r\n");
          break;

        default:
          printf("Invalid command\r\n");
      }
  }
}

int main() {
    stdio_init_all();

    // Allow time for serial monitor to start
    sleep_ms(2000);

    // Configure servo
    configure_servo(DOOR_SERVO_PIN);

    // Configure NRF24 radio
    if (!radio.begin()) {
        printf("Radio hardware is not responding\n");
        while(1);
    }
    printf("Radio initialized\n");
    radio.openReadingPipe(0, address);
    radio.setDataRate(RF24_250KBPS);
    radio.setPALevel(RF24_PA_MIN);
    radio.startListening();

    // Configure encoders
    configure_encoder_counter(&left_encoder, ENC_PINS[0]);
    configure_encoder_counter(&right_encoder, ENC_PINS[1]);
    
    // Configure motors
    configure_motor(&left_motor, MOTOR_PINS[0][0], MOTOR_PINS[0][1]);
    configure_motor(&right_motor, MOTOR_PINS[1][0], MOTOR_PINS[1][1]);

    // Configure PID controllers
    configure_pid_control(&left_pid_controller, PID_LOOP_S, 100, 1000, 10);
    configure_pid_control(&right_pid_controller, PID_LOOP_S, 100, 1000, 10);

    struct repeating_timer timer;
    add_repeating_timer_ms(PID_LOOP_MS, pid_control_run, NULL, &timer);

    while(1) {
      rf24_interface_run();
    }
}
