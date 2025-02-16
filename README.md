# libbot_hw

Code files for connecting libbot core systems with hardware systems. Current implementation uses Raspberry Pi Pico and PicoSDK to make things work.

## Features:
- Wireless handling of robot with transmitter and receiver nodes using NRF24 based communication.
- Standard optical encoder based RPM estimation that uses PIO state machines for counting pulses
- Precise dual channel motor speed control using PID control loop
- Servo based door angle control
