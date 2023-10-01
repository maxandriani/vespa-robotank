#ifndef MOTOR_H
#define MOTOR_H

#if !defined(ARDUINO_ESP32_DEV) // ESP32
#error Use this library with the ESP32
#endif

#include <Arduino.h>

extern "C" {
  #include <stdarg.h>
  #include <stdint.h>
  #include <stdlib.h>
  
  #include <esp_adc_cal.h>
  #include <esp32-hal-ledc.h>
}

#define LEFT_MOTOR_CHANNEL ((uint8_t) 14)
#define LEFT_MOTOR_FORWARD ((uint8_t) 13)
#define LEFT_MOTOR_BACKWARD ((uint8_t) 14)
#define RIGHT_MOTOR_CHANNEL ((uint8_t) 15)
#define RIGHT_MOTOR_FORWARD ((uint8_t) 4)
#define RIGHT_MOTOR_BACKWARD ((uint8_t) 27)
#define MOTOR_DEFAULT_POWER ((double) 0.6)

class Motor {
  public:
    Motor(uint8_t _pinForward,
          uint8_t _pinBackward,
          uint8_t _pwmChannel,
          double _power);
    ~Motor(void);
    void velocity(int8_t);
    void backward(int8_t);
    void forward(int8_t);
    void stop(void);

  private:
    uint8_t _pinForward;
    uint8_t _pinBackward;
    double _power;
    uint8_t _active_pin;
    uint16_t _pwm_velocity;
    double _pwm_frequency; // [Hz]
    uint8_t _pwm_resolution;
    uint8_t _pwmChannel;
    uint16_t _max_duty_cyle;

    void _attachPin(uint8_t);
    void _configurePWM(void);
    int8_t _assertSpeed(int8_t);
    void _writePwmSpeed(uint8_t);
};

#endif // MOTORS_H
