#include "Motor.h"

Motor::Motor( uint8_t pinForward,
              uint8_t pinBackward,
              uint8_t pwmChannel,
              double_t power) :
  _pinForward(pinForward),
  _pinBackward(pinBackward),
  _pwmChannel(pwmChannel),
  _power(power),
  _pwm_frequency(5000),
  _pwm_resolution(10)
{
  pinMode(this->_pinBackward, OUTPUT);
  pinMode(this->_pinForward, OUTPUT);

  // turn motor off
  digitalWrite(this->_pinBackward, LOW);
  digitalWrite(this->_pinForward, LOW);

  this->_configurePWM();
  this->stop();
}

Motor::~Motor(void) {
  // clear Pwm channels
  ledcDetachPin(this->_active_pin);
  // reset pin mode configurations
  pinMode(this->_pinBackward, INPUT);
  pinMode(this->_pinForward, INPUT);
}

void Motor::_configurePWM(void) {
  // configure the channels and timers (use the same frequency)
  this->_pwm_frequency = ledcSetup(this->_pwmChannel, this->_pwm_frequency, this->_pwm_resolution);
  
  // calculate the maximum duty cycle
  this->_max_duty_cyle = (uint16_t)(pow(2, this->_pwm_resolution) - 1);

  // attach the pins
  this->_active_pin = this->_pinForward;
  ledcAttachPin(this->_active_pin, this->_pwmChannel);
}

void Motor::_attachPin(uint8_t pin) {
  Serial.print("Motor::_attachPin(");
  Serial.printf("%d", pin);
  Serial.println(")");
  // check if is already the active pin
  if((pin == this->_active_pin)) {
    return;
  }

  if((pin == this->_pinBackward) || (pin == this->_pinForward)){
    ledcDetachPin(this->_active_pin);
    digitalWrite(this->_active_pin, LOW);
    this->_active_pin = pin;
    ledcAttachPin(this->_active_pin, this->_pwmChannel);
  }
}

void Motor::stop(void) {
  this->_pwm_velocity = 0; // reset
  ledcWrite(this->_pwmChannel, this->_pwm_velocity); // update
}

int8_t Motor::_assertSpeed(int8_t speed) {
  if (speed > 100) {
    return 100;
  }

  if (speed <= 0) {
    return 0;
  }

  return speed;
}

void Motor::_writePwmSpeed(uint8_t speed) {
  Serial.print("Motor::_writePwmSpeed(");
  Serial.printf("%d", speed);
  Serial.println(")");

  this->_pwm_velocity = map(speed, 0, 100, this->_power * this->_max_duty_cyle, this->_max_duty_cyle);
  ledcWrite(this->_pwmChannel, this->_pwm_velocity);
}

void Motor::backward(int8_t speed) {
  speed = this->_assertSpeed(speed);
  
  if (speed == 0) {
    this->stop();
    return;
  }

  // update the directions
  this->_attachPin(this->_pinBackward);
  this->_writePwmSpeed(speed);
}

void Motor::forward(int8_t speed) {
  speed = this->_assertSpeed(speed);
  if (speed == 0) {
    this->stop();
    return;
  }
  this->_attachPin(this->_pinForward);
  this->_writePwmSpeed(speed);
}


void Motor::velocity(int8_t velocity) {
  if (velocity >= 0) {
    this->forward(velocity);
  } else {
    this->backward((velocity * -1));
  }
}