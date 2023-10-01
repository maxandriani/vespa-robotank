#ifndef ROBOTANK_H
#define ROBOTANK_H

typedef enum onboard_pins
{
    LED = 15,
    BUTTON = 35
} onboard_pins_t;

typedef struct remote_control_motors
{
    int8_t leftAxis;
    int8_t rightAxis;
} remote_control_motors_t;

typedef struct remote_control_arm
{
    int8_t horizontalAxis;
    int8_t verticalAxis;
    int8_t distanceAxis;
    int8_t clawAxis;
} remote_control_arm_t;

typedef struct remote_control
{
    remote_control_motors_t motors;
    remote_control_arm arm;
} remote_control_t;

#endif
