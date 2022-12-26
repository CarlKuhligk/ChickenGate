#ifndef MOTOR_H
#define MOTOR_H

#include <Preferences.h>

// PWM FREQUENCY
#define PWM_FREQ 5000  // frequency
#define PWM_RES_BIT 16 // 16 bit pwm
#define PWM_RES 65535  // max counter value
#define PWM_PERCENT_SCALE (PWM_RES / 100.0f)

// H_BRIDGE CURRENT SENSE
#define ADC_RESOLUTION 12       // bit
#define ADC_ATTENUATION ADC_0db //

// PREFERENCES KEYS
#define INVERT_DIRECTION_KEY "invertDirection"
#define INVERT_ENABLE_KEY "invertEnable"
#define CURRENT_LIMIT_KEY "currentLimit"

// RORATION DIRECTION
#define CLOCKWISE 1
#define COUNTERCLOCKWISE 0
#define CW 1
#define CCW 0

// ESP32 PWM CHANNEL
#define PWM_CHANNEL_0 0

class MotorStatus;

// This Class controls a dc-brushmotor via a h-bridge
// H-Bridge Type: VNH3ASP30
class Motor
{
private:
    // connections
    uint8_t pinEnable;
    uint8_t pinPWM;
    uint8_t pinCurrentSense;
    uint8_t pinChannelA;
    uint8_t pinChannelB;

    // current monitoring
    uint16_t currentADC;
    const float currentScaleFactorChannelA = 0.00116f; //
    const float currentScaleFactorChannelB = 0.00101f; //
    float current;
    float currentLimit; // save in preferences

    // motion
    float throttle;
    bool direction;
    bool enabled;
    bool channelA;
    bool channelB;
    uint16_t pwm;

    // settings
    Preferences *Settings;
    bool invertDirection; // save in preferences
    bool invertEnable;    // save in preferences

    void readPreferences();

    void setDirection();
    void setThrottle();

    friend class MotorStatus;

public:
    Motor(uint8_t pinEN, uint8_t pinPWM, uint8_t pinCS, uint8_t pinCHA, uint8_t pinCHB, Preferences *Preference);

    ~Motor();

    // configurate
    void setCurrentLimit(float value);

    void setInvertDirection(bool inverted);

    void setInvertEnable(bool inverted);

    // motion control
    void start(bool direction, float throttle);

    void stop(bool ebrake = true);

    void enable();

    void disable();

    // response
    float getCurrent();
    float getCurrentLimit();

    MotorStatus getStatus();
};

class MotorStatus
{
public:
    MotorStatus(Motor *MotorX);

    // current monitoring
    uint16_t currentADC;
    float current;
    float currentLimit; // save in eeprom

    // motion
    float throttle;
    bool direction;
    bool enabled;
    bool channelA;
    bool channelB;
    uint16_t pwm;

    // settings
    bool invertDirection; // Motion Flag 0
    bool invertEnable;    // Motion Flag 1

    // string output
    char *toString();
};
#endif
