#include "Motor.h"

void Motor::readPreferences()
{
  try{
    this->currentLimit = this->Settings->getFloat(CURRENT_LIMIT_KEY, 0);
    this->invertDirection = this->Settings->getBool(INVERT_DIRECTION_KEY, false);
    this->invertEnable = this->Settings->getBool(INVERT_ENABLE_KEY, false);
  }catch(...){
    
  }
}

Motor::Motor(uint8_t pinEN, uint8_t pinPWM, uint8_t pinCS, uint8_t pinCHA, uint8_t pinCHB, Preferences *Preference)
{
    this->Settings = Preference;

    // store connection
    pinEnable = pinEN;
    pinPWM = pinPWM;
    pinCurrentSense = pinCS;
    pinChannelA = pinCHA;
    pinChannelB = pinCHB;

    // set pinmode
    pinMode(pinEN, OUTPUT);
    pinMode(pinPWM, OUTPUT);
    pinMode(pinChannelA, OUTPUT);
    pinMode(pinChannelB, OUTPUT);

    // set adc settings
    adcAttachPin(pinCurrentSense);
    analogReadResolution(ADC_RESOLUTION);
    analogSetWidth(ADC_RESOLUTION);
    analogSetAttenuation((adc_attenuation_t)ADC_0db);

    // set pwm settings
    ledcAttachPin(pinPWM, PWM_CHANNEL_0);
    ledcSetup(PWM_CHANNEL_0, PWM_FREQ, PWM_RES_BIT - 1);

    // load settings
    readPreferences();
}

Motor::~Motor()
{
}

void Motor::setThrottle()
{
    // calculate the pwm counter value
    this->pwm = this->throttle * (float)(PWM_PERCENT_SCALE);
    // set pwm
    ledcWrite(PWM_CHANNEL_0, this->pwm);
}

void Motor::setDirection()
{
    this->channelA = invertDirection ? !this->direction : this->direction;
    this->channelB = !this->channelA;

    digitalWrite(pinChannelA, this->channelA);
    digitalWrite(pinChannelB, this->channelB);
}

void Motor::setCurrentLimit(float value)
{
    // write only different values
    if (value != currentLimit)
    {
        this->currentLimit = value;
        this->Settings->putFloat(CURRENT_LIMIT_KEY, this->currentLimit);
    }
    // duplicated code 1!
}

void Motor::setInvertDirection(bool inverted)
{
    // write only different values
    if (invertDirection != inverted)
    {
        this->invertDirection = inverted;
        this->Settings->putBool(INVERT_DIRECTION_KEY, this->invertDirection);
    }
    // duplicated code 1!
}

void Motor::setInvertEnable(bool inverted)
{
    // write only different values
    if (invertEnable != inverted)
    {
        this->invertEnable = inverted;
        this->Settings->putBool(INVERT_ENABLE_KEY, this->invertEnable);
    }
    // duplicated code 1!
}

// motion control
void Motor::start(bool direction, float throttle)
{
    // no instant direction change
    if (this->direction != direction)
    {
        // stop with ebreak
        stop();
    }

    this->direction = direction;
    this->throttle = throttle;

    setDirection();
    setThrottle();
}

void Motor::stop(bool ebrake)
{
    this->throttle = 0;
    setThrottle();

    if (ebrake)
    {
        digitalWrite(pinChannelA, HIGH);
        digitalWrite(pinChannelB, HIGH);
    }
    else
    {
        digitalWrite(pinChannelA, LOW);
        digitalWrite(pinChannelB, LOW);
    }
    delay(300);
}

void Motor::enable()
{
    this->enabled = true;
    digitalWrite(pinEnable, invertEnable ? LOW : HIGH);
}

void Motor::disable()
{
    this->enabled = false;
    digitalWrite(pinEnable, invertEnable ? HIGH : LOW);
}

float Motor::getCurrent()
{
    float adcMean = 0;
    for (int8_t i = 0; i < 50; i++)
    {
        adcMean += (float)analogRead(pinCurrentSense) / 50.0f;
    }
    currentADC = (uint16_t)adcMean;
    current = (float)currentADC * (channelA ? currentScaleFactorChannelA : currentScaleFactorChannelB);
    return current;
}

float Motor::getCurrentLimit()
{
    return this->currentLimit;
}

MotorStatus Motor::getStatus()
{
    // update current
    getCurrent();
    return MotorStatus(this);
}

MotorStatus::MotorStatus(Motor *MotorX)
{
    this->throttle = MotorX->throttle;
    this->direction = MotorX->direction;
    this->enabled = MotorX->enabled;
    this->channelA = MotorX->channelA;
    this->channelB = MotorX->channelB;
    this->pwm = MotorX->pwm;
    this->invertDirection = MotorX->invertDirection;
    this->invertEnable = MotorX->invertEnable;
    this->currentADC = MotorX->currentADC;
    this->current = MotorX->current;
    this->currentLimit = MotorX->currentLimit;
}

char *MotorStatus::toString()
{
    char *buffer = new char[200];
    sprintf(buffer, "###Motorstatus###\n  throttle: %2.f\n  direction: %d\n  enabled: %d\n  channelA: %d\n  channelB: %d\n  pwm: %d\n  invertDirection: %d\n  invertEnable: %d\n  currentADC: %i\n  current: %.3f\n  currentLimit: %.2f\n\n\n",
            this->throttle, this->direction, this->enabled, this->channelA, this->channelB, this->pwm, this->invertDirection, this->invertEnable, this->currentADC, this->current, this->currentLimit);

    return buffer;
}
