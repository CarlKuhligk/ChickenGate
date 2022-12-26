#ifndef LINEARACTUATOR_H
#define LINEARACTUATOR_H

#include <Preferences.h>
#include <Arduino_JSON.h>

#include "Motor.h"
#include "Encoder.h"

#define MOVE_IN 0
#define MOVE_OUT 1

#define MINIMUM_THROTTLE 70.0f
#define ACCELERATION_RATE 2.5f

// PREFERENCES KEYS
#define COUNT_PER_UNIT_KEY "countPerUnit"
#define HOME_DIRECTION_KEY "homeDirection"

typedef enum ActuatorTaskType
{
  Stop,
  Homing,
  MoveAbsolute,
  Calibration,
  Calibration2
};

typedef enum ActuatorStateType
{
  Idle,
  Moving,
  Done,
  Error,
};

typedef enum ActuatorErrorType
{
  NoError,
  CurrentProtection,
  CalibrationOrder,
};

typedef struct ActuatorTaskProperties
{
  ActuatorTaskType taskType = Stop;
  float throttle = 0;
  float startPosition = 0;
  float targetPosition = 0;
  uint16_t movementTime = 0;

  ActuatorStateType state = Idle;
  float actualPosition = 0;
  float startDistance = 0;
  float targetDistance = 0;
  float current = 0;
  float currentLimit = 0;
  bool direction = MOVE_IN;
  bool enabled = false;
  ActuatorErrorType error = NoError;
};

class LinearActuator
{
  private:
    Motor *Motor1;
    Encoder *Encoder1;
    Preferences *Settings;
    ActuatorTaskProperties prop;

    // values for calibration
    uint16_t calibrationCounter; // contains the encoder impulses while calibration movement
    float countsPerUnit;         // for example, value 6800 equals = 1 unit = 1mm

    bool homeDirection;
    bool isEnabled;

    void readPreferences();

    void setThrottle(float throttle);
    void startTask(ActuatorTaskType taskType);

    const float getPosition(); // contains the automatic converted unit value that is calculated by counter value * countsPerUnit

    // actuator tasks
    // 1. axis gets stoped
    // 2. encoder value is stored
    // 3. axis will move in the given direction at a given throttle for a given time
    // 4. axis gets stoped
    // 5. next step call finishCalibration(float traveledWayInMM) methods to finish the calibration
    void taskStartCalibration();
    // 1. the traveled way gets divided by the difference between the encoder values, the result (countsPerUnit) is set and stored in the eeprom
    void taskFinishCalibration();
    void setCountPerUnit(float value);
    void taskMoveHome();
    void taskMoveAbsolute();

  public:
    LinearActuator(uint8_t pinEN, uint8_t pinPWM, uint8_t pinCS, uint8_t pinDirA, uint8_t pinDirB, uint8_t pinEncChA, uint8_t pinEncChB, Preferences *Settings);
    void loop();

    void enable();
    void disable();
    void stop();
    void update();

    void setHomeDirection(bool direction);
    void setCurrentLimit(float value);
    void setInvertDirection(bool value);

    void startCalibration(bool direction, float throttle, uint16_t movementTime);
    void finishCalibration(float distanceTraveledInMM);
    void moveHome();
    void moveAbsolute(float position, float throttle = 80.f);
    bool taskDone();
    const String toJSON();
};

#endif
