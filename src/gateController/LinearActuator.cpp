#include "LinearActuator.h"

LinearActuator::LinearActuator(uint8_t pinEN, uint8_t pinPWM, uint8_t pinCS, uint8_t pinDirA, uint8_t pinDirB, uint8_t pinEncChA, uint8_t pinEncChB, Preferences *Settings)
{
  this->Settings = Settings;
  readPreferences();

  this->Motor1 = new Motor(pinEN, pinPWM, pinCS, pinDirA, pinDirB, this->Settings);
  this->Encoder1 = new Encoder(pinEncChA, pinEncChB);

  this->prop.currentLimit = this->Motor1->getCurrentLimit();

  this->disable();
}

void LinearActuator::readPreferences()
{
  this->homeDirection = this->Settings->getBool(HOME_DIRECTION_KEY, false);
  this->countsPerUnit = this->Settings->getFloat(COUNT_PER_UNIT_KEY, 0);
}

void LinearActuator::setCountPerUnit(float value)
{
  // write only different values
  if (value != countsPerUnit)
  {
    this->countsPerUnit = value;
    this->Settings->putFloat(COUNT_PER_UNIT_KEY, this->countsPerUnit);
  }
}

void LinearActuator::loop()
{
  if (this->isEnabled)
  {
    this->update();

    // skip tasks if state is done
    if (this->prop.state == Done)
    {
      return;
    }

    switch (this->prop.taskType)
    {
      case Stop:
        this->stop();
        break;
      case Homing:
        this->taskMoveHome();
        break;

      case MoveAbsolute:
        this->taskMoveAbsolute();
        break;

      case Calibration:
        this->taskStartCalibration();
        break;

      case Calibration2:
        this->taskFinishCalibration();
        break;

      // idle
      default:
        return;
        break;
    }
  }
}

void LinearActuator::taskStartCalibration()
{ /*
    this->Motor1->stop();
    this->calibrationCounter = (uint16_t)this->Encoder1->getCount();
    this->Motor1->start(this->dir, this->throttle);
    delay(this->movementTime);
    this->Motor1->stop();
    this->taskState = Done;
    return;*/
}

void LinearActuator::taskFinishCalibration()
{ /*
    float factor = this->distanceTraveledInMM / ((uint16_t)Encoder1->getCount() - this->calibrationCounter);
    this->setCountPerUnit(factor);
    this->taskState = Done;
    return;*/
}

void LinearActuator::taskMoveHome()
{
  this->setThrottle(this->prop.throttle + 1);
  Motor1->start(homeDirection, this->prop.throttle);

  // wait 100ms to measure a current after the motor has started
  if (this->prop.state == Idle)
  {
    delay(100);
    this->prop.current = this->Motor1->getCurrent();
  }

  // condition of the task
  // check if the motor has reached an end switch and turned off
  if (this->prop.current > 0)
  {
    this->prop.state = Moving;
  }
  else
  {
    Motor1->stop();
    Encoder1->clear();
    this->prop.state = Done;
  }
  return;
}

void LinearActuator::moveAbsolute(float position, float throttle)
{
  if (this->prop.state != Moving)
  {
    this->prop.startPosition = this->getPosition();
    this->prop.targetPosition = position;
    this->startTask(MoveAbsolute);
    this->setThrottle(throttle);
  }
}

void LinearActuator::taskMoveAbsolute()
{
  float accelerationDistance = (this->prop.throttle - MINIMUM_THROTTLE) / ACCELERATION_RATE; // acceleration distance
  float pn = this->prop.actualPosition;                                                      //
  float delta = this->prop.targetPosition - pn;                                              //
  float sX;                                                                                  // can be accelerationDistance or delta/2
  float throttle;                                                                            //

  /*
         p1,p2
           ^
          / \
         /   \
        /     \
       /       \
      /         \
    ___/           \___
     |<- delta ->|  distanceToTargetPosition
     |           |
     p0          p4
  */
  if (this->prop.targetDistance > (2.0f * accelerationDistance))
  {
    sX = accelerationDistance;
  }
  else
  {
    sX = this->prop.targetDistance / 2.0f;
  }

  /*
      ____________MOVE OUT____________            _____________MOVE IN______________

           p1                 p2                        p2                 p1
     |< sX >|________V_________|< sX >|           |< sX >|________V_________|< sX >|
     |      /        |         \      |           |      /        |         \      |
     |     /         pn         \     |           |     /         pn         \     |
     |    /          |           \    |           |    /          |           \    |
     |   /           |->          \   |           |   /         <-|            \   |
     |  /            |             \  |           |  /            |             \  |
     | /             |              \ |           | /             |              \ |
    __!/             |               \!__       __!/              |               \!__
     |               |<--- delta ---->|           |<--- delta --->|
     |-- startDist --|               p4           p4              |<-- startDist-->|
     p0                                                                            p0
  */

  if (delta > 0.0f)
    this->prop.direction = MOVE_OUT;
  else
    this->prop.direction = MOVE_IN;

  if (((int)this->prop.startDistance < (int)sX) && (this->prop.targetDistance > sX))
  {
    // accelerate
    throttle = MINIMUM_THROTTLE + this->prop.startDistance * ACCELERATION_RATE;
  }
  else if ((int)this->prop.targetDistance < ((int)sX + 50))
  {
    // decelerate
    throttle = MINIMUM_THROTTLE + this->prop.targetDistance * ACCELERATION_RATE;
  }
  else
  {
    throttle = this->prop.throttle;
  }

  if (throttle > this->prop.throttle)
    throttle = this->prop.throttle;

  if (throttle < MINIMUM_THROTTLE)
    throttle = MINIMUM_THROTTLE;

  Motor1->start(this->prop.direction, throttle);

  // wait 100ms to measure a current after the motor has started
  if ((this->prop.state == Idle) && (this->prop.targetDistance > 0.5f))
  {
    delay(200);
    this->prop.current = this->Motor1->getCurrent();
  }

  // complete condition of the task
  if (this->prop.targetDistance <= 0.5f || this->prop.current <= 0.0f)
  {
    Motor1->stop();
    this->prop.state = Done;
  }
  else
  {
    this->prop.state = Moving;
  }
  return;
}

void LinearActuator::setThrottle(float throttle)
{
  if (throttle < MINIMUM_THROTTLE)
    this->prop.throttle = MINIMUM_THROTTLE;
  else if (throttle > 100.0f)
    this->prop.throttle = 100.;
  else
    this->prop.throttle = throttle;
}

void LinearActuator::startTask(ActuatorTaskType taskType)
{
  this->prop.taskType = taskType;
  this->prop.state = Idle;
}

////////////
// PUBLIC //
////////////

void LinearActuator::setCurrentLimit(float value)
{
  this->Motor1->setCurrentLimit(value);
}

void LinearActuator::setInvertDirection(bool value)
{
  this->Motor1->setInvertDirection(value);
}

void LinearActuator::setHomeDirection(bool dir)
{
  // write only different values
  if (dir != homeDirection)
  {
    this->homeDirection = dir;
    this->Settings->putBool(HOME_DIRECTION_KEY, this->homeDirection);
  }
}

void LinearActuator::enable()
{
  this->Motor1->enable();
  this->isEnabled = true;
  this->prop.enabled = true;
}

void LinearActuator::disable()
{
  this->Motor1->disable();
  this->isEnabled = false;
  this->prop.enabled = false;
}

void LinearActuator::startCalibration(bool dir, float throttle, uint16_t movementTime)
{
  /*this->dir = dir;
    this->throttle = throttle;
    this->movementTime = movementTime;
    this->taskState = idle;
    this->command = calibration;*/
}

void LinearActuator::finishCalibration(float distanceTraveledInMM)
{
  /*if (this->command == calibration && this->taskState == done)
    {
    this->distanceTraveledInMM = distanceTraveledInMM;
    this->taskState = idle;
    this->command = calibration2;
    }
    else
    {
    this->errorState = calibrationOrder;
    }*/
}

void LinearActuator::moveHome()
{
  this->startTask(Homing);
  this->setThrottle(MINIMUM_THROTTLE);
  this->prop.startPosition = this->getPosition();
}

const float LinearActuator::getPosition()
{
  return (float)this->Encoder1->getCount() * countsPerUnit;
}

void LinearActuator::stop()
{
  this->Motor1->stop();
  this->prop.state == Done;
}

void LinearActuator::update()
{
  // read actual values for the cycle
  this->prop.current = this->Motor1->getCurrent();
  this->prop.actualPosition = this->getPosition();
  this->prop.startDistance = abs(this->prop.startPosition - this->prop.actualPosition);
  this->prop.targetDistance = abs(this->prop.targetPosition - this->prop.actualPosition);

  // check current overload
  if (this->prop.current >= this->prop.currentLimit)
  {
    this->prop.taskType = Stop;
    this->prop.state == Error;
    this->prop.error = CurrentProtection;
  }
}

bool LinearActuator::taskDone()
{
  return this->prop.state == Done ? true : false;
}

const String LinearActuator::toJSON()
{
  JSONVar json;
  json["p"] = (int)this->prop.actualPosition;
  json["t"] = this->prop.taskType;
  json["s"] = this->prop.state;
  json["d"] = this->prop.direction;
  json["c"] = this->prop.current;
  // json["cl"] = this->prop.currentLimit;
  // json["er"] = this->prop.error;
  // json["en"] = this->prop.enabled;
  json["sD"] = (int)this->prop.startDistance;
  json["tD"] = (int)this->prop.targetDistance;

  const String result = JSON.stringify(json);
  return result;
}
