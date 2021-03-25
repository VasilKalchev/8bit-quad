#include "PID.hpp"


Pcontroller::Pcontroller(int16_t* setpoint, float* input, int16_t* output, float P) {
  _setpoint = setpoint;
  _input = input;
  _output = output;
  _state = false;
  _rampRate = 0;
  _lastOutput = *_output;
  setOutputLimits(0, 255);
  direct();
  setTunings(P);
  setUpdateRate(2000);
}

bool Pcontroller::setTunings(float P) {
  if (P < 0) return false;
  _p = P;

  if (_direct == false) {
    _p = 0.0 - _p;
  }
  return true;
}

bool Pcontroller::setOutputLimits(int16_t minimum, int16_t maximum) {
  if (minimum >= maximum) return false;
  _minimumOutput = minimum;
  _maximumOutput = maximum;

  if (_state) {
    if (*_output > _maximumOutput) { *_output = _maximumOutput; _lastOutput = *_output; }
    else if (*_output < _minimumOutput) { *_output = _minimumOutput; _lastOutput = *_output; }
  }
  return true;
}

bool Pcontroller::setOutputLimits(int16_t range) {
  int16_t max = abs(range);
  int16_t min = max * -1;
  return setOutputLimits(min, max);
}

void Pcontroller::setUpdateRate(uint32_t updateRate) {
  _updateRate = updateRate;
}

void Pcontroller::on() {
  _state = true;
}

void Pcontroller::off() {
  _state = false;
}

void Pcontroller::direct() {
  if (_direct == false) {
    _p = (0 - _p);
  }
  _direct = true;
}

void Pcontroller::reverse() {
  if (_direct == true) {
    _p = (0 - _p);
  }
  _direct = false;
}

float Pcontroller::getP() {
  return _p;
}

uint32_t Pcontroller::getUpdateRate() {
  return _updateRate;
}

void Pcontroller::setRampRate(int16_t rampRate) {
  rampRate = abs(rampRate);
  if (rampRate >= _maximumOutput) return;
  _rampRate = rampRate;
}

void Pcontroller::compute() {
  if (!_state) return;
  // uint32_t now = micros();
  // static uint32_t lastTime = 0;
  // if (now - lastTime >= _updateRate) {
  // Serial.print("Inner "); Serial.print(*_setpoint);
  // Serial.print(" - "); Serial.println(micros()/1000);
  *_output = (int16_t)round(((float) * _setpoint - *_input) * _p);
  if (_rampRate != 0) {
    int16_t currentMaximum = _lastOutput + _rampRate;
    int16_t currentMinimum = _lastOutput - _rampRate;
    if (*_output > currentMaximum) *_output = currentMaximum;
    else if (*_output < currentMinimum) *_output = currentMinimum;
  }
  if (*_output > _maximumOutput) *_output = _maximumOutput;
  else if (*_output < _minimumOutput) *_output = _minimumOutput;
  _lastOutput = *_output;
  // lastTime = now;
  // } else {
  //   return false;
  // }
}


PID::PID(int16_t* setpoint, float* input, int16_t* output,
         float P, float I, float D, int POn, int ControllerDirection) {
  _setpoint = setpoint;
  _input = input;
  _output = output;
  _inAuto = false;
  _lastOutput = *_output;

  PID::setOutputLimits(0, 255);
  PID::setIntegralLimit(0, 255);
  _rampRate = 0;

  _updateRate = 100000; //default Controller Sample Time is 0.015 seconds
  // PID::setUpdateRate(15000);

  PID::setControllerDirection(ControllerDirection);
  PID::setTunings(P, I, D, POn);

  // _lastTime = micros() - _updateRate;
}

/*Constructor (...)*********************************************************
 *    To allow backwards compatability for v1.1, or for people that just want
 *    to use Proportional on Error without explicitly saying so
 ***************************************************************************/

PID::PID(int16_t* setpoint, float* input, int16_t* output,
         float P, float I, float D, int ControllerDirection)
  : PID::PID(setpoint, input, output, P, I, D, P_ON_E, ControllerDirection) {}


/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/
void PID::compute() {
  if (!_inAuto) return;
  // uint32_t now = micros();
  // uint32_t timeChange = (now - _lastTime);
  // if ((now - _lastTime) >= _updateRate) {
  // Serial.print("Outer "); Serial.print(*_setpoint);
  // Serial.print(" - "); Serial.println(micros()/1000);

  /*Compute all the working error variables*/
  float input = *_input;
  float error = (float) * _setpoint - input;
  float dInput = (input - _lastInput);
  _outputSum += (_i * error);

  /*Add Proportional on Measurement, if P_ON_M is specified*/
  if (!_pOnE) _outputSum -= _p * dInput;

  if (_outputSum > _maximumIntegralSum) _outputSum = _maximumIntegralSum;
  else if (_outputSum < _minimumIntegralSum) _outputSum = _minimumIntegralSum;

  /*Add Proportional on Error, if P_ON_E is specified*/
  float output;
  if (_pOnE) output = _p * error;
  else output = 0;

  /*Compute Rest of PID output*/
  output += _outputSum - _d * dInput;

  if (_rampRate != 0) {
    int16_t currentMaximum = _lastOutput + _rampRate;
    int16_t currentMinimum = _lastOutput - _rampRate;
    if (output > currentMaximum) output = currentMaximum;
    else if (output < currentMinimum) output = currentMinimum;
  }

  if (output > _maximumOutput) output = _maximumOutput;
  else if (output < _minimumOutput) output = _minimumOutput;
  *_output = round(output);

  /*Remember some variables for next time*/
  _lastInput = input;
  _lastOutput = *_output;
// _lastTime = now;
//   return true;
// }
// else return false;
}

/* setTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/
bool PID::setTunings(float P, float I, float D, int8_t POn) {
  if (P < 0.0 || I < 0.0 || D < 0.0) return false;
  _pOn = POn;
  _pOnE = POn == P_ON_E;

  _pOriginal = P; _iOriginal = I; _dOriginal = D;

  float SampleTimeInSec = ((float)_updateRate) / 1000000.0;
  _p = P;
  _i = I * SampleTimeInSec;
  _d = D / SampleTimeInSec;

  if (_controllerDirection == REVERSE) {
    _p = (0.0 - _p);
    _i = (0.0 - _i);
    _d = (0.0 - _d);
  }
  return true;
}

/* setTunings(...)*************************************************************
 * Set Tunings using the last-rembered POn setting
 ******************************************************************************/
bool PID::setTunings(float P, float I, float D) {
  return setTunings(P, I, D, _pOn);
}

/* setSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed
 ******************************************************************************/
void PID::setUpdateRate(uint32_t updateRate) {
  float ratio  = (float)updateRate
                 / (float)_updateRate;
  _i *= ratio;
  _d /= ratio;
  _updateRate = updateRate;
}

bool PID::setIntegralLimit(int16_t range) {
  int16_t max = abs(range);
  int16_t min = max * -1;
  return setIntegralLimit(min, max);
}

bool PID::setIntegralLimit(int16_t minimum, int16_t maximum) {
  if (minimum >= maximum) return false;
  _minimumIntegralSum = minimum;
  _maximumIntegralSum = maximum;

  if (_inAuto) {
    if (_outputSum > _maximumIntegralSum) _outputSum = _maximumIntegralSum;
    else if (_outputSum < _minimumIntegralSum) _outputSum = _minimumIntegralSum;
  }
  return true;
}

/* setOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
bool PID::setOutputLimits(int16_t minimum, int16_t maximum) {
  if (minimum >= maximum) return false;
  _minimumOutput = minimum;
  _maximumOutput = maximum;

  if (_inAuto) {
    if (*_output > _maximumOutput) { *_output = _maximumOutput; _lastOutput = *_output; }
    else if (*_output < _minimumOutput) { *_output = _minimumOutput; _lastOutput = *_output; }

    if (_outputSum > _maximumOutput) _outputSum = _maximumOutput;
    else if (_outputSum < _minimumOutput) _outputSum = _minimumOutput;
  }
  return true;
}

bool PID::setOutputLimits(int16_t range) {
  int16_t max = abs(range);
  int16_t min = max * -1;
  return setOutputLimits(min, max);
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/
void PID::setMode(int8_t Mode) {
  bool newAuto = (Mode == AUTOMATIC);
  if (newAuto && !_inAuto) { /*we just went from manual to auto*/
    PID::initialize();
  }
  _inAuto = newAuto;
}
void PID::on() {
  setMode(AUTOMATIC);
}
void PID::off() {
  setMode(MANUAL);
}

/* initialize()****************************************************************
 *  does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/
void PID::initialize() {
  _outputSum = *_output;
  _lastInput = *_input;
  if (_outputSum > _maximumOutput) _outputSum = _maximumOutput;
  else if (_outputSum < _minimumOutput) _outputSum = _minimumOutput;
}

/* setControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void PID::setControllerDirection(int8_t Direction) {
  if (_inAuto && Direction != _controllerDirection) {
    _p = (0 - _p);
    _i = (0 - _i);
    _d = (0 - _d);
  }
  _controllerDirection = Direction;
}

void PID::setRampRate(int16_t rampRate) {
  rampRate = abs(rampRate);
  if (rampRate >= _maximumOutput) return;
  _rampRate = rampRate;
}

void PID::unwind() {
  _outputSum = 0;
  _lastInput = *_input;
}

/* Status Funcions*************************************************************
 * Just because you set the P=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
float PID::getP() { return _pOriginal; }
float PID::getI() { return _iOriginal; }
float PID::getD() { return _dOriginal; }
int8_t PID::getMode() { return  _inAuto ? AUTOMATIC : MANUAL; }
int8_t PID::getDirection() { return _controllerDirection; }
uint32_t PID::getUpdateRate() { return _updateRate; }
