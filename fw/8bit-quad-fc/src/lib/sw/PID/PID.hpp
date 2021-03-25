#pragma once
#include <inttypes.h>
#include <Arduino.h>


class Pcontroller {
public:
  Pcontroller(int16_t* setpoint, float* input, int16_t* output, float P = 1);

  bool setTunings(float P);
  bool setOutputLimits(int16_t range);
  bool setOutputLimits(int16_t minimum, int16_t maximum);
  void setUpdateRate(uint32_t updateRate);
  void setRampRate(int16_t rampRate);
  void on();
  void off();
  void direct();
  void reverse();

  float getP();
  uint32_t getUpdateRate();

  void compute();

private:
  float *_input;
  int16_t *_output;
  int16_t *_setpoint;

  float _p;
  bool _direct;
  bool _state;
  int16_t _minimumOutput, _maximumOutput;
  int16_t _lastOutput;
  int16_t _rampRate;
  uint32_t _updateRate;
};


#define AUTOMATIC 1
#define MANUAL    0
#define DIRECT  0
#define REVERSE  1
#define P_ON_M 0
#define P_ON_E 1

class PID {
public:
  PID(int16_t* setpoint, float* input, int16_t* output,
      float P, float I, float D,
      int POn, int controllerDirection);
  PID(int16_t* setpoint, float* input, int16_t* output,
      float P = 1.0, float I = 0.0, float D = 0.0,
      int controllerDirection = DIRECT);

  void compute();
  void setMode(int8_t mode);
  void on();
  void off();
  bool setIntegralLimit(int16_t range);
  bool setIntegralLimit(int16_t minimum, int16_t maximum);
  bool setOutputLimits(int16_t range);
  bool setOutputLimits(int16_t minimum, int16_t maximum);
  bool setTunings(float P, float I = 0.0, float D = 0.0);
  bool setTunings(float P, float I, float D, int8_t PonM);
  void setControllerDirection(int8_t controllerDirection);
  void setUpdateRate(uint32_t updateRate);
  void setRampRate(int16_t rampRate);
  void unwind();

  float getP();
  float getI();
  float getD();
  int8_t getMode();
  int8_t getDirection();
  uint32_t getUpdateRate();

private:
  void initialize();

  float _pOriginal;
  float _iOriginal;
  float _dOriginal;

  float _p;
  float _i;
  float _d;

  int8_t _controllerDirection;
  int8_t _pOn;

  float *_input;
  int16_t *_output;
  int16_t *_setpoint;

  uint32_t _lastTime;
  float _outputSum, _lastInput;
  int16_t _lastOutput;

  uint32_t _updateRate;
  int16_t _minimumIntegralSum, _maximumIntegralSum;
  int16_t _minimumOutput, _maximumOutput;
  int16_t _rampRate;
  bool _inAuto, _pOnE;
};
