/*
 * VelocityPI.h
 * The PI wheel-velocity controller developed in Activity 4, packaged for reuse
 * by Activities 5, 6, 7, Task 4 and the line follower.
 *
 *     u(t) = Kp e(t) + Ki integral( e(t) dt )        (PDF p.25)
 *
 * plus the Activity 1 feedforward term, so the integrator only has to absorb
 * the residual rather than build the whole operating point from zero.
 *
 * An identical copy lives in every sketch folder that uses it.
 */

#ifndef VELOCITY_PI_H
#define VELOCITY_PI_H

#include <Arduino.h>
#include "RobotBase.h"

struct VelocityPI {
  float kp;
  float ki;
  float integral;    // of the error, in RPM*seconds
  float error;       // most recent error, for display
  int output;        // most recent PWM command, for display
  bool useFeedforward;
};

void piReset(VelocityPI &c) {
  c.integral = 0.0f;
  c.error = 0.0f;
  c.output = 0;
}

void piInit(VelocityPI &c, float kp, float ki, bool useFeedforward = true) {
  c.kp = kp;
  c.ki = ki;
  c.useFeedforward = useFeedforward;
  piReset(c);
}

/*
 * One control step. dt is in seconds.
 *
 * The integrator is conditionally frozen when the output is already saturated
 * and the error would push it further into the rail. Without that, a setpoint
 * the motor physically cannot reach makes the integral grow without bound, and
 * the controller then ignores the setpoint coming back down until it unwinds -
 * classic integral windup.
 */
int piUpdate(VelocityPI &c, MotorId m, float setpointRpm, float measuredRpm,
             float dt) {
  const float e = setpointRpm - measuredRpm;
  c.error = e;

  const float ff = c.useFeedforward ? (float)feedforwardPwm(m, setpointRpm) : 0.0f;
  const float candidate = ff + c.kp * e + c.ki * (c.integral + e * dt);

  const bool saturated = (candidate > 255.0f && e > 0.0f) ||
                         (candidate < -255.0f && e < 0.0f);
  if (!saturated) {
    c.integral += e * dt;
  }

  const float u = ff + c.kp * e + c.ki * c.integral;
  c.output = constrain((int)lroundf(u), -255, 255);
  return c.output;
}

#endif  // VELOCITY_PI_H
