#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "foc_utils.h"

/**
 * State of a trapezoidal motion profile
 */
struct TrapezoidalProfileState {
  float position = 0.0f;
  float velocity = 0.0f;
  // float acceleration = 0.0f; // not used so commented to save memory
  // float target = 0.0f;
  bool initialized = false;
};

/**
 *  Output of a trapezoidal motion profile step
 */
struct TrapezoidalProfileOutput {
  float position = 0.0f;
  float velocity = 0.0f;
  float acceleration = 0.0f;
};

/**
 *  Resets the trapezoidal profile state to the current position and velocity, and sets the target position.
 * @param state - the current state of the trapezoidal profile (position, velocity, acceleration, target, initialized)
 * @param current_position - the current position of the system
 * @param current_velocity - the current velocity of the system
 * @param target_position - the desired target position to reach
 */
void trajectoryResetTrapezoidal(TrapezoidalProfileState& state,
                                float current_position,
                                float current_velocity,
                                float target_position);

/**
 *  Calculates the next step of a trapezoidal motion profile given the current state, target position, and limits.
 *  @param state - the current state of the trapezoidal profile (position, velocity, acceleration, target, initialized)
 *  @param current_position - the current position of the system
 *  @param current_velocity - the current velocity of the system
 *  @param dt - the time step in seconds
 *  @param target_position - the desired target position to reach
 *  @param velocity_limit - the maximum allowed velocity (absolute value)
 *  @param acceleration_limit - the maximum allowed acceleration (absolute value)
 *  @returns a TrapezoidalProfileOutput struct containing the updated position, velocity, and acceleration for the next step of the profile
 * 
 */
TrapezoidalProfileOutput trajectoryStepTrapezoidal(TrapezoidalProfileState& state,
                                                   float current_position,
                                                   float current_velocity,
                                                   float dt,
                                                   float target_position,
                                                   float velocity_limit,
                                                   float acceleration_limit);

#endif