#include "trajectory.h"

// for more information about the trapezoidal profile, see https://en.wikipedia.org/wiki/Trapezoidal_velocity_profile

// Resets the trapezoidal profile state to the current position and velocity, 
// and sets the target position.
void trajectoryResetTrapezoidal(TrapezoidalProfileState& state,
                                float current_position,
                                float current_velocity,
                                float target_position) {
  state.position = current_position;
  state.velocity = current_velocity;
  state.acceleration = 0.0f;
  state.target = target_position;
  state.initialized = true;
}
// Bang-bang acceleration control with trapezoidal velocity profile
TrapezoidalProfileOutput trajectoryStepTrapezoidal(TrapezoidalProfileState& state,
                                                   float current_position,
                                                   float current_velocity,
                                                   float dt,
                                                   float target_position,
                                                   float velocity_limit,
                                                   float acceleration_limit) {
  TrapezoidalProfileOutput out;

  // Guard against bad dt values at startup or timer glitches.
  if (dt <= 0.0f || dt > 0.5f) {
    dt = 1e-3f;
  }

  float velocity_abs_limit = fabsf(velocity_limit);
  float acceleration_abs_limit = fabsf(acceleration_limit);

  // On first call, seed profile state from measured position/velocity.
  if (!state.initialized) {
    trajectoryResetTrapezoidal(state, current_position, current_velocity, target_position);
  }
  state.target = target_position;

  // Disabled acceleration limit means direct target snap.
  if (!_isset(acceleration_abs_limit) || acceleration_abs_limit <= 0.0f) {
    state.position = state.target;
    state.velocity = 0.0f;
    state.acceleration = 0.0f;
    out.position = state.position;
    out.velocity = state.velocity;
    out.acceleration = state.acceleration;
    return out;
  }

  float position_error = state.target - state.position;
  float direction_to_target = _sign(position_error);
  float commanded_accel = 0.0f;
  float distance_remaining = fabsf(position_error);
  float velocity_along_path = state.velocity * direction_to_target;
  float speed_magnitude = fabsf(state.velocity);

  // IMPORTANT
  // for given velocity v and acceleration a, the time to stop t = v/a - (following from v = a.t )
  // and the distance traveled during that time is d = v.t - 0.5.a.t^2 = v^2 / (2.a)
  float stopping_distance = (speed_magnitude * speed_magnitude) / (2.0f * acceleration_abs_limit);

  // If nearly at target and almost stopped, finalize immediately.
  if (distance_remaining < 1e-6f && speed_magnitude < (acceleration_abs_limit * dt)) {
    state.position = state.target;
    state.velocity = 0.0f;
    commanded_accel = 0.0f;
  } else {

    // Bang-bang acceleration control with trapezoidal velocity profile.
    // Trapezoid phase selection:
    // 1) moving away -> accelerate toward target
    // 2) close enough to need braking -> decelerate
    // 3) below vmax and far enough -> accelerate
    // 4) else cruise
    if (velocity_along_path < 0.0f) {
      commanded_accel = direction_to_target * acceleration_abs_limit;
    } else if (stopping_distance >= distance_remaining) {
      commanded_accel = -direction_to_target * acceleration_abs_limit;
    } else if (speed_magnitude < velocity_abs_limit) {
      commanded_accel = direction_to_target * acceleration_abs_limit;
    } else {
      commanded_accel = 0.0f;
    }

    // integrate acceleration to get new velocity
    state.velocity += commanded_accel * dt;
    state.velocity = _constrain(state.velocity, -velocity_abs_limit, velocity_abs_limit);

    // integrate velocity to get new position
    float position_step = state.velocity * dt;
    if (fabsf(position_error) <= fabsf(position_step)) {
      // Clamp final step to avoid overshoot around the endpoint.
      state.position = state.target;
      state.velocity = 0.0f;
      commanded_accel = 0.0f;
    } else {
      state.position += position_step;
    }
  }

  state.acceleration = commanded_accel;
  out.position = state.position;
  out.velocity = state.velocity;
  out.acceleration = state.acceleration;
  return out;
}