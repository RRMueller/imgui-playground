/**
 * @file trackDriveSimulator.cpp
 * @brief Implementation of hydraulic track drive simulator
 */

#include "trackDriveSimulator.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ============================================================================
 * UTILITY FUNCTIONS
 * ============================================================================ */

/**
 * @brief Clamp value between min and max
 */
static float clamp(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

/**
 * @brief First-order low-pass filter
 * @param current Current filtered value
 * @param target Target value
 * @param time_constant Time constant (seconds)
 * @param dt Time step (seconds)
 * @return New filtered value
 */
static float low_pass_filter(float current, float target, float time_constant, float dt) {
    if (time_constant <= 0.0f) return target;
    float alpha = dt / (time_constant + dt);
    return current + alpha * (target - current);
}

/**
 * @brief Apply deadband to input
 */
static float apply_deadband(float value, float deadband) {
    if (fabs(value) < deadband) return 0.0f;
    if (value > 0) return value - deadband;
    return value + deadband;
}

/**
 * @brief Sign function
 */
static float sign(float value) {
    if (value > 0.0f) return 1.0f;
    if (value < 0.0f) return -1.0f;
    return 0.0f;
}

/* ============================================================================
 * INITIALIZATION
 * ============================================================================ */

void track_sim_init_default_params(track_drive_sim_t* sim) {
    // Hydraulic parameters (typical for medium construction equipment)
    sim->hydraulic_config.valve_time_constant = 0.08f;      // 80ms valve response
    sim->hydraulic_config.valve_deadband = 50.0f;           // 50mA deadband (5% of range)
    sim->hydraulic_config.valve_current_to_flow_gain = 0.012f; // 0.012 L/min per mA (max 12 L/min)

    sim->hydraulic_config.motor_displacement = 100.0f;      // 100 cc/rev
    sim->hydraulic_config.motor_efficiency = 0.92f;         // 92% efficient
    sim->hydraulic_config.motor_leakage = 0.001f;           // Small leakage

    sim->hydraulic_config.pressure_buildup_time = 0.15f;    // 150ms pressure buildup
    sim->hydraulic_config.max_system_pressure = 350.0f;     // 350 bar max
    sim->hydraulic_config.min_operating_pressure = 20.0f;   // 20 bar minimum

    sim->hydraulic_config.left_efficiency_factor = 1.0f;    // Start symmetrical
    sim->hydraulic_config.right_efficiency_factor = 1.0f;

    // Mechanical parameters
    sim->mechanical_config.vehicle_mass = 5000.0f;          // 5000 kg vehicle
    sim->mechanical_config.track_width = 2.0f;              // 2m track width
    sim->mechanical_config.track_radius = 0.25f;            // 0.25m sprocket radius

    sim->mechanical_config.static_friction = 0.8f;          // High static friction
    sim->mechanical_config.coulomb_friction = 0.5f;         // Lower sliding friction
    sim->mechanical_config.viscous_friction = 20.0f;        // Viscous damping
    sim->mechanical_config.stiction_velocity = 0.05f;       // 0.05 m/s stiction threshold

    sim->mechanical_config.base_load = 500.0f;              // 500N base resistance
    sim->mechanical_config.load_variation = 100.0f;         // +/- 100N variation

    // PID parameters (tuned for speed difference in m/s)
    sim->pid_config.kp = 2000.0f;                           // Proportional gain (0.1 m/s diff = 200 mA)
    sim->pid_config.ki = 100.0f;                            // Integral gain
    sim->pid_config.kd = 50.0f;                             // Derivative gain
    sim->pid_config.integral_limit = 2.0f;                  // Anti-windup limit (in m/s*s)
    sim->pid_config.output_limit = 400.0f;                  // Max 400 mA correction (40% of full scale)
    sim->pid_config.enable = false;                         // Start disabled
}

void track_sim_init(track_drive_sim_t* sim) {
    // Zero out entire structure
    memset(sim, 0, sizeof(track_drive_sim_t));

    // Set default parameters
    track_sim_init_default_params(sim);

    // Initialize sensor validity
    sim->speed_sensors.sensor_valid = true;
    sim->pressure_sensors.sensor_valid = true;
    sim->flow_sensors.sensor_valid = true;
    sim->temp_sensors.sensor_valid = true;

    // Initialize temperature
    sim->temp_sensors.hydraulic_oil_temp = 40.0f;  // Start at 40C
    sim->hydraulic.oil_temperature = 40.0f;

    // System pressure
    sim->pressure_sensors.system_pressure = sim->hydraulic_config.min_operating_pressure;

    sim->dt = 0.016f; // Default 60 Hz
}

void track_sim_reset(track_drive_sim_t* sim) {
    // Save configuration
    hydraulic_params_t hyd_cfg = sim->hydraulic_config;
    mechanical_params_t mech_cfg = sim->mechanical_config;
    pid_params_t pid_cfg = sim->pid_config;

    // Reinitialize
    track_sim_init(sim);

    // Restore configuration
    sim->hydraulic_config = hyd_cfg;
    sim->mechanical_config = mech_cfg;
    sim->pid_config = pid_cfg;
}

/* ============================================================================
 * INPUT/OUTPUT FUNCTIONS
 * ============================================================================ */

void track_sim_set_input(track_drive_sim_t* sim, float forward, float turn, bool enable) {
    sim->operator_input.forward_cmd = clamp(forward, -1000.0f, 1000.0f);
    sim->operator_input.turn_cmd = clamp(turn, -1000.0f, 1000.0f);
    sim->operator_input.enable = enable;
}

void track_sim_set_pid_gains(track_drive_sim_t* sim, float kp, float ki, float kd) {
    sim->pid_config.kp = kp;
    sim->pid_config.ki = ki;
    sim->pid_config.kd = kd;
}

void track_sim_enable_pid(track_drive_sim_t* sim, bool enable) {
    if (enable && !sim->pid_config.enable) {
        // Reset integral when enabling
        sim->pid.integral = 0.0f;
    }
    sim->pid_config.enable = enable;
}

void track_sim_set_asymmetry(track_drive_sim_t* sim, float left_efficiency, float right_efficiency) {
    sim->hydraulic_config.left_efficiency_factor = clamp(left_efficiency, 0.5f, 1.5f);
    sim->hydraulic_config.right_efficiency_factor = clamp(right_efficiency, 0.5f, 1.5f);
}

float track_sim_get_heading_error(const track_drive_sim_t* sim) {
    return sim->mechanical.heading;
}

float track_sim_get_speed_difference(const track_drive_sim_t* sim) {
    return sim->mechanical.left_velocity - sim->mechanical.right_velocity;
}

/* ============================================================================
 * PID CONTROLLER
 * ============================================================================ */

/**
 * @brief Update PID controller
 * @param pid PID state
 * @param params PID parameters
 * @param error Current error
 * @param dt Time step
 * @return PID output
 */
static float update_pid(pid_state_t* pid, const pid_params_t* params, float error, float dt) {
    if (!params->enable) {
        pid->integral = 0.0f;
        pid->output = 0.0f;
        return 0.0f;
    }

    // Store error
    pid->error = error;

    // Proportional term
    pid->p_term = params->kp * error;

    // Integral term with anti-windup
    pid->integral += error * dt;
    pid->integral = clamp(pid->integral, -params->integral_limit, params->integral_limit);
    pid->i_term = params->ki * pid->integral;

    // Derivative term
    pid->derivative = (error - pid->error_prev) / dt;
    pid->d_term = params->kd * pid->derivative;

    // Total output
    pid->output = pid->p_term + pid->i_term + pid->d_term;
    pid->output = clamp(pid->output, -params->output_limit, params->output_limit);

    // Save for next iteration
    pid->error_prev = error;

    return pid->output;
}

/* ============================================================================
 * HYDRAULIC SYSTEM SIMULATION
 * ============================================================================ */

/**
 * @brief Simulate single hydraulic channel (valve + motor)
 */
static void simulate_hydraulic_channel(
    float command,                      // Input command (mA)
    float efficiency_factor,            // Asymmetry factor
    float* valve_state,                 // Valve state
    float* pressure,                    // Motor pressure
    float* flow,                        // Motor flow
    float velocity,                     // Current track velocity
    const hydraulic_params_t* params,
    float dt)
{
    // Apply deadband to command
    float effective_command = apply_deadband(command, params->valve_deadband);

    // Valve dynamics (first-order lag)
    float target_valve = clamp(effective_command / 1000.0f, -1.0f, 1.0f); // Normalize to -1 to +1
    *valve_state = low_pass_filter(*valve_state, target_valve, params->valve_time_constant, dt);

    // Flow from valve position
    float commanded_flow = (*valve_state) * params->valve_current_to_flow_gain * 1000.0f; // L/min
    commanded_flow *= efficiency_factor; // Apply asymmetry

    // Pressure buildup (simplified model based on load)
    // Pressure scales with flow rate
    float max_flow = 12.0f; // Max flow at full valve ~12 L/min
    float pressure_ratio = clamp(fabs(commanded_flow) / max_flow, 0.0f, 1.0f);
    float target_pressure = pressure_ratio * params->max_system_pressure * 0.7f;
    *pressure = low_pass_filter(*pressure, target_pressure, params->pressure_buildup_time, dt);

    // Actual flow (accounting for leakage)
    // Leakage reduces flow magnitude but doesn't reverse direction
    float leakage = params->motor_leakage * (*pressure);
    if (fabs(commanded_flow) > leakage) {
        *flow = commanded_flow - sign(commanded_flow) * leakage;
    } else {
        *flow = 0.0f; // Flow too small to overcome leakage
    }
}

/* ============================================================================
 * MECHANICAL SYSTEM SIMULATION
 * ============================================================================ */

/**
 * @brief Calculate friction force
 */
static float calculate_friction(float velocity, const mechanical_params_t* params) {
    float friction = 0.0f;

    // Stiction model (static vs coulomb friction)
    if (fabs(velocity) < params->stiction_velocity) {
        friction = params->static_friction * sign(velocity);
    } else {
        friction = params->coulomb_friction * sign(velocity);
    }

    // Viscous friction
    friction += params->viscous_friction * velocity;

    return friction;
}

/**
 * @brief Simulate track mechanics
 */
static void simulate_track_mechanics(
    float flow,                         // Flow to motor (L/min)
    float* velocity,                    // Track velocity (m/s)
    float* rpm,                         // Track RPM
    const hydraulic_params_t* hyd_params,
    const mechanical_params_t* mech_params,
    float dt)
{
    // Convert flow to RPM
    // Flow (L/min) = Displacement (L/rev) * RPM
    float displacement_liters = hyd_params->motor_displacement / 1000.0f; // cc to liters
    float target_rpm = 0.0f;
    if (fabs(displacement_liters) > 0.001f) {
        target_rpm = (flow / displacement_liters) * hyd_params->motor_efficiency;
    }

    // Apply simple first-order dynamics (inertia)
    float rpm_time_constant = 0.2f; // 200ms mechanical time constant
    *rpm = low_pass_filter(*rpm, target_rpm, rpm_time_constant, dt);

    // Convert RPM to linear velocity
    // v = omega * r, where omega = RPM * 2*pi / 60
    *velocity = (*rpm) * (2.0f * M_PI / 60.0f) * mech_params->track_radius;

    // Apply friction (simplified - reduces velocity slightly)
    float friction = calculate_friction(*velocity, mech_params);
    float friction_accel = -friction / mech_params->vehicle_mass;
    *velocity += friction_accel * dt;
}

/* ============================================================================
 * MAIN UPDATE FUNCTION
 * ============================================================================ */

void track_sim_update(track_drive_sim_t* sim, float dt) {
    sim->dt = dt;
    sim->sim_time += dt;

    // Get operator input
    float forward_cmd = sim->operator_input.forward_cmd;
    float turn_cmd = sim->operator_input.turn_cmd;

    if (!sim->operator_input.enable) {
        forward_cmd = 0.0f;
        turn_cmd = 0.0f;
    }

    // Convert joystick to left/right track commands (ISO-S pattern)
    // Forward: both tracks same speed
    // Turn: differential speed
    float left_base_cmd = forward_cmd + turn_cmd;
    float right_base_cmd = forward_cmd - turn_cmd;

    // Clamp to valve range
    left_base_cmd = clamp(left_base_cmd, -1000.0f, 1000.0f);
    right_base_cmd = clamp(right_base_cmd, -1000.0f, 1000.0f);

    // Calculate PID correction (based on speed difference - real machines use speed sensors, not heading)
    float pid_output = 0.0f;
    if (sim->pid_config.enable && fabs(forward_cmd) > 10.0f) { // Only when moving forward/back
        // Use speed difference as error: if left > right (turning right), error is positive
        // We want to slow down the faster side, so negate: error = right - left
        float speed_diff = sim->mechanical.left_velocity - sim->mechanical.right_velocity;
        float speed_error = -speed_diff; // Negate so positive error = turning right, need to correct left
        pid_output = update_pid(&sim->pid, &sim->pid_config, speed_error, dt);
    } else {
        sim->pid.integral = 0.0f; // Reset when not active
    }

    // Apply PID correction
    // Positive pid_output = left track too fast, so reduce left / increase right
    float left_cmd = left_base_cmd + pid_output;
    float right_cmd = right_base_cmd - pid_output;

    // Clamp final commands
    left_cmd = clamp(left_cmd, -1000.0f, 1000.0f);
    right_cmd = clamp(right_cmd, -1000.0f, 1000.0f);

    // Store valve commands
    sim->valve_commands.left_valve_current = left_cmd;
    sim->valve_commands.right_valve_current = right_cmd;

    // Simulate hydraulic system for each track
    simulate_hydraulic_channel(
        left_cmd,
        sim->hydraulic_config.left_efficiency_factor,
        &sim->hydraulic.left_valve_state,
        &sim->hydraulic.left_pressure,
        &sim->hydraulic.left_flow,
        sim->mechanical.left_velocity,
        &sim->hydraulic_config,
        dt
    );

    simulate_hydraulic_channel(
        right_cmd,
        sim->hydraulic_config.right_efficiency_factor,
        &sim->hydraulic.right_valve_state,
        &sim->hydraulic.right_pressure,
        &sim->hydraulic.right_flow,
        sim->mechanical.right_velocity,
        &sim->hydraulic_config,
        dt
    );

    // Simulate mechanical system for each track
    simulate_track_mechanics(
        sim->hydraulic.left_flow,
        &sim->mechanical.left_velocity,
        &sim->mechanical.left_rpm,
        &sim->hydraulic_config,
        &sim->mechanical_config,
        dt
    );

    simulate_track_mechanics(
        sim->hydraulic.right_flow,
        &sim->mechanical.right_velocity,
        &sim->mechanical.right_rpm,
        &sim->hydraulic_config,
        &sim->mechanical_config,
        dt
    );

    // Calculate vehicle heading from track speed difference
    // heading_rate = (v_left - v_right) / track_width
    // Positive heading = right turn, Negative heading = left turn
    float velocity_diff = sim->mechanical.left_velocity - sim->mechanical.right_velocity;
    sim->mechanical.heading_rate = (velocity_diff / sim->mechanical_config.track_width) * (180.0f / M_PI); // rad/s to deg/s
    sim->mechanical.heading += sim->mechanical.heading_rate * dt;

    // Calculate ground speed (average of tracks)
    float avg_velocity = (sim->mechanical.left_velocity + sim->mechanical.right_velocity) / 2.0f;

    // Update position
    float heading_rad = sim->mechanical.heading * (M_PI / 180.0f);
    sim->mechanical.position_x += avg_velocity * cos(heading_rad) * dt;
    sim->mechanical.position_y += avg_velocity * sin(heading_rad) * dt;
    sim->mechanical.travel_distance += fabs(avg_velocity) * dt;

    // Update sensor outputs
    sim->speed_sensors.left_track_rpm = sim->mechanical.left_rpm;
    sim->speed_sensors.right_track_rpm = sim->mechanical.right_rpm;
    sim->speed_sensors.left_ground_speed = sim->mechanical.left_velocity;
    sim->speed_sensors.right_ground_speed = sim->mechanical.right_velocity;

    sim->pressure_sensors.left_motor_pressure = sim->hydraulic.left_pressure;
    sim->pressure_sensors.right_motor_pressure = sim->hydraulic.right_pressure;
    sim->pressure_sensors.system_pressure = fmax(sim->hydraulic.left_pressure, sim->hydraulic.right_pressure);

    sim->flow_sensors.left_motor_flow = sim->hydraulic.left_flow;
    sim->flow_sensors.right_motor_flow = sim->hydraulic.right_flow;
    sim->flow_sensors.pump_flow = fabs(sim->hydraulic.left_flow) + fabs(sim->hydraulic.right_flow);

    sim->vehicle_sensors.heading = sim->mechanical.heading;
    sim->vehicle_sensors.heading_rate = sim->mechanical.heading_rate;
    sim->vehicle_sensors.ground_speed = avg_velocity;
    sim->vehicle_sensors.travel_distance = sim->mechanical.travel_distance;

    // Simple temperature model (increases with flow)
    float heat_input = (fabs(sim->hydraulic.left_flow) + fabs(sim->hydraulic.right_flow)) * 0.01f;
    float heat_dissipation = (sim->hydraulic.oil_temperature - 40.0f) * 0.1f;
    sim->hydraulic.oil_temperature += (heat_input - heat_dissipation) * dt;
    sim->temp_sensors.hydraulic_oil_temp = sim->hydraulic.oil_temperature;
    sim->temp_sensors.left_motor_temp = sim->hydraulic.oil_temperature + fabs(sim->hydraulic.left_flow) * 0.05f;
    sim->temp_sensors.right_motor_temp = sim->hydraulic.oil_temperature + fabs(sim->hydraulic.right_flow) * 0.05f;

    // Store valve positions (for display)
    sim->valve_commands.left_valve_position = sim->hydraulic.left_valve_state * 100.0f;
    sim->valve_commands.right_valve_position = sim->hydraulic.right_valve_state * 100.0f;
}
