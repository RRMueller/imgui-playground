/**
 * @file trackDriveSimulator.h
 * @brief Hydraulic track drive simulator with PID straight-tracking control
 *
 * This simulator models a dual hydraulic track system for construction/agricultural
 * equipment with realistic hydraulic delays, friction, and asymmetries that require
 * PID compensation to maintain straight travel.
 */

#ifndef TRACK_DRIVE_SIMULATOR_H
#define TRACK_DRIVE_SIMULATOR_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * SENSOR READINGS - Mirror real machine sensor interfaces
 * ============================================================================ */

/**
 * @brief Speed sensor readings (RPM or ground speed)
 */
typedef struct {
    float left_track_rpm;       // Left track motor RPM
    float right_track_rpm;      // Right track motor RPM
    float left_ground_speed;    // Left track ground speed (m/s)
    float right_ground_speed;   // Right track ground speed (m/s)
    bool sensor_valid;          // Sensor health flag
} speed_sensors_t;

/**
 * @brief Hydraulic pressure sensors (bar or PSI)
 */
typedef struct {
    float system_pressure;      // Main hydraulic system pressure (bar)
    float left_motor_pressure;  // Left motor working pressure (bar)
    float right_motor_pressure; // Right motor working pressure (bar)
    bool sensor_valid;
} pressure_sensors_t;

/**
 * @brief Flow sensors (liters/min)
 */
typedef struct {
    float left_motor_flow;      // Flow to left motor (L/min)
    float right_motor_flow;     // Flow to right motor (L/min)
    float pump_flow;            // Pump output flow (L/min)
    bool sensor_valid;
} flow_sensors_t;

/**
 * @brief Temperature sensors (Celsius)
 */
typedef struct {
    float hydraulic_oil_temp;   // Hydraulic oil temperature
    float left_motor_temp;      // Left motor case temperature
    float right_motor_temp;     // Right motor case temperature
    bool sensor_valid;
} temp_sensors_t;

/**
 * @brief Vehicle state sensors
 */
typedef struct {
    float heading;              // Vehicle heading (degrees, 0 = straight)
    float heading_rate;         // Rate of heading change (deg/s)
    float ground_speed;         // Average ground speed (m/s)
    float travel_distance;      // Total distance traveled (m)
} vehicle_sensors_t;

/* ============================================================================
 * ACTUATOR COMMANDS - Mirror real machine control interfaces
 * ============================================================================ */

/**
 * @brief Valve control commands
 */
typedef struct {
    float left_valve_current;   // Left valve current command (mA), 0-1000
    float right_valve_current;  // Right valve current command (mA), 0-1000
    float left_valve_position;  // Actual left valve spool position (%), 0-100
    float right_valve_position; // Actual right valve spool position (%), 0-100
} valve_commands_t;

/**
 * @brief Operator inputs (joystick)
 */
typedef struct {
    float forward_cmd;          // Forward/reverse command (-100 to +100)
    float turn_cmd;             // Turn command (-100 to +100, negative = left)
    bool enable;                // Dead-man switch / enable
} operator_input_t;

/* ============================================================================
 * HYDRAULIC SYSTEM MODEL
 * ============================================================================ */

/**
 * @brief Hydraulic system parameters (configuration)
 */
typedef struct {
    // Valve characteristics
    float valve_time_constant;  // Valve response time constant (s), typical 0.05-0.15
    float valve_deadband;       // Valve deadband (mA), typical 50-100
    float valve_current_to_flow_gain; // Flow gain (L/min per mA)

    // Motor characteristics
    float motor_displacement;   // Motor displacement (cc/rev)
    float motor_efficiency;     // Motor efficiency (0.0-1.0)
    float motor_leakage;        // Internal leakage (L/min per bar)

    // System dynamics
    float pressure_buildup_time; // Time to build pressure (s), typical 0.1-0.3
    float max_system_pressure;   // Maximum system pressure (bar)
    float min_operating_pressure; // Minimum operating pressure (bar)

    // Asymmetries (to simulate real-world differences)
    float left_efficiency_factor;  // Left side efficiency multiplier (0.9-1.1)
    float right_efficiency_factor; // Right side efficiency multiplier (0.9-1.1)
} hydraulic_params_t;

/**
 * @brief Hydraulic system state (internal simulation state)
 */
typedef struct {
    // Valve states
    float left_valve_state;     // Internal valve state (filtered position)
    float right_valve_state;    // Internal valve state (filtered position)

    // Pressure states
    float left_pressure;        // Left motor pressure (bar)
    float right_pressure;       // Right motor pressure (bar)

    // Flow states
    float left_flow;            // Actual flow to left motor (L/min)
    float right_flow;           // Actual flow to right motor (L/min)

    // Temperature model (simplified)
    float oil_temperature;      // Hydraulic oil temperature (C)
} hydraulic_state_t;

/* ============================================================================
 * MECHANICAL SYSTEM MODEL
 * ============================================================================ */

/**
 * @brief Mechanical system parameters
 */
typedef struct {
    // Vehicle properties
    float vehicle_mass;         // Vehicle mass (kg)
    float track_width;          // Distance between tracks (m)
    float track_radius;         // Track drive sprocket radius (m)

    // Friction model
    float static_friction;      // Static friction coefficient
    float coulomb_friction;     // Coulomb (sliding) friction coefficient
    float viscous_friction;     // Viscous friction coefficient
    float stiction_velocity;    // Velocity threshold for stiction (m/s)

    // Load variations
    float base_load;            // Base resistive load (N)
    float load_variation;       // Random load variation amplitude (N)
} mechanical_params_t;

/**
 * @brief Mechanical system state
 */
typedef struct {
    // Track states
    float left_velocity;        // Left track velocity (m/s)
    float right_velocity;       // Right track velocity (m/s)
    float left_rpm;             // Left track RPM
    float right_rpm;            // Right track RPM

    // Vehicle state
    float heading;              // Vehicle heading (degrees)
    float heading_rate;         // Heading rate (deg/s)
    float position_x;           // X position (m)
    float position_y;           // Y position (m)
    float travel_distance;      // Total distance (m)

    // Load state
    float current_load;         // Current resistive load (N)
} mechanical_state_t;

/* ============================================================================
 * PID CONTROLLER
 * ============================================================================ */

/**
 * @brief PID controller parameters
 */
typedef struct {
    float kp;                   // Proportional gain
    float ki;                   // Integral gain
    float kd;                   // Derivative gain

    float integral_limit;       // Anti-windup limit for integral term
    float output_limit;         // Output limit (% of command)

    bool enable;                // PID enable flag
} pid_params_t;

/**
 * @brief PID controller state
 */
typedef struct {
    float error;                // Current error
    float error_prev;           // Previous error (for derivative)
    float integral;             // Integral accumulator
    float derivative;           // Derivative term

    float output;               // PID output
    float p_term;               // Proportional term (for debugging)
    float i_term;               // Integral term (for debugging)
    float d_term;               // Derivative term (for debugging)
} pid_state_t;

/* ============================================================================
 * COMPLETE SIMULATOR STATE
 * ============================================================================ */

/**
 * @brief Complete track drive simulator
 */
typedef struct {
    // Configuration
    hydraulic_params_t hydraulic_config;
    mechanical_params_t mechanical_config;
    pid_params_t pid_config;

    // State
    hydraulic_state_t hydraulic;
    mechanical_state_t mechanical;
    pid_state_t pid;

    // Inputs (from operator)
    operator_input_t operator_input;

    // Outputs (to actuators)
    valve_commands_t valve_commands;

    // Sensor readings (mirrors real machine)
    speed_sensors_t speed_sensors;
    pressure_sensors_t pressure_sensors;
    flow_sensors_t flow_sensors;
    temp_sensors_t temp_sensors;
    vehicle_sensors_t vehicle_sensors;

    // Simulation time
    float sim_time;
    float dt;                   // Time step (s)
} track_drive_sim_t;

/* ============================================================================
 * FUNCTION DECLARATIONS
 * ============================================================================ */

/**
 * @brief Initialize simulator with default parameters
 * @param sim Pointer to simulator structure
 */
void track_sim_init(track_drive_sim_t* sim);

/**
 * @brief Initialize with realistic default parameters
 * @param sim Simulator structure
 */
void track_sim_init_default_params(track_drive_sim_t* sim);

/**
 * @brief Update simulator (call once per time step)
 * @param sim Simulator structure
 * @param dt Time step (seconds)
 */
void track_sim_update(track_drive_sim_t* sim, float dt);

/**
 * @brief Reset simulator to initial state
 * @param sim Simulator structure
 */
void track_sim_reset(track_drive_sim_t* sim);

/**
 * @brief Set operator input (joystick command)
 * @param sim Simulator structure
 * @param forward Forward/reverse command (-100 to +100)
 * @param turn Turn command (-100 to +100)
 * @param enable Enable flag
 */
void track_sim_set_input(track_drive_sim_t* sim, float forward, float turn, bool enable);

/**
 * @brief Set PID parameters
 * @param sim Simulator structure
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 */
void track_sim_set_pid_gains(track_drive_sim_t* sim, float kp, float ki, float kd);

/**
 * @brief Enable/disable PID controller
 * @param sim Simulator structure
 * @param enable Enable flag
 */
void track_sim_enable_pid(track_drive_sim_t* sim, bool enable);

/**
 * @brief Inject asymmetry (simulate worn/mismatched components)
 * @param sim Simulator structure
 * @param left_efficiency Left side efficiency (0.8-1.2, 1.0 = nominal)
 * @param right_efficiency Right side efficiency (0.8-1.2, 1.0 = nominal)
 */
void track_sim_set_asymmetry(track_drive_sim_t* sim, float left_efficiency, float right_efficiency);

/**
 * @brief Get heading error (for PID)
 * @param sim Simulator structure
 * @return Heading error in degrees
 */
float track_sim_get_heading_error(const track_drive_sim_t* sim);

/**
 * @brief Get speed difference between tracks
 * @param sim Simulator structure
 * @return Speed difference (left - right) in m/s
 */
float track_sim_get_speed_difference(const track_drive_sim_t* sim);

#ifdef __cplusplus
}
#endif

#endif /* TRACK_DRIVE_SIMULATOR_H */
