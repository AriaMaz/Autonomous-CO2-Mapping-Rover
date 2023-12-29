// Motor control pins for PWM output
int EA = 3; // Left wheel PWM control pin (must be a PWM pin)
int EB = 5; // Right wheel PWM control pin (must be a PWM pin)

// Motor direction control digital pins
int I1 = 2; // Left wheel direction pin 1
int I2 = 4; // Left wheel direction pin 2 (complementary to I1)
int I3 = 7; // Right wheel direction pin 1
int I4 = 8; // Right wheel direction pin 2 (complementary to I3)

// Wheel encoder digital pins
const byte L_SIGNAL_A = 12; // Left wheel encoder signal A
const byte L_SIGNAL_B = 13; // Left wheel encoder signal B
const byte R_SIGNAL_A = 10; // Right wheel encoder signal A
const byte R_SIGNAL_B = 11; // Right wheel encoder signal B

// Sharp sensor analog pins
const byte SHARP_PIN_F = A4; // Front Sharp sensor pin
const byte SHARP_PIN_L = A5; // Left Sharp sensor pin
const byte SHARP_PIN_R = A3; // Right Sharp sensor pin

// Encoder ticks per revolution (TPR) of the motor
const int TPR = 3000;

// Physical dimensions of the robot
const double ELL = 0.2775; // Robot body width in meters
const double RHO = 0.0625; // Wheel radius in meters

// Variables for encoder tick counts
volatile long L_encoder_ticks = 0; // Left wheel encoder ticks
volatile long R_encoder_ticks = 0; // Right wheel encoder ticks
double l_distance; // Cumulative distance traveled by the left wheel
double r_distance; // Cumulative distance traveled by the right wheel

// Variables for storing proximity measurements from Sharp sensors
float sharp_range_F; // Front sensor range in centimeters
float sharp_range_L; // Left sensor range in centimeters
float sharp_range_R; // Right sensor range in centimeters

// Structure to hold the current velocity of each wheel
typedef struct {
    double v_left; // Left wheel velocity in meters per second
    double v_right; // Right wheel velocity in meters per second
} State;

State current; // Current state containing the velocities of each wheel

// Desired motion parameters
double desired_velocity = 0; // Desired forward velocity of the robot in meters per second
double desired_steering_rate = 0; // Desired steering rate (angular velocity)
double default_speed = 0.6; // Default speed of the robot when not avoiding obstacles

// PID controller variables for navigation and control
int kp = 300; // Proportional gain for PID controller
int ki = 250; // Integral gain for PID controller
double omega_L, omega_R; // Angular velocities of the left and right wheels
double v_L, v_R; // Linear velocities of the left and right wheels
double right_desired, left_desired; // Desired linear velocities for the left and right wheels
double error; // Error term used in PID calculations
double proportional; // Proportional term for PID controller
double integral_L = 0, integral_R = 0; // Integral terms for PID controller
short u_L = 0, u_R = 0; // Control signals for the left and right wheels

// Timekeeping variables to track time intervals for PID updates
unsigned long tick_clock; // Timestamp of the last encoder tick
unsigned long controller_clock; // Timestamp of the last PID controller update
unsigned long stop_clock; // Timestamp used for implementing stopping durations

// Variables to detect when the wheels have stopped moving
unsigned long last_left_zero = 0; // Timestamp when the left wheel last had zero velocity
unsigned long last_right_zero = 0; // Timestamp when the right wheel last had zero velocity
boolean turn_left = false; // Flag indicating whether the robot should turn left during self-correction

// Interrupt service routine for left wheel encoder
void L_decodeEncoderTicks() {
    // Check the state of signal B to determine the direction
    if (digitalRead(L_SIGNAL_B) == LOW) {
        // If signal B is LOW, decrement the tick count (wheel moving forward)
        L_encoder_ticks--;
    } else {
        // If signal B is HIGH, increment the tick count (wheel moving backward)
        L_encoder_ticks++;
    }
}

// Interrupt service routine for right wheel encoder
void R_decodeEncoderTicks() {
    // Check the state of signal B to determine the direction
    if (digitalRead(R_SIGNAL_B) == LOW) {
        // If signal B is LOW, increment the tick count (wheel moving forward)
        R_encoder_ticks++;
    } else {
        // If signal B is HIGH, decrement the tick count (wheel moving backward)
        R_encoder_ticks--;
    }
}

// Compute the current state of the vehicle including the velocity of each wheel
void compute_vehicle_state() {
    // Calculate the time elapsed since the last tick count
    unsigned long current_time = millis();
    unsigned long time_difference = current_time - tick_clock;

    // Estimate the rotational speed in radians per second
    omega_L = 2.0 * PI * (static_cast<double>(L_encoder_ticks) / TPR) * (1000.0 / static_cast<double>(time_difference));
    omega_R = 2.0 * PI * (static_cast<double>(R_encoder_ticks) / TPR) * (1000.0 / static_cast<double>(time_difference));

    // Calculate the linear distance traveled by each wheel
    l_distance += (L_encoder_ticks / static_cast<double>(TPR)) * (2.0 * PI * RHO);
    r_distance += (R_encoder_ticks / static_cast<double>(TPR)) * (2.0 * PI * RHO);

    // Reset the encoder ticks for the next interval
    L_encoder_ticks = 0;
    R_encoder_ticks = 0;
    tick_clock = current_time; // Update the tick clock to the current time

    // Calculate the linear velocity for each wheel
    v_L = RHO * omega_L;
    v_R = RHO * omega_R;

    // Update the current state with the new velocities
    current.v_left = v_L;
    current.v_right = v_R;
}

// Right wheel PI controller to maintain desired velocity and steering rate
short right_pi_controller(double estimated_vr, double vd, double wd) {
    // Calculate the desired velocity for the right wheel based on steering rate
    right_desired = vd + (wd * ELL / 2);
    // Compute the error between estimated and desired velocities
    error = right_desired - estimated_vr;
    // Calculate the proportional term
    proportional = kp * error;
    // Calculate the integral term, integrating over time
    integral_R += ki * error * ((millis() - controller_clock) / 1000.0);

    // Reset the integral term if it becomes NaN due to some error
    if (isnan(integral_R)) {
        integral_R = 0;
    }

    // Calculate the control signal for the right wheel
    u_R = static_cast<short>(proportional + integral_R);

    // Saturate the control signal to the maximum PWM value
    if (u_R > 255) {
        integral_R = 0; // Reset the integral term to prevent windup
        u_R = 255; // Cap the control signal
    } else if (u_R < -255) {
        integral_R = 0; // Reset the integral term to prevent windup
        u_R = -255; // Cap the control signal at the negative limit
    }

    // Set the motor direction based on the sign of the control signal
    if (u_R < 0) {
        u_R = abs(u_R); // Take the absolute value for PWM
        // Set motor to move backward
        digitalWrite(I1, LOW);
        digitalWrite(I2, HIGH);
    } else {
        // Set motor to move forward
        digitalWrite(I1, HIGH);
        digitalWrite(I2, LOW);
    }

    // Return the control signal for the right motor
    return u_R;
}

// Left wheel PI controller (mirrors the right wheel PI controller)
short left_pi_controller(double estimated_vl, double vd, double wd) {
    // Calculate the desired velocity for the left wheel based on steering rate
    left_desired = vd - (wd * ELL / 2);
    // Compute the error between estimated and desired velocities
    error = left_desired - estimated_vl;
    // Calculate the proportional term
    proportional = kp * error;
    // Calculate the integral term, integrating over time
    integral_L += ki * error * ((millis() - controller_clock) / 1000.0);

    // Reset the integral term if it becomes NaN due to some error
    if (isnan(integral_L)) {
        integral_L = 0;
    }

    // Calculate the control signal for the left wheel
    u_L = static_cast<short>(proportional + integral_L);

    // Saturate the control signal to the maximum PWM value
    if (u_L > 255) {
        integral_L = 0; // Reset the integral term to prevent windup
        u_L = 255; // Cap the control signal
    } else if (u_L < -255) {
        integral_L = 0; // Reset the integral term to prevent windup
        u_L = -255; // Cap the control signal at the negative limit
    }

    // Set the motor direction based on the sign of the control signal
    if (u_L < 0) {
        u_L = abs(u_L); // Take the absolute value for PWM
        // Set motor to move backward
        digitalWrite(I3, HIGH);
        digitalWrite(I4, LOW);
    } else {
        // Set motor to move forward
        digitalWrite(I3, LOW);
        digitalWrite(I4, HIGH);
    }

    // Return the control signal for the left motor
    return u_L;
}

// Function to convert the raw sensor reading to distance in centimeters
float sharp_distance(int integer_val) {
    // Use the sensor-specific conversion formula
    float distance = 85.48 * exp(-0.004 * integer_val);
    return distance;
}

// Function for controlled slow driving based on desired distances
void slow_drive(float l_distance_des, float r_distance_des) {
    // Reset the cumulative distances
    l_distance = 0;
    r_distance = 0;
    // Set the slow speeds for precise maneuvers
    l_slow_speed = l_distance_des < 0 ? -0.3 : 0.3;
    r_slow_speed = r_distance_des < 0 ? -0.3 : 0.3;

    // Continue driving until the desired distances are reached
    while (fabs(l_distance) < fabs(l_distance_des) || fabs(r_distance) < fabs(r_distance_des)) {
        // Update the vehicle state for the current wheel velocities
        compute_vehicle_state();

        // Drive the left wheel
        if (fabs(l_distance) < fabs(l_distance_des)) {
            // Continue driving at slow speed
            analogWrite(EB, left_pi_controller(current.v_left, l_slow_speed, desired_steering_rate));
        } else {
            // Stop the left wheel
            analogWrite(EB, left_pi_controller(current.v_left, 0, desired_steering_rate));
        }

        // Drive the right wheel
        if (fabs(r_distance) < fabs(r_distance_des)) {
            // Continue driving at slow speed
            analogWrite(EA, right_pi_controller(current.v_right, r_slow_speed, desired_steering_rate));
        } else {
            // Stop the right wheel
            analogWrite(EA, right_pi_controller(current.v_right, 0, desired_steering_rate));
        }

        // Update the controller clock for the next PID cycle
        controller_clock = millis();
        // Introduce a short delay to allow other processes
        delay(50);
    }

    // Reset the integral terms of the PID controller
    integral_L = 0;
    integral_R = 0;
}

// Function to stop the robot for a specified duration
void stop(float stop_length) {
    // Record the time at which the stop was initiated
    stop_clock = millis();
    // Set the desired velocities to zero for stopping
    desired_velocity = 0;
    desired_steering_rate = 0;

    // Continue stopping until the specified duration has passed
    while (millis() < stop_clock + stop_length) {
        // Update the vehicle state for the current wheel velocities
        compute_vehicle_state();
        // Apply the PID control to maintain zero velocity
        analogWrite(EB, left_pi_controller(current.v_left, desired_velocity, desired_steering_rate));
        analogWrite(EA, right_pi_controller(current.v_right, desired_velocity, desired_steering_rate));
        // Update the controller clock for the next PID cycle
        controller_clock = millis();
        // Introduce a short delay to allow other processes
        delay(50);
    }

    // Reset the integral terms of the PID controller
    integral_L = 0;
    integral_R = 0;
}

// Function to correct the orientation of the robot if it has stopped due to an obstacle
void correct_self() {
    // Reset the timestamps for when the wheels last had zero velocity
    last_left_zero = 0;
    last_right_zero = 0;
    // Stop the robot briefly
    stop(150);
    // Drive backwards a short distance to disengage from the obstacle
    slow_drive(-0.2, -0.2);
    // Stop again briefly
    stop(150);
    // Determine the direction to turn based on the flag set
    if (turn_left) {
        // Turn left
        slow_drive(-0.3, 0.3);
    } else {
        // Turn right
        slow_drive(0.3, -0.3);
    }
    // Stop once more to complete the self-correction
    stop(150);
}

// Microcontroller setup function, called once when the sketch starts
void setup() {
    // Configure motor control and direction pins for output
    pinMode(EA, OUTPUT);
    pinMode(EB, OUTPUT);
    pinMode(I1, OUTPUT);
    pinMode(I2, OUTPUT);
    pinMode(I3, OUTPUT);
    pinMode(I4, OUTPUT);

    // Configure encoder signal pins for input
    pinMode(L_SIGNAL_A, INPUT);
    pinMode(L_SIGNAL_B, INPUT);
    pinMode(R_SIGNAL_A, INPUT);
    pinMode(R_SIGNAL_B, INPUT);

    // Initialize timekeeping variables
    tick_clock = millis();
    controller_clock = millis();

    // Attach interrupt routines to the encoder signal pins
    attachInterrupt(digitalPinToInterrupt(L_SIGNAL_A), L_decodeEncoderTicks, RISING);
    attachInterrupt(digitalPinToInterrupt(R_SIGNAL_A), R_decodeEncoderTicks, RISING);

    // Initialize the serial communication
    Serial.begin(115200);
}

// Main loop function, called repeatedly
void loop() {
    // Read the Sharp sensor values and convert to distances
    sharp_range_F = sharp_distance(analogRead(SHARP_PIN_F));
    sharp_range_L = sharp_distance(analogRead(SHARP_PIN_L));
    sharp_range_R = sharp_distance(analogRead(SHARP_PIN_R));

    // Check the front sensor for obstacles
    if (sharp_range_F < 35) {
        // If an obstacle is detected, print a message and adjust velocity
        Serial.println("Obstacle");
        // Decrease velocity based on the distance to the obstacle
        desired_velocity = default_speed - ((35 - sharp_range_F) * (0.04));
        desired_steering_rate = 0;

        // If the velocity is very low, set it to zero to stop the robot
        if (desired_velocity < 0 && desired_velocity > -0.4) {
            desired_velocity = 0;
        }

        // If the robot is stopped and should not be moving, attempt self-correction
        if (current.v_left == 0 && current.v_right == 0 && desired_velocity == 0) {
            Serial.println("SHARP FIX");
            correct_self();
        }
    } else {
        // If no obstacle is detected in front, check the side sensors
        if (sharp_range_L < 20 && sharp_range_R > 20) {
            // If the left side is too close to an obstacle, veer right
            Serial.println("Veer Right");
            desired_velocity = 0.4;
            desired_steering_rate = -1.5;
        } else if (sharp_range_L > 20 && sharp_range_R < 20) {
            // If the right side is too close to an obstacle, veer left
            Serial.println("Veer Left");
            desired_velocity = 0.4;
            desired_steering_rate = 1.5;
        } else {
            // If no obstacles are close, proceed with the standard velocity
            Serial.println("Standard");
            desired_velocity = default_speed;
            desired_steering_rate = 0;
        }

        // Check if the wheels have stopped moving and correct if necessary
        if (current.v_left == 0.0) {
            if (last_left_zero == 0) {
                last_left_zero = millis();
            } else if (millis() - last_left_zero > 1000) {
                turn_left = false;
                Serial.println("WHEEL FIX L");
                correct_self();
            }
        }

        if (current.v_right == 0.0) {
            if (last_right_zero == 0) {
                last_right_zero = millis();
            } else if (millis() - last_right_zero > 2000) {
                turn_left = true;
                Serial.println("WHEEL FIX R");
                correct_self();
            }
        }
    }

    // Update the vehicle state with current wheel velocities
    compute_vehicle_state();
    // Apply the PID controllers to drive the motors
    analogWrite(EB, left_pi_controller(current.v_left, desired_velocity, desired_steering_rate));
    analogWrite(EA, right_pi_controller(current.v_right, desired_velocity, desired_steering_rate));
    // Update the controller clock for the next PID cycle
    controller_clock = millis();
    // Introduce a short delay to allow other processes
    delay(50);
}
