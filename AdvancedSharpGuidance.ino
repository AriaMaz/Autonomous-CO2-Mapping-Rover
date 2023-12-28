int EA = 3; // Wheel PWM pin (must be a PWM pin)
int EB = 5; // Wheel PWM pin (must be a PWM pin)
int I1 = 2; // Wheel direction digital pin 2
int I2 = 4; // Wheel direction digital pin 4
int I3 = 7; // Wheel direction digital pin 7
int I4 = 8; // Wheel direction digital pin 8

const byte L_SIGNAL_A = 12; // Left wheel encoder digital pins
const byte L_SIGNAL_B = 13;
const byte R_SIGNAL_A = 10; // Right wheel encoder digital pins
const byte R_SIGNAL_B = 11;

const byte SHARP_PIN_F = A4;
const byte SHARP_PIN_L = A5;
const byte SHARP_PIN_R = A3;

// Encoder ticks per (motor) revolution (TPR)
const int TPR = 3000;

// robot body width
const double ELL = 0.2775;

// Wheel radius [m]
const double RHO = 0.0625;

// Counter to keep track of encoder ticks [integer]
volatile long L_encoder_ticks = 0;
volatile long R_encoder_ticks = 0;
double l_distance;
double r_distance;

// Variables to store the proximity measurement
float sharp_range_F; // range measurement [cm] (F)
float sharp_range_L; // range measurement [cm] (L)
float sharp_range_R; // range measurement [cm] (R)

// Current velocity and steering rate
typedef struct State
{
    double v_left, v_right;
} state;

state current;

// Set desired velocity and steering rate
double desired_velocity = 0;
double desired_steering_rate = 0;
double default_speed = 0.6;

// Creating variables for navigation and controll
int kp = 300, ki = 250;
double omega_L, omega_R, v_L, v_R, right_desired, left_desired, error, proportional, integral_L = 0, integral_R = 0, l_slow_speed, r_slow_speed;
short u_L = 0, u_R = 0;

// Create variables to track time intervals
unsigned long tick_clock;
unsigned long controller_clock;
unsigned long stop_clock;

// Remember current velocities
unsigned long last_left_zero = 0, last_right_zero = 0;
boolean turn_left = false;

void L_decodeEncoderTicks()
{
    if (digitalRead(L_SIGNAL_B) == LOW)
    {
        // SIGNAL_A leads SIGNAL_B, so count one way
        L_encoder_ticks--;
    }
    else
    {
        // SIGNAL_B leads SIGNAL_A, so count the other way
        L_encoder_ticks++;
    }
}

void R_decodeEncoderTicks()
{
    // Right wheel encoder ticks
    if (digitalRead(R_SIGNAL_B) == LOW)
    {
        R_encoder_ticks++;
    }
    else
    {
        R_encoder_ticks--;
    }
}

// compute vehicle speed [m/s]
void compute_vehicle_state()
{
    // Estimate the rotational speed [rad/s]
    omega_L = 2.0 * PI * ((double)L_encoder_ticks / (double)TPR) * 1000.0 / (double)((millis()) - tick_clock);
    omega_R = 2.0 * PI * ((double)R_encoder_ticks / (double)TPR) * 1000.0 / (double)((millis()) - tick_clock);

    // Reset the encoder ticks counter and tick_clock

    l_distance += (L_encoder_ticks / 3000.0) * (2.0 * PI * RHO);
    r_distance += (R_encoder_ticks / 3000.0) * (2.0 * PI * RHO);

    L_encoder_ticks = 0;
    R_encoder_ticks = 0;
    tick_clock = millis();

    // estimate translational velocity [m/s]
    v_L = RHO * omega_L;
    v_R = RHO * omega_R;

    // Place in struct
    current.v_left = v_L;
    current.v_right = v_R;
}

short right_pi_controller(double estimated_vr, double vd, double wd)
{
    right_desired = vd + (wd * ELL / 2);
    error = right_desired - estimated_vr;
    proportional = kp * error;
    integral_R += ki * error * ((millis() - controller_clock) / 1000.0);

    if (isnan(integral_R))
    {
        integral_R = 0;
    }

    u_R = (short)(proportional + integral_R);

    if (u_R > 255)
    {
        integral_R = 0;
        u_R = 255;
    }
    if (u_R < -255)
    {
        integral_R = 0;
        u_R = -255;
    }
    if (u_R < 0)
    {
        u_R = abs(u_R);
        digitalWrite(I1, LOW);
        digitalWrite(I2, HIGH);
    }
    else
    {
        digitalWrite(I1, HIGH);
        digitalWrite(I2, LOW);
    }

    return u_R;
};

short left_pi_controller(double estimated_vl, double vd, double wd)
{
    left_desired = vd - (wd * ELL / 2);
    error = left_desired - estimated_vl;
    proportional = kp * error;
    integral_L += ki * error * ((millis() - controller_clock) / 1000.0);

    if (isnan(integral_L))
    {
        integral_L = 0;
    }

    u_L = (short)(proportional + integral_L);

    if (u_L > 255)
    {
        integral_L = 0;
        u_L = 255;
    }
    if (u_L < -255)
    {
        integral_L = 0;
        u_L = -255;
    }
    if (u_L < 0)
    {
        u_L = abs(u_L);
        digitalWrite(I3, HIGH);
        digitalWrite(I4, LOW);
    }
    else
    {
        digitalWrite(I3, LOW);
        digitalWrite(I4, HIGH);
    }

    return u_L;
};

float sharp_distance(int integer_val)
{
    float distance;
    distance = 85.48 * exp(-0.004 * integer_val);
    return distance;
}

void slow_drive(float l_distance_des, float r_distance_des)
{
    l_distance = 0;
    r_distance = 0;
    l_slow_speed = 0.3;
    r_slow_speed = 0.3;
    if (l_distance_des < 0)
    {
        l_slow_speed = -0.3;
    };
    if (r_distance_des < 0)
    {
        r_slow_speed = -0.3;
    };

    while (fabs(l_distance) < fabs(l_distance_des) || fabs(r_distance) < fabs(r_distance_des))
    {
        compute_vehicle_state();

        if (fabs(l_distance) < fabs(l_distance_des))
        {
            analogWrite(EB, left_pi_controller(current.v_left, l_slow_speed, desired_steering_rate));
        }
        else
        {
            analogWrite(EB, left_pi_controller(current.v_left, 0, desired_steering_rate));
        }
        if (fabs(r_distance) < (fabs(r_distance_des)))
        {
            analogWrite(EA, right_pi_controller(current.v_right, r_slow_speed, desired_steering_rate));
        }
        else
        {
            analogWrite(EA, right_pi_controller(current.v_right, 0, desired_steering_rate));
        }
        controller_clock = millis();
        delay(50);
    }
    integral_L = 0;
    integral_R = 0;
}

void stop(float stop_length)
{
    stop_clock = millis();
    desired_velocity = 0;
    desired_steering_rate = 0;
    while (stop_clock + stop_length > millis()) // Stop
    {
        compute_vehicle_state();
        analogWrite(EB, left_pi_controller(current.v_left, desired_velocity, desired_steering_rate));
        analogWrite(EA, right_pi_controller(current.v_right, desired_velocity, desired_steering_rate));
        controller_clock = millis();
        delay(50);
    };
    integral_L = 0;
    integral_R = 0;
}
void correct_self()
{
    last_left_zero = 0;
    last_right_zero = 0;
    stop(150);
    slow_drive(-0.2, -0.2);
    stop(150);
    if (turn_left)
    {
        slow_drive(-0.3, 0.3);
    }
    else
    {
        slow_drive(0.3, -0.3);
    }

    stop(150);
}

void setup()
{
    // Configure digital pins for output
    pinMode(EA, OUTPUT);
    pinMode(EB, OUTPUT);
    pinMode(I1, OUTPUT);
    pinMode(I2, OUTPUT);
    pinMode(I3, OUTPUT);
    pinMode(I4, OUTPUT);

    // Set the pin modes for the encoders
    pinMode(L_SIGNAL_A, INPUT);
    pinMode(L_SIGNAL_B, INPUT);
    pinMode(R_SIGNAL_A, INPUT);
    pinMode(R_SIGNAL_B, INPUT);

    // Start counting time just after tick counting begins
    tick_clock = millis();
    controller_clock = millis();

    // Every time the pin goes high, this is a pulse
    attachInterrupt(pdigitalPinToInterrupt(L_SIGNAL_A), L_decodeEncoderTicks, RISING);
    attachInterrupt(pdigitalPinToInterrupt(R_SIGNAL_A), R_decodeEncoderTicks, RISING);

    Serial.begin(115200);
}

void loop()
{
    sharp_range_F = sharp_distance(analogRead(SHARP_PIN_F));
    sharp_range_L = sharp_distance(analogRead(SHARP_PIN_L));
    sharp_range_R = sharp_distance(analogRead(SHARP_PIN_R));

    if (sharp_range_F < 35)
    {
        Serial.println("Obstacle");
        desired_velocity = default_speed - ((35 - sharp_range_F) * (0.04));
        desired_steering_rate = 0;

        if (desired_velocity < 0 && desired_velocity > -0.4)
        {
            desired_velocity = 0;
        }

        if (current.v_left == 0 && current.v_right == 0 && desired_velocity == 0)
        {
            Serial.println("SHARP FIX");
            correct_self();
        };
    }

    else
    {

        if (sharp_range_L < 20 && sharp_range_R > 20)
        {
            Serial.println("Veer Right");
            desired_velocity = 0.4;
            desired_steering_rate = -1.5;
        }
        else if (sharp_range_L > 20 && sharp_range_R < 20)
        {
            Serial.println("Veer Left");
            desired_velocity = 0.4;
            desired_steering_rate = 1.5;
        }
        else
        {
            Serial.println("Standard");
            desired_velocity = default_speed;
            desired_steering_rate = 0;
        }

        if (current.v_left == 0.0)
        {
            if (last_left_zero == 0)
            {
                last_left_zero = millis();
            }
            else if (last_left_zero + 1000 < millis() || last_left_zero + 1400 < millis())
            {
                turn_left = false;
                Serial.println("WHEEL FIX L");
                correct_self();
            }
        }

        if (current.v_right == 0.0)
        {

            if (last_right_zero == 0)
            {
                last_right_zero = millis();
            }
            else if (last_right_zero + 2000 < millis() || last_right_zero + 2400 < millis())
            {
                turn_left = true;
                Serial.println("WHEEL FIX R");
                correct_self();
            }
        }
    }
    compute_vehicle_state();
    analogWrite(EB, left_pi_controller(current.v_left, desired_velocity, desired_steering_rate));
    analogWrite(EA, right_pi_controller(current.v_right, desired_velocity, desired_steering_rate));
    controller_clock = millis();
    delay(50);
}