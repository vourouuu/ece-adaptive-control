#include <math.h>
#define MOTOR_FWD     4
#define MOTOR_REV     5
#define ENCODER_PIN1  2
#define ENCODER_PIN2  3
#define PPR           7
#define GEAR_RATIO  300

volatile int pulseCount = 0;
volatile long encoderValue = 0;

unsigned long lastTime = 0;
unsigned long currentTime = 0;
float t = 0;
float dt = 0;
float Ts = 100;

// State vector -> x = [θ ω]
float theta = 0;
float omega = 0;

// Reference model: x_dot = A_m * x + B_m * r
// x_m = [θ_m ω_m]
float theta_m = 0, omega_m = 0;
float theta_m_dot = 0, omega_m_dot = 0;

// A_m
float am11 = 0,    am12 = 0.5; 
float am21 = -2.0, am22 = -1.0;
float bm2 = 2.0;

// Lyapunov P -> P * A_m+ A_m^T * P = -Q
float p11 = 3.0;
float p12 = 0.25;
float p22 = 0.625;

// Adaptive gains -> Kx, Kr
float kx_theta = 0, kx_omega = 0;
float kr = 0;

float kx_theta_dot = 0, kx_omega_dot = 0;
float kr_dot = 0;

// Learning rates
float gamma_theta = 0.001;
float gamma_omega = 0.01;
float gamma_r = 0.05;

// σ-modification
float sigma_theta = 0.01;
float sigma_omega = 0.01;
float sigma_r = 0.1;

// Errors
float e_theta = 0;
float e_omega = 0;

// Reference Input
float ref_theta = 90.0;
float r = 0;

int sgn_Lambda; // sign of Λ
float s;        // e^T * P * B

void setup() {
    Serial.begin(9600);
    pinMode(MOTOR_FWD, OUTPUT); 
    pinMode(MOTOR_REV, OUTPUT);
    pinMode(ENCODER_PIN1, INPUT_PULLUP);
    pinMode(ENCODER_PIN2, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN1), countEncoder, RISING);
    lastTime = millis();
}

void loop() {
    currentTime = millis();
    dt = currentTime - lastTime;

    if (dt >= Ts) {
        dt = dt / 1000.0;
        
        // 1. Measurment of θ and ω
        theta = (encoderValue / ((float)PPR * GEAR_RATIO)) * 360.0;
        omega = (pulseCount / ((float)PPR * GEAR_RATIO)) * 60.0 / dt_sec;
        
        pulseCount = 0;

        // 2.1 Reference input: constant
        // r = ref_theta; 

        // 2.2 Reference input: sinusoidal
        // float t = millis() * 0.001;
        // r = ref_theta + 90 * sin(0.5 * t); 

        // 2.3 Reference input: step
        t = millis();
        r = 90 + 90 * (t > 8500) - 60 * (t > 17000); 

        // 3. Errors
        e_theta = theta - theta_m;
        e_omega = omega - omega_m;

        // 4. Reference model
        theta_m_dot = am11 * theta_m + am12 * omega_m;
        omega_m_dot = am21 * theta_m + am22 * omega_m + bm2 * r;
        
        theta_m += theta_m_dot * dt_sec;
        omega_m += omega_m_dot * dt_sec;

        // 5. Adaptive laws
        s = (e_theta*p12 + e_omega*p22);
        sgn_Lambda = -1;
        kx_theta_dot = -gamma_theta * theta * s * sgn_Lambda;
        kx_omega_dot = -gamma_omega * omega * s * sgn_Lambda;
        kr_dot = -gamma_r * r * s * sgn_Lambda;

        // 6. Adaptive laws MRAC with σ-modification
        kx_theta_dot = -gamma_theta * (theta * s + sigma_theta * kx_theta) * sgn_Lambda;
        kx_omega_dot = -gamma_omega * (omega * s + sigma_omega * kx_omega) * sgn_Lambda;
        kr_dot = -gamma_r * (r * s + sigma_r * kr) * sgn_Lambda;
        
        // Euler integration
        kx_theta += kx_theta_dot * dt_sec;
        kx_omega += kx_omega_dot * dt_sec;
        kr += kr_dot * dt_sec;

        // 7. Control law
        float u = kx_theta * theta + kx_omega * omega + kr * r;

        // 9. PWM
        driveMotor(u);

        Serial.print(theta);
        Serial.print(",");
        Serial.print(theta_m);
        Serial.print(",");
        Serial.print(omega);
        Serial.print(",");
        Serial.print(omega_m);
        Serial.print(",");
        Serial.print(kx_theta);
        Serial.print(",");
        Serial.print(kx_omega);
        Serial.print(",");
        Serial.println(kr);

        lastTime = currentTime;
    }
}

void driveMotor(float controlSignal) {
    int pwm = constrain(abs(controlSignal), 0, 255);
    if (controlSignal >= 0) {
        analogWrite(MOTOR_FWD, pwm);
        analogWrite(MOTOR_REV, 0);
    } else {
        analogWrite(MOTOR_FWD, 0);
        analogWrite(MOTOR_REV, pwm);
    }
}

void countEncoder() {
    if(digitalRead(ENCODER_PIN2) == LOW) {
        encoderValue++;
        pulseCount++;
    } else {
        encoderValue--;
        pulseCount--;
    }
}
