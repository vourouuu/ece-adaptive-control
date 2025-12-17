#define MOTOR_FWD     4
#define MOTOR_REV     5
#define ENCODER_PIN1  2
#define ENCODER_PIN2  3
#define PPR           7
#define GEAR_RATIO  300

volatile int lastEncoded = 0;
volatile int pulseCount = 0;
volatile long encoderValue = 0;

unsigned long lastTime = 0;
unsigned long currentTime = 0;
float t = 0;
float dt = 0;
float Ts = 100;

int motorSpeed = 0;
int u = 0;
float RPM = 0;

float ref = 0;
float r = 0;

float error = 0;
float x = 0;
float xm = 0, xm_dot = 0;
float kx = 0, kx_dot = 0;
float kr = 0, kr_dot = 0;

float am = 0.9, bm = 0.9;
float gamma_x = 0.001, gamma_r = 0.01;
float sigma_x = 0.05, sigma_r = 0.05;

// Reference sequence
int refs[]   = {18, 18, 18};       // r1, r2, r3 (reference signals)
int delays[] = {8000, 6000, 6000}; // delay1, delay2, delay3 (ms)

int numSteps = sizeof(refs) / sizeof(refs[0]);
int currentStep = 0;
unsigned long stepStartTime = 0;

void setup() {
    Serial.begin(9600);

    pinMode(MOTOR_FWD, OUTPUT); 
    pinMode(MOTOR_REV, OUTPUT);
    pinMode(ENCODER_PIN1, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN1), countPulse, RISING);

    lastTime = millis();
    // stepStartTime = millis();
}

void loop() {
    currentTime = millis();
    dt = currentTime - lastTime;

    // For step function
    if (currentTime - stepStartTime >= delays[currentStep]) {
        currentStep++;
        if (currentStep >= numSteps) {
            currentStep = 0;
        }

        ref = refs[currentStep];
        stepStartTime = currentTime;
    }

    // For sinusoidal function
    // t = currentTime * 0.001; // (sec)
    // ref = 16 + 3 * sin(0.6 * t);

    if (dt >= Ts) {
        dt = dt/1000;
        
        // Reference signal
        r = ref;

        // System measurement
        x = RPM;

        // Model
        error = x - xm;
        xm_dot = -am * xm + bm * r;
        xm += xm_dot * dt;

        // 1. Adaptive laws MRAC
        kx_dot = -gamma_x * x * error * (+1) ;
        kr_dot = -gamma_r * r * error * (+1);

        // 2. Adaptive laws MRAC with sigma modification
        // kx_dot = -gamma_x * (x * error + sigma_x * kx);
        // kr_dot = -gamma_r * (r * error + sigma_r * kr);

        // 3. Adaptive laws MRAC with epsilon modification
        // kx_dot = -gamma_x * (x * error + sigma_x * abs(error) * kx);
        // kr_dot = -gamma_r * (r * error + sigma_r * abs(error) * kr);

        // 4. Dead zone for MRAC
        // if(abs(error) < 2) {
        //     kx_dot = 0;
        //     kr_dot = 0;
        // }
        // else {
        //     kx_dot = -gamma_x * x * error * (+1) ;
        //     kr_dot = -gamma_r * r * error * (+1);
        // }

        kx += kx_dot * dt;
        kr += kr_dot * dt;

        // Control
        u = kx * x + kr * r;

        if (u >= 0 && u <= 255) {
            motorSpeed = u; 
            analogWrite(MOTOR_FWD, motorSpeed);
            analogWrite(MOTOR_REV, 0);
        }
        else if (u < 0 && u >= -255) {
            motorSpeed = -u; 
            analogWrite(MOTOR_REV, motorSpeed);
            analogWrite(MOTOR_FWD, 0);
        }

        detachInterrupt(digitalPinToInterrupt(ENCODER_PIN1));

        // RPM calculation
        RPM = (pulseCount / ((float)PPR * GEAR_RATIO)) * 60.0 / (Ts/1000);

        pulseCount = 0;
        lastTime = currentTime;

        attachInterrupt(digitalPinToInterrupt(ENCODER_PIN1), countPulse, RISING);

        // Plots
        Serial.print(x);
        Serial.print(",");
        Serial.print(xm);
        Serial.print(",");
        Serial.print(kr);
        Serial.print(",");
        Serial.println(kx);
    }
}

void countPulse() {
    pulseCount++;
}