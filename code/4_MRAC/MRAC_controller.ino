#define MOTOR_FWD     5
#define MOTOR_REV     6
#define ENCODER_PIN1  2
#define ENCODER_PIN2  3
#define PPR           7
#define GEAR_RATIO  300

volatile int pulseCount = 0;

unsigned long lastTime = 0;
unsigned long currentTime = 0;
float Ts = 100;
float t = 0;

int motorSpeed = 0;
int u = 0;
float RPM = 0;

int ref = 18; // Reference applied to controller
int r = 18;

float error = 0;
float x = 0;
float xm = 0, xm_dot = 0;

float kx_dot=0, kr_dot=0, kx=0, kr=0;
float gamma_x = 0.01, gamma_r = 0.005;
float sigma_x = 1, sigma_r = 1;

float am = 2, bm = 2;

// ------------------- NEW REFERENCE SEQUENCE -------------------
// Βάλε εδώ τις δικές σου τιμές
int refs[]   = {15, 18, 20};          // r1, r2, r3...
int delays[] = {8000, 6000, 6000};    // delay1, delay2, delay3 (ms)

int numSteps = sizeof(refs) / sizeof(refs[0]);
int currentStep = 0;
unsigned long stepStartTime = 0;
// --------------------------------------------------------------

void setup() {
    Serial.begin(9600);

    pinMode(MOTOR_FWD, OUTPUT); 
    pinMode(MOTOR_REV, OUTPUT);
    pinMode(ENCODER_PIN1, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN1), countPulse, RISING);

    lastTime = millis();
    stepStartTime = millis();   // ξεκινάει το πρώτο ref
}

void loop() {

    // ------------------- ΕΦΑΡΜΟΓΗ ΑΚΟΛΟΥΘΙΑΣ REF -------------------
    // for step
    unsigned long now = millis();
    if (now - stepStartTime >= delays[currentStep]) {
        currentStep++;
        if (currentStep >= numSteps) {
            currentStep = 0;  // loop sequence
        }

        ref = refs[currentStep];  // step
        stepStartTime = now;
    }

    // for sin
    // t = millis() * 0.001;  // seconds
    // ref = 16 + 3*sin(0.4*t);

    // --------------------------------------------------------------

    currentTime = millis();
    float dt = currentTime - lastTime;

    if (dt >= Ts) {
        dt = dt/1000.0;

        // reference applied
        r = ref;

        // System measurement
        x = RPM;

        // Model
        error = x - xm;
        xm_dot = -am * xm + bm * r;
        xm += xm_dot * dt;

        // Adaptive laws MRAC
        kx_dot = -gamma_x * x * error * (+1) ;
        kr_dot = -gamma_r * r * error * (+1);

        // Adaptive laws MRAC with sigma modification
        // kx_dot = -gamma_x * (x * error + sigma_x * kx);
        // kr_dot = -gamma_r * (r * error + sigma_r * kr);

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

        // ------------------- SERIAL OUTPUT για Python -------------------
        Serial.print(RPM);
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
