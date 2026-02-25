#define MOTOR_FWD     5
#define MOTOR_REV     4
#define ENCODER_PIN1  2
#define ENCODER_PIN2  3
#define PPR           7
#define GEAR_RATIO  300

volatile int lastEncoded = 0; // Here updated value of encoder store.
volatile long encoderValue = 0; // Raw encoder value
unsigned long lastTime = 0;
unsigned long currentTime = 0;
float u=0;
int motorSpeed = 0;
int ref = 20; // rpm
float Ts = 100;
float error = 0;
float error_integral = 0;

volatile int pulseCount = 0;
float RPM = 0;
float dt = 0;

float Kp = 10;
float Ki = 3;

// ------------------- NEW REFERENCE SEQUENCE -------------------
// Βάλε εδώ τις δικές σου τιμές
int refs[]   = {14, 14, 14};          // r1, r2, r3...
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

        // System measurement

        error = (ref - RPM);
        error_integral += error * dt;
        u = 200 + Kp * error + Ki * error_integral;

        if (u>255) u = 255;
        else if (u<-255) u = -255;

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
        Serial.println(ref);

    }
}

void countPulse() {
    pulseCount++;
}

