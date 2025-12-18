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
float a_hat = 2, a_hat_dot = 0;
float b_hat = 0.1, b_hat_dot = 0;

float am = 0.9, bm = 0.9;
float gamma_a = 0.01, gamma_b = 0.000001;
float sigma_a = 0.1, sigma_b = 0.01;

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
    // if (currentTime - stepStartTime >= delays[currentStep]) {
    //     currentStep++;
    //     if (currentStep >= numSteps) {
    //         currentStep = 0;
    //     }

    //     ref = refs[currentStep];
    //     stepStartTime = currentTime;
    // }

    // For sinusoidal function
    t = currentTime * 0.001; // (sec)
    ref = 16 + 4 * sin(0.5 * t);

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

        // 1. Adaptive laws ADI
        a_hat_dot = gamma_a * x * error;
        b_hat_dot = gamma_b * u * error;

        // 2. Adaptive laws ADI with sigma modification
        // a_hat_dot = gamma_a * (x * error - sigma_a * a_hat);
        // b_hat_dot = gamma_b * (u * error - sigma_b * b_hat);

        // 3. Adaptive laws ADI with epsilon modification
        // a_hat_dot = gamma_a * (x * error - sigma_a * abs(error) * a_hat);
        // b_hat_dot = gamma_b * (u * error - sigma_b * abs(error) * b_hat);

        // 4. Dead zone for ADI
        // if(abs(error) < 1.5) {
        //     a_hat_dot = 0;
        //     b_hat_dot = 0;
        // }
        // else {
        //     a_hat_dot = gamma_a * x * error;
        //     b_hat_dot = gamma_b * u * error;
        // }

        // 5. Projection Operator
        // th_max = [[20.0, 35.0], [0.0, 0.5]];
        // eps = 
        // f = 



        a_hat += a_hat_dot * dt;
        b_hat += b_hat_dot * dt;

        // Control
        u = 1/b_hat * ((am - a_hat) * x + bm * r);

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
        Serial.print(a_hat);
        Serial.print(",");
        Serial.println(b_hat);
    }
}

void countPulse() {
    pulseCount++;
}