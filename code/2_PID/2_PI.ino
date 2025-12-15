#define MOTOR_FWD     5 // Motor Forward pin
#define MOTOR_REV     4 // Motor Reverse pin
#define ENCODER_PIN1  2 // Encoder Output 'A' must connected with intreput pin of arduino.
#define ENCODER_PIN2  3 // Encoder Output 'B' must connected with intreput pin of arduino.
#define PPR           7 // Encoder Pulses per Revolution (from datasheet)
#define GEAR_RATIO  300 // Gear Ratio from experiment (with load R = 5kΩ)

volatile int lastEncoded = 0;
volatile int pulseCount = 0;
volatile long encoderValue = 0;

unsigned long currentTime = 0;
unsigned long lastTime = 0;
unsigned long dt = 0;
float Ts = 100;

int motorSpeed = 0;
int u = 0;
float RPM = 0;

int ref = 0; // Reference Signal in RPM
float Kp = 3;
float Ki = 300;
float error = 0;
float error_integral = 0;

// Reference sequence
int refs[]   = {22, 22, 22};       // r1, r2, r3 (reference signals)
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
    stepStartTime = millis();
}

void loop() {
    // For step function
    currentTime = millis();
    dt = currentTime - lastTime;
    
    if (currentTime - stepStartTime >= delays[currentStep]) {
        currentStep++;
        if (currentStep >= numSteps) {
            currentStep = 0;
        }

        ref = refs[currentStep];
        stepStartTime = currentTime;
    }

    // For sinusoidal function
    // t = millis() * 0.001;  // (sec)
    // ref = 16 + 3*sin(0.4*t);

    if (dt >= Ts) {
        dt = dt / 1000;
        error = (ref - RPM);
        error_integral += error * dt;

        // PWM calculation
        u = 200 + Kp * error + Ki * error_integral;
        
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

        // RPM calculation
        detachInterrupt(digitalPinToInterrupt(ENCODER_PIN1));

        RPM = (pulseCount / ((float)PPR * GEAR_RATIO)) * 60.0 / (Ts/1000);

        pulseCount = 0;
        lastTime = currentTime;

        attachInterrupt(digitalPinToInterrupt(ENCODER_PIN1), countPulse, RISING);
        
        // Plots
        Serial.print(RPM);
        Serial.print(",");
        Serial.println(ref);
    }
}

// ISR: count pulses via encoder
void countPulse() {
    pulseCount++;
}
