#define MOTOR_FWD     5 // Motor Forward pin
#define MOTOR_REV     6 // Motor Reverse pin
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
int PWM_in = 0;
float RPM = 0;

int ref = 0; // Reference Signal in RPM
float Kp = 3;
float Ki = 300;
float error = 0;
float error_integral = 0;

void setup() {
    Serial.begin(9600);

    pinMode(MOTOR_FWD, OUTPUT); 
    pinMode(MOTOR_REV, OUTPUT);

    pinMode(ENCODER_PIN1, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN1), countPulse, RISING);
    
    lastTime = millis();
}

void loop() {
    // If there are new data of serial monitor
    if (Serial.available() > 1 || Serial.available() < -1) {
        ref = Serial.parseInt();
        error_integral = 0;
    }

    currentTime = millis();
    dt = currentTime - lastTime;

    if (dt >= Ts) {
        error = (ref - RPM); // normalized error
        error_integral += error * (dt / 1000);

        // PWM calculation
        PWM_in = 200 + Kp * error + Ki * error_integral;
        
        if (PWM_in >= 0 && PWM_in <= 255) {
            motorSpeed = PWM_in; 
            analogWrite(MOTOR_FWD, motorSpeed); 
        }
        else if (PWM_in < 0 && PWM_in >= -255) {
            motorSpeed = -PWM_in; 
            analogWrite(MOTOR_REV, motorSpeed);
        }

        // RPM calculation
        detachInterrupt(digitalPinToInterrupt(ENCODER_PIN1));

        RPM = (pulseCount / ((float)PPR * GEAR_RATIO)) * 60.0 / (Ts/1000);

        pulseCount = 0;
        lastTime = currentTime;

        attachInterrupt(digitalPinToInterrupt(ENCODER_PIN1), countPulse, RISING);

        // Helper signals
        Serial.print("u:");
        Serial.println(28);
        Serial.print("l:");
        Serial.println(0);

        // Plots
        Serial.print("ref:");
        Serial.println(ref);

        Serial.print("RPM:");
        Serial.println(RPM);

        Serial.print("PWM:");
        Serial.println(PWM_in);
        
        Serial.print("error:");
        Serial.println(error);
    }
}

// ISR: count pulses via encoder
void countPulse() {
    pulseCount++;
}
