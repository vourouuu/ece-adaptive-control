#define MOTOR_FWD     4 // Motor Forward pin
#define MOTOR_REV     5 // Motor Reverse pin
#define ENCODER_PIN1  2 // Encoder Output 'A' must connected with intreput pin of arduino.
#define ENCODER_PIN2  3 // Encoder Output 'B' must connected with intreput pin of arduino.
#define PPR           7 // Encoder Pulses per Revolution (from datasheet)
#define GEAR_RATIO  300 // Gear Ratio from experiment (with load R = 5kΩ)

volatile int lastEncoded = 0;
volatile long encoderValue = 0;
volatile int pulseCount = 0;

unsigned long lastTime = 0;
unsigned long currentTime = 0;
float Ts = 100;
float dt = 0;

int PWM_in = 0;
int motorSpeed = 0;
float RPM = 0;

int ref = 0; // Reference Signal in RPM
float Kp = 4;
float Ki = 4;
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

    Serial.print("ref = ");
    Serial.println(ref);

    currentTime = millis();
    dt = currentTime - lastTime;

    // RPM calculation any Ts millisecond
    if (dt >= Ts) {
        error = (ref - RPM)/2; // normalized
        error_integral += error * (dt / 1000);
        PWM_in = 200 + Kp * error + Ki * error_integral;

        // Helper signals
        Serial.print("lower:");
        Serial.println(0.1);
        Serial.print("upper:");
        Serial.println(28);

        // Other plots
        Serial.print("error =");
        Serial.println(error);
        Serial.print("PWM =");
        Serial.println(PWM_in);

        if (PWM_in >= 0 && PWM_in <= 255) {
            motorSpeed = PWM_in; 
            analogWrite(MOTOR_FWD, motorSpeed);       
        }
        else if (PWM_in < 0 && PWM_in >= -255) {
            motorSpeed = -PWM_in; 
            analogWrite(MOTOR_REV, motorSpeed);   
        }

        detachInterrupt(digitalPinToInterrupt(ENCODER_PIN1));

        """
            +----------------+
            |RPM calculaition|
            +----------------+

            RPM = (pulses / revolutions per cycle) * (60 sec./min.)

            Example: the encoder has 9 pulses/rev. --> PPR = 9
        """

        RPM = (pulseCount / ((float)PPR * GEAR_RATIO)) * 60.0 / (Ts/1000);
        Serial.print("RPM = ");
        Serial.println(RPM);

        pulseCount = 0;
        lastTime = currentTime;

        attachInterrupt(digitalPinToInterrupt(ENCODER_PIN1), countPulse, RISING);
    }
}

// ISR: count pulses via encoder
void countPulse() {
    pulseCount++;
}
