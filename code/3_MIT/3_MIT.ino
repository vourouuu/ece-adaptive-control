
#define MOTOR_FWD     5 // Motor Forward pin
#define MOTOR_REV     4 // Motor Reverse pin
#define ENCODER_PIN1  2 // Encoder Output 'A' must connected with intreput pin of arduino.
#define ENCODER_PIN2  3 // Encoder Output 'B' must connected with intreput pin of arduino.
#define PPR           7 // Encoder Pulses per Revolution (from datasheet)
#define GEAR_RATIO  300 // Gear Ratio from experiment (with load R = 5kΩ)

volatile int lastEncoded = 0;
volatile int pulseCount = 0;
volatile long encoderValue = 0;

unsigned long lastTime = 0;
unsigned long currentTime = 0;
float Ts = 100;
float dt = 0;

int motorSpeed = 0;
int u = 0;
float RPM = 20;

int ref = 0;
int r = 0;

float e0 = 0;
float yp = 0;
float ym = 0, ym_dot = 0;
float r_filtered = 0, r_filtered_dot = 0;
float yp_filtered = 0, yp_filtered_dot = 0;
float theta1 = 0, theta2 = 0, theta1_dot = 0, theta2_dot = 0;

float am = 1, bm = 1;
float gamma = 1;

// Reference sequence
int refs[]   = {20, 20, 20};       // r1, r2, r3 (reference signals)
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
    t = currentTime * 0.001;  // (sec)
    ref = 16 + 5 * sin(3 * t);

    if (dt >= Ts) {
        dt = dt/1000;
        r = ref;

        // Plant: dyp/dt = -a * yp + b* u ---> implemented inside the real system
        yp = RPM;

        // Error: e0 = yp - ym
        e0 = yp - ym;

        /*
            +--------+
            |MIT Rule|
            +--------+

            dθ1/dt = -γ * ( am / (s+am) *  r) * e0
            dθ2/dt =  γ * ( am / (s+am) * yp) * e0

            ---> r_filtered_dot  = ( am / (s+am) *  r)
            ---> yp_filtered_dot = ( am / (s+am) * yp)
        */
        
        r_filtered_dot  = -am *  r_filtered + am * r;
        yp_filtered_dot = -am * yp_filtered + am * yp;
        
        // Euler integration
        r_filtered  = r_filtered  +  r_filtered_dot * dt;
        yp_filtered = yp_filtered + yp_filtered_dot * dt;

        // Model: dym/dt = -am * ym + bm * r
        ym_dot = -am * ym + bm * r;
        ym = ym + ym_dot * dt;

        denom = 0.001 + r_filtered * r_filtered + yp_filtered * yp_filtered;
        
        // MIT
        theta1_dot = -gamma *  (r_filtered/denom) * e0;
        theta2_dot =  gamma * (yp_filtered/denom) * e0;

        // Euler integration
        theta1 = theta1 + theta1_dot * dt;
        theta2 = theta2 + theta2_dot * dt;

        // Controller
        u = theta1 * r - theta2 * yp;

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

        // RPM calculaition
        RPM = (pulseCount / ((float)PPR * GEAR_RATIO)) * 60.0 / (Ts/1000);

        pulseCount = 0;
        lastTime = currentTime;

        attachInterrupt(digitalPinToInterrupt(ENCODER_PIN1), countPulse, RISING);
        
        // Plots
        Serial.print(RPM);
        Serial.print(",");
        Serial.print(ym);
        Serial.print(",");
        Serial.print(theta1);
        Serial.print(",");
        Serial.println(theta2);
    }
}

// ISR: count pulses via encoder
void countPulse() {
    pulseCount++;
}





