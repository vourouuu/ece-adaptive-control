
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

int u = 0; // before: PWM_in, now: u
int motorSpeed = 0;
float RPM = 0;

int ref = 20; // Reference Signal in RPM
int r = 0;
float e0 = 0;

float yp = 0;
float ym = 0, ym_dot = 0;
float r_filtered = 0, r_filtered_dot = 0;
float yp_filtered = 0, yp_filtered_dot = 0;
float theta1 = 0, theta2 = 0, theta1_dot = 0, theta2_dot = 0;

float am = 5, bm = 5;
float gamma = 0.01;
int A = 7, omega = 1, t = 0; // Amplitude, ω (rad/sec), time (sec)

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
    }

    // Serial.print("ref = ");
    // Serial.println(ref);

    currentTime = millis();
    dt = currentTime - lastTime;

    // RPM calculation any Ts millisecond
    if (dt >= Ts) {
        dt = dt/1000;
        
        """
            Testing with different signals:
                1. step function
                2. sinusoidal function
            in RPM.
        """
        
        // 1. step function
        r = ref;
        // 2. sinusoidal function
        // t = millis() / 700.0;
        // r = ref + A * sin(omega * t);

        // Plant: dyp/dt = -a * yp + b* u ---> impemented inside the real system
        yp = RPM;

        // Error: e0 = yp - ym
        e0 = yp - ym;

        """
            +--------+
            |MIT Rule|
            +--------+

            dθ1/dt = -γ * ( am / (s+am) *  r) * e0
            dθ2/dt =  γ * ( am / (s+am) * yp) * e0

            ---> r_filtered_dot  = ( am / (s+am) *  r)
            ---> yp_filtered_dot = ( am / (s+am) * yp)
        """
        
        r_filtered_dot  = -am *  r_filtered + am * r;
        yp_filtered_dot = -am * yp_filtered + am * yp;
        
        // Euler integration
        r_filtered  = r_filtered  +  r_filtered_dot * dt;
        yp_filtered = yp_filtered + yp_filtered_dot * dt;

        // Model: dym/dt = -am * ym + bm * r
        ym_dot = -am * ym + bm * r;
        ym = ym + ym_dot * dt;
        
        // MIT
        theta1_dot = -gamma *  r_filtered * e0;
        theta2_dot =  gamma * yp_filtered * e0;

        // Euler integration
        theta1 = theta1 + theta1_dot * dt;
        theta2 = theta2 + theta2_dot * dt;
        
        // Clamp θ1, θ2 for better tracking
        if(theta1 > 10) theta1 = 10;
        if(theta2 > 10) theta2 = 10;

        // Controller
        u = theta1 * r - theta2 * yp;

        // Plots of helper signals
        Serial.print("lower:");
        Serial.println(0.1);
        Serial.print("upper:");
        Serial.println(28);

        // Other plots
        Serial.print("theta1 =");
        Serial.println(theta1);
        Serial.print("theta2 =");
        Serial.println(theta2);

        // Serial.print("e0 = ");
        // Serial.println(e0);

        // Serial.print("ym_dot = ");
        // Serial.println(ym_dot);
        
        Serial.print("ym = ");
        Serial.println(ym);
        
        // Serial.print("u =");
        // Serial.println(u);

        // -255 <= u <= 255
        if (u >= 0 && u <= 255) {
            motorSpeed = u; 
            analogWrite(MOTOR_FWD, motorSpeed);       
        }
        else if (u < 0 && u >= -255) {
            motorSpeed = -u; 
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





