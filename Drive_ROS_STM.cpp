#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Int16.h>

// Motor Drivers Pin Declaration
// Front left
#define dir_fl PC15    
#define pwm_fl PA0

// Front right
#define dir_fr PB9
#define pwm_fr PB8

// Back left
#define dir_bl PA5
#define pwm_bl PA6

// Back right
#define dir_br PB15
#define pwm_br PB14

// Choice and PWM variables
int input_pwm = 0;
int choice = 0;

// Class declaration for motor
class motor {
private:
    int dir_pin;
    int pwm_pin;

public:
    motor(int pin1, int pin2);
    void clockwise();
    void anticlockwise();
    void pwm(int input_pwm);
};




motor::motor(int pin1, int pin2) {
    dir_pin = pin1;
    pwm_pin = pin2;
    pinMode(dir_pin, OUTPUT);
    pinMode(pwm_pin, OUTPUT);
}

void motor::clockwise() {
    digitalWrite(dir_pin, LOW); // Experimentally found
}

void motor::anticlockwise() {
    digitalWrite(dir_pin, HIGH);
}

void motor::pwm(int input_pwm) {
    analogWrite(pwm_pin, input_pwm);
}

// Motor objects
motor motorFL(dir_fl, pwm_fl);
motor motorFR(dir_fr, pwm_fr);
motor motorBR(dir_br, pwm_br);
motor motorBL(dir_bl, pwm_bl);

// ROS NodeHandle
ros::NodeHandle nh;

// Movement functions
void forward() {
    motorFR.clockwise();
    motorBR.clockwise();
    motorFL.anticlockwise();
    motorBL.anticlockwise();
}

void backward() {
    motorFR.anticlockwise();
    motorBR.anticlockwise();
    motorFL.clockwise();
    motorBL.clockwise();
}


void spotleft() {
    motorFR.clockwise();
    motorBR.clockwise();
    motorFL.clockwise();
    motorBL.clockwise();
}

void spotright() {
    motorFR.anticlockwise();
    motorBR.anticlockwise();
    motorFL.anticlockwise();
    motorBL.anticlockwise();
}

// Callback for choice topic
void choiceCallback(const std_msgs::Int16 &msg) {
    choice = msg.data;

    switch (choice) {
        case 1:
            forward();
            break;
        case 2:
            backward();
            break;
        case 3:
            spotleft();
            break;
        case 4:
            spotright();
            break;
        case 0:
            input_pwm = 0;
    }
        // Apply updated PWM
    motorFR.pwm(input_pwm);
    motorBR.pwm(input_pwm);
    motorFL.pwm(input_pwm);
    motorBL.pwm(input_pwm);
}




// Callback for PWM topic
void pwmCallback(const std_msgs::Int16 &msg) {
    input_pwm = constrain(msg.data, 0, 255);
}


// ROS Subscribers
ros::Subscriber<std_msgs::Int16> pwm_sub("/rover/pwm", &pwmCallback);
ros::Subscriber<std_msgs::Int16> choice_sub("/rover/choice", &choiceCallback);

void setup() {
    Serial.begin(9600);
    nh.initNode();
    nh.subscribe(choice_sub);
    nh.subscribe(pwm_sub);
}

void loop() {
    nh.spinOnce();
    delay(10); // Short delay for stability
}
