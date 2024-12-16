#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Int16.h>


#define LA1dir PC15
#define LA1pwm PA0
#define LA2dir PA1
#define LA2pwm PA2
#define M1dir PA5
#define M1pwm PA6
#define basedir PB5
#define basepwm PB4


class LA {
 private:
   int pwm_pin;
   int dir_pin;


 public:
   LA(int, int);
   void assign_pwm(int);
   void forward();
   void backward();
   void stop();
};


LA::LA(int DIR_PIN, int PWM_PIN) {
 dir_pin = DIR_PIN;
 pwm_pin = PWM_PIN;
 pinMode(dir_pin, OUTPUT);
 pinMode(pwm_pin, OUTPUT);
 digitalWrite(dir_pin, LOW);
 analogWrite(pwm_pin, 0);
}


void LA::assign_pwm(int pwm) {
 analogWrite(pwm_pin, pwm);
}


void LA::forward() {
 digitalWrite(dir_pin, HIGH);
}


void LA::backward() {
 digitalWrite(dir_pin, LOW);
}


void LA::stop() {
 digitalWrite(dir_pin, LOW);
 analogWrite(pwm_pin, 0);
}


class motor {
 private:
   int pwm_pin;
   int dir_pin;


 public:
   motor(int, int);
   void assign_pwm(int);
   void CW();
   void ACW();
   void stop();
};


motor::motor(int DIR_PIN, int PWM_PIN) {
 dir_pin = DIR_PIN;
 pwm_pin = PWM_PIN;
 pinMode(dir_pin, OUTPUT);
 pinMode(pwm_pin, OUTPUT);
 digitalWrite(dir_pin, LOW);
 analogWrite(pwm_pin, 0);
}


void motor::assign_pwm(int pwm) {
 analogWrite(pwm_pin, pwm);
}


void motor::CW() { digitalWrite(dir_pin, HIGH); }


void motor::ACW() { digitalWrite(dir_pin, LOW); }


void motor::stop() {
 digitalWrite(dir_pin, LOW);
 analogWrite(pwm_pin, 0);
}




LA link1(LA1dir, LA1pwm);
LA link2(LA2dir, LA2pwm);
motor link3(M1dir, M1pwm);
motor base(basedir, basepwm);


ros::NodeHandle nh;
std_msgs::Int16 pwm_msg;
std_msgs::Int16 choice_msg;


// ros subscriber callbacks
void choiceCallback(const std_msgs::Int16 &msg) {
 int choice = msg.data;
 int pwm = 100;


 switch (choice) {
   case 1:
     link1.assign_pwm(pwm);
     link1.forward();
     break;


   case 2:
     link1.assign_pwm(pwm);
     link1.backward();
     break;


   case 3:
     link2.assign_pwm(pwm);
     link2.forward();
     break;


   case 4:
     link2.assign_pwm(pwm);
     link2.backward();
     break;


   case 5:
     link3.assign_pwm(pwm);
     link3.CW();
     break;


   case 6:
     link3.assign_pwm(pwm);
     link3.ACW();
     break;


   case 7:
     base.assign_pwm(pwm);
     base.CW();
     break;


   case 8:
     base.assign_pwm(pwm);
     base.ACW();
     break;


   default:
     link1.stop();
     link2.stop();
     link3.stop();
     base.stop();
     break;
 }
}


void pwmCallback(const std_msgs::Int16 &msg) {
 int pwm = msg.data;
 link1.assign_pwm(pwm);
 link2.assign_pwm(pwm);
 link3.assign_pwm(pwm);
 base.assign_pwm(pwm);
}


ros::Subscriber<std_msgs::Int16> choice_sub("choice", &choiceCallback);
ros::Subscriber<std_msgs::Int16> pwm_sub("pwm", &pwmCallback);


void setup() {
 nh.initNode();
 nh.subscribe(choice_sub);
 nh.subscribe(pwm_sub);
 nh.loginfo("Arduino Node Initialized");
}


void loop() {
 nh.spinOnce();
 delay(10);
}
