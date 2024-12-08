#include <ros.h>
#include <std_msgs/Int32.h>

// Motor Drivers Pin Declaration
#define dir_fl PC15    
#define pwm_fl PA0
#define dir_fr PB9
#define pwm_fr PB8
#define dir_bl PA5
#define pwm_bl PA6
#define dir_br PB15
#define pwm_br PB14

// Prototypes
void forward();
void backward();
void spotleft();
void spotright();

// Class declaration of motor
class motor
{
  private:
    int dir_pin;
    int pwm_pin;
  public:
    motor(int pin1, int pin2);
    void clockwise();
    void anticlockwise();
    void pwm(int input_pwm);
};

motor::motor(int pin1, int pin2)
{
  dir_pin = pin1;
  pwm_pin = pin2;    
  pinMode(dir_pin, OUTPUT);
  pinMode(pwm_pin, OUTPUT);
}

void motor::clockwise() 
{
  digitalWrite(dir_pin, LOW);
}

void motor::anticlockwise()
{
  digitalWrite(dir_pin, HIGH);
}

void motor::pwm(int input_pwm)
{
  analogWrite(pwm_pin, input_pwm);
}

// Create motor objects
motor motorFL(dir_fl, pwm_fl);
motor motorFR(dir_fr, pwm_fr);
motor motorBR(dir_br, pwm_br);
motor motorBL(dir_bl, pwm_bl);

// Variables
int input_pwm = 0;
int choice = 0;

// ROS Node handle
ros::NodeHandle nh;

// ROS Callback to receive PWM
void pwmCallback(const std_msgs::Int32& msg)
{
  input_pwm = constrain(msg.data, 0, 255);

  motorFR.pwm(input_pwm);
  motorBR.pwm(input_pwm);
  motorFL.pwm(input_pwm);
  motorBL.pwm(input_pwm);
}

// ROS Callback to receive direction choice
void directionCallback(const std_msgs::Int32& msg)
{
  choice = msg.data;

  switch (choice)
  {
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
    default:
      input_pwm = 0;
      motorFR.pwm(input_pwm);
      motorBR.pwm(input_pwm);
      motorFL.pwm(input_pwm);
      motorBL.pwm(input_pwm);
      break;
  }
}

// Subscribers
ros::Subscriber<std_msgs::Int32> pwm_sub("pwm", pwmCallback);
ros::Subscriber<std_msgs::Int32> direction_sub("direction", directionCallback);

// Motor Control Functions
void forward()
{
  motorFR.clockwise();
  motorBR.clockwise();
  motorFL.anticlockwise();
  motorBL.anticlockwise();
}

void backward()
{
  motorFR.anticlockwise();
  motorBR.anticlockwise();
  motorFL.clockwise();
  motorBL.clockwise();
}

void spotleft()
{
  motorFR.clockwise();
  motorBR.clockwise();
  motorFL.clockwise();
  motorBL.clockwise();
}

void spotright()
{
  motorFR.anticlockwise();
  motorBR.anticlockwise();
  motorFL.anticlockwise();
  motorBL.anticlockwise();
}

void setup()
{
  nh.initNode();
  nh.subscribe(pwm_sub);
  nh.subscribe(direction_sub);
}

void loop()
{
  nh.spinOnce();
}
