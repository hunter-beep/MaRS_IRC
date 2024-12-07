#include <Arduino.h>

void forward();
void backward();
void spotleft();
void spotright();

//Motor Drivers Pin Declaration

//front left
#define dir_fl PC15    
#define pwm_fl PA0

//front right
#define dir_fr PB9
#define pwm_fr PB8

//back left--
#define dir_bl PA5
#define pwm_bl PA6

//back right
#define dir_br PB15
#define pwm_br PB14

//choice
int choice;
int input_pwm;


//class declration of motor
class motor
{
  private:
  
    int dir_pin;
    int pwm_pin;
  
  public:
    motor(int pin1,int pin2);
    void clockwise();
    void anticlockwise();
    void pwm(int input_pwm);
};

motor::motor(int pin1,int pin2)
{
  dir_pin = pin1;
  pwm_pin = pin2;    
  pinMode(dir_pin,OUTPUT);
  pinMode(pwm_pin,OUTPUT);
}

void motor::clockwise() // Viewing the rover form the right side with respect to front of the rover
{
  digitalWrite(dir_pin,HIGH);
}

void motor::anticlockwise()
{
  digitalWrite(dir_pin,LOW);
}

void motor::pwm(int input_pwm)
{
  analogWrite(pwm_pin,input_pwm);
}


motor motorFL(dir_fl,pwm_fl);
motor motorFR(dir_fr,pwm_fr);
motor motorBR(dir_br,pwm_br);
motor motorBL(dir_bl,pwm_bl);

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  Serial.print("Enter pwm: ");
  while(Serial.available()==0);
  input_pwm = Serial.parseInt();
  input_pwm = constrain(input_pwm,0,255);

  Serial.println("Enter choice: ");
  while(Serial.available()==0);
  choice = Serial.parseInt();

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
  }

  motorFR.pwm(input_pwm);
  motorBR.pwm(input_pwm);
  motorFL.pwm(input_pwm);
  motorBL.pwm(input_pwm);

}

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

  motorFL.anticlockwise();
  motorBL.anticlockwise();
}

void spotright()
{
  motorFR.anticlockwise();
  motorBR.anticlockwise();

  motorFL.clockwise();
  motorBL.clockwise();
}