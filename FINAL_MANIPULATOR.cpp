#include <Arduino.h>

// #define LA1dir PC15 L1-on PCB
// #define LA1pwm PA0

// #define LA2dir PA1  L2 -on PCB
// #define LA2pwm PA2

// #define M1dir PA5  L3 -on PCB
// #define M1pwm PA6 

// #define basedir PB5 L7 -on PCB
// #define basepwm PB4

// #define M2dir PB9
// #define M2pwm PB8

class LA
{
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

LA::LA(int DIR_PIN, int PWM_PIN)
{
  dir_pin = DIR_PIN;
  pwm_pin = PWM_PIN;
  pinMode(dir_pin,OUTPUT);
  pinMode(pwm_pin,OUTPUT);
  digitalWrite(dir_pin,LOW);
  analogWrite(pwm_pin,0);
}

void LA::assign_pwm(int pwm)
{
  analogWrite(pwm_pin,pwm);
}

void LA::forward()
{
  digitalWrite(dir_pin,HIGH);
}

void LA::backward()
{
  digitalWrite(dir_pin,LOW);
}

void LA::stop()
{
  digitalWrite(dir_pin,LOW);
  analogWrite(pwm_pin,0); 
}
///////////////////////////////////////////////////////////
class motor
{
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

motor::motor(int DIR_PIN, int PWM_PIN)
{
  dir_pin = DIR_PIN;
  pwm_pin = PWM_PIN;
  pinMode(dir_pin,OUTPUT);
  pinMode(pwm_pin,OUTPUT);
  digitalWrite(dir_pin,LOW);
  analogWrite(pwm_pin,0);
}

void motor::assign_pwm(int pwm)
{
  analogWrite(pwm_pin,pwm);
}

void motor::CW()    //clockwise
{
  digitalWrite(dir_pin,HIGH);
}

void motor::ACW()   //anticlockwise
{
  digitalWrite(dir_pin,LOW);
} 

void motor::stop()
{
  digitalWrite(dir_pin,LOW);
  analogWrite(pwm_pin,0); 
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////


LA link1(PC15, PA0);
LA link2(PA1, PA2);
motor link3(PA5,PA6);
motor base(PB5,PB4);

int choice;
int pwm; //common pwm for all links. if needed, make one pwm for actuators and one for motors later

void setup()
{
  Serial.begin(9600);
  // Serial.println("pwm:");
  // while (Serial.available() == 0);
  // pwm = Serial.parseInt();
  // pwm = 80;
}

void loop()
{
  // link1.assign_pwm(pwm);
  // link2.assign_pwm(pwm);
  //link1.forward();

  Serial.println("choice:");
  while (Serial.available() == 0);
  choice = Serial.parseInt(); 

  

  switch(choice)
  {
    
    case 1: //knob 1 forward
      Serial.println("pwm:");
      while (Serial.available() == 0);
      pwm = Serial.parseInt();
      link1.assign_pwm(pwm); link1.forward(); break; 

    case 2:  //knob 1 backward
      Serial.println("pwm:");
      while (Serial.available() == 0);
      pwm = Serial.parseInt();
      link1.assign_pwm(pwm); link1.backward(); break;

    case 3: //knob 2 forward
      Serial.println("pwm:");
      while (Serial.available() == 0);
      pwm = Serial.parseInt();
      link2.assign_pwm(pwm); link2.forward(); break;

    case 4: //knob 2 backward
      Serial.println("pwm:");
      while (Serial.available() == 0);
      pwm = Serial.parseInt();
      link2.assign_pwm(pwm); link2.backward(); break;

    case 5: //knob 1 right
      Serial.println("pwm:");
      while (Serial.available() == 0);
      pwm = Serial.parseInt();
      link3.assign_pwm(pwm); link3.CW(); break;  

    case 6: //knob 1 left
      Serial.println("pwm:");
      while (Serial.available() == 0);
      pwm = Serial.parseInt();
      link3.assign_pwm(pwm); link3.ACW(); break;

    case 7: //knob 2 right
      Serial.println("pwm:");
      while (Serial.available() == 0);
      pwm = Serial.parseInt();
      base.assign_pwm(pwm); base.CW(); break;

    case 8: //knob 2 left
      Serial.println("pwm:");
      while (Serial.available() == 0);
      pwm = Serial.parseInt();
      base.assign_pwm(pwm); base.ACW(); break;

    default:
       link1.stop(); link2.stop(); link3.stop(); base.stop(); break;
  }
}
