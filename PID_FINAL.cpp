#include <Arduino.h>


class encoder{
  public:


  //////// ENCODER ////////


  //PINS
  int pin_A;
  int pin_B;


  //COUNT
  unsigned long int encoder_count;
  unsigned long int  prev_encoder_count;


  //TIME
  unsigned long int current_time;
  unsigned long int prev_time;


  //SPEED
  float speed;


  //////// PID ////////


  //ERROR VARIABLES
  float error_difference;
  float error_integral;
  float error;
  float prev_error;


  //PID PARAMETERS
  int setpoint;
  float Kp;
  float Ki;
  float Kd;


  //PWM for PID
  float input_pwm;
  float prev_pwm;


  //FUNCTION DECLRATION  
  encoder(int pin_A, int pin_B);


  void begin();
  static void countISR();
  void count();
  float rpm();
  int angle();


  int PID(float _setpoint, float _Kp, float _Ki, float _Kd); //returns PWM
};


encoder* encoderInstance = nullptr;


encoder::encoder(int pin_a ,int pin_b)
{
  pin_A = pin_a;
  pin_B = pin_b;


  encoder_count = 0;
  prev_encoder_count = 0;


  current_time = 0;
  prev_time = 0;


  error_difference = 0;
  error_integral = 0;
  error = 0;
  prev_error = 0;


  setpoint = 0;
  Kp = 0;
  Ki = 0;
  Kd = 0;


  input_pwm = 0;
  prev_pwm = 0;


  pinMode(pin_A,INPUT);
  pinMode(pin_B,INPUT);
}


void encoder:: count()
{
  if (digitalRead(pin_A) == digitalRead(pin_B))
  {
    encoder_count++;
  }
  else
  {
    encoder_count--;
  }
  //Serial.println(encoder_count);
}


int encoder:: PID(float _setpoint, float _Kp, float _Ki, float _Kd)
{
    setpoint = _setpoint;
    Kp = _Kp;
    Ki = _Ki;
    Kd = _Kd;


    current_time = micros();


    if(current_time-prev_time>=500000)
    {
      //ERROR


      error = setpoint - rpm();


      error_integral = error_integral + error;


      error_difference = error - prev_error;


      //PID CONTROL LOOP EQUATION


      input_pwm = (Kp*error)*(200/12.9) + (Ki*error_integral)*(200/12.9) +(Kd*error_difference)*(200/12.9);  // 200 rpm for 12.9 pwm so that fraction for convertion


      if(input_pwm!=prev_pwm)
      {
        input_pwm = input_pwm+prev_pwm;
      }


      //CONTRAINT


      input_pwm = constrain(input_pwm,0,255);


      //EXTRA THINGS


      prev_pwm = input_pwm;


      prev_error = error;
    }


    return input_pwm;


}


float encoder:: rpm()
{




  current_time = micros();


  if(current_time-prev_time>=500000)
  {
    speed = ((encoder_count - prev_encoder_count)/39000.0)*60*2; //(no of ticks/total ticks per rotation)*1000000*60
    prev_time =  current_time;  
    prev_encoder_count = encoder_count;
    Serial.println(speed);
  }
  return speed;


}


int encoder:: angle()
{
  Serial.println(encoder_count);
  int count_angle = map(encoder_count%39000,0,39000,0,359);
  return count_angle;


}


void encoder::begin() {
    encoderInstance = this; // Assign this instance to the global pointer
    attachInterrupt(digitalPinToInterrupt(pin_A), countISR, RISING);
}


void encoder::countISR() {
    if (encoderInstance) {
        encoderInstance->count(); // Forward to the actual instance
    }
}
