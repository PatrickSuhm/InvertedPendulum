#include <Encoder.h>
  
  Encoder linEnc(2, 4);    // create encoder object for linear encoder
  Encoder rotEnc(3, 5);    // create encoder object for rotary encoder
  
  double rot = 0.0, lin=0.0, rot_old=0.0, lin_old=0.0, lin_dot=0.0, rot_dot=0.0;
  unsigned long micros_old = 0;
  double u = 0.0;
  double integral = 0.0;
  
  const int u_hat = 50;                   // friction compensator
  const int maxu = 255 - u_hat;
  const int minu = u_hat - 255;
  const double Pi = 3.141592654;
  const double TwoPi = 2 * Pi;
  const double rot_conv = TwoPi / 2880;     // rad/step
  const double lin_conv = 0.299 / 7060;     // m/step
  const double m = 0.02;                
  const double g = 9.81;                
  const double L = 0.08;               
  const double e_const = 2 / 3 * L * L * m;
  const double mgl = m * g * L; 
  const double crit_angle = 30 * Pi / 180;
  const double loop_freq = 1000.0;
  const int loop_time_micros = 1000;  
  
//////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(115200);
  pinMode(22, OUTPUT);    // pin for motor dir
  pinMode(24, OUTPUT);    // pin for motor dir
  pinMode(10, OUTPUT);    // pin for motor speed
  pinMode(7, OUTPUT);     // pin to determine loopetime with oszi
}

///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////

void loop() {
  while(micros() - micros_old<loop_time_micros){}  // create fixed loop time of 1ms
  micros_old = micros();
      
  rot = wrapNegPiToPi( rot_conv * rotEnc.read() + Pi );     // read sensors and transform it to SI units and add Pi to have 0 at the top and then normalize it from -Pi to Pi
  lin = - lin_conv * linEnc.read() - 0.15;                  // add -0.15 because we start on the left
   
  rot_dot = (rot - rot_old) * loop_freq;    // angular velocity
  lin_dot = (lin - lin_old) * loop_freq;    // velocity 
  lin_old = lin;
  rot_old = rot;
  
  if(abs(rot) < crit_angle){       // control upper position only when abs(angle) < xy°
    u = 1 * lin + 1 * lin_dot + 1 * rot + 1 * rot_dot + 1 * integral;   // adjust gains here
    integral += lin;               // integral action on position of cart
  }
  else{                            // else do swing-up procedere 
    integral = 0.0;
    const double energy = e_const * rot_dot * rot_dot + mgl * (cos(rot) - 1);
    u = static_cast<int>(1 * energy * rot_dot * cos(rot));            // adjust gain here
  }   
  motor(u);                        // write pwm 
}
   
  
//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////

template <typename T> int sgn(T val) {    
    return (T(0) < val) - (val < T(0));
}

double wrapNegPiToPi(double x){ 
    x = fmod(x + Pi, TwoPi);
    if (x < 0)
        x += TwoPi;
    return x - Pi;
}

void motor(int u){
  if(u>0){
    if((u + u_hat)>255){u = maxu;}
    PORTA |=  B00000001;    // digitalWrite(22,HIGH);
    PORTA &= B11111011;     // digitalWrite(24,LOW);
    analogWrite(10,u + u_hat);
  }
  else{
      if(u==0){analogWrite(10,0);}
      else{
      if((u - u_hat)< - 255){u = minu;}
      PORTA |=  B00000100;    // digitalWrite(24,HIGH);
      PORTA &= B11111110;     // digitalWrite(22,LOW);
      analogWrite(10, - u + u_hat);
      }
  }
}
