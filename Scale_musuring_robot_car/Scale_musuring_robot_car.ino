#include <NewPing.h>
#define TRIG_PIN A0
#define ECHO_PIN A1
NewPing sonar(TRIG_PIN, ECHO_PIN);
#include<AFMotor.h>
AF_DCMotor M_fr(1);
AF_DCMotor M_bl(2);
AF_DCMotor M_br(3);
AF_DCMotor M_fl(4);
#define MAX_SPEED_OFFSET 40
#define MAX_SPEED 160
#include<Servo.h>
Servo X_axis;
Servo Y_axis;
const int xpin = A3;
const int ypin = A4;
const int zpin = A5;
int speedSet = 0;
#define ADC_ref 2.56
#define zero_x 1.569 
#define zero_y 1.569
#define zero_z 1.569
#define sensitivity_x 0.3
#define sensitivity_y 0.3 
#define sensitivity_z 0.3
char bt='0';
int y_1=0;
int y_2=0;
int x_1=0;
int x_2=0;
int c1=2;
int c2=1;
int c3=0;

void setup() {
Serial.begin(9600);
X_axis.attach(9);
Y_axis.attach(10);
}

void loop() {
  bt=Serial.read();
  switch(bt){
    case 'F':
      Forward();
      break; 
    case 'B':
      Backward();
      break;   
    case 'L':
      Turn_l();
      break; 
    case 'R':
      Turn_r();
      break;
    case 'S':
      Stop_car();
      break;  
    }
    if(bt =='C'){
      Stop_car();
      int h_val=0;
      int v_val=0;
      int len=0;
      int last_val=0;  
      int xlevel= analogRead(xpin);
      int ylevel= analogRead(ypin);
      int zlevel= analogRead(zpin);
      long xv=(xlevel/1024.0*ADC_ref-zero_x)/sensitivity_x;
      long yv=(ylevel/1024.0*ADC_ref-zero_y)/sensitivity_y;
      long zv=(zlevel/1024.0*ADC_ref-zero_z)/sensitivity_z;
      long angle_x =atan2(-yv,-zv)*57.2957795+180;
      long angle_y =atan2(-xv,-zv)*57.2957795+180;
      int n=90-angle_y;
      Y_axis.write(n);
      delay(500);
      Y_axis.write(90);
      while(c2-c1<30){
          X_axis.write(c3-=2);
          delay(50);
          c1=c2;
          c2=readPing();}
      x_1=c1;
      X_axis.write(c3+5);
      c3=c3+5;    
      c1=c2;
      while( c1>= c2){
          X_axis.write(c3+=2);
          delay(50);
          c1=c2;
          c2=readPing();    
      }
      x_1=(sqrt(x_1^2-c1^2));
      int m=c3-2;
      X_axis.write(m);
      
      h_val=readPing();
      len=h_val;
      while(len-last_val<30){
            Y_axis.write(n+=1); 
            delay(50);
            last_val=len;
            len = readPing();} 
      y_1=(sqrt(last_val^2-h_val^2));
      n=90-angle_y;
      Y_axis.write(n);
      h_val=readPing();
      len=h_val;
      while(len-last_val<30){
            Y_axis.write(n-=1); 
            delay(50);
            last_val=len;
            len = readPing();}
      y_2=(sqrt(last_val^2-h_val^2));
      n=90-angle_y;
      Y_axis.write(n);      
      delay(1000);
      Serial.print(y_1+y_2);
 
      X_axis.write(m);
      delay(100);
      v_val=readPing();
      len=0;
      last_val=0;
      while(len-last_val<30){
            X_axis.write(m+=2); 
            delay(50);
            last_val=len;
            len= readPing();  }
           x_2=(sqrt(last_val^2-v_val^2));
           m=90;
           X_axis.write(m);
           delay(1000);
           Serial.print(0-x_1-x_2);
  }
}


int readPing() { 
  delay(70);   
  unsigned int uS = sonar.ping();
  int cm = uS/US_ROUNDTRIP_CM;
  return cm;
}

void Stop_car() {M_fr.run(RELEASE); M_fl.run(RELEASE); M_bl.run(RELEASE); M_br.run(RELEASE);}

void Forward(){
    M_fr.run(FORWARD);
    M_fl.run(FORWARD);
    M_bl.run(FORWARD);
    M_br.run(FORWARD);
  for (speedSet = 0; speedSet < MAX_SPEED; speedSet +=2)
  {M_fr.setSpeed(speedSet);
    M_fl.setSpeed(speedSet);
    M_bl.setSpeed(speedSet); 
    M_br.setSpeed(speedSet);
    delay(5);
  }
  }
void Backward(){
    M_fr.run(BACKWARD);
    M_fl.run(BACKWARD);
    M_bl.run(BACKWARD);
    M_br.run(BACKWARD);
  for (speedSet = 0; speedSet < MAX_SPEED; speedSet +=2)
  { M_fr.setSpeed(speedSet);
    M_fl.setSpeed(speedSet);
    M_bl.setSpeed(speedSet); 
    M_br.setSpeed(speedSet);
    delay(5);
  }
  }
void Turn_r(){
   M_fl.run(FORWARD);
   M_bl.run(FORWARD);
   M_fr.run(BACKWARD);
   M_br.run(BACKWARD);
   M_br.setSpeed(speedSet+MAX_SPEED_OFFSET);      
   M_br.setSpeed(speedSet+MAX_SPEED_OFFSET);       
}
void Turn_l(){
   M_fl.run(BACKWARD); 
   M_bl.run(BACKWARD); 
   M_fl.setSpeed(speedSet+MAX_SPEED_OFFSET);     
   M_bl.setSpeed(speedSet+MAX_SPEED_OFFSET);    
   M_br.run(FORWARD); 
   M_bl.run(FORWARD);
}
