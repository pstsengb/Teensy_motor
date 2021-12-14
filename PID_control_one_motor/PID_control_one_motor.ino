#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

Encoder motor1(14, 15);
double timeunit=40;
double sensor_R = 13;
double time_s = timeunit/1000;
double time_s_ratio = 1/time_s;
double time_s_m = time_s_ratio*60;
double tar_rpm = 3000.0;
double Kp = 0.04;
double error_i = 0.0;
double Ki = 0.03;
double Kd = 0.0001;
int B_PWM = 9;
int B_1 = 10;
int B_2 = 11;

void setup() {
  Serial.begin(9600);
  pinMode(B_PWM, OUTPUT);
  pinMode(B_1, OUTPUT);
  pinMode(B_2, OUTPUT);
  analogWriteFrequency(9,93750);
  analogWriteResolution(10);
}

void loop() {
  long pluse_1;
  long rpm1;
  pluse_1=motor1.read();
  rpm1=(pluse_1*time_s_m)/sensor_R;
  double error = tar_rpm-rpm1;
  error_i = error_i+error*time_s;
  double error_d = error/time_s;
  double out_put = Kp*error+Ki*error_i+Kd*error_d;
  if(out_put>0){
    digitalWrite(B_1,LOW);
    digitalWrite(B_2,HIGH);
    out_put = min(out_put,900);
    analogWrite(B_PWM,out_put);  
  }
  else{
    digitalWrite(B_1,HIGH);
    digitalWrite(B_2,LOW);
    out_put = out_put*-1.0;
    out_put = min(out_put,900);
    analogWrite(B_PWM,out_put);      
  }   
  //Serial.printf("motor tick: %ld\n", pluse_1);
  Serial.printf("motor rpm: %ld,%.2f,%.2f\n", rpm1,error,out_put);
  motor1.write(0);
  delay(timeunit);
}
