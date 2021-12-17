#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

//B is left_motor form back see
//A is right_motor from back see

Encoder motorB(14, 15);
Encoder motorA(16, 17);
double pi=3.1415;
double wheel_diameter=0.065; //unit=m
double gear_ratio=30.0;
double timeunit=40.0;
double sensor_R = 13.0;
double time_s = timeunit/1000.0;
double time_s_ratio = 1/time_s;
double time_s_m = time_s_ratio*60.0;
double Kp = 0.02;
double error_i_B = 0.0;
double error_i_A = 0.0;
double Ki = 0.6;
double Kd = 0.0000000000001;
double z=1.0;
double tar_rpm_B=0.0;
double tar_rpm_A=0.0;
double wheel_rps_B=0.0;
double wheel_rps_A=0.0;
double veolity_B=0.0;//unit=m/s use B wheel caluate
double veolity_A=0.0;
double displacement_B=0.0; //unit=m
double displacement_A=0.0; 
double rad=0;
int B_PWM = 9;
int B_1 = 10;
int B_2 = 11;
int A_PWM = 5;
int A_1 = 6;
int A_2 = 7;
int tar_rpm_BB;
int tar_rpm_AA;
double last_error_B=0.0;
double last_error_A=0.0;

void setup() {
  Serial.begin(115200);
  if(!bno.begin())
  {
    while(1);
  }
  delay(1000);
  pinMode(B_PWM, OUTPUT);
  pinMode(B_1, OUTPUT);
  pinMode(B_2, OUTPUT);
  pinMode(A_PWM, OUTPUT);
  pinMode(A_1, OUTPUT);
  pinMode(A_2, OUTPUT);
  analogWriteFrequency(9,93750);
  analogWriteFrequency(5,93750);
  analogWriteResolution(10);
}

void loop() {
  long pulse_B;
  long rpmB;
  long pulse_A;
  long rpmA;
  pulse_B=motorB.read();
  pulse_A=motorA.read();
  rpmB=(pulse_B*time_s_m)/sensor_R;
  rpmA=(pulse_A*time_s_m)/sensor_R;
  wheel_rps_B = ((rpmB*z)/gear_ratio)/60;
  wheel_rps_A = ((rpmA*z)/gear_ratio)/60;
  veolity_B = (wheel_rps_B*pi*wheel_diameter)/4.0;
  veolity_A = (wheel_rps_A*pi*wheel_diameter)/4.0;
  displacement_B = displacement_B+(veolity_B*time_s);
  displacement_A = displacement_A+(veolity_A*time_s);
  sensors_event_t angVelocityData;
  bno.getEvent(&angVelocityData,Adafruit_BNO055::VECTOR_GYROSCOPE);
  rad = rad +angVelocityData.gyro.z*time_s;
  Serial.printf("motor B rpm: %ld, motor A rpm: %ld, veolity_B: %.2f, veolity_A: %.2f, displacement_B: %.2f, displacement_A: %.2f, ang_veolity: %.2f, ang_displacement: %.2f\n", rpmB,rpmA,veolity_B,veolity_A,displacement_B,displacement_A,angVelocityData.gyro.z,rad);
  motorB.write(0);
  motorA.write(0);
  if(Serial.available()>9){
    int A = Serial.read();
    int B = Serial.read()-48;
    int C = Serial.read()-48;
    int D = Serial.read()-48;
    int E = Serial.read()-48;
    int F = Serial.read();
    int G = Serial.read()-48;
    int H = Serial.read()-48;
    int I = Serial.read()-48;
    int J = Serial.read()-48;
    tar_rpm_BB = B*1000.0+C*100.0+D*10.0+E;
    tar_rpm_AA = G*1000.0+H*100.0+I*10.0+J; 
    tar_rpm_B=tar_rpm_BB*z;
    tar_rpm_A=tar_rpm_AA*z;
    if(A == 45){
      tar_rpm_B = tar_rpm_BB*-1.0;
    }
    else if(A == 43){
      tar_rpm_B = tar_rpm_BB*1.0;  
    }
    if(F == 45){
      tar_rpm_A = tar_rpm_AA*-1.0;  
    }
    else if(F == 43){
      tar_rpm_A = tar_rpm_AA*1.0;  
    }
  }
  else{
    int serial_length = Serial.available();
    for(int i=0;i<serial_length;i++){
      int dummy_reader = Serial.read();
      Serial.print("dummy_reader:");
      Serial.println(dummy_reader);
    }
  }
  double error_B = tar_rpm_B-rpmB;
  double error_A = tar_rpm_A-rpmA;
  error_i_B = error_i_B+error_B*time_s;
  error_i_A = error_i_A+error_A*time_s;
  double error_d_B = (error_B - last_error_B)/time_s;
  double error_d_A = (error_A - last_error_A)/time_s;
  last_error_B = error_B;
  last_error_A = error_A;
  
  double out_put_B = Kp*error_B+Ki*error_i_B+Kd*error_d_B;
  double out_put_A = Kp*error_A+Ki*error_i_A+Kd*error_d_A;
  if(tar_rpm_B ==0){
    digitalWrite(B_1,LOW);
    digitalWrite(B_2,LOW);
  }
  else if(tar_rpm_A ==0){
    digitalWrite(A_1,HIGH);
    digitalWrite(A_2,HIGH);
  }
  if((out_put_B>0 && out_put_A>0) ||(out_put_B<0 && out_put_A<0)){
    if(out_put_B>0 && out_put_A>0){
      digitalWrite(B_1,HIGH);
      digitalWrite(B_2,LOW);
      digitalWrite(A_1,HIGH);
      digitalWrite(A_2,LOW);
      out_put_B = min(out_put_B,950);
      out_put_A = min(out_put_A,950);
      analogWrite(B_PWM,out_put_B);
      analogWrite(A_PWM,out_put_A);
    }
    else{
      digitalWrite(B_1,LOW);
      digitalWrite(B_2,HIGH);
      digitalWrite(A_1,LOW);
      digitalWrite(A_2,HIGH);
      out_put_B = out_put_B*-1.0;
      out_put_A = out_put_A*-1.0;
      out_put_B = min(out_put_B,950);
      out_put_A = min(out_put_A,950);
      analogWrite(B_PWM,out_put_B);
      analogWrite(A_PWM,out_put_A);
    }
  }
  else if((out_put_B>0 && out_put_A<0) ||(out_put_B<0 && out_put_A>0)){
    if(out_put_B>0 && out_put_A<0){
      digitalWrite(B_1,HIGH);
      digitalWrite(B_2,LOW);
      digitalWrite(A_1,LOW);
      digitalWrite(A_2,HIGH);
      out_put_B = min(out_put_B,950);
      out_put_A = out_put_A*-1.0;
      out_put_A = min(out_put_A,950);
      analogWrite(B_PWM,out_put_B);
      analogWrite(A_PWM,out_put_A);
    }
    else{
      digitalWrite(B_1,LOW);
      digitalWrite(B_2,HIGH);
      digitalWrite(A_1,HIGH);
      digitalWrite(A_2,LOW);
      out_put_B = out_put_B*-1.0;
      out_put_B = min(out_put_B,950);
      out_put_A = min(out_put_A,950);
      analogWrite(B_PWM,out_put_B);
      analogWrite(A_PWM,out_put_A);
    }
  }
  delay(timeunit);
}
