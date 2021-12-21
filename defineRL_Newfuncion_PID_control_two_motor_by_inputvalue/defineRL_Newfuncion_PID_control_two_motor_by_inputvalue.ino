#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

//B is left_motor form back see
//A is right_motor from back see

Encoder motorL(14, 15);
Encoder motorR(16, 17);
double pi=3.1415;
double wheel_diameter=0.065; //unit=m
double gear_ratio=30.0;
double timeunit=40.0;
double sensor_R = 13.0;
double time_s = timeunit/1000.0;
double time_s_ratio = 1/time_s;
double time_s_m = time_s_ratio*60.0;
double Kp = 0.02;
double error_i_L = 0.0;
double error_i_R = 0.0;
double Ki = 0.6;
double Kd = 0.0000000000001;
double z=1.0;
double tar_rpm_L=0.0;
double tar_rpm_R=0.0;
double wheel_rps_L=0.0;
double wheel_rps_R=0.0;
double veolity_L=0.0;//unit=m/s use B wheel caluate
double veolity_R=0.0;
double displacement_L=0.0; //unit=m
double displacement_R=0.0; 
double rad=0;
int L_PWM = 9;
int L_1 = 10;
int L_2 = 11;
int R_PWM = 5;
int R_1 = 6;
int R_2 = 7;
int tar_rpm_LL;
int tar_rpm_RR;
double last_error_L=0.0;
double last_error_R=0.0;

void setup() {
  Serial.begin(115200);
  if(!bno.begin())
  {
    while(1);
  }
  delay(1000);
  pinMode(L_PWM, OUTPUT);
  pinMode(L_1, OUTPUT);
  pinMode(L_2, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  pinMode(R_1, OUTPUT);
  pinMode(R_2, OUTPUT);
  analogWriteFrequency(9,93750);
  analogWriteFrequency(5,93750);
  analogWriteResolution(10);
}

void loop() {
  long pulse_L;
  long rpmL;
  long pulse_R;
  long rpmR;
  pulse_L=motorL.read();
  pulse_R=motorR.read();
  rpmL=(pulse_L*time_s_m)/sensor_R;
  rpmR=(pulse_R*time_s_m)/sensor_R;
  wheel_rps_L = ((rpmL*z)/gear_ratio)/60;
  wheel_rps_R = ((rpmR*z)/gear_ratio)/60;
  veolity_L = (wheel_rps_L*pi*wheel_diameter)/4.0;
  veolity_R = (wheel_rps_R*pi*wheel_diameter)/4.0;
  displacement_L = displacement_L+(veolity_L*time_s);
  displacement_R = displacement_R+(veolity_R*time_s);
  sensors_event_t angVelocityData;
  bno.getEvent(&angVelocityData,Adafruit_BNO055::VECTOR_GYROSCOPE);
  rad = rad +angVelocityData.gyro.z*time_s;
  Serial.printf("motor L rpm: %ld, motor R rpm: %ld, veolity_L: %.2f, veolity_R: %.2f, displacement_L: %.2f, displacement_R: %.2f, ang_veolity: %.2f, ang_displacement: %.2f\n", rpmL,rpmR,veolity_L,veolity_R,displacement_L,displacement_R,angVelocityData.gyro.z,rad);
  motorL.write(0);
  motorR.write(0);
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
    tar_rpm_LL = B*1000.0+C*100.0+D*10.0+E;
    tar_rpm_RR = G*1000.0+H*100.0+I*10.0+J; 
    tar_rpm_L=tar_rpm_LL*z;
    tar_rpm_R=tar_rpm_RR*z;
    if(A == 45){
      tar_rpm_L = tar_rpm_LL*-1.0;
    }
    else if(A == 43){
      tar_rpm_L = tar_rpm_LL*1.0;  
    }
    if(F == 45){
      tar_rpm_R = tar_rpm_RR*-1.0;  
    }
    else if(F == 43){
      tar_rpm_R = tar_rpm_RR*1.0;  
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
  double error_L = tar_rpm_L-rpmL;
  double error_R = tar_rpm_R-rpmR;
  error_i_L = error_i_L+error_L*time_s;
  error_i_R = error_i_R+error_R*time_s;
  double error_d_L = (error_L - last_error_L)/time_s;
  double error_d_R = (error_R - last_error_R)/time_s;
  last_error_L = error_L;
  last_error_R = error_R;
  
  double out_put_L = Kp*error_L+Ki*error_i_L+Kd*error_d_L;
  double out_put_R = Kp*error_R+Ki*error_i_R+Kd*error_d_R;
  if(tar_rpm_L ==0){
    digitalWrite(L_1,LOW);
    digitalWrite(L_2,LOW);
  }
  else if(tar_rpm_R ==0){
    digitalWrite(R_1,HIGH);
    digitalWrite(R_2,HIGH);
  }
  if((out_put_L>0 && out_put_R>0) ||(out_put_L<0 && out_put_R<0)){
    if(out_put_L>0 && out_put_R>0){
      digitalWrite(L_1,HIGH);
      digitalWrite(L_2,LOW);
      digitalWrite(R_1,HIGH);
      digitalWrite(R_2,LOW);
      out_put_L = min(out_put_L,950);
      out_put_R = min(out_put_R,950);
      analogWrite(L_PWM,out_put_L);
      analogWrite(R_PWM,out_put_R);
    }
    else{
      digitalWrite(L_1,LOW);
      digitalWrite(L_2,HIGH);
      digitalWrite(R_1,LOW);
      digitalWrite(R_2,HIGH);
      out_put_L = out_put_L*-1.0;
      out_put_R = out_put_R*-1.0;
      out_put_L = min(out_put_L,950);
      out_put_R = min(out_put_R,950);
      analogWrite(L_PWM,out_put_L);
      analogWrite(R_PWM,out_put_R);
    }
  }
  else if((out_put_L>0 && out_put_R<0) ||(out_put_L<0 && out_put_R>0)){
    if(out_put_L>0 && out_put_R<0){
      digitalWrite(L_1,HIGH);
      digitalWrite(L_2,LOW);
      digitalWrite(R_1,LOW);
      digitalWrite(R_2,HIGH);
      out_put_L = min(out_put_L,950);
      out_put_R = out_put_R*-1.0;
      out_put_R = min(out_put_R,950);
      analogWrite(L_PWM,out_put_L);
      analogWrite(R_PWM,out_put_R);
    }
    else{
      digitalWrite(L_1,LOW);
      digitalWrite(L_2,HIGH);
      digitalWrite(R_1,HIGH);
      digitalWrite(R_2,LOW);
      out_put_L = out_put_L*-1.0;
      out_put_L = min(out_put_L,950);
      out_put_R = min(out_put_R,950);
      analogWrite(L_PWM,out_put_L);
      analogWrite(R_PWM,out_put_R);
    }
  }
  delay(timeunit);
}
