#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

Encoder motor1(14, 15);
Encoder motor2(18, 19);
double timeunit=40;
double sensor_R = 13;
double time_s = timeunit/1000;
double time_s_ratio = 1/time_s;
double time_s_m = time_s_ratio*60;
double tar_rpm_B = 3000.0;
double tar_rpm_A = 3000.0;
double Kp = 0.03;
double error_i_B = 0.0;
double error_i_A = 0.0;
double Ki = 0.03;
double Kd = 0.000;
int B_PWM = 9;
int B_1 = 10;
int B_2 = 11;
int A_PWM = 5;
int A_1 = 6;
int A_2 = 7;

void setup() {
  Serial.begin(9600);
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
  long pluse_1;
  long rpm1;
  long pluse_2;
  long rpm2;
  pluse_1=motor1.read();
  pluse_2=motor2.read();
  rpm1=(pluse_1*time_s_m)/sensor_R;
  rpm2=(pluse_2*time_s_m)/sensor_R;
  double error_B = tar_rpm_B-rpm1;
  double error_A = tar_rpm_A-rpm2;
  error_i_B = error_i_B+error_B*time_s;
  error_i_A = error_i_A+error_A*time_s;
  double error_d_B = error_B/time_s;
  double error_d_A = error_A/time_s;
  double out_put_B = Kp*error_B+Ki*error_i_B+Kd*error_d_B;
  double out_put_A = Kp*error_A+Ki*error_i_A+Kd*error_d_A;
    if((out_put_B>0 && out_put_A>0) ||(out_put_B<0 && out_put_A<0)){
    if(out_put_B>0 && out_put_A>0){
      digitalWrite(B_1,LOW);
      digitalWrite(B_2,HIGH);
      digitalWrite(A_1,HIGH);
      digitalWrite(A_2,LOW);
      out_put_B = min(out_put_B,900);
      out_put_A = min(out_put_A,900);
      analogWrite(B_PWM,out_put_B);
      analogWrite(A_PWM,out_put_A);
    }
    else{
      digitalWrite(B_1,HIGH);
      digitalWrite(B_2,LOW);
      digitalWrite(A_1,LOW);
      digitalWrite(A_2,HIGH);
      out_put_B = out_put_B*-1.0;
      out_put_A = out_put_A*-1.0;
      out_put_B = min(out_put_B,900);
      out_put_A = min(out_put_A,900);
      analogWrite(B_PWM,out_put_B);
      analogWrite(A_PWM,out_put_A);
    }
  }
  else{
    if(out_put_B>0 && out_put_A<0){
      digitalWrite(B_1,LOW);
      digitalWrite(B_2,HIGH);
      digitalWrite(A_1,LOW);
      digitalWrite(A_2,HIGH);
      out_put_B = min(out_put_B,900);
      out_put_A = out_put_A*-1.0;
      out_put_A = min(out_put_A,900);
      analogWrite(B_PWM,out_put_B);
      analogWrite(A_PWM,out_put_A);
    }
    else if((out_put_B>0 && out_put_A<0) ||(out_put_B<0 && out_put_A>0)){
      digitalWrite(B_1,HIGH);
      digitalWrite(B_2,LOW);
      digitalWrite(A_1,HIGH);
      digitalWrite(A_2,LOW);
      out_put_B = out_put_B*-1.0;
      out_put_B = min(out_put_B,900);
      out_put_A = min(out_put_A,900);
      analogWrite(B_PWM,out_put_B);
      analogWrite(A_PWM,out_put_A);
    }
  }
  //digitalWrite(B_1,LOW);
  //digitalWrite(B_2,HIGH);
  //digitalWrite(A_1,HIGH);
  //digitalWrite(A_2,LOW);
  //analogWrite(B_PWM,out_put_B);
  //analogWrite(A_PWM,out_put_A);    
  Serial.printf("motor rpm: %ld,%ld\n", rpm1,rpm2);
  motor1.write(0);
  motor2.write(0);
  delay(timeunit);
}
