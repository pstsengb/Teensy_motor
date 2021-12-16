#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

//B is left_motor
//A is right_motor

Encoder motor1(14, 15);
Encoder motor2(18, 19);
double timeunit=40;
double sensor_R = 13;
double time_s = timeunit/1000;
double time_s_ratio = 1/time_s;
double time_s_m = time_s_ratio*60;
//double tar_rpm_B = 9000.0;
//double tar_rpm_A = 9000.0;
double Kp = 0.02;
double error_i_B = 0.0;
double error_i_A = 0.0;
double Ki = 0.6;
double Kd = 0.0000000000001;
double z=1.0;
int B_PWM = 9;
int B_1 = 10;
int B_2 = 11;
int A_PWM = 5;
int A_1 = 6;
int A_2 = 7;
int tar_rpm_BB;
int tar_rpm_AA;
double tar_rpm_B;
double tar_rpm_A;

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
  motor1.write(0);
  motor2.write(0);
    if(Serial.available()>7){
      int A = Serial.read();
      int B = Serial.read();
      int C = Serial.read();
      int D = Serial.read();
      int E = Serial.read();
      int F = Serial.read();
      int G = Serial.read();
      int H = Serial.read();
      A = A-48.0;
      B = B-48.0;
      C = C-48.0;
      D = D-48.0;
      E = E-48.0;
      F = F-48.0;
      G = G-48.0;
      H = H-48.0;
      tar_rpm_BB = A*1000.0+B*100.0+C*10.0+D;
      tar_rpm_AA = E*1000.0+F*100.0+G*10.0+H; 
      tar_rpm_B=tar_rpm_BB*z;
      tar_rpm_A=tar_rpm_AA*z;
    }
     else{
        int serial_length = Serial.available();
        for(int i=0;i<serial_length;i++){
          int dummy_reader = Serial.read();
          Serial.print("dummy_reader:");
          Serial.println(dummy_reader);
        }
     }

      double error_B = tar_rpm_B-rpm1;
      double error_A = tar_rpm_A-rpm2;
      error_i_B = error_i_B+error_B*time_s;
      error_i_A = error_i_A+error_A*time_s;
      double error_d_B = error_B/time_s;
      double error_d_A = error_A/time_s;
      double out_put_B = Kp*error_B+Ki*error_i_B+Kd*error_d_B;
      double out_put_A = Kp*error_A+Ki*error_i_A+Kd*error_d_A;
      Serial.printf("motor rpm: %ld,%ld,%.2f,%.2f\n", rpm1,rpm2,error_B,error_A);
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
