#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

Encoder motorB(14, 15);
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
double Ki = 0.6;
double Kd = 0.0000000000001;
double z=1.0;
double tar_rpm_B=0.0;
int B_PWM = 9;
int B_1 = 10;
int B_2 = 11;
int tar_rpm_BB;
double last_error_B=0.0;

void setMotorForward(int pin1, int pin2){
  digitalWrite(pin1,HIGH);
  digitalWrite(pin2,LOW);
}
void setMotorBackward(int pin1, int pin2){
  digitalWrite(pin1,LOW);
  digitalWrite(pin2,HIGH);
}

double caluate_PID(double x,double y){
  double result;
    result = x - y;
    double result1;
      result1 = result1+result*time_s;
      double result2;
        result2 =(result-last_error_B)/time_s;
        last_error_B =result;
        double result3;
          result3 = Kp*result+Ki*result1+Kd*result2;
        return result3;
      return result2;
    return result1;
  return result;
  }
  //double error_B = tar_rpm_B-rpmB;
  //error_i_B = error_i_B+error_B*time_s;
  //double error_d_B = (error_B - last_error_B)/time_s;
  //last_error_B = error_B;  
  //double out_put_B = Kp*error_B+Ki*error_i_B+Kd*error_d_B;
  
//double caluate_PID(double x,double y,double z){
  //double result;
  //result = Kp*x+Ki*y+Kd*z;
  //return result;
//}

void setup() {
  Serial.begin(9600);
  pinMode(B_PWM, OUTPUT);
  pinMode(B_1, OUTPUT);
  pinMode(B_2, OUTPUT);
  analogWriteFrequency(9,93750);
  analogWriteResolution(10);
}

void loop() {
  long pulse_B;
  long rpmB;
  pulse_B=motorB.read();
  rpmB=(pulse_B*time_s_m)/sensor_R;
  Serial.printf("motor B rpm: %ld\n", rpmB);
  motorB.write(0);
  if(Serial.available()>4){
    int A = Serial.read();
    int B = Serial.read()-48;
    int C = Serial.read()-48;
    int D = Serial.read()-48;
    int E = Serial.read()-48;
    tar_rpm_BB = B*1000.0+C*100.0+D*10.0+E;
    tar_rpm_B=tar_rpm_BB*z;
    if(A == 45){
      tar_rpm_B = tar_rpm_BB*-1.0;
    }
    else if(A == 43){
      tar_rpm_B = tar_rpm_BB*1.0;  
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

  double out_put_B = caluate_PID(tar_rpm_B,rpmB);

  if(tar_rpm_B ==0){
    digitalWrite(B_1,LOW);
    digitalWrite(B_2,LOW);
  }
  
  if(out_put_B>0){
      setMotorForward(B_1, B_2);
      out_put_B = min(out_put_B,950);
      analogWrite(B_PWM,out_put_B);
    }
    else{
      setMotorBackward(B_1,B_2);
      out_put_B = out_put_B*-1.0;
      out_put_B = min(out_put_B,950);
      analogWrite(B_PWM,out_put_B);
    }

  delay(timeunit);
}
