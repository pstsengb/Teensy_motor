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
double z=1.0;
double tar_rpm_B=0.0;
int B_PWM = 9;
int B_1 = 10;
int B_2 = 11;
int tar_rpm_BB;

class Motor
{
  public:
    double kp=0.02, ki=0.6, kd=0.0000000000001;
    double err=0, err_i=0, err_d=0, err_last=0;
    double target_rpm;
    double measure_rpm;
    double output;
};

Motor m1;


void setMotorForward(int pin1, int pin2){
  digitalWrite(pin1,HIGH);
  digitalWrite(pin2,LOW);
}
void setMotorBackward(int pin1, int pin2){
  digitalWrite(pin1,LOW);
  digitalWrite(pin2,HIGH);
}

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

  m1.measure_rpm = rpmB;
  m1.target_rpm = tar_rpm_B;
  m1.err =  m1.target_rpm-m1.measure_rpm ;
  m1.err_i = m1.err_i + m1.err * time_s;
  m1.err_d = (m1.err - m1.err_last)/time_s;
  m1.err_last = m1.err;
  m1.output = m1.kp*m1.err + m1.ki*m1.err_i+m1.kd*m1.err_d;
  
  if(tar_rpm_B ==0){
    digitalWrite(B_1,LOW);
    digitalWrite(B_2,LOW);
  }
  
  if(m1.output>0){
      setMotorForward(B_1, B_2);
      m1.output = min(m1.output,950);
      analogWrite(B_PWM,m1.output);
    }
    else{
      setMotorBackward(B_1,B_2);
      m1.output = m1.output*-1.0;
      m1.output = min(m1.output,950);
      analogWrite(B_PWM,m1.output);
    }

  delay(timeunit);
}
