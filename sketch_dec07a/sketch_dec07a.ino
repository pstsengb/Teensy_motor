#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

Encoder motor1(10, 11);
Encoder motor2(15, 16);
double timeunit=1000;
double sensor_R=13;
double time_s=timeunit/1000;
double time_s_ratio=1/time_s;
double time_s_m=time_s_ratio*60;

void setup() {
  Serial.begin(9600);

}

void loop() {
  long pluse_1;
  long pluse_2;
  long rpm1;
  long rpm2;
  pluse_1=motor1.read();
  pluse_2=motor2.read();
  rpm1=(pluse_1*time_s_m)/sensor_R;
  rpm2=(pluse_2*time_s_m)/sensor_R;
  Serial.printf("L-motor pluse: %ld R-motor pluse: %ld \n", pluse_1, pluse_2);
  Serial.printf("L-motor rpm: %ld   R-motor rpm: %ld \n", rpm1, rpm2);
  motor1.write(0);
  motor2.write(0);
  delay(timeunit);
}
