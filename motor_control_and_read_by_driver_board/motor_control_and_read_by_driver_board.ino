#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

Encoder motor1(14, 15);
double timeunit = 100.0;
double sensor_R = 13.0;
double time_s = timeunit/1000.0;
double time_s_ratio = 1/time_s;
double time_s_m = time_s_ratio*60.0;
int A_PWM = 9 ;
int A_1 = 10;
int A_2 = 11; 

void setup(){
  Serial.begin(9600);
  pinMode(A_PWM, OUTPUT);
  pinMode(A_1, OUTPUT);
  pinMode(A_2, OUTPUT);
}
void loop(){
  long pluse_1;
  long rpm1;
  pluse_1 = motor1.read();
  rpm1 = (pluse_1*time_s_m)/sensor_R;
  Serial.printf("motor pluse: %ld \n", pluse_1);
  Serial.printf("motor rpm: %ld  \n", rpm1);
  motor1.write(0);
  delay(timeunit);
  if(Serial.available()>0){
    if(Serial.available()==3){
     int A = Serial.read();
     int B = Serial.read();
     int C = Serial.read();
     A = A-48;
     B = B-48;
     C = C-48;
     int OUT = A*100+B*10+C;
       if(OUT<=255){
         Serial.printf("OUT:%d\n",OUT);
         digitalWrite(A_1,LOW);
         digitalWrite(A_2,HIGH);
         analogWrite(A_PWM,OUT);
       }
    }
    if(Serial.available()==2){
     int A = Serial.read();
     int B = Serial.read();
     A = A-48;
     B = B-48;
     int OUT = A*10+B;
       if(OUT<=255){
         Serial.printf("OUT:%d\n",OUT);
         digitalWrite(A_1,LOW);
         digitalWrite(A_2,HIGH);
         analogWrite(A_PWM,OUT);
       }
    }
    if(Serial.available()==1){
      int A = Serial.read();
      A = A-48;
      int OUT = A;
      if(OUT<=255){
      Serial.printf("OUT:%d\n",OUT);
      digitalWrite(A_1,LOW);
      digitalWrite(A_2,HIGH);
      analogWrite(A_PWM,OUT);
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
  }
}
