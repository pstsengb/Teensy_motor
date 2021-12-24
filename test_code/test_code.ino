#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

class Motor{

  public:
    Motor(double m_dt, int m_dir_pin1, int m_dir_pin2, int m_pwm_pin,int m_read_pin1,int m_read_pin2);
    Encoder* motorencoder;
    double target_rpm_l;
    double target_rpm_r;
    double target_rpm;
    double measured_rpm_l;
    double measured_rpm_r;
    double measured_rpm;
    double last_err;
    double dt,dtt;
    double err, err_i, err_d;
    double kp, kd, ki;
    double output;
    long pulseL;
    long pulseR;
    long pulse;

    
    void controlByPID();
    void computeError();
    void inputdata();
    void readdata();
    void setupPID(double m_kp, double m_ki, double m_kd);
    void setRPMs(double m_target_rpm, double m_measured_rpm);
    
  private:
    int dir_pin1, dir_pin2, pwm_pin;
  
};

Motor m1(0.04, 10, 11, 9, 14, 15), m2(0.04, 6, 7, 5, 16, 17);  

Motor::Motor(double m_dt, int m_dir_pin1, int m_dir_pin2, int m_pwm_pin,int m_read_pin1,int m_read_pin2){
  motorencoder = new Encoder(m_read_pin1,m_read_pin2);
  dt = m_dt;
  dtt= m_dt*1000;
  err = 0.0;
  err_i = 0.0;
  err_d = 0.0;
  last_err = 0.0;
  output = 0.0;
  pulseL =0.0;
  pulseR =0.0;
  pulse =0.0;
  dir_pin1 = m_dir_pin1;
  dir_pin2 = m_dir_pin2;
  pwm_pin = m_pwm_pin;

  pinMode(m_dir_pin1, OUTPUT);
  pinMode(m_dir_pin2, OUTPUT);
  pinMode(m_pwm_pin, OUTPUT);
  analogWriteFrequency(m_pwm_pin,93750);
  analogWriteResolution(10);
}

void Motor::setRPMs(double m_target_rpm, double m_measured_rpm){
  target_rpm = m_target_rpm;
  measured_rpm = m_measured_rpm;
}

void Motor::setupPID(double m_kp, double m_ki, double m_kd){
  kp = m_kp;
  ki = m_ki;
  kd = m_kd;
}

void Motor::controlByPID(){
  
  output = kp * err + ki *err_i + kd* err_d;
  //once we get output we can set direction
  if(output>0){
    output = min(output,950);
    digitalWrite(dir_pin1,HIGH);
    digitalWrite(dir_pin2,LOW);
    analogWrite(pwm_pin, output);
  }
  else{
    output = output * -1.0;
    output = min(output,950);
    digitalWrite(dir_pin1,LOW);
    digitalWrite(dir_pin2,HIGH);
    analogWrite(pwm_pin, output);
  }
  
}

void Motor::computeError(){
  err = target_rpm - measured_rpm;
  err_i = err_i + err * dt;
  err_d = (err - last_err)/dt;  
  last_err = err;
}


void Motor::readdata(){
  pulse = motorencoder->read();
  measured_rpm = (pulse*3000)/13;
  motorencoder->write(0);
}

double getAngvel(){
  double result;
  sensors_event_t angVelocityData;
  bno.getEvent(&angVelocityData,Adafruit_BNO055::VECTOR_GYROSCOPE);
  result = angVelocityData.gyro.z;
  return result;
}


void setup() { 
  Serial.begin(115200);
  if(!bno.begin())
  {
    while(1);
  }
  delay(1000);
}

void loop() {
  double angvel = getAngvel();

  m1.readdata();
  m2.readdata();
   
  m1.setupPID(0.02, 0.6, 0.00000001);
  m2.setupPID(0.02, 0.6, 0.00000001);

  m1.setRPMs(3000, m1.measured_rpm);
  m2.setRPMs(3000, m2.measured_rpm);

  m1.computeError();
  m2.computeError();

  m1.controlByPID();
  m2.controlByPID();

  delay(m1.dtt);
  
Serial.printf("m1 cur rpm: %.2f  ,m2 cur rpm: %.2f  ,m1 tar rpm: %.2f  ,m2 tar rpm: %.2f\n", m1.measured_rpm,m2.measured_rpm,m1.target_rpm_l,m1.target_rpm_r);
Serial.printf("angv: %.2f\n", angvel);
;
}
