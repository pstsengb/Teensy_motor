#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

double v_in =0;
double w_in =0;

ros::NodeHandle  nh;
void messageCb( const geometry_msgs::Twist& toggle_msggg){
  v_in = toggle_msggg.linear.x;
  w_in = toggle_msggg.angular.z;
}


ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("cmd_vel", messageCb );

class Motor{

  public:
    Motor(double m_dt, int m_dir_pin1, int m_dir_pin2, int m_pwm_pin);
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
    double gear_ratio;
    double wheel;
    double base_long;

    
    void controlByPID();
    void computeError();
    void inputdata();
    void readdata();
    void setupPID(double m_kp, double m_ki, double m_kd);
    void setRPMs(double m_target_rpm, double m_measured_rpm);
    
  private:
    int dir_pin1, dir_pin2, pwm_pin;
  
};

Motor m1(0.02, 10, 11, 9), m2(0.02, 6, 7, 5);  

Motor::Motor(double m_dt, int m_dir_pin1, int m_dir_pin2, int m_pwm_pin){
  gear_ratio = 30.0;
  wheel = 0.065;
  base_long = 0.081;
  dt = m_dt;
  dtt= m_dt*1000;
  err = 0.0;
  err_i = 0.0;
  err_d = 0.0;
  last_err = 0.0;
  output = 0.0;
  pulseL =0.0;
  pulseR =0.0;
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

void Motor::inputdata(){

    double m_v_input = v_in*1.0;  
    double m_w_input = w_in*1.0;  

    double target_r = m_v_input*gear_ratio/3.14159/wheel*4*60;
    double target_l = m_v_input*gear_ratio/3.14159/wheel*4*60;
    double m_required_v_r = base_long * m_w_input;
    double m_required_v_l = -1.0*base_long * m_w_input;
    double target_rotation_r = m_required_v_r*gear_ratio/3.14159/wheel*4*60;
    double target_rotation_l = m_required_v_l*gear_ratio/3.14159/wheel*4*60;
    double tar_rpm_L=target_r + target_rotation_r;
    double tar_rpm_R=target_l + target_rotation_l;
    target_rpm_l = tar_rpm_L;
    target_rpm_r = tar_rpm_R;
  
  }

Encoder motorL(14, 15);
Encoder motorR(16, 17);
void Motor::readdata(){
  pulseL = motorL.read();
  pulseR = motorR.read();
  measured_rpm_l = (pulseL*3000)/13;
  measured_rpm_r = (pulseR*3000)/13;
  motorL.write(0);
  motorR.write(0);
  delay(dtt);
}

double getAngvel(){
  double result;
  sensors_event_t angVelocityData;
  bno.getEvent(&angVelocityData,Adafruit_BNO055::VECTOR_GYROSCOPE);
  result = angVelocityData.gyro.z;
  return result;
}


void setup() { 
  nh.initNode();
  nh.subscribe(sub_cmd_vel);
  //Serial.begin(115200);
  if(!bno.begin())
  {
    while(1);
  }
  delay(1000);
}

void loop() {
  double angvel = getAngvel();

  m1.inputdata();
  m1.readdata();
   
  m1.setupPID(0.03, 0.6, 0.00000001);
  m2.setupPID(0.025, 0.6, 0.00000001);

  m1.setRPMs(m1.target_rpm_l, m1.measured_rpm_l);
  m2.setRPMs(m1.target_rpm_r, m1.measured_rpm_r);

  m1.computeError();
  m2.computeError();

  m1.controlByPID();
  m2.controlByPID();
  nh.spinOnce();
  
//Serial.printf("m1 cur rpm: %.2f  ,m2 cur rpm: %.2f  ,m1 tar rpm: %.2f  ,m2 tar rpm: %.2f\n", m1.measured_rpm_l,m1.measured_rpm_r,m1.target_rpm_l,m1.target_rpm_r);
//Serial.printf("angv: %.2f\n", angvel);
;
}
