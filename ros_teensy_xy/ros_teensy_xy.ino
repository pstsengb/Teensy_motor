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
#include <nav_msgs/Odometry.h>

ros::NodeHandle  nh;

double v_in =0;
double w_in =0;
void messageCb( const geometry_msgs::Twist& toggle_msggg){
  v_in = toggle_msggg.linear.x;
  w_in = toggle_msggg.angular.z;
}
ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("cmd_vel", messageCb );

nav_msgs::Odometry odo_type;
ros::Publisher sub_publish("odomdata", &odo_type);

class Motor{

  public:
    Motor(double m_dt, int m_dir_pin1, int m_dir_pin2, int m_pwm_pin,int m_read_pin1,int m_read_pin2);
    Encoder* motorencoder;
    double target_rpm_l;
    double target_rpm_r;
    double target_rpm;
    double measured_rpm;
    double last_err;
    double dt,dtt;
    double err, err_i, err_d;
    double kp, kd, ki;
    double output;
    long pulse;
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

Motor m1(0.04, 10, 11, 9, 14, 15), m2(0.04, 6, 7, 5, 16, 17);   

Motor::Motor(double m_dt, int m_dir_pin1, int m_dir_pin2, int m_pwm_pin,int m_read_pin1,int m_read_pin2){
  motorencoder = new Encoder(m_read_pin1,m_read_pin2);
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



void Motor::readdata(){
  pulse = motorencoder->read();
  measured_rpm = (pulse*1/dt*60)/13;
  motorencoder->write(0);
}

double get_estimate_V(){
  double result;
  result = (m1.target_rpm_l/30*3.14159*0.065/4/60)/2+(m1.target_rpm_r/30*3.14159*0.065/4/60)/2;
  return result;
}

double get_estimate_W(){
 double result;
  result = (m1.target_rpm_l/30*3.14159*0.065/4/60/ 0.081)/2 + (-m1.target_rpm_r/30*3.14159*0.065/4/60/ 0.081)/2;
  return result;
}

double get_really_V(){
  double result;
  result = (m1.measured_rpm/30*3.14159*0.065/4/60)/2+(m2.measured_rpm/30*3.14159*0.065/4/60)/2;
  return result;
}

double get_really_W(){
  double result;
  sensors_event_t angVelocityData;
  bno.getEvent(&angVelocityData,Adafruit_BNO055::VECTOR_GYROSCOPE);
  result = angVelocityData.gyro.z;
  return result;
}
double displacement =0;
double rad =0;
double x_pos =0;
double y_pos =0;
  
void setup() { 
  nh.initNode();
  nh.subscribe(sub_cmd_vel);
  nh.advertise(sub_publish);
  //Serial.begin(115200);
  if(!bno.begin())
  {
    while(1);
  }
  delay(1000);
}

void loop() {
  
  double estimateV = get_estimate_V();
  double estimateW = get_estimate_W();
  double reallyV = get_really_V();
  double reallyW = get_really_W();
  
  displacement = estimateV *(m1.dt);
  rad = rad + estimateW *(m1.dt);
  x_pos = x_pos + reallyV*cos(rad);
  y_pos = y_pos + reallyV*sin(rad);
  
  
  odo_type.twist.twist.linear.x = estimateV;
  odo_type.twist.twist.linear.y = x_pos;
  odo_type.twist.twist.linear.z = y_pos;
  odo_type.twist.twist.angular.z = estimateW ;
  
  sub_publish.publish( &odo_type );
  
  m1.inputdata();
  m1.readdata();
  m2.readdata();
   
  m1.setupPID(0.02, 0.6, 0.00000001);
  m2.setupPID(0.02, 0.6, 0.00000001);

  m1.setRPMs(m1.target_rpm_l, m1.measured_rpm);
  m2.setRPMs(m1.target_rpm_r, m2.measured_rpm);

  m1.computeError();
  m2.computeError();

  m1.controlByPID();
  m2.controlByPID();
  
  nh.spinOnce();
  delay(m1.dtt);
  
//Serial.printf("m1 cur rpm: %.2f  ,m2 cur rpm: %.2f  ,m1 tar rpm: %.2f  ,m2 tar rpm: %.2f\n", m1.measured_rpm,m2.measured_rpm,m1.target_rpm_l,m1.target_rpm_r);
//Serial.printf("angv: %.2f\n", angvel);
;
}
