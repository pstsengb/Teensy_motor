#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#define timeunit (40)
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

double rad=0;
void setup(void)
{
  Serial.begin(115200);
  if(!bno.begin())
  {
    while(1);
  }
  delay(1000);
}

void loop(void){
  sensors_event_t angVelocityData;
  bno.getEvent(&angVelocityData,Adafruit_BNO055::VECTOR_GYROSCOPE);
  rad = rad +angVelocityData.gyro.z*(timeunit/1000.0);
  Serial.printf("%.2f,%.2f\n",angVelocityData.gyro.z,rad);
  delay(timeunit);
}
