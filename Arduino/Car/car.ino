#include <QTRSensors.h>
#include <Servo.h>
//Cách QTR hoạt động
//SETUP:
//10s cho việc calibrate sensor
//loop sẽ chạy từ 0-5000
QTRSensors qtr;
QTRSensors qtr2;
double kp=0;
double ki=0;
double kd=0;
int SERVOTURN=0;
int error=0;
int previouserror=0;
int angleturn=0;
uint16_t position=0;
uint16_t position2=0;
float previous_error = 0, previous_I = 0;
byte motor[4]={3,5,6,7}; 
const uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];
uint16_t sensorValues2[SensorCount];
Servo steering;

/////////////////////////////////////////////////////////////////////////////////////////

class newpidConfig(){
  public:
    newpidConfig(float kpp,float kip,float kdp){
      kp=kpp;
      ki=kip;
      kd=kdp;
    }

    float getKP(){
      return kp;
    }

    float getKD(){
      return kd;
    }

    float getKI(){
      return ki
    }

    float getError(){
      return error
    }

    int PIDloop(int error){
      output=kp*error + kd*(error - previous_error);
      previous_error = error;
      return output;
    }

  private:
    float kp;
    float ki;
    float kd;
    int previous_error=0;
}

float pidConfig(float p,float i,float d){
  kp=p;
  ki=i;
  kd=d;
}

void ServoDefault(){
  steering.write(90);
}

void readline(){
    Serial.print(position);
    Serial.print('\t');
    Serial.print(position2);
    Serial.print('\t');
    Serial.print(error);
    delay(250);
}

void SensorCalibrate(){
  // configure the sensors
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){14,15,16,17,18,11,10,9,8,4}, 10);
  qtr.setEmitterPin(2);

  delay(500);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(13, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
}

void SensorCalibrate1(){
  // configure the sensors
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){14,15,16,17,18}, SensorCount);
  qtr.setEmitterPin(2);

  delay(500);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(13, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
}

void SensorCalibrate2(){
  // configure the sensors
  qtr2.setTypeRC();
  qtr2.setSensorPins((const uint8_t[]){11,10,9,8,4}, SensorCount);
  qtr2.setEmitterPin(2);

  delay(500);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr2.calibrate();
  }
  digitalWrite(13, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr2.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();
  
  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr2.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
}


//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
  steering.attach(motor[0]);
  ServoDefault();
  for (int i=0; i<4; i++){
    pinMode(motor[i],OUTPUT);
  }
  Serial.begin(9600);
  SensorCalibrate1();
  SensorCalibrate2();
  pidConfig(0.1,0,0); //0.02 0 0.01
}

void loop()
{
  digitalWrite(motor[2],1);
  digitalWrite(motor[3],0);
  analogWrite(motor[1],180);
  position = qtr.readLineBlack(sensorValues);
  position2= 4000-qtr2.readLineBlack(sensorValues2);
  Serial.print(position);
  Serial.print(" ");
  Serial.println(position2);
  error=position2-position;
  SERVOTURN = kp*error + kd*(error - previouserror);
  previouserror = error;
  angleturn=SERVOTURN;
  if (angleturn>40) angleturn=40;
  if (angleturn<-30) angleturn=-30;
  steering.write(90-angleturn);
  
  readline();
}
