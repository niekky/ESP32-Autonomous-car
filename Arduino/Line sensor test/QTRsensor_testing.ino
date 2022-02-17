// Thư viện đèn
#include <QTRSensors.h>

QTRSensors qtr;
QTRSensors qtr2;

uint16_t position=0;
uint16_t position2=0;
float previous_error = 0, previous_I = 0;
const uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];
uint16_t sensorValues2[SensorCount];
// Servo steering;

unsigned long previousTime=0;
unsigned long previousTime2=0;


/////////////////////////////////////////////////////////////////////////////////////////

void SensorCalibrate(){
  // configure the sensors
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){16, 17, 18, 19, 21, 25, 26, 27,32,33}, 10);
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(2, LOW); // turn off Arduino's LED to indicate we are through with calibration

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
  qtr.setSensorPins((const uint8_t[]){25,26,27,32,33}, SensorCount);
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(2, LOW); // turn off Arduino's LED to indicate we are through with calibration

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
  qtr2.setSensorPins((const uint8_t[]){16, 17, 18, 19, 21, 25, 26, 27,32,33}, SensorCount);

  delay(500);
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr2.calibrate();
  }
  digitalWrite(2, LOW); // turn off Arduino's LED to indicate we are through with calibration

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

void setup()
{
  Serial.begin(115200);
  // SensorCalibrate1();
  // SensorCalibrate2();
  SensorCalibrate();
}

void loop()
{
  // Đọc inputs từ Firebase
  position = qtr.readLineBlack(sensorValues);
  Serial.println("Sensor: "+String(position));
  // position2= 4000-qtr2.readLineBlack(sensorValues2);
}