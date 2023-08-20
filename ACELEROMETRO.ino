#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <BluetoothSerial.h>
BluetoothSerial SerialBT;
#define BNO055_SAMPLERATE_DELAY_MS (10)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
int c_ms = 0;
void setup(void)
{
  Serial.begin(9600);
  SerialBT.begin("ESP32_ACCELERACIÓN");
  while (!Serial) delay(10);  // wait for serial port to open!

  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void)
{
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  imu::Vector<3> ACC = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  /* Display the floating point data */
  float x = ACC.x();
  Serial.print (c_ms);
  SerialBT.print(c_ms);
  Serial.print(";X:");
  Serial.print(x);
  Serial.print(";");
  SerialBT.print(";X:");
  SerialBT.print(x);
  SerialBT.print(";");

  float y = ACC.y();
  Serial.print(" Y:");
  Serial.print(y);
  Serial.print(";");
  SerialBT.print("Y:");
  SerialBT.print(y);
  SerialBT.print(";");

  float z = ACC.z();
  Serial.print(" Z:");
  Serial.print(z);
  Serial.print(";");
  SerialBT.print("Z:");
  SerialBT.print(z);
 SerialBT.print(";");
  
  Serial.print("\t");
  
 
  /* Display calibration status for each sensor. */
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
 
  Serial.print(" Cal_Acc=");
  Serial.println(accel, DEC);

  SerialBT.print("AC");
  SerialBT.println(accel); 
   
  delay(BNO055_SAMPLERATE_DELAY_MS);
  c_ms += 1; 
}
