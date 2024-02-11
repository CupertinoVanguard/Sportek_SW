#include <ArduinoBLE.h>
#include <Wire.h>
//y axis
#define INTEREST_AXES 2

BLEService sensorService("19B10013-E8F2-537E-4F6C-D104768A1214");
BLEIntCharacteristic interestGyr("19B10011-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEIntCharacteristic interestAngle("19B10012-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);

/*Vars bby*/
const float GYRO_SENSITIVITY_SCALE_FACTOR = 131.0; //for 200 degrees / sec
const int MPU6050_ADDR = 0b1101000;
const byte PWR_REG_ADDR = 0x6B;
const byte GYRO_CONFIG_REG_ADDR = 0x1B;
const byte GYRO_READ_START_ADDR = 0x43;

float gyrVals[3];
float accVals[3];
float offsets[3]; 
float offsets_accl[3];
float rotX;
float prevRotX=0;
float netAngles[3];
float accAngles[2] = {5, 5};

int i;
unsigned long prevTime=0,currentTime;
float offsetVal;
const int noOfSamplesForOffset = 1000;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while(!Serial); 
  if(!BLE.begin()){
    while(1);
  }

  BLE.setLocalName("Elbow BLE");
  BLE.setAdvertisedService(sensorService);
  sensorService.addCharacteristic(interestGyr);
  sensorService.addCharacteristic(interestAngle);
  BLE.addService(sensorService);

  interestGyr.writeValue(0);
  interestAngle.writeValue(0); 
  
  /*offsetting*/
  gyroSetup();
  float totalValX = 0;
  float totalValY = 0;
  float totalValZ = 0;
  for (int i=0; i< noOfSamplesForOffset; i++){
    readGyroX();
    totalValX += (gyrVals[0]);
    totalValY += gyrVals[1];
    totalValZ += gyrVals[2];
  }
  offsets[0] = totalValX/1000;
  offsets[1] = totalValY/1000;
  offsets[2] = totalValZ/1000;

  totalValX = 0;
  totalValY = 0;
  totalValZ = 0;
  for (int k = 0; k < noOfSamplesForOffset; k++){
    readAccData();
    float accAngleX = (atan2(accVals[1], accVals[2]) * 180 / PI); //maybe offset could be fixed)
    float accAngleY = (atan2(-1*accVals[0], accVals[2])*180/PI);
    totalValX += accAngleX;
    totalValY += accAngleY;
  }
  offsets_accl[0] = totalValX/noOfSamplesForOffset;
  offsets_accl[1] = totalValY/noOfSamplesForOffset;

  prevTime = millis();
  
  BLE.advertise();
}

void loop() {
  // put your main code here, to run repeatedly:
  BLEDevice central = BLE.central();
  if (central){
    Serial.println(central.address());
    while(central.connected()){
      
      updateAngleVals();
      
      interestGyr.writeValue((int)gyrVals[INTEREST_AXES]);
      interestAngle.writeValue((int)netAngles[INTEREST_AXES]);
      
      delay(1);
    }
    Serial.print(F("Disconnected from central: "));
    Serial.println(central.address());
  }
}
void updateAngleVals(){
   readGyroX(); //transition this into interrupt based read, running updates on its own time so likely to get better more updated values?
   readAccData();
   currentTime = millis();
   double dt = (currentTime - prevTime)/1000.0;
   prevTime = currentTime;
   //Serial.println(sqrt(pow(accVals[0],2) + pow(accVals[2],2)));
   float accAngleX = (atan2(accVals[1], accVals[2]) * 180 / PI) - offsets_accl[0]; //maybe offset could be fixed)
   float accAngleY = (atan2(-1*accVals[0], accVals[2]) * 180/PI) - offsets_accl[1];
   for (int y = 0; y < 3; y++){
      netAngles[y] = netAngles[y] + (gyrVals[y] - offsets[y])*dt;
   }
   if (abs(netAngles[0] - accAngleX) <= 5){
      netAngles[0] = (0.9*netAngles[0]) + (0.1*accAngleX);
   }
}
void readGyroX(){
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x43);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 6,true);  
  int16_t gyroX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  int16_t gyroY=Wire.read()<<8|Wire.read();
  int16_t gyroZ=Wire.read()<<8|Wire.read();
  gyrVals[0] = gyroX/131.0;
  gyrVals[1] = gyroY/131.0;
  gyrVals[2] = gyroZ/131.0;
  //return gyroX / GYRO_SENSITIVITY_SCALE_FACTOR;
}
void readAccData(){
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0X3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 6, true);
  float acc_x = Wire.read()<<8|Wire.read();
  float acc_y = Wire.read()<<8|Wire.read();
  float acc_z = Wire.read()<<8|Wire.read();
  accVals[0] = acc_x/16384.0;
  accVals[1] = acc_y/16384.0;
  accVals[2] = acc_z/16384.0;
}
void gyroSetup(){
 Wire.begin();
 Wire.beginTransmission(0x68);
 Wire.write(0x6B);//Access the power register
 Wire.write((byte)0x00);
 Wire.endTransmission();

  //Gyro Config
 Wire.beginTransmission(0x68);
 Wire.write(0x1B);
 Wire.write((byte)0x00);
 Wire.endTransmission();
}
