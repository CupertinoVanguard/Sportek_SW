//Assumptions of this code
/*
 * single axis dependence on both sensors
 * Begin indexing only when the latest value comes in and then close up whent the earliest comes in?
 * 3-state state machine - all processing inside main controlller - signal recognitioon on the the main microcontroller
 */
#include <ArduinoBLE.h>

#define MAX_ITER 10
#define ELBOW 0
#define SHOULDER 1
#define FIR_SIZE 10
#define BUFFER_SIZE 10
#define TRIGGER CRITERION 0.
const char* SHOULDER_UUID = "19B10016-E8F2-537E-4F6C-D104768A1214";
const char* ELBOW_UUID = "19B10013-E8F2-537E-4F6C-D104768A1214";
const char* FEEDBACK_CHAR_UUID_ANGLS = "19B10012-E8F2-537E-4F6C-D104768A1214";
const char* FEEDBACK_CHAR_UUID_ACCLS = "19B10011-E8F2-537E-4F6C-D104768A1214";

BLECharacteristic feedback_keys[2]; //why can't I make it volatile?

int feedback_values[2];
BLECharacteristic characteristicFind(BLEDevice* dev, const char* feedback_ID){
  if (dev->discoverAttributes()){
    return dev->characteristic(feedback_ID);
  }
  //else what though??
}
int triggerDetect(int* buffer){
    for(int i = 1; i < BUFFER_SIZE; i++){
      if (fabs(buffer[i-1] - buffer[i]) > TRIGGER_CRITERION){
        return i;
      }
    }
    return -1;
}
bool deviceConnect(BLEDevice* dev, int maxIterConnect){
  bool sensor_connect = dev->connect();
  int max_it = 0;
  while(!sensor_connect && max_it <= maxIterConnect){
    sensor_connect = dev->connect();
    max_it++;
  }
  if (max_it == maxIterConnect){
    return false; //what is the alternative to NULL
  }
  return sensor_connect;
}
void setup() {
  Serial.begin(9600);
  while(!Serial);

  BLE.begin();

  BLE.scanForUuid(ELBOW_UUID);
  BLEDevice elbow_device = BLE.available();
  while(!elbow_device){
    BLE.scanForUuid(ELBOW_UUID);
    elbow_device = BLE.available();
  }
  BLE.stopScan();
  if (deviceConnect(&elbow_device, MAX_ITER)){
    BLECharacteristic charac = characteristicFind(&elbow_device, FEEDBACK_CHAR_UUID);
    if (charac){
      feedback_keys[0] = charac;
    }
  }

  BLE.scanForUuid(SHOULDER_UUID);
  BLEDevice shoulder_device = BLE.available();
  while(!shoulder_device){
    BLE.scanForUuid(ELBOW_UUID);
    shoulder_device = BLE.available();
  }
  BLE.stopScan();
  if (deviceConnect(&shoulder_device, MAX_ITER)){
    BLECharacteristic charac_dev = characteristicFind(&shoulder_device, FEEDBACK_CHAR_UUID);
    if (charac_dev){ //no isValid?
      feedback_keys[1] = charac_dev;
    }
  }

}

void loop() {
  if (charsValid()){
    /*
    Should be buffer store logic right here right?, but for now print?
    */
    feedback_values[ELBOW] = readVal(ELBOW);
    feedback_values[SHOULDER] = readVal(SHOULDER);
  }
  Serial.print(feedback_values[ELBOW]);
  Serial.print(" ");
  Serial.print(feedback_values[SHOULDER]);
  Serial.println();
} 

bool charsValid(){
  bool checkVal = true;
  for (int i = 0; i < 2; i++){
    checkVal &= feedback_keys[i];
  }
  return checkVal;
}
int readVal(int index){
  feedback_keys[index].read();
  return byteArrayToInt(feedback_keys[index].value(), feedback_keys[index].valueLength());
}

union ArrayToInteger {
  byte array[4];
  uint32_t integer;
};
int byteArrayToInt(const byte data[], int length) {
  byte dataW[length];
  for (int i = 0; i < length; i++) {
       byte b = data[i]; 
        dataW[i] = data[i];    
  }
  ArrayToInteger converter; //Create a converter
  converter.array[0] = dataW[0];
  converter.array[1] = dataW[1];
  converter.array[2] = dataW[2];
  converter.array[3] = dataW[3];
  return converter.integer ;
}