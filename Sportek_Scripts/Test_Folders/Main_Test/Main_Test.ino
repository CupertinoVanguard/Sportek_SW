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
#define TRIGGER_CRITERION 0.5
#define FIR_BUFFER_LENGTH 10


#define ELBOW_GYR 0
#define ELBOW_ANGL 1
#define SHOULDER_GYR 2
#define SHOULDER_ANGL 3
     
const char* SHOULDER_UUID = "19B10016-E8F2-537E-4F6C-D104768A1214";
const char* ELBOW_UUID = "19B10013-E8F2-537E-4F6C-D104768A1214";
const char* FEEDBACK_CHAR_UUID_ANGLS = "19B10012-E8F2-537E-4F6C-D104768A1214";
const char* FEEDBACK_CHAR_UUID_GYR = "19B10011-E8F2-537E-4F6C-D104768A1214";

BLECharacteristic feedback_keys[4]; //why can't I make it volatile?


float FIR_WEIGHTS[FIR_BUFFER_LENGTH] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
float FIR_BUFFER[FIR_BUFFER_LENGTH];
int fir_pointer_1 = 0;
int fir_pointer_2 = 9;

int feedback_values[4];
bool fir_buffer_met = false;
int num_values_added = 0;

void insertFIRValue(float value){
  FIR_BUFFER[fir_pointer_1] = value;
  fir_pointer_1++;
  fir_pointer_2++;
  if (fir_pointer_1 >= FIR_BUFFER_LENGTH){
    fir_pointer_1 = 0;
  }
  if (fir_pointer_2 >= FIR_BUFFER_LENGTH){
    fir_pointer_2 = 0;
  }
}
float lowPass(){
  float lowPassValue = 0;
  for (int i = 0; i < FIR_BUFFER_LENGTH; i++){
    lowPassValue += (FIR_BUFFER[i] * FIR_WEIGHTS[i]);
  }
  return lowPassValue;
}
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
  Serial.println("Made it here");
  while(!elbow_device){
    Serial.println("In here searching for the device");
    BLE.scanForUuid(ELBOW_UUID);
    elbow_device = BLE.available();
  }
  BLE.stopScan();
  if (deviceConnect(&elbow_device, MAX_ITER)){
    Serial.println("Device connected");
    BLECharacteristic charac_gyr= characteristicFind(&elbow_device, FEEDBACK_CHAR_UUID_GYR);
    BLECharacteristic charac_angl = characteristicFind(&elbow_device, FEEDBACK_CHAR_UUID_ANGLS);
    if (charac_gyr && charac_angl){
      feedback_keys[0] = charac_gyr;
      feedback_keys[1] = charac_angl;
    }
  }

  /*  
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
  }*/

}
int readVal(int index){
  feedback_keys[index].read();
  return byteArrayToInt(feedback_keys[index].value(), feedback_keys[index].valueLength());
}

void loop() {
  if (charsValid()){
    num_values_added++;
    if(num_values_added < FIR_BUFFER_LENGTH + 1){
      FIR_BUFFER[num_values_added] = readVal(ELBOW_ANGL);
      Serial.print("Adding");
    }else{
      insertFIRValue(readVal(ELBOW_ANGL));
      Serial.println(lowPass());
    }
    for (int i = 0; i < FIR_BUFFER_LENGTH; i++){
      Serial.print(FIR_BUFFER[i]);
      Serial.print(", ");
    }
    Serial.println();
    //feedback_values[ELBOW] = readVal(ELBOW);
  }
} 

bool charsValid(){
  bool checkVal = true;
  for (int i = 0; i < 2; i++){
    checkVal |= feedback_keys[i]; //have to change to and
  }
  return checkVal;
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
