 //Assumptions of this code
/*
 * single axis dependence on both sensors
 * Begin indexing only when the latest value comes in and then close up whent the earliest comes in?
 * 3-state state machine - all processing inside main controlller - signal recognitioon on the the main microcontroller
 */
#include <ArduinoBLE.h>

#define MAX_ITER 10
#define ANGLE_SIZE 100
#define ELBOW 0
#define SHOULDER 1
#define FIR_SIZE 10
#define BUFFER_SIZE 10
#define TRIGGER_CRITERION 0.5
#define FIR_BUFFER_LENGTH 10
#define POSITIVE true
#define NEGATIVE false
#define SNAP_COUNT 3

//Values for the Array References
#define ELBOW_GYR 0
#define ELBOW_ANGL 1
#define SHOULDER_GYR 2
#define SHOULDER_ANGL 3

typedef enum{
  IDLE,
  START_TRIGGER,
  SIGN_TRIGGER,
  STOP_TRIGGER,
  FEEDBACK_GENERATION
} DATA_COLLECT_STATE;

const char* SHOULDER_UUID = "19B10016-E8F2-537E-4F6C-D104768A1214";
const char* ELBOW_UUID = "19B10013-E8F2-537E-4F6C-D104768A1214";
const char* FEEDBACK_CHAR_UUID_ANGLS = "19B10012-E8F2-537E-4F6C-D104768A1214";
const char* FEEDBACK_CHAR_UUID_GYR = "19B10011-E8F2-537E-4F6C-D104768A1214";

BLECharacteristic feedback_keys[4]; //why can't I make it volatile?

float ANGLE_STORE[ANGLE_SIZE];
int num_angles = 0;

float FIR_WEIGHTS[FIR_BUFFER_LENGTH] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
float FIR_BUFFER[FIR_BUFFER_LENGTH];
int fir_pointer_1 = 0;
int fir_pointer_2 = 9;

bool running_sign = NEGATIVE;

//Single Circular buffer
float GYR_BUFF_1[BUFFER_SIZE];
int gyr_buf_ind_1 = 0;
int gyr_buf_ind_2 = BUFFER_SIZE - 1;

int feedback_values[4];
bool fir_buffer_met = false;
int num_values_added = 0;
int snap_count = 0;

DATA_COLLECT_STATE movement_state;
bool start_trigger = false;
bool end_trigger = false; 
bool add_angles = false;
int ANGLE_INTERM_BUFF[BUFFER_SIZE];

int insertStopBuff(float value, float angleVal, int (*cond_detect)(float*)){
  if (gyr_buf_ind_1 == 0 && gyr_buf_ind_2 == BUFFER_SIZE - 1){
    add_angles = true;
    int condition_detected = cond_detect(GYR_BUFF_1);
    if (condition_detected != -1){
      return condition_detected;
    }
  }
  GYR_BUFF_1[gyr_buf_ind_1] = value;
  ANGLE_INTERM_BUFF[gyr_buf_ind_1] = angleVal;
  gyr_buf_ind_1++;
  gyr_buf_ind_2++;
  if (gyr_buf_ind_1 >= FIR_BUFFER_LENGTH){
    gyr_buf_ind_1 = 0;
  }
  if (gyr_buf_ind_2 >= FIR_BUFFER_LENGTH){
    gyr_buf_ind_2 = 0;
  }
  return -1;
}

int insertSingleBuff(float value, int (*cond_detect)(float*)){
  if (gyr_buf_ind_1 == 0 && gyr_buf_ind_2 == BUFFER_SIZE - 1){
    for (int i = 0; i < BUFFER_SIZE; i++){
      Serial.println(GYR_BUFF_1[i]);
    }
    int start_detected = cond_detect(GYR_BUFF_1);
    if (start_detected != -1){
      return start_detected;
    }
  }
  GYR_BUFF_1[gyr_buf_ind_1] = value;
  gyr_buf_ind_1++;
  gyr_buf_ind_2++;
  if (gyr_buf_ind_1 >= FIR_BUFFER_LENGTH){
    gyr_buf_ind_1 = 0;
  }
  if (gyr_buf_ind_2 >= FIR_BUFFER_LENGTH){
    gyr_buf_ind_2 = 0;
  }
  return -1;
}
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
int triggerStartDetect(float* buffer){
    for(int i = 3; i < BUFFER_SIZE; i++){
      if (fabs(buffer[i-3] - buffer[i]) > TRIGGER_CRITERION){
        if (buffer[i - 3] - buffer[i] > 0){
          running_sign = !running_sign;
        }
        return i;
      }
    }
    return -1;
}
int signDetect(float* buffer){
  for (int i = 3; i < BUFFER_SIZE; i++){
    if (buffer[i-3] - buffer[i] < 0 && running_sign){
      return i;
    }else if (buffer[i - 3] - buffer[i] > 0 && !running_sign){
      return i;
    }
  }
}
int triggerStopDetect(float* buffer){
    for(int i = 1; i < BUFFER_SIZE; i++){
      if (fabs(buffer[i-1] - buffer[i]) > TRIGGER_CRITERION){
        return i;
      }
    }
    add_angles = true;
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
int readVal(int index){
  feedback_keys[index].read();
  return byteArrayToInt(feedback_keys[index].value(), feedback_keys[index].valueLength());
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
    Serial.println("Still searching");
  }
  Serial.println(elbow_device);
  Serial.println("its connected");
  BLE.stopScan();
  if (deviceConnect(&elbow_device, MAX_ITER)){
    BLECharacteristic charac_gyr= characteristicFind(&elbow_device, FEEDBACK_CHAR_UUID_GYR);
    BLECharacteristic charac_angl = characteristicFind(&elbow_device, FEEDBACK_CHAR_UUID_ANGLS);
    if (charac_gyr & charac_angl){
      feedback_keys[0] = charac_gyr;
      feedback_keys[1] = charac_angl;
    }
  }
  movement_state = IDLE;
}
void loop(){
  if (charsValid()){
    Serial.println(movement_state);
    switch (movement_state){
      case IDLE:
        if (num_values_added < FIR_BUFFER_LENGTH){
          FIR_BUFFER[num_values_added] = readVal(ELBOW_GYR);
          num_values_added++;
        }else{
          movement_state = START_TRIGGER;
        }
        break;
      case START_TRIGGER:
        if (!start_trigger){
          Serial.print("THis is the state that I am in: ");
          Serial.println(movement_state);
          Serial.println("In here, start hasn't been detected yet");
          Serial.print("THis is the value that I am reading right now ");
          int val_test = readVal(ELBOW_GYR);
          Serial.println(val_test);
          insertFIRValue(readVal(ELBOW_GYR));
          int rec_index = insertSingleBuff(lowPass(), triggerStartDetect);
          Serial.println(rec_index);
          
          if (rec_index != -1){
            start_trigger = true;
            Serial.println("Right abot to switch out to sign trigger");
            movement_state = SIGN_TRIGGER;
            /*
             This buffer has to be cleared at each snap. 
             Snap Clear: for the start snap
            */
            for (int index = 0; index < BUFFER_SIZE; index++){
              GYR_BUFF_1[index] = 0;
            }
          }
        }else{
          insertFIRValue(readVal(ELBOW_GYR));
        }
        break;
      case SIGN_TRIGGER:
        //track angles along each snap and update the overall angles
        Serial.println("Inside SIGN_TRIGGER");
        if (num_angles >= ANGLE_SIZE){
          movement_state = FEEDBACK_GENERATION;
          Serial.println("Inside angle's expired condition");
        }else if (snap_count <= SNAP_COUNT){
          //Serial.println("The SNAP count is yet to hit condition");
          insertFIRValue(readVal(ELBOW_GYR));
          float angle_read = readVal(ELBOW_ANGL);
          Serial.print("This is the angle that was recorded: ");
          Serial.println(angle_read);
          Serial.print("This is the current running sign - Negative: false, Positive: True");
          Serial.println(running_sign);
          float val = insertStopBuff(lowPass(), angle_read, signDetect);
          if (add_angles){
            if (val != -1){
              snap_count++;
              //update overall angle array till val but considering circular buffer - don't have to since the check is only when the pointers circle back to start and end
              //just read to val here:
              for (int i = 0; i < val + 1 && num_angles < ANGLE_SIZE; i++){
                ANGLE_STORE[num_angles] = ANGLE_INTERM_BUFF[i];
                num_angles++;
              }
            }else{
              for (int i = 0; i < BUFFER_SIZE && num_angles < ANGLE_SIZE; i++){
                ANGLE_STORE[num_angles] = ANGLE_INTERM_BUFF[i];
                num_angles++;
              }
            }
            add_angles = !add_angles;
          }
        }else if(snap_count == SNAP_COUNT){
          Serial.println("The SNAP count has hit condition");
          movement_state = STOP_TRIGGER;
        }
        break;
      case STOP_TRIGGER:
        if (num_angles >= ANGLE_SIZE){
          movement_state = FEEDBACK_GENERATION;
        }else if (!end_trigger){
          insertFIRValue(readVal(ELBOW_GYR));
          float angle_read = readVal(ELBOW_ANGL);
          float val = insertStopBuff(lowPass(), angle_read, triggerStopDetect);
          if (add_angles){
            if (val != -1){
              //add only until val but gotta know which order right
              for (int i = 0; i < val + 1 && num_angles < ANGLE_SIZE; i++){
                ANGLE_STORE[num_angles] = ANGLE_INTERM_BUFF[i];
                num_angles++;
              }
            }else{
               for (int i = 0; i < BUFFER_SIZE && num_angles < ANGLE_SIZE; i++){
                  ANGLE_STORE[num_angles] = ANGLE_INTERM_BUFF[i];
                  num_angles++;
               }
            }
          }
        }else{
          //movement_state = FEEDBACK_GENERATION;
        }
        break;
      case FEEDBACK_GENERATION:
        //Serial.println(readVal(ELBOW_GYR));
        int d = 5;
        break;
    }
  }
}

/*
    each instance check if one of those is full, 
    the moment one of those is full, check for the trigger by passing in that array
    if a trigger is detected, gotta used a 4th fat buffer for value storage - how much tho???
    The buffer is gonna be cleared though shouldn't be a problem???

    States
    1) state one - entered if the trigger is sensed on either one right. Knowing this start collecting values. 
    Should start swiching the entire logic over to a statemachine as designed
    */

/*
void loop() {

  //Serial.println(charsValid());
  if (charsValid()){
    
    //Should be buffer store logic right here right?, but for now print?
    
    //Serial.println("The chars are valid");
    num_values_added++;
    if(num_values_added < FIR_BUFFER_LENGTH){
      FIR_BUFFER[num_values_added] = readVal(UUID);
    }else{
      insertBuffValue(lowPass());  
      insertFIRValue(readVal(UUID));
    }
    feedback_values[ELBOW] = readVal(ELBOW);
    
    //feedback_values[SHOULDER] = readVal(SHOULDER);
  }
  //Serial.println(feedback_values[ELBOW]);
  //Serial.print(" ");
  
  //Serial.print(feedback_values[SHOULDER]);
  
  //Serial.println();
}
*/
/*
void loop(){
  if (charsValid()){
    if (DATA_COLLECT_STATE == 1){
      if (num_values_added < FIR_BUFFER_LENGTH){ 
        FIR_BUFFER[num_values_added] = readVal(ELBOW_GYR);
        num_values_added++;
      }else if (!start_trigger){
        float lp_val_insert = insertSingleBuff(lowPass(), triggerStartDetect);
        if (lp_val_insert != -1){
          start_trigger = true;
          DATA_COLLECT_STATE = 2;
          for (int index = 0; index < BUFFER_SIZE; index++){
            GYR_BUFF_1[index] = 0;
          }
          //Do I have to insert new FIR value inside?
        }else{
          insertFIRValue(readVal(ELBOW_GYR));
        }
      }
    }else if(DATA_COLLECT_STATE == 2){
      if (num_angles >= ANGLE_SIZE){
        DATA_COLLECT_STATE = 5;
      }else if (!end_trigger){
        insertFIRValue(readVal(ELBOW_GYR));
        float angle_read = readVal(ELBOW_ANGL);
        if (snap_count < SNAP_COUNT){
          float val = insertStopBuff(lowPass(), angle_read, signDetect);
          if (val != -1 && add_angles){
            snap_count++; // it shouldn't be right. It should be on transition checks or does add_angles mean i've gotten a snap add the values till then?
          }
          if (add_angles){
            for (int i = 0; i < BUFFER_SIZE; i++){
              ANGLE_STORE[num_angles] = ANGLE_INTERM_BUFF[i];
              num_angles++;
            }
            add_angles = !add_angles;
          }
        }else{
          float val = insertStopBuff(lowPass(), angle_read, triggerStopDetect);
          if (val == -1){
            if (add_angles){ //this will lead to the first 10 points in the array being 0?
              for (int i = 0; i < BUFFER_SIZE; i++){
                ANGLE_STORE[num_angles] = ANGLE_INTERM_BUFF[i];
                num_angles++;
              }
              add_angles = !add_angles;
            }
          }else{
            for (int i = 0; i < val + 1; i++){
              ANGLE_STORE[num_angles] = ANGLE_INTERM_BUFF[i];
              num_angles++;
            }
            DATA_COLLECT_STATE = 3;
          }
        }
      }
    }else if (DATA_COLLECT_STATE == 3){

    }
  }
}
void loop(){
  if (charsValid()){
    if(DATA_COLLECT_STATE == 1){
      if (num_values_added < FIR_BUFFER_LENGTH){ 
        FIR_BUFFER[num_values_added] = readVal(ELBOW_GYR);
        num_values_added++;
      }else if (!start_trigger){
        float lp_val_insert = insertSingleBuff(lowPass());
        if (lp_val_insert != -1){
          start_trigger = true;
          DATA_COLLECT_STATE = 2;
          for (int index = 0; index < BUFFER_SIZE; index++){
            GYR_BUFF_1[index] = 0;
          }
          //Do I have to insert new FIR value inside?
        }else{
          insertFIRValue(readVal(ELBOW_GYR));
        }
      }
    }else if(DATA_COLLECT_STATE == 2){
      if (num_angles >= ANGLE_SIZE){
        DATA_COLLECT_STATE = 3;
      }else if (!end_trigger){
        insertFIRValue(readVal(ELBOW_GYR));
        float angle_read = readVal(ELBOW_ANGL);
        float val = insertStopBuff(lowPass(), angle_read);
        if (val == -1){
          if (add_angles){ //this will lead to the first 10 points in the array being 0?
            for (int i = 0; i < BUFFER_SIZE; i++){
              ANGLE_STORE[num_angles] = ANGLE_INTERM_BUFF[i];
              num_angles++;
            }
            add_angles = !add_angles;
          }
        }else{
          for (int i = 0; i < val + 1; i++){
            ANGLE_STORE[num_angles] = ANGLE_INTERM_BUFF[i];
            num_angles++;
          }
          DATA_COLLECT_STATE = 3;
        }
      } 
    }else if (DATA_COLLECT_STATE == 3){
      //ML Logic
      //send back signal to say that the feedback is received and send it back to state 1?
      DATA_COLLECT_STATE = 1;
    }
  }
}
*/
