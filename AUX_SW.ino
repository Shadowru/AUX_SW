#include <SoftwareSerial.h>

#include "./mavlink/common/mavlink.h"
#include "MedianFilter.h"
//#include "./sonar/Sonar.h"

//UART
#define RX_PIN 6
#define TX_PIN 5

SoftwareSerial SerialMAV(RX_PIN, TX_PIN, false);

//mv buffer
uint8_t buf[MAVLINK_MAX_PACKET_LEN];
int packet_sequence = 0;

//MAVLINK MODES
#define MANUAL_MODE 0
#define AUTO_MODE 10


//MAVLINK CONSTANTS
#define CENTER_STEER 1500
#define CENTER_SPEED 1500

//SONAR 
#define SONAR_INTERVAL 20000
//SONAR FOV radians
#define SONAR_HFOV 0.174533
#define SONAR_VFOV 0.174533

const int SONAR_THRESHOLD = 30;
unsigned long sonarTimer = 0;

const int MIN_DISTANCE = 20;
const int MAX_DISTANCE = 3000;

//SONAR 1
#define SONAR1_TRIG_PIN 12
#define SONAR1_ECHO_PIN 11
MedianFilter sonar_filter_1(3, 0);

//SONAR 2
#define SONAR2_TRIG_PIN 10
#define SONAR2_ECHO_PIN 9
MedianFilter sonar_filter_2(3, 0);

//BUMPER
#define BUMPER_INTERVAL 20000

#define BUMPER_TRIG_PIN 7

const boolean DEBUG_FLAG = true;

unsigned long bumperTimer = 0;


// WORK MODES
const int INIT_MODE = 1;
const int IDLE_MODE = 2;
const int DRIVE_MODE = 3;
const int DRIVE_BUMPERHIT_MODE = 4;

// WORK MODE
int current_work_mode = INIT_MODE;

//EVASIVE MANUVER
#define EVASIVE_STEER 1700
#define EVASIVE_SPEED 1300
//msecs
#define EVASIVE_INTERVAL 10000000

const bool evasiveManeuverEnabled = true;
bool evasiveManeuverInit = true;
bool evasiveManeuverFinished = false;
unsigned long evasiveTimer = 0;

//=======================================================
void setup() {
  //Init SONAR 1
  pinMode(SONAR1_TRIG_PIN, OUTPUT); 
  pinMode(SONAR1_ECHO_PIN, INPUT);    
  //Init SONAR 1
  pinMode(SONAR2_TRIG_PIN, OUTPUT); 
  pinMode(SONAR2_ECHO_PIN, INPUT);    
  //Init BUMPER
  pinMode(BUMPER_TRIG_PIN, INPUT_PULLDOWN); 
  // init timer
  sonarTimer = micros();
  bumperTimer = micros();
  
  //DEBUG INTERFACE
  if(DEBUG_FLAG){
    Serial.begin(115200);
    Serial.println("AUX SW v0.1");
  }
  
  //Mavlink serial init
  SerialMAV.begin(115200);

  //Start
  changeMode(IDLE_MODE);
}

//=======================================================
void loop() {

  switch(current_work_mode){
    case IDLE_MODE:
      doIdleMode();
      break;
    case DRIVE_MODE:
      doDriveRoutine();      
      break;
    case DRIVE_BUMPERHIT_MODE:
      doBumperHitRoutine();      
      break;
    default:
      break;
  }
  delay(10);
}

//=======================================================
//DRIVE MODES

void doIdleMode(){
  sendInfoMessage("AUX SW v0.1");
  delay(500);
  stopMotors();
  delay(500);
  disarm();
  delay(500);
  setAutoPilotMode(MANUAL_MODE);
  delay(500);
  changeMode(DRIVE_MODE);
}

void doDriveRoutine(){
  mav_distance_sensor(3, MAX_DISTANCE-1);            

  //BUMPER HIT
  if(doBumperHit()){
    
    changeMode(DRIVE_BUMPERHIT_MODE);
    bumperTimer = micros();
    
    if(evasiveManeuverEnabled){
      initEvasive();
    }
    
    return;
  }

  if(sonarTimer < micros()){
    sonarTimer = micros() + SONAR_INTERVAL;
    doSonar();
  }

}

void doBumperHitRoutine(){
 
  mav_distance_sensor(3, MIN_DISTANCE + 1);
  
  if(bumperTimer < micros()){
    bumperTimer = micros() + BUMPER_INTERVAL;

    if(evasiveManeuverEnabled){
      if(evasiveTimer > micros()){
        //Serial.print(evasiveTimer);
        //Serial.print(" - ");
        //Serial.print(micros());
        doEvasiveManeuver();
        return;
      } else {
        doEvasiveManeuverFinished();
      }
    }
    
    Serial.println("Check bumper");    
    if(isBumperHit(BUMPER_TRIG_PIN)){
      //BUMPER STILL HIT
      emergencyStop();
    } else {
      mav_distance_sensor(3, MAX_DISTANCE-1);            
      changeMode(DRIVE_MODE);
    }

  }
}

//======================================================
//EVASIVE FUNCTIONS

void initEvasive(){
  evasiveManeuverInit = true;
  evasiveTimer = micros() + EVASIVE_INTERVAL;
  evasiveManeuverFinished = false;
}

void doEvasiveManeuver(){

  if(evasiveManeuverInit){
    disarm();
    delay(100);
    setAutoPilotMode(MANUAL_MODE);
    delay(200);
    arm();
    evasiveManeuverInit = false;
    evasiveManeuverFinished = false;
    sendInfoMessage("Init EVM");
  } else {
    evasiveManeuver();     
  }
}

void evasiveManeuver(){
  rc_override(EVASIVE_STEER, EVASIVE_SPEED);
}

void doEvasiveManeuverFinished(){
  if(evasiveManeuverFinished){
    //
  } else{
    if(DEBUG_FLAG){
      Serial.println("Stop EMV");
    }
    sendInfoMessage("Stop EMV");
    evasiveManeuverFinished = true;
    mav_distance_sensor(3, MIN_DISTANCE + 1);
    delay(100);
    stopMotors();
    mav_distance_sensor(3, MIN_DISTANCE + 1);
    delay(100);
    mav_distance_sensor(3, MIN_DISTANCE + 1);
    disarm();
    mav_distance_sensor(3, MIN_DISTANCE + 1);
    delay(100);
    mav_distance_sensor(3, MIN_DISTANCE + 1);
    setAutoPilotMode(AUTO_MODE);
    delay(300);
    mav_distance_sensor(3, MIN_DISTANCE + 1);
    arm();
  }
}

//=======================================================
//BUMPER FUNCTIONS
boolean doBumperHit(){
  if(isBumperHit(BUMPER_TRIG_PIN)){
    if(DEBUG_FLAG){
      Serial.println("BUMPER HIT");
    }
    emergencyStop();
    mav_distance_sensor(3, MIN_DISTANCE + 1);
    sendEmergencyMessage("BUMPER HIT");
    return true;
  }
  return false;
}

boolean isBumperHit(int bumper_pin){
  boolean isBumperHit = digitalRead(bumper_pin) == 0;
  //TODO: filter contact bounce
  return isBumperHit;
}

//=======================================================
//SONAR FUNCTIONS
void doSonar(){

    int sonar_1_distance = readSonar(SONAR1_TRIG_PIN, SONAR1_ECHO_PIN);
    sonar_filter_1.in(sonar_1_distance);
    sonar_1_distance = sonar_filter_1.out(); 

    int sonar_2_distance = readSonar(SONAR2_TRIG_PIN, SONAR2_ECHO_PIN);
    sonar_filter_2.in(sonar_2_distance);
    sonar_2_distance = sonar_filter_2.out(); 

    if(sonarDataValid(sonar_1_distance, sonar_2_distance)){
      
      if(DEBUG_FLAG){
        Serial.print("SONAR 1 DIST : ");
        Serial.print(sonar_1_distance);
        Serial.println();
        Serial.print("SONAR 2 DIST : ");
        Serial.print(sonar_2_distance);
        Serial.println();
      }
      
      mav_distance_sensor(1, sonar_1_distance);
      mav_distance_sensor(2, sonar_2_distance);
      
    } else {
      
      if(DEBUG_FLAG){
        Serial.println("SONAR UNRELIABLE");
      }
      
    }
}

boolean sonarDataValid(int sonar_1_distance, int sonar_2_distance){
    int minDistanceSonar = min(sonar_1_distance, sonar_2_distance);
    int maxDistanceSonar = max(sonar_1_distance, sonar_2_distance);

    if(minDistanceSonar < SONAR_THRESHOLD){
      return false;
    }

    if(maxDistanceSonar > MAX_DISTANCE){
      return false;
    }

    return true;
}

int readSonar(int trigger_pin, int echo_pin){
  digitalWrite(trigger_pin, LOW); 
  delayMicroseconds(2); 
  digitalWrite(trigger_pin, HIGH); 
  delayMicroseconds(10); 
  digitalWrite(trigger_pin, LOW); 
  int duration = pulseIn(echo_pin, HIGH,50000); 
  int cm = duration / 58;
  return cm;  
}

//=======================================================
//DRIVE FUNCTIONS

void emergencyStop(){
  if(DEBUG_FLAG){
    Serial.println("EMERGENCY STOP");
  }
  sendEmergencyMessage("EMERGENCY STOP");
  stopMotors();
}

void stopMotors(){
  rc_override(1500, 1500);
}

void disarm(){
  mav_arm_pack(false);
}

void arm(){
  mav_arm_pack(true);
}

//=======================================================
// MESSAGE LAYER
void sendEmergencyMessage(String message){
  mavlink_message(MAV_SEVERITY_EMERGENCY, message);
}

void sendErrorMessage(String message){
  mavlink_message(MAV_SEVERITY_ERROR, message);
}

void sendNoticeMessage(String message){
  mavlink_message(MAV_SEVERITY_NOTICE, message);
}

void sendInfoMessage(String message){
  mavlink_message(MAV_SEVERITY_INFO, message);
}

// SERVICE LAYER

void changeMode(int mode){
   current_work_mode = mode;
   if(DEBUG_FLAG){
    Serial.println("SET MODE : " + String(mode)); 
   }
   sendInfoMessage("MODE:" + String(mode));
}

void rc_override(int mav_steer, int mav_speed){
  mavlink_message_t msg;
  
  mavlink_msg_rc_channels_override_pack(
    0xFF, 0x54, &msg, 0, 0,
    mav_steer, 
    65535, 
    mav_speed,
    65535,65535,65535,65535,65535,0,0,0,0,0,0,0,0,0,0
  );    
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SerialMAV.write(buf, len);
}

// distance - mm
void mav_distance_sensor(int sensor_id, int distance){
  int covariance = 255;
  
  float quaternion[4]; 
  quaternion[0] = 1;
  quaternion[1] = 0;
  quaternion[2] = 0;
  quaternion[3] = 0;
  
  mavlink_message_t msg;

  mavlink_msg_distance_sensor_pack(0x01, MAV_COMP_ID_OBSTACLE_AVOIDANCE, &msg, 0, 
    MIN_DISTANCE, MAX_DISTANCE, distance,
    MAV_DISTANCE_SENSOR_ULTRASOUND, sensor_id,
    MAV_SENSOR_ROTATION_NONE, covariance,
    SONAR_HFOV, SONAR_VFOV,
    quaternion
  );
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SerialMAV.write(buf, len);
}

void mavlink_message(int severity, String message){
  mavlink_message_t msg;
  
  int str_len = message.length()+1;
  if(str_len > 50){
    str_len = 50;
  }
  
  char message_buf[str_len];
  message.toCharArray(message_buf, str_len);
  
  mavlink_msg_statustext_pack(0x01, MAV_COMP_ID_OBSTACLE_AVOIDANCE, &msg, severity, message_buf);
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SerialMAV.write(buf, len);
}


void mav_arm_pack(boolean state) {
  mavlink_message_t msg;

  // 1 an 8'th argument is for ARM (0 for DISARM)
  if(state) {
    //ARM
    mavlink_msg_command_long_pack(0xFF, 0xBE, &msg, 1, 1, 400, 1,1.0,0,0,0,0,0,0);
  }else {
    //DISARM
    mavlink_msg_command_long_pack(0xFF, 0xBE, &msg, 1, 1, 400, 1,0.0,0,0,0,0,0,0);
  }
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SerialMAV.write(buf, len);
}

void sendServoLong(int servo_num, int servo_pwm){
  mavlink_message_t msg;
  
  mavlink_msg_command_long_pack(0xFF, 0x00, &msg, 1, 1, 183, servo_num, servo_pwm, 0,0,0,0,0,0);
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SerialMAV.write(buf, len);
}

void setAutoPilotModeLong(int mode){
  mavlink_message_t msg;
  
  mavlink_msg_command_long_pack(0xFF, 0xBE, &msg, 1, 1, MAV_CMD_DO_SET_MODE, mode, 0, 0,0,0,0,0,0);
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SerialMAV.write(buf, len);
}

void setAutoPilotMode(int mode){
  mavlink_message_t msg;
  
  mavlink_msg_set_mode_pack(0xFF, 190, &msg, 1, 1, mode);
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SerialMAV.write(buf, len);
}

void send_rc_override_packet(uint16_t channel1, uint16_t channel3) {
  int pos = 0;

  buf[pos++] = 0xFE;
  buf[pos++] = 16;
  buf[pos++] = packet_sequence++;
  buf[pos++] = 0xFF;
  buf[pos++] = 0x54;
  buf[pos++] = 70;
  //CHANNELS
  buf[pos++] = channel1 % 256;
  buf[pos++] = channel1 / 256;
  buf[pos++] = 0;
  buf[pos++] = 0;
  buf[pos++] = channel3 % 256;
  buf[pos++] = channel3 / 256;
  buf[pos++] = 0;
  buf[pos++] = 0;
  buf[pos++] = 0;
  buf[pos++] = 0;
  buf[pos++] = 0;
  buf[pos++] = 0;
  buf[pos++] = 0;
  buf[pos++] = 0;
  buf[pos++] = 0;
  buf[pos++] = 0;

  uint16_t crc = checksum(pos, 124);
  buf[pos++] = crc % 256;
  buf[pos++] = crc / 256;

  SerialMAV.write(buf, pos);
}

uint16_t checksum(uint16_t len, uint16_t extra) { // # https://github.com/mavlink/c_library_v1/blob/master/checksum.h
  uint16_t output = 0xFFFF;
  uint16_t tmp;
  for (int i = 1; i < len; i++) {
    tmp = buf[i] ^ (output & 0xFF);
    tmp = (tmp ^ (tmp << 4)) & 0xFF;
    output = ((output >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF;
  }
  tmp = extra ^ (output & 0xFF);
  tmp = (tmp ^ (tmp << 4)) & 0xFF;
  output = ((output >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF;
  return output;
}
