//#include "./DFRobotDFPlayerMini/DFRobotDFPlayerMini.cpp"

#include "./mavlink/common/mavlink.h"
#include "MedianFilter.h"
//#include "./sonar/Sonar.h"

//UART
#define RX_PIN 12
#define TX_PIN 10

//Cable finder
//int analogInPin[2]={A2,A4};//left,right

//SoftwareSerial SerialMAV(RX_PIN, TX_PIN, false);
#define SerialMAV Serial1

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

//Cablefinder
int i,j,v1,toq,minv=1000,maxv=0,q=0,arr_q[2];
int maxq[2] = {8,8};
int errorv=200,toi=20,toj=20,looseevenone,steer,speedf,angle;
int antidancer=100;
int lastturn=1;//1=left,2=right
int cabletouch=0;//0-gotobasestation cable; 1-folow the cable
int lastdat=1;
const int centersteer=1500;
const int timetosteer=100;
const int speedsteer=70;
const int centerspeed=1500;
const int speedfront=70;
const int errorpq=30; 
uint32_t dancelimit=850000;
uint32_t myTimerdancer;

//SONAR 1
#define SONAR1_TRIG_PIN 12
#define SONAR1_ECHO_PIN 11
MedianFilter sonar_filter_1(3, 0);

//SONAR 2
#define SONAR2_TRIG_PIN 10
#define SONAR2_ECHO_PIN 9
MedianFilter sonar_filter_2(3, 0);

//BUMPER
#define BUMPER_INTERVAL 1

#define BUMPER_TRIG_PIN 2

const boolean DEBUG_FLAG = true;

unsigned long bumperTimer = 0;

//MODE_SWITCHER

#define DO_PARKING 1

// WORK MODES
const int INIT_MODE = 1;
const int IDLE_MODE = 2;
const int DRIVE_MODE = 3;
const int DRIVE_BUMPERHIT_MODE = 4;
const int DRIVE_ACRO_MODE = 5;
const int DRIVE_PARKING_MODE = 6;

// WORK MODE
int current_work_mode = INIT_MODE;

//EVASIVE MANEUVER
#define EVASIVE_STEER_LEFT 1600
#define EVASIVE_STEER_RIGHT 1400

#define EVASIVE_SPEED_BACK 1400 //Разбираюсь с git
#define EVASIVE_SPEED_FORWARD 1700

//msecs
#define EVASIVE_INTERVAL 1000000

const bool evasiveManeuverEnabled = true;
bool evasiveManeuverInit = true;
bool evasiveManeuverFinished = false;
unsigned long evasiveTimer = 0;

#define EVASIVE_MANEUVER_START 0
#define EVASIVE_MANEUVER_BACK 1
#define EVASIVE_MANEUVER_LEFT 2
#define EVASIVE_MANEUVER_RIGHT 3
#define EVASIVE_MANEUVER_FORWARD 4
#define EVASIVE_MANEUVER_STOP 5
#define HATCH_OPEN 6
#define HATCH_CLOSE 7
#define SWITCH_TO_AUTO 8

const int evasive_maneuver_list[] = {
  EVASIVE_MANEUVER_BACK, 
  EVASIVE_MANEUVER_BACK, 
  EVASIVE_MANEUVER_LEFT, 
  EVASIVE_MANEUVER_FORWARD, 
  EVASIVE_MANEUVER_FORWARD, 
  EVASIVE_MANEUVER_RIGHT, 
  EVASIVE_MANEUVER_FORWARD, 
  EVASIVE_MANEUVER_FORWARD, 
  EVASIVE_MANEUVER_STOP
};

const int evasive_maneuver_list_length = sizeof(evasive_maneuver_list) / sizeof(evasive_maneuver_list[0]);  

const bool acroManeuverEnabled = true;
bool acroManeuverInit = true;
bool acroManeuverFinished = false;
unsigned long acroTimer = 0;

const int acro_maneuver_list[] = {
  HATCH_OPEN,
  EVASIVE_MANEUVER_BACK,
  EVASIVE_MANEUVER_BACK,
  EVASIVE_MANEUVER_BACK,
  EVASIVE_MANEUVER_RIGHT, 
  EVASIVE_MANEUVER_BACK,
  EVASIVE_MANEUVER_BACK,
  EVASIVE_MANEUVER_BACK,
  HATCH_CLOSE,
  HATCH_CLOSE
};

const int acro_maneuver_list_length = sizeof(acro_maneuver_list) / sizeof(acro_maneuver_list[0]);  


//TODO: FIX TO COMMON FUNCTIONS
int current_evasive_maneuver_pos = 0;
unsigned long current_evasive_maneuver_timer = 0;

int current_acro_maneuver_pos = 0;
unsigned long current_acro_maneuver_timer = 0;

//=======================================================

mavlink_message_t recv_msg;
mavlink_status_t  recv_status;

unsigned long heartbeat_timer = 0;

uint32_t current_autopilot_mode = 0xFF;

//DF player
//SoftwareSerial Serialmp3(4, 5, false);
//DFRobotDFPlayerMini myDFPlayer;


//=======================================================
void setup() {
  //Init SONAR 1
  pinMode(SONAR1_TRIG_PIN, OUTPUT); 
  pinMode(SONAR1_ECHO_PIN, INPUT);    
  //Init SONAR 1
  pinMode(SONAR2_TRIG_PIN, OUTPUT); 
  pinMode(SONAR2_ECHO_PIN, INPUT);    
  //Init BUMPER
  pinMode(BUMPER_TRIG_PIN, INPUT_PULLUP); 
  // init timer
  sonarTimer = micros();
  bumperTimer = micros();
  heartbeat_timer = millis();
  
  //DEBUG INTERFACE
  if(DEBUG_FLAG){
    Serial.begin(115200);
    Serial.println("AUX SW v0.1");
  }
  
  //Mavlink serial init
  //SerialMAV.begin(115200);
  SerialMAV.begin(19200);
  //SerialMAV.begin(9600);

  //MP3
  //Serialmp3.begin(9600);
  //myDFPlayer.begin(Serialmp3);
  //myDFPlayer.volume(30);  //Set volume value. From 0 to 30
  
  //Start
  changeMode(IDLE_MODE);
}



//=======================================================
void loop() {
  //TODO: bumper hit in main cycle
  //doSendHeartbeat();
  
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
    case DRIVE_ACRO_MODE:
      doAcroMode();
      break;
    case DRIVE_PARKING_MODE:
      doParkingMode();
    default:
      break;
  }
  //TODO : FIX SET ACRO DRIVE MODE IN RCV PROCEDURE
  comm_receive(recv_msg, recv_status);//delay(10);
}

//=======================================================
//DRIVE MODES

void doIdleMode(){
  sendInfoMessage("AUX SW v0.2");
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
    //doSonar();
  }

}

void doBumperHitRoutine(){
 
  //mav_distance_sensor(3, MIN_DISTANCE + 1);
  
  if(bumperTimer < micros()){
    bumperTimer = micros() + BUMPER_INTERVAL;

    if(evasiveManeuverEnabled){
      if(evasiveTimer > micros()){
        doEvasiveManeuver();
        return;
      } else {
        doEvasiveManeuverFinished();
        changeMode(DRIVE_MODE);
      }
    }
  }
}

void doAcroMode(){
    if(acroTimer > micros()){
        doAcroManeuver();
        return;
      } else {
        doAcroManeuverFinished();
        changeMode(DRIVE_MODE);
      }
}

//=====================================================
//PARKING MODE
void initParkingMode(){
  
}

void doParkingMode(){
//Получаем в процентном отношении данные с каждого датчика к максимуму этого датчика
  cableper();
  checkcable();

    if(arr_q[1]>errorpq and arr_q[2]>errorpq){
      //----------------------------- Обработка следования кабелю.
      //Если оба датчика видят кабель более errorpq %, правим угол во время движения
      looseevenone=0;//Оба датчика видят поле
      cabletouch=1; //Кабель видел
      speedf=centerspeed+speedfront;
      if(arr_q[1]>arr_q[2]){
          angle=arr_q[1]-arr_q[2];
          steer=centersteer-(angle*speedsteer)/100;
          lastturn=1;
        }
      if(arr_q[2]>arr_q[1]){
          angle=arr_q[2]-arr_q[1];
          steer=centersteer+(angle*speedsteer)/100;
          lastturn=2;
        }
       }else if(arr_q[1]<errorpq and arr_q[2]<errorpq){
        Serial.println("!!!No cable!!!");
        looseevenone=1; //Если потерял оба датчика, смотрим последний поворот и крутим в другую сторону
        steer=centersteer;
        
        if(lastturn==1){
          //Если поворачивали влево, поворачиваем вправо
             steer=centersteer+speedsteer;
          }else{
             steer=centersteer-speedsteer;
          }
       }else{
        looseevenone=1;
        //Если один из датчиков видит, а другой нет. Крутимся на месте
        //Увеличиваем или уменьшаем коэффициент для угла поворота, чтобы не танцевать
        if(micros()-myTimerdancer>dancelimit){
          //Serial.println("DANCE PLUS+++++ \t");
          if(antidancer<100){
              antidancer+=10;
              //Serial.println("++++++++++++++++ \t");
            }
          }else {
            if(antidancer>10){
              antidancer-=10;
              //Serial.println("------------------ \t");
              }
          }
//                  Serial.print("mydancer=");
//                  Serial.print(micros()-myTimerdancer);
//                  Serial.print("\t antidancer=");
//                  Serial.print(antidancer);
//                  Serial.print("\t lastdat=");
//                  Serial.print(lastdat);
//                  Serial.print("\t myTimerdancer=");
//                  Serial.print(myTimerdancer);
//                  Serial.print("\t micros=");
//                  Serial.print(micros());
//                  Serial.println("\t");
          speedf=centerspeed; 
          
          if(arr_q[1]>arr_q[2]){
              steer=centersteer-speedsteer*antidancer/100;
              if(lastdat==2){
                myTimerdancer = micros(); 
                //Обновляем таймер когда начал видеть один лишь датчик, после другого
              }
              lastdat=1;
            }
          if(arr_q[2]>arr_q[1]){
                steer=centersteer+speedsteer*antidancer/100;
                if(lastdat==1){
                myTimerdancer = micros(); 
                //Обновляем таймер когда начал видеть один лишь датчик, после другого
              }
                lastdat=2;
              }
          }

          
//Если еще не видел кабель просто едем тихо вперед
      if(cabletouch==0){          
          steer=centersteer;
          speedf=centerspeed+speedfront;
          Serial.println("Cable search");
      }
}

void checkcable() {
  for(i=1;i<=2;i++){
    minv=1000;
    maxv=0;
    q=0;
    //Собираем измерения макс и мин напряжения в цикле 
    for (i = 0; i < toi; i++) {
        for (j = 0; j < toj; j++) {
          v1 = analogRead(i);
          if(v1>maxv){
            maxv=v1;
          }
          if(v1<minv){
            minv=v1;
          }
        }
        toq=maxv-minv;
        q=q+toq;
    }
    if(q<errorv){
      q=0;
    }
      arr_q[i]=q;
  }
}

void cableper(){
  for(i=1;i<=2;i++){
    if(arr_q[i]>maxq[i] and arr_q[i]<2*maxq[i]){
      maxq[i]=arr_q[i];
    }
    arr_q[i]=arr_q[i]*100/maxq[i];
  }
}

//=====================================================
//ACRO MODE

void initAcroMode(){
  acroManeuverInit = true;
  
  acroTimer = micros() + (EVASIVE_INTERVAL * acro_maneuver_list_length);
  
  acroManeuverFinished = false;
}

void doAcroManeuver(){

  if(acroManeuverInit){
    disarm();
    delay(100);
    setAutoPilotMode(MANUAL_MODE);
    delay(200);
    arm();
    acroManeuverInit = false;
    acroManeuverFinished = false;
    sendInfoMessage("Init Acro");
    current_acro_maneuver_pos = 0;
    current_acro_maneuver_timer = micros() + EVASIVE_INTERVAL;
  } else {
    acroManeuver();     
  }
}

void acroManeuver(){
  //Check bumper hit in evasive mode

  if(isBumperHit(BUMPER_TRIG_PIN)){
    emergencyStop();  
    initEvasive();
    return;
  }
  
  if(current_acro_maneuver_timer > micros()){
    executeManeuver(acro_maneuver_list[current_acro_maneuver_pos]);
  } else {
    current_acro_maneuver_timer = micros() + EVASIVE_INTERVAL;
    if(current_acro_maneuver_pos < acro_maneuver_list_length-1) 
    {
      current_acro_maneuver_pos++;
    }
  }
}

void doAcroManeuverFinished(){
  if(acroManeuverFinished){
    //
  } else{
    if(DEBUG_FLAG){
      Serial.println("Stop ACRO");
    }
    sendInfoMessage("Stop ACRO");
    current_acro_maneuver_pos = 0;
    current_acro_maneuver_timer = micros();
    acroManeuverFinished = true;
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
//======================================================
//EVASIVE FUNCTIONS

void initEvasive(){
  evasiveManeuverInit = true;
  
  evasiveTimer = micros() + (EVASIVE_INTERVAL * evasive_maneuver_list_length);
  
  evasiveManeuverFinished = false;
}

void doEvasiveManeuver(){

  if(evasiveManeuverInit){
    //disarm();
    //delay(100);
    setAutoPilotMode(MANUAL_MODE);
    //delay(200);
    //arm();
    evasiveManeuverInit = false;
    evasiveManeuverFinished = false;
    Serial.println("Init EVM");
    //sendInfoMessage("Init EVM");
    current_evasive_maneuver_pos = 0;
    current_evasive_maneuver_timer = micros() + EVASIVE_INTERVAL;
  } 
  evasiveManeuver();     
}

void evasiveManeuver(){
  //Check bumper hit in evasive mode
  //TOOD: Fix Mess

  int current_maneuver = evasive_maneuver_list[current_evasive_maneuver_pos];
  
  if(current_evasive_maneuver_timer > micros()){
    if(current_maneuver != EVASIVE_MANEUVER_START && current_maneuver != EVASIVE_MANEUVER_BACK && current_maneuver != EVASIVE_MANEUVER_STOP){
      if(isBumperHit(BUMPER_TRIG_PIN)){
        emergencyStop();  
        initEvasive();
        return;
      }
    }

    executeManeuver(current_maneuver);            
  } else {
    current_evasive_maneuver_timer = micros() + EVASIVE_INTERVAL;
    if(current_evasive_maneuver_pos < evasive_maneuver_list_length-1) 
    {
      current_evasive_maneuver_pos++;
    }
  }
 
}

void executeManeuver(int maneuver){
  switch(maneuver){
    case EVASIVE_MANEUVER_BACK:
      rc_override(CENTER_STEER, EVASIVE_SPEED_BACK);
      break;
    case EVASIVE_MANEUVER_LEFT:
      rc_override(EVASIVE_STEER_LEFT, CENTER_SPEED);
      break;
    case EVASIVE_MANEUVER_RIGHT:
      rc_override(EVASIVE_STEER_RIGHT, CENTER_SPEED);
      break;
    case EVASIVE_MANEUVER_FORWARD:
      rc_override(CENTER_STEER, EVASIVE_SPEED_FORWARD);
      break;
    case EVASIVE_MANEUVER_STOP:
      rc_override(CENTER_STEER, CENTER_SPEED);
      break;
    case HATCH_OPEN:
      openHatch();
      break;      
    case HATCH_CLOSE:
      closeHatch();
      break;
    case SWITCH_TO_AUTO:
      disarm();
      mav_distance_sensor(3, MIN_DISTANCE + 1);
      delay(100);
      mav_distance_sensor(3, MIN_DISTANCE + 1);
      setAutoPilotMode(AUTO_MODE);
      delay(300);
      arm();
      break;
    default:
      Serial.println("Unknown maneuver");  
  }
}

void doEvasiveManeuverFinished(){
  if(evasiveManeuverFinished){
    //
  } else{
    if(DEBUG_FLAG){
      Serial.println("Stop EMV");
    }
    sendInfoMessage("Stop EMV");
    current_evasive_maneuver_pos = 0;
    current_evasive_maneuver_timer = micros();
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
    //mav_distance_sensor(3, MIN_DISTANCE + 1);
    //sendEmergencyMessage("BUMPER HIT");
    return true;
  }
  return false;
}

boolean isBumperHit(int bumper_pin){
  boolean isBumperHit = digitalRead(bumper_pin) == 1;
  //TODO: filter contact bounce
  //isBumperHit = false;
  return isBumperHit;
}

//=======================================================
//HEARTBEAT FUNCTIONS
void doSendHeartbeat(){
  if(heartbeat_timer < millis()){
    mav_heartbeat_pack();
    heartbeat_timer = millis() + 1000;
  }
}
//=======================================================
//SONAR FUNCTIONS
void doSonar(){

    return;
    
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
        //Serial.println("SONAR UNRELIABLE");
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
  //sendEmergencyMessage("EMERGENCY STOP");
  stopMotors();
}

void stopMotors(){
  rc_override(CENTER_STEER, CENTER_SPEED);
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

//=============================================

#define HATCH_SERVO 5.0

void openHatch(){
  sendServoLong(2000);
}

void closeHatch(){
  sendServoLong(850);
}

// SERVICE LAYER

void changeMode(int mode){
   current_work_mode = mode;
   if(DEBUG_FLAG){
    Serial.println("SET MODE : " + String(mode)); 
   }
   //myDFPlayer.play(mode);
   //sendInfoMessage("MODE:" + String(mode));
}

int getMode(){
  return current_work_mode;
}


void rc_override(int mav_steer, int mav_speed){
  Serial.println("RC_OVERRIDE");
  mavlink_message_t msg;
  
  mavlink_msg_rc_channels_override_pack(
    0xFF, 0x54, &msg, 0, 0,
    mav_steer, 
    mav_speed, 
    65535,
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

void sendServoLong2(int servo_pwm){
  mavlink_message_t msg;
  
  mavlink_msg_rc_channels_override_pack(
    0xFF, 0x54, &msg, 0, 0,
    65535, 
    65535, 
    65535,
    65535,65535,servo_pwm,65535,65535,0,0,0,0,0,0,0,0,0,0
  );    
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SerialMAV.write(buf, len);

}

void sendServoLong(float servo_pwm){
  mavlink_message_t msg;
  
  mavlink_msg_command_long_pack(0xFF, 0x00, &msg, 
  0x01, 0x01, 
  MAV_CMD_DO_SET_SERVO, 
  0,
  5.0, 
  servo_pwm, 
  0,0,0,0,0);
  
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

#define ARDUPILOT_ACRO_MODE 1

/* This function gets message from the APM and interprete for Mavlink common messages */
void comm_receive(mavlink_message_t recv_msg, mavlink_status_t recv_status) {

  unsigned long read_timer = millis() + 10;
  
  while(SerialMAV.available() > 0) {

    if(read_timer < millis()){
      //Serial.println("RCV_TIMEOUT");
      return;
    }
    
    uint8_t c = SerialMAV.read();
    
    // Try to get a new message
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &recv_msg, &recv_status)) {
      switch(recv_msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:  // #0: Heartbeat
          {
            uint8_t base_mode = mavlink_msg_heartbeat_get_base_mode(&recv_msg);
            if((MAV_MODE_FLAG_CUSTOM_MODE_ENABLED & base_mode) > 0){
              uint32_t custom_autopilot_mode = mavlink_msg_heartbeat_get_custom_mode(&recv_msg);
              if(current_autopilot_mode != custom_autopilot_mode){
                current_autopilot_mode = custom_autopilot_mode;

                if(DEBUG_FLAG){
                  Serial.print("Mode : ");
                  Serial.println(current_autopilot_mode);
                }
                //TODO: MOVE TO ACRO ROUTINE
                if(current_autopilot_mode == ARDUPILOT_ACRO_MODE && getMode() != DRIVE_ACRO_MODE){
                  if(DO_PARKING){
                    changeMode(DRIVE_PARKING_MODE);
                    initParkingMode();
                  } else {
                    changeMode(DRIVE_ACRO_MODE);
                    initAcroMode();
                  }
                }
                
              }
            }
          }
          break;
      }
    }
  }
}

void mav_heartbeat_pack() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  // Pack the message
  mavlink_msg_heartbeat_pack(0x01, MAV_COMP_ID_OBSTACLE_AVOIDANCE, &msg, MAV_TYPE_CAMERA, MAV_AUTOPILOT_INVALID, 0x01, 0x00, MAV_STATE_ACTIVE);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SerialMAV.write(buf, len);
}
