 
/********************************************************
  TEC controller
  Nicola Maghelli
********************************************************/

#include <DallasTemperature.h>
#include <errno.h>
#include <EEPROM.h>
#include "errorCodes.h"
#include "pinout.h"
#include <SerialCommand.h>
#include <avr/io.h>
#include <PID_AutoTune_v0.h>
#include <PID_v1.h>

//#define _TEST

//******************************************************************************************
//**********************************   OPTIONS   *******************************************
//******************************************************************************************

//evaluation interval (in ms) for updating control value
#define _EVALINT 3000

//timeout (in ms) for entering standalone mode
#define _STANDALONEINT 5000

//TEC output generation: define _FASTPWM to use fast modulation, otherwise comment to use slower analog.write()
#define _FASTPWM

//PWM prescaler when _FASTPWM is defined
#define _PWMPRE 256

//temperature sensors resolution in bit: valid values are 9, 10, 12, and 12 corresponding to resolution of
//0.5°C, 0.25°C, 0.125°C, or 0.0625°C, respectively. Conversion time is max. 1s at 12-bit resolution
#define _TEMPRESOLUTION 11

//maximum TEC output (0-255)
#define _MAXTEC 255

//minimum TEC output (0-255)
#define _MINTEC 0

//Minumum and makimum Kp, Ki, Kd
#define _MINKP (-100)
#define _MINKI (-100)
#define _MINKD (-100)
#define _MAXKP 100
#define _MAXKI 100
#define _MAXKD 100

//struct for temperature sensors linked list
typedef struct OneWireNodes {
  byte addr[8];
  struct OneWireNodes *next;
} OneWireNode;

byte thisaddr[8];

//TEC status
bool TECrunning = FALSE;
bool TECerror = TRUE;
bool TECenabled = FALSE;

//Sensor status
bool sensor_error = TRUE;

//algorithm used: 0=autotune, 1=manual PID, 2=ONOFF
int algorithm = 0;

//Define control and output variables
double Setpoint, T1, TEC_out;

//External temperature
double T2;

//Addresses for temperature sensors
DeviceAddress insideTemp;
DeviceAddress outsideTemp;

//error management
int errnum;

//************ AUTOTUNE CONFIG ************
#define MAX_AUTOTUNE_ITER 1000
#define _LOOKBACK 20

//Creates an autotune
PID_ATune myPID_AT(&T1, &TEC_out);

int i = 0;
int AT_out = 0;

//Autotune Inital parameters
double Kp = 2.0;
double Ki = 5;
double Kd = 3;

//noise level for autotune
double ATnoise = 0.1;

//************ MANUAL PID CONFIG ************  
//create a PID
PID myPID(&T1, &TEC_out, &Setpoint, Kp, Ki, Kd, REVERSE);


//************ ONOFF CONFIG ************  
//Hysteresis values for ON/OFF algorithm
double Hys_cool = 2;
double Hys_heat = 0.75;

//variables for remembering time
double now;
double lastTime;

//acknowledge external connection
bool IsConnected = FALSE;
bool IsStandalone = FALSE;

//************ TEMPERATURE SENSORS CONFIG ************  
//Setup a OneWire instance for T1 and T2 on ONE_WIRE_BUS
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

//************ SERIAL COMMAND ************  
//create a SeriallCommand  object
SerialCommand SCmd;

void setup()
{
  //Hardware stuff
  pinMode(TEC_OUT_PIN, OUTPUT);

  //Fast PWM
  #ifdef _FASTPWM
    TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
    TCCR2B = _BV(WGM22) | _BV(CS20);
    OCR2A = _PWMPRE;
    OCR2B = 0;
  #endif
    
  //Setpoint
  Setpoint = -10;
  
  //set autotune controller to PID
  myPID_AT.SetControlType(1);

  //set autotune noise level (in degrees C)
  myPID_AT.SetNoiseBand(ATnoise);

  //set lookback (in s, should be 1/2 to 1/4 of the characteristic time of the system)
  myPID_AT.SetLookbackSec(_LOOKBACK);

  //set output step (0-255)
  myPID_AT.SetOutputStep(_MAXTEC);

  //turn the PID on
   myPID.SetMode(AUTOMATIC);
   
   //set sample time
   myPID.SetSampleTime(_EVALINT);
   
   TEC_out = 0;

  Serial.begin(9600);

  //************ SERIAL COMMAND ************  
  //Command list
  
  //Add commands
  SCmd.addCommand("a", SetAlgorithm);
  SCmd.addCommand("k", PrintStatus);
  SCmd.addCommand("O", EmergencyStop);
  SCmd.addCommand("s", SetSetpoint);
  SCmd.addCommand("p", SetP);
  SCmd.addCommand("i", SetI);
  SCmd.addCommand("d", SetD);
  SCmd.addCommand("n", SetNoiseLevel);
  SCmd.addCommand("r", ClearError);
  SCmd.addCommand("e", EnableTEC);
  SCmd.addCommand("h", SetHh);
  SCmd.addCommand("c", SetHc);
  SCmd.addCommand("w", SaveState);
  SCmd.addDefaultHandler(unrecognized);


//******************************************************************************************
//**********************************   CHECK   *********************************************
//******************************************************************************************

  //OneWire stuff: populate linked-list with all addresses of found sensors
  OneWireNode *curr;
  OneWireNode *head;

  head = NULL;

#ifndef _TEST
  //Starts up OneWire library
  sensors.begin();

  //reset search
  oneWire.reset_search();

  //search devices
  while(oneWire.search(thisaddr)) {
    Serial.println("Start searching for devices...");
    curr = (OneWireNode *)malloc(sizeof(OneWireNode));
    memcpy(curr->addr, thisaddr, sizeof(thisaddr));

    Serial.println("Found \'1-Wire\' device with address:");
   
    for( int i = 0; i < 8; i++) {
      Serial.print("0x");
      if (curr->addr[i] < 16) {
        Serial.print('0');
      }
      Serial.print(curr->addr[i], HEX);
      if (i < 7) {
        Serial.print(", ");
      }
    }

    Serial.println();
    
    if ( OneWire::crc8( curr->addr, 7) != curr->addr[7]) {
        errnum = tempSensorCRCError;
        sensor_error = TRUE;
        Serial.print("CRC is not valid!\n");
        //exit(tempSensorCRCError);
    }
    curr->next = head;
    head = curr;
  }

  curr = head;
  
  Serial.print("Found "); Serial.print(sensors.getDeviceCount(), DEC); Serial.println(" sensors");

  if(sensors.getDeviceCount() == 2){   
    memcpy(&insideTemp, curr->addr, sizeof insideTemp);
    curr=curr->next;
    memcpy(&outsideTemp, curr->addr, sizeof outsideTemp);
    sensor_error = FALSE;
  }else{
    errnum = wrongTempSensorNumber;
    sensor_error = TRUE;
    //Serial.println("Wrong number of sensor detected!");
    //exit(errnum);
  }
  
  //set resolution
  sensors.setResolution(_TEMPRESOLUTION);

  
#elif defined(_TEST)

 Serial.println("Found 2 fake sensors");
 sensor_error = FALSE;
#endif

//******************************************************************************************
//**********************************   CHECK   *********************************************
//******************************************************************************************

  //Set TEC off
  shutTECoff();
  
}

void loop()
{

#ifndef _TEST

  //check if sensors are connected
  if(sensors.isConnected(insideTemp)&&sensors.isConnected(outsideTemp)){
    //all sensors connected, reading temperatures
    //Request temperatures from sensors
    sensors.requestTemperatures();
  
    //Read T1 and T2 uning OneWire protocol
    T1 = sensors.getTempC(insideTemp);
    T2 = sensors.getTempC(outsideTemp);

    //clear sensor error flag
    sensor_error = FALSE;
    
  }else{
    //at least one sensor in not connected: shutting TEC OFF and troubleshooting
    sensor_error = TRUE;
    
    shutTECoff();
    
    if(!(sensors.isConnected(insideTemp)) && !(sensors.isConnected(outsideTemp))){
      //both sensors disconnected
       errnum = allTempSensorError;
       Serial.println("!3");
       //exit(errnum);
      
    }else{
      //only one sensor is disconnected
      if(sensors.isConnected(insideTemp)){
        //insideTemp disconnected
        errnum = insideTempSensorError;
        Serial.println("!1");
        //exit(errnum);
       
    }else{
        //outTemp disconnected
        errnum = outsideTempSensorError;
        Serial.println("!2");
        //exit(errnum);
       }
     }
  
  } 
#elif defined(_TEST)
  T1 = -10+5*sin((float) millis()/20000*3.1415)+random(-100,100)/100;
  T2 = 25;
  sensor_error = FALSE;   
#endif

 //******************************************************************************************
 //********************************   STANDALONE   ******************************************
 //******************************************************************************************

if(!IsStandalone){
  if ((millis() > _STANDALONEINT)&&!IsConnected){
    //no connection for more than _STANDALONEINT ms, entering standalone mode
    RetrieveState();
    TECenabled = TRUE;
    TECrunning = TRUE;
    TECerror = FALSE;
    Serial.println("Entering standalnoe mode...");
    IsStandalone = TRUE;
  }
}


 if(TECenabled){
  if(algorithm == 0){
  //*************** AUTOTUNE ***************
    for(i = 0; i < MAX_AUTOTUNE_ITER; i++){
      //Serial.print("AT iteration:"); Serial.println(i+1);
      AT_out = myPID_AT.Runtime();
      if(AT_out == 1){
        //Serial.print("AT finished at iteration "); Serial.println(i);
          //get autotune parameters
       
         Kp = constrain (myPID_AT.GetKp(), _MINKP, _MAXKP);
         Ki = constrain (myPID_AT.GetKi(), _MINKI, _MAXKI);
         Kd = constrain (myPID_AT.GetKd(), _MINKD, _MAXKD);        
          
       break;
      }
      
    }
  }else if(algorithm == 1){
    //*************** MANUAL PID ***************
    //update manual PID parameters
    myPID.SetTunings(Kp, Ki, Kd);
    myPID.Compute();
  }else if (algorithm == 2){
    //*************** ONOFF ***************
    now = millis();
  
    if((now-lastTime)>=_EVALINT){
      //need to update status
   
      if(TEC_out >= _MAXTEC){
        //We are already cooling
      
        if(T1<=(Setpoint-Hys_cool)){
          TEC_out = _MINTEC;
          }
          else{
          TEC_out = _MAXTEC;
          }
      }
      else{
       //we were not cooling
        if(T1>(Setpoint+Hys_heat)){
          TEC_out = _MAXTEC;
        }
        else{
          TEC_out = _MINTEC;
        }
          
        }
      lastTime = millis();  
    }
  
  }

 

  //******************************************************************************************
  //********************************   TEC output   ******************************************
  //******************************************************************************************
 
  //Updating TEC control if TECerror and sensor_error are FLASE
  if((!TECerror)&&(!sensor_error)){

      TEC_out = constrain (TEC_out, _MINTEC, _MAXTEC);
      updateTEC(TEC_out);
    
  }

  
  
  //******************************************************************************************
  //********************************   Serial comm   *****************************************
  //******************************************************************************************
 }else
 {
  updateTEC(0);
  TECrunning = FALSE;
 }

  SCmd.readSerial(); 

}

void updateTEC(double value){
  //casting to byte (0-255) and constrain between _MINTEC and _MAXTEC
  byte writeTEC = (byte) value;
  if(writeTEC>=_MAXTEC){
    writeTEC = _MAXTEC;
  }
  if(writeTEC<=_MINTEC){
    writeTEC = _MINTEC;
  }

    #ifdef _FASTPWM
      //Fast PWM: duty cycle set by changing OCR2B value
      OCR2B = writeTEC/_MAXTEC*_PWMPRE;
    #elif !defined(_FASTPWM)
      //output control value
      analogWrite(TEC_OUT_PIN, writeTEC);
      TECrunning = TRUE;
    #endif
    
    return;
}

void shutTECoff(){
  //turns TEC control to 0
  analogWrite(TEC_OUT_PIN, 0);
  TECrunning = FALSE;
  TECerror = TRUE;
  return;
}

void PrintStatus()
{
    Serial.println(algorithm);
    Serial.println(Setpoint);
    Serial.println(T1);
    Serial.println(T2);
    Serial.println(TEC_out);
    Serial.println(Kp);
    Serial.println(Ki);
    Serial.println(Kd);
    Serial.println(Hys_cool);
    Serial.println(Hys_heat);
    Serial.println(TECerror);
    Serial.println(TECrunning);
    Serial.println(TECenabled);
    Serial.println(AT_out);
    Serial.println(ATnoise);
    Serial.println(sensor_error);
    IsConnected = TRUE;
}

void SetAlgorithm()
{
  char *arg;
  arg = SCmd.next();
  if(arg!=NULL){
    algorithm = atof(arg);
  }
}

void EmergencyStop()
{
  analogWrite(TEC_OUT_PIN, 0);
  TECrunning = FALSE;
  TECerror = TRUE;
}

void SetSetpoint()
{
  char *arg;
  arg = SCmd.next();
  if(arg!=NULL){
    Setpoint = atof(arg);
  }
}

void SetP()
{
  char *arg;
  arg = SCmd.next();
  if(arg!=NULL){
    Kp = atof(arg);
  }
}

void SetI()
{
  char *arg;
  arg = SCmd.next();
  if(arg!=NULL){
    Ki = atof(arg);
  }
}

void SetD()
{
  char *arg;
  arg = SCmd.next();
  if(arg!=NULL){
    Kd = atof(arg);
  }
}

void SetNoiseLevel()
{
  char *arg;
  arg = SCmd.next();
  if(arg!=NULL){
     ATnoise=atof(arg);
     myPID_AT.SetNoiseBand(ATnoise);
  }
}

void SetHh()
{
  char *arg;
  arg = SCmd.next();
  if(arg!=NULL){
    Hys_heat = atof(arg);
  }
}

void SetHc()
{
  char *arg;
  arg = SCmd.next();
  if(arg!=NULL){
    Hys_cool = atof(arg);
  }
}

void EnableTEC()
{
  char *arg;
  arg = SCmd.next();
  if(arg!=NULL){
    if (atoi(arg) == 0) {
      TECenabled = FALSE;
    } else {
      TECenabled = TRUE;
    }
  }
}

void ClearError()
{
 TECerror = FALSE;
}

void SaveState()
{
  int addr = 0;
 
  struct TEC_config {
    double SetT;
    int alg;
    double K_p;
    double K_i;
    double K_d;
    double H_c;
    double H_h;
  } CurrentState;

  CurrentState.SetT = Setpoint;
  CurrentState.alg = algorithm;
  CurrentState.K_p = Kp;
  CurrentState.K_i = Ki;
  CurrentState.K_d = Kd;
  CurrentState.H_c = Hys_cool;
  CurrentState.H_h = Hys_heat;
  
  EEPROM.put(addr, CurrentState);
}

void RetrieveState()
{
   int addr = 0;
 
  struct TEC_config {
    double SetT;
    int alg;
    double K_p;
    double K_i;
    double K_d;
    double H_c;
    double H_h;
  } CurrentState;

  EEPROM.get(addr, CurrentState);

  Setpoint = CurrentState.SetT;
  algorithm = CurrentState.alg;
  Kp = CurrentState.K_p;
  Ki = CurrentState.K_i;
  Kd = CurrentState.K_d;
  Hys_cool = CurrentState.H_c;
  Hys_heat = CurrentState.H_h; 
}

void unrecognized()
{
  char *arg;
  arg = SCmd.next();
  Serial.print("? "); Serial.println(arg);
}

