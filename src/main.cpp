#include <main.h>

#define CHILD_ID_DIM_1 4
#define CHILD_ID_DIM_2 5

#define LED_PIN_1	5
#define LED_PIN_2	6

#define MAX_DIMMERS 2
#define DIMMER_DEFAULT_DIMMING  10

DimmerClass dimmer[2];

ControllerMonitor controller;

ParametersManager   parameters;
MyMessage dimmerMsg;

/* defined project specific, used to store values in EEPROM */
typedef struct 
{
    uint16_t fadeIn[2];
    uint16_t fadeOut[2];
    uint16_t onValue;
    uint16_t onTime;
    uint16_t localMode;
    uint16_t qualSendFrequency;
    uint16_t pollTime; 
}CFG_VAL;


CFG_VAL configValues;

/* defined project specific */

#define CFG_SIZE    9

extern const PROGMEM CFG_PAR configPar[] = 
{ // address, min, max, default, (type?)
/*  000 */    {&configValues.fadeIn[0], 0, 100, DIMMER_DEFAULT_DIMMING},
/*  001 */    {&configValues.fadeOut[0], 1, 101, DIMMER_DEFAULT_DIMMING},
/*  002 */    {&configValues.fadeIn[1], 2, 102, DIMMER_DEFAULT_DIMMING},
/*  003 */    {&configValues.fadeOut[1], 3, 103, DIMMER_DEFAULT_DIMMING},
/*  004 */    {&configValues.onValue, 10, 100, 50},
/*  005 */    {&configValues.onTime, 5, 300, 30},
/*  006 */    {&configValues.localMode, 0, 1, 0},
/*  007 */    {&configValues.qualSendFrequency, 1, 1000, 50},
/*  008 */    {&configValues.pollTime, 1, 1000, 30},
};


boolean metric = true, motion, lastMotion, motionChange; 
#define QUAL_BFR_SIZE 5
uint16_t    sendOk, sendQual, msgSendTotal, sendFail;
static  uint16_t    totalSends;/* 
uint16_t    sendQualBfr[QUAL_BFR_SIZE];
uint8_t     sendQualIdx = 0;
uint16_t    sendTotalBfr[QUAL_BFR_SIZE];
uint16_t    msgTotalBfr[QUAL_BFR_SIZE];
uint16_t    sendFailBfr[QUAL_BFR_SIZE],  ;*/
uint32_t qual; 
uint32_t qualFail;
boolean waitForSettings = false, sendQuality = false;

#define CHILD_ID_HUM 0
#define CHILD_ID_TEMP 1
#define CHILD_ID_MOT 2   // Id of the sensor child
#define CHILD_ID_LIGHT 3

#define CHILD_ID_CONN   6
#define CHILD_ID_CONN_QUAL   7

#define CHILD_ID_GET_SETT   8
MyMessage msgGetSettings(CHILD_ID_GET_SETT, V_LIGHT_LEVEL);


#define HUMIDITY_SENSOR_DIGITAL_PIN 4
#define LIGHT_SENSOR_ANALOG_PIN A0
#define MOTION_INPUT 3   // The digital input you attached your motion sensor.  (Only 2 and 3 generates interrupt!)
#define INTERRUPT MOTION_INPUT-2 // Usually the interrupt = pin -2 (on uno/nano anyway)

#define INTERVAL_DHT    900L  // seconds
#define INTERVAL_LIGHT   60L
#define FAST_LIGHT_CHANGE_DIFF 50 

MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
MyMessage msgMot(CHILD_ID_MOT, V_TRIPPED);
MyMessage msgLight(CHILD_ID_LIGHT, V_LIGHT_LEVEL);

MyMessage msgConnectionQual(CHILD_ID_CONN, V_LEVEL);
MyMessage msgConnectionFail(CHILD_ID_CONN_QUAL, V_LEVEL);

unsigned long SLEEP_TIME = 30000; // Sleep time between reads (in milliseconds)

// MySensor gw;
DHT dht;
float lastTemp;
float lastHum;
//int   lightLevel;
int   lastLightLevel;
//int 	intervalDHT;
unsigned long lastDhtRead;
unsigned long lastLightRead;
unsigned long motionTimeout, motionTimer;
byte  state;


void setup()  
{ 
  //char dimPeriod;
  Serial.println("setup");
  dimmer[0].begin(0, LED_PIN_1);
  dimmer[1].begin(1, LED_PIN_2);
  controller.set_callback(request_values);
  dht.setup(HUMIDITY_SENSOR_DIGITAL_PIN); 
  pinMode(MOTION_INPUT, INPUT);      // sets the motion sensor digital pin as input
  parameters.begin(CFG_SIZE, (uint8_t *)&configValues, sizeof(configValues));
 // parameters.begin(CFG_SIZE);
  parameters.setRequestCallback(requestParameters);
  parameters.setStatusCallback(paramStatusReport);
  parameters.load();
  /* for (uint8_t j = 0; j < 2; j++) 
  {
    Serial.print("loaded fade in: ");
    Serial.print(j, DEC);
    Serial.print(" val: ");
    Serial.println(configValues.fadeIn[j], DEC);  
  }
  for (uint8_t j = 0; j < 2; j++) 
  {
    Serial.print("loaded fade out: ");
    Serial.print(j, DEC);
    Serial.print(" val: ");
    Serial.println(configValues.fadeOut[j], DEC);
  } */
  
  parameters.request(0, 3, 20);
  
  
}

uint8_t request_values(uint8_t set)
{
  // request the current values
  static uint8_t requestToggle;
  if (++requestToggle > 1) // nr of childs to request
    requestToggle = 0;
  request(CHILD_ID_DIM_1 + requestToggle, V_DIMMER);  
  // request(CHILD_ID_DIM_2, V_DIMMER);  
  
  Serial.print("Request values ");
  Serial.println(CHILD_ID_DIM_1 + requestToggle, DEC);
  #ifdef DIS_INT_ON_TX
    wait(10);
    EIFR = bit (INTF1);  // clear flag for interrupt 1 to avoid triggering on sending msg
  #endif
  return (true);
}

void configLoad()
{
  for (uint8_t i = 0; i < MAX_DIMMERS; i++) 
  {
    dimmer[i].setFadeIn(configValues.fadeIn[i]);
    dimmer[i].setFadeOut(configValues.fadeOut[i]);
  }
  controller.begin(configValues.pollTime);
}

uint8_t requestParameters(uint8_t parameterToRead)
{
  send(msgGetSettings.set(parameterToRead));  // request settings from controller
  /* Serial.print("Requesting ");
  Serial.println(parameterToRead); */
  
}

uint8_t paramStatusReport(uint8_t status)
{
  if (status == ST_NEW_PAR_AV)
  {
    configLoad();
  }
}

void presentation()  
{ 
  
  Serial.println("presentation  ");
  // Send the Sketch Version Information to the Gateway
  sendSketchInfo(SN, SV);

  present(CHILD_ID_DIM_1, S_DIMMER);  // present as dimmer
  present(CHILD_ID_DIM_2, S_DIMMER);  // present as dimmer


//  lastControllerMsgTime = millis();
 
  present(CHILD_ID_HUM, S_HUM);
  present(CHILD_ID_TEMP, S_TEMP);
  present(CHILD_ID_MOT, S_MOTION);
  present(CHILD_ID_LIGHT, S_LIGHT_LEVEL);
  present(CHILD_ID_CONN, S_GAS); //??
  present(CHILD_ID_CONN_QUAL, S_GAS); //??
}


uint16_t getInt(uint32_t value)
{
    return (uint16_t) (value & 0x0000FFFF);
}

uint16_t highInt(uint32_t value)
{
    return (uint16_t) ((value & 0xFFFF0000) >> 16);
}

uint8_t getIndex(uint32_t value)
{
    return (uint8_t)  ((value & 0xFF000000) >> 24);
}

void receive(const MyMessage &message)
{
 // int dimValue;
  uint32_t  input;
  byte index = message.sensor == CHILD_ID_DIM_1?0:1;
  switch (message.type)
  {
    case V_LIGHT:
    case V_DIMMER:
      #ifdef DEBUG
      Serial.print("dim value ");
      Serial.println(atoi(message.data));  
      #endif
      dimmer[index].setLevel(atoi(message.data));
      break;
    

    case V_VAR1:  // used to set parameters on sensor
      input = atol(message.data);
      parameters.set(getIndex(input), getInt(input)); 
   /*   Serial.print("setting ");
      Serial.println(getIndex(input));*/
     /*
     for (uint8_t i = 0; i < MAX_DIMMERS; i++) 
      {
        dimmer[i].setFadeIn(configValues.fadeIn[i]);
        dimmer[i].setFadeOut(configValues.fadeOut[i]);
      }*/
      break;
    
    default:
      break;

  }
/*   Serial.println("receive"); */
  controller.answer();
/* 
  wait(10);
  EIFR = bit (INTF1);  // clear flag for interrupt 1 to avoid triggering on sending msg
   */
}

boolean resend(MyMessage &msg, int repeats) // Resend messages if not received by gateway
{
  int repeat = 0;
  int repeatDelay = 0;
  boolean ack = false;
    
  msgSendTotal++; // number of messages sent (without retries)
  
  while (ack == false && repeat < repeats) 
  {
    wait(repeatDelay); 
    if (send(msg)) 
    {
      ack = true;
      sendOk++;
    }
    else 
    {
      ack = false;
      if (repeatDelay < 500)
        repeatDelay += 250 + (MY_NODE_ID * 4);
      sendQual++;
    } 
   /*  Serial.println(totalSends); */
    totalSends++; // number of messages sent (including retries)
   /*  Serial.println(totalSends);
     
        Serial.print("total bfr ");
        Serial.print(sendTotalBfr[sendQualIdx]);
        Serial.print(" index ");
        Serial.print(sendQualIdx); */
    if ((totalSends % configValues.qualSendFrequency) == 0)
    {
       qual = ((totalSends - sendQual) * 1000UL) / totalSends;
        qualFail = ((msgSendTotal - sendFail) * 1000UL) / msgSendTotal;
       #ifdef DEBUG
    Serial.print("Quality ");
    Serial.print(qual);
    Serial.print(" Total ");
    Serial.print(totalSends);
    Serial.print(" send ok ");
    Serial.print(sendOk);
    Serial.print(" fail ");
    Serial.println(qualFail);
  #endif
        sendOk = 0;
        totalSends = 0;
        sendQual = 0;
        sendFail = 0;
        msgSendTotal = 0;
/* 
        sendQualBfr[sendQualIdx] = sendQual;
        sendQual = 0;
        sendTotalBfr[sendQualIdx] = totalSends;
        totalSends = 0;
        sendFailBfr[sendQualIdx] = sendFail;
        sendFail = 0;
        msgTotalBfr[sendQualIdx] = msgSendTotal;
        msgSendTotal = 0;
        if (++sendQualIdx >= QUAL_BFR_SIZE)
            sendQualIdx = 0; */
        sendQuality = true;
       
    }
    repeat++;
  }
  
  if (ack == false) // after repeated sending still no ack
    sendFail++;
  #ifdef DIS_INT_ON_TX
    wait(10);
    EIFR = bit (INTF1);  // clear flag for interrupt 1 to avoid triggering on sending msg
  #endif
  return (ack);
}


void sendConnectionQuality(void)
{
  /* 
  unsigned long sendQualTotal = 0, sendTotal = 0, sendFailTotal = 0, msgTotal = 0;
  for (uint8_t i = 0; i < QUAL_BFR_SIZE; i++)
    sendQualTotal += sendQualBfr[i];
  for (uint8_t i = 0; i < QUAL_BFR_SIZE; i++)
    sendTotal += sendTotalBfr[i];
  for (uint8_t i = 0; i < QUAL_BFR_SIZE; i++)
    sendFailTotal += sendFailBfr[i];
  for (uint8_t i = 0; i < QUAL_BFR_SIZE; i++)
    msgTotal += msgTotalBfr[i];
    
  uint32_t qual = ((sendTotal - sendQualTotal) * 1000UL) / sendTotal;
  uint32_t qualFail = ((msgTotal - sendFailTotal) * 1000UL) / msgTotal; */
  resend(msgConnectionFail.set(qualFail), 2);
  resend(msgConnectionQual.set(qual), 2);
 /*  #ifdef DEBUG
    Serial.print("Quality ");
    Serial.print(qual);
    Serial.print(" Total ");
    Serial.print(sendTotal);
    Serial.print(" send ok ");
    Serial.print(sendOk);
    Serial.print(" fail ");
    Serial.println(qualFail);
  #endif */
}

void pir_sensor(void)
{
  static uint32_t motionMsgResendTimer;
  static uint32_t retryTimer;
  // Read digital motion value
  boolean motion = digitalRead(MOTION_INPUT) == HIGH; 
  if (lastMotion != motion || millis() > motionMsgResendTimer) 
  {
    if (lastMotion != motion)
    {
      lastMotion = motion;     
      motionChange = true;
    }
    if (!resend(msgMot.set(motion ? "1" : "0" ), 2))  // Send motion value to gw
      retryTimer = (5 * 100L);
    else
      retryTimer = (30 * 1000L);
    #ifdef DEBUG 
    Serial.print("motion ");
    Serial.print(motion ? "1" : "0" );
    Serial.print(" sends ");
    Serial.print(totalSends); 
    Serial.print(" resend in ");
    Serial.println(retryTimer); 
    #endif
    motionMsgResendTimer = millis() + retryTimer;
    if (motion)
    {
      motionTimeout = millis();
      // requestedLevel[0] = preferredLevel[0];
    }
  }
}

void dht_sensor(void)
{
  if (secTimeout(lastDhtRead, INTERVAL_DHT))
  {
    lastDhtRead = millis();
    delay(dht.getMinimumSamplingPeriod()); 
    float temperature = dht.getTemperature();
    if (isnan(temperature)) {
      Serial.println("Failed reading temperature from DHT");
    } 
    else// if (temperature != lastTemp) 
    {
      lastTemp = temperature;
      resend(msgTemp.set(temperature, 1), 1);
      Serial.print("T: ");
      Serial.println(temperature);
    }
  
    float humidity = dht.getHumidity();
    if (isnan(humidity)) {
      Serial.println("Failed reading humidity from DHT");
    } else // if (humidity != lastHum) 
    {
      lastHum = humidity;
      resend(msgHum.set(humidity, 1), 1);
      Serial.print("H: ");
      Serial.println(humidity);
    }
  }
  
}

void light_sensor(void)
{
  int delta;
  
  int lightLevel = (1023-analogRead(LIGHT_SENSOR_ANALOG_PIN))/10.23; 
  delta = lightLevel - lastLightLevel;
  
  if (secTimeout(lastLightRead, INTERVAL_LIGHT) /* || motionChange  */|| (delta >= FAST_LIGHT_CHANGE_DIFF) || (delta <= -FAST_LIGHT_CHANGE_DIFF) )
  {
    motionChange = false;
    lastLightRead = millis();
    
    if (lightLevel != lastLightLevel) 
    {
      resend(msgLight.set(lightLevel), 2);
      lastLightLevel = lightLevel;
      #ifdef DEBUG
      Serial.print("L: ");
      Serial.println(lightLevel);
      #endif
    }
  }
} 

void localFunctionality()
{
  switch (state)
  {
    case 0:   // idle
      if (digitalRead(MOTION_INPUT))
      {
        dimmer[0].setLevel(configValues.onValue);
        motionTimer = millis();
        state = 1;
        Serial.println("Motion, switch on");
      }
      break;
    
    case 1:   // lights on
      if (digitalRead(MOTION_INPUT))
        motionTimer = millis();
      else if (secTimeout(motionTimer, configValues.onTime))
      {
        Serial.println("Switching off");
        dimmer[0].setLevel(0);
        motionTimer = millis();
        state = 0;
      }
      break;
    
    default:
      break;
  }

}


void loop()      
{
  for (byte i = 0; i < 2; i++)
    dimmer[i].process();
  controller.monitor();
  pir_sensor();
  dht_sensor();
  light_sensor();
  parameters.process();
  if (sendQuality)
  {  
    sendQuality = false;
    sendConnectionQuality();
  }
  if (!controller.alive() || configValues.localMode)
    localFunctionality();
}





