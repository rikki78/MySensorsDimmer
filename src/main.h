#include <Arduino.h>
#define TEST_DIMMER


#define RF24_PA_LEVEL RF24_PA_MAX
#ifdef SLAAPKAMER
  #define MY_NODE_ID 6  // Slaapkamer
  #define MY_BAUD_RATE  115200 // Sets the serial baud rate for console and serial gateway

  // Enable repeater functionality for this node
  #define MY_REPEATER_FEATURE
  #define MY_RX_MESSAGE_BUFFER_FEATURE
  #define MY_RX_MESSAGE_BUFFER_SIZE 10  
  #define MY_RF24_IRQ_PIN   2
#endif
#ifdef TEST_DIMMER
  #define MY_NODE_ID 34  // test dimmer
  #define MY_BAUD_RATE  9600 // Sets the serial baud rate for console and serial gateway
  
  // Enable repeater functionality for this node
  #define MY_REPEATER_FEATURE
  
#define MY_PARENT_NODE_IS_STATIC
#define MY_PARENT_NODE_ID 40
// #define MY_RX_MESSAGE_BUFFER_FEATURE
 // #define MY_RX_MESSAGE_BUFFER_SIZE 20  
  #define MY_RF24_IRQ_PIN   2

#endif

#define MY_DEBUG_VERBOSE_TRANSPORT_HAL
#define MY_DEBUG_VERBOSE_TRANSPORT
//#define MY_DEBUG_VERBOSE_RF24 
#define MY_RADIO_RF24
#define DEBUG
#define MY_DEBUG    // Enables debug messages in the serial log
//#define MY_NODE_ID 34  // Sets a static id for a node

#define SN "Dimmer with PIR"
#define SV "3.0"

#include <MySensors.h>  
#include <Dimmer.h>
#include <ControllerMonitor.h>
#include <timeout.h>

#include "MySensorsParameters.h"
//#include "ConfigParameters.h"

#include <SPI.h>
#include <DHT.h>  
void setup();
uint8_t request_values(uint8_t set);
void configLoad();
uint8_t requestParameters(uint8_t parameterToRead);
uint8_t paramStatusReport(uint8_t status);
void presentation();
uint16_t getInt(uint32_t value);
uint16_t highInt(uint32_t value);
uint8_t getIndex(uint32_t value);
void receive(const MyMessage &message);
boolean resend(MyMessage &msg, int repeats); // Resend messages if not received by gateway;
void sendConnectionQuality(void);
void pir_sensor(void);
void dht_sensor(void);
void light_sensor(void);
void localFunctionality();
void loop();