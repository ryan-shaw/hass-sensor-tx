
/** RF24Mesh_Example.ino by TMRh20

   This example sketch shows how to manually configure a node via RF24Mesh, and send data to the
   master node.
   The nodes will refresh their network address as soon as a single write fails. This allows the
   nodes to change position in relation to each other and the master node.
*/


#include "RF24.h"
#include "RF24Network.h"
#include "RF24Mesh.h"
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <LowPower.h>
//#include <printf.h>

#define TEMP 0
#define HUMIDITY 1


/**** Configure DHT11 */
#define DHT_PIN 2
#define DHT_PWR 3
#define DHTTYPE DHT11     // DHT 11 
#define TX_PWR 4
DHT_Unified dht(DHT_PIN, DHTTYPE);

/**** Configure the nrf24l01 CE and CS pins ****/
RF24 radio(7, 8);
RF24Network network(radio);
RF24Mesh mesh(radio, network);

/**
   User Configuration: nodeID - A unique identifier for each radio. Allows addressing
   to change dynamically with physical changes to the mesh.

   In this example, configuration takes place below, prior to uploading the sketch to the device
   A unique value from 1-255 must be configured for each node.
   This will be stored in EEPROM on AVR devices, so remains persistent between further uploads, loss of power, etc.

 **/
#define nodeID 1
#define DEBUG true


uint32_t displayTimer = 0;

struct dataStruct{
  uint16_t sensor;
  uint16_t type;
  float value;
};

dataStruct myData;

void setup() {

  if(DEBUG) Serial.begin(115200);

  // radio.setPALevel(RF24_PA_MAX);
  // radio.setCRCLength(RF24_CRC_8);

  // Setup  DHT11 sensor
  pinMode(DHT_PIN, INPUT);
  pinMode(DHT_PWR, OUTPUT);
  pinMode(TX_PWR, OUTPUT);
  // Set the nodeID manually
  mesh.setNodeID(nodeID);
  // Connect to the mesh
  if(DEBUG) Serial.println(F("Connecting to the mesh..."));
  digitalWrite(TX_PWR, HIGH);
  if(DEBUG) Serial.flush();
  LowPower.powerDown(SLEEP_120MS, ADC_OFF, BOD_OFF);
  mesh.begin(MESH_DEFAULT_CHANNEL, RF24_250KBPS, 15000);
}

void send(struct dataStruct msg) {
  // Send an 'M' type message containing the current millis()
  if (!mesh.write((uint8_t*) &msg, 'M', sizeof(msg))) {
    // If a write fails, check connectivity to the mesh network
    if ( ! mesh.checkConnection() ) {
      //refresh the network address
      if(DEBUG) Serial.println("Renewing Address");
      mesh.renewAddress(15000);
    } else {
      if(DEBUG) Serial.println("Send fail, Test OK");
    }
  } else {
    if(DEBUG) Serial.print("Send OK: "); Serial.println(displayTimer);
  }
}

void pUp(){
  if(DEBUG) Serial.println("Powering up");
  for(int i = 7; i < 14; i++){
    pinMode(i, OUTPUT);
  }
  // pinMode(7, OUTPUT); // set ce pin output
  // pinMode(8, OUTPUT); // set csn pin output
  pinMode(TX_PWR, OUTPUT);
  digitalWrite(TX_PWR, HIGH);
  if(DEBUG) Serial.flush();
  LowPower.powerDown(SLEEP_120MS, ADC_OFF, BOD_OFF);
  if(DEBUG) Serial.println("Mesh begin");
  mesh.begin(MESH_DEFAULT_CHANNEL, RF24_250KBPS, 15000);
  if(DEBUG) Serial.println("Radio power up");
  radio.powerUp();
  if(DEBUG) Serial.println("Mesh update");
  mesh.update();
  digitalWrite(DHT_PWR, HIGH);
  if(DEBUG) Serial.flush();
  displayTimer = millis();
  LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);
  dht.begin();
}

void pDown(){
  if(DEBUG) Serial.println("Powering down");
  if(DEBUG) Serial.flush();
  radio.powerDown();
  LowPower.powerDown(SLEEP_120MS, ADC_OFF, BOD_OFF);
  digitalWrite(TX_PWR, LOW);
  pinMode(TX_PWR, INPUT_PULLUP);
  digitalWrite(DHT_PWR, LOW);
  for(int i = 7; i < 14; i++){
    pinMode(i, INPUT_PULLUP);
  }
  for ( int i = 0; i < 75; i++){
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  }
}

void loop() {
  pUp();
  if(DEBUG) Serial.println("Completed powerup");
  sensors_event_t event;  
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    if(DEBUG) Serial.println("Error reading temperature!");
  } else {
    if(DEBUG) Serial.print("Temperature: ");
    if(DEBUG) Serial.print(event.temperature);
    if(DEBUG) Serial.println(" *C");
    
    // dataStruct myData;
    myData.sensor = nodeID;
    myData.type = TEMP;
    myData.value = event.temperature;
    send(myData);
  }

  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    // Serial.println("Error reading humidity!");
  } else {
    if(DEBUG) Serial.print("Humidity: ");
    if(DEBUG) Serial.print(event.relative_humidity);
    if(DEBUG) Serial.println("%");

    // dataStruct myData;
    myData.sensor = nodeID;
    myData.type = HUMIDITY;
    myData.value = event.relative_humidity;
    send(myData);
  }
  pDown();
}