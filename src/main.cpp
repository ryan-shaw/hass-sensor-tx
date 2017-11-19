
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
#define DHTPIN 2
#define DHTTYPE DHT11     // DHT 11 
DHT_Unified dht(DHTPIN, DHTTYPE);

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
#define nodeID 2


uint32_t displayTimer = 0;

struct payload_t {
  unsigned long ms;
  unsigned long counter;
};

struct dataStruct{
  uint16_t sensor;
  uint16_t type;
  float value;
};

dataStruct myData;

void setup() {

  Serial.begin(115200);

  // Setup  DHT11 sensor
  pinMode(DHTPIN, INPUT);
  // Set the nodeID manually
  mesh.setNodeID(nodeID);
  // Connect to the mesh
  Serial.println(F("Connecting to the mesh..."));
  mesh.begin();
}

void send(struct dataStruct msg) {
  // Send an 'M' type message containing the current millis()
  if (!mesh.write((uint8_t*) &msg, 'M', sizeof(msg))) {
    // If a write fails, check connectivity to the mesh network
    if ( ! mesh.checkConnection() ) {
      //refresh the network address
      Serial.println("Renewing Address");
      mesh.renewAddress();
    } else {
      Serial.println("Send fail, Test OK");
    }
  } else {
    Serial.print("Send OK: "); Serial.println(displayTimer);
  }
}

void loop() {
  radio.powerUp();
  mesh.update();
  uint32_t delay = 1000;
  // if (displayTimer == 0 || millis() - displayTimer >= delay) {
    displayTimer = millis();
    pinMode(DHTPIN, OUTPUT);
    digitalWrite(DHTPIN, HIGH);
    dht.begin();
    LowPower.powerDown(SLEEP_500MS, ADC_OFF, BOD_OFF);
    digitalWrite(DHTPIN, HIGH);
    sensors_event_t event;  
    dht.temperature().getEvent(&event);
    if (isnan(event.temperature)) {
      Serial.println("Error reading temperature!");
    } else {
      Serial.print("Temperature: ");
      Serial.print(event.temperature);
      Serial.println(" *C");
      
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
      Serial.print("Humidity: ");
      Serial.print(event.relative_humidity);
      Serial.println("%");

      // dataStruct myData;
      myData.sensor = nodeID;
      myData.type = HUMIDITY;
      myData.value = event.relative_humidity;
      send(myData);
    }
    radio.powerDown();
    digitalWrite(DHTPIN, LOW);
    pinMode(DHTPIN, INPUT);
  // }
  for ( int i = 0; i < 75; i++){
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  }

  // while (network.available()) {
  //   RF24NetworkHeader header;
  //   payload_t payload;
  //   network.read(header, &payload, sizeof(payload));
  //   Serial.print("Received packet #");
  //   Serial.print(payload.counter);
  //   Serial.print(" at ");
  //   Serial.println(payload.ms);
  // }
}