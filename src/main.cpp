// PowMr mqtt modbus bridge


#include <FS.h>                   //this needs to be first, or it all crashes and burns...
#include "LittleFS.h"
#include <Arduino.h>

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

//needed for library
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson

#include <PubSubClient.h>         // MQTT client

#include <SimpleTimer.h>          // Simple Task Time Manager
SimpleTimer timer;

#include <ModbusMaster.h>
ModbusMaster node;

// R232 level shifter connections

#define SS_TX_PIN   D8    // GPIO15
#define SS_RX_PIN   D7    // GPIO13
#include <SoftwareSerial.h>
SoftwareSerial Ser1(SS_RX_PIN,SS_TX_PIN);

#define VERSION             1.04

#define DEBUG(x)            Serial.println(x);


ADC_MODE(ADC_VCC);
int vdd;

// stay alive 
bool stay_alive = false;

//flag for saving data
bool shouldSaveConfig = false;

// MQTT
char conf_server_ip[32] = "mqtt.local";
char conf_server_port[6] = "1883";
uint16_t conf_server_port_int;

#define     MAX_MSG     50
WiFiClient espClient;
PubSubClient client(espClient);
char msg[MAX_MSG + 1];

#define PUBLISH_PREFIX            "/iot/node/powmr/s"
const char* publishTopic_prefix = "/iot/node/powmr/s";
const char* publishTopic        = "/iot/node/powmr/log/modbus";
const char* publishTopic_raw1   = "/iot/node/powmr/log/modbus45";
const char* publishTopic_raw2   = "/iot/node/powmr/log/modbus16";
const char* publishTopicLog     = "/iot/node/powmr/log/console";
const char* mqtt_topic_cmd      = "/iot/node/powmr/c/+";


// global invertor state

uint8_t i_state_available = 0;
float batt_v_compensation_k = 0.01;  // about 0.4v at 60a, 60 * x = 0.4 
float batt_v_corrected; 

uint8_t charger_active = 0;

uint16_t op_mode;
float ac_voltage;
float ac_freq;
float pv_voltage;
float pv_power;
float batt_voltage;
float batt_voltage_;
float batt_charge_current;
float batt_charge_current_;
float batt_discharge_current;
float batt_discharge_current_;
float output_power;
float output_load;

float batt_soc = 0;

// battery upper float voltage setpoint
float batt_voltage_charged = 24.2;
uint8_t batt_max_charge_current = 60;
uint8_t charge_current = 0;


void set_util_charge_current(uint8_t); 

/**
 * @fn int strend(const char *s, const char *t)
 * @brief Searches the end of string s for string t
 * @param s the string to be searched
 * @param t the substring to locate at the end of string s
 * @return one if the string t occurs at the end of the string s, and zero otherwise
 */
int strend(const char *s, const char *t) {
    size_t ls = strlen(s); // find length of s
    size_t lt = strlen(t); // find length of t
    if (ls >= lt)  // check if t can fit in s
    {
        // point s to where t should start and compare the strings from there
        return (0 == memcmp(t, s + (ls - lt), lt));
    }
    return 0; // t was longer than s
}

//callback notifying us of the need to save config
void saveConfigCallback() {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

void alivePrint() {
  Serial.print(".");
}

void hexDump(const uint8_t*b, int len){
  //#ifdef DEBUG_PRINTS
  Serial.println();
  for (int i=0; i < len; i = i + 16) {
    Serial.print("           ");
    for(int x=0; x<16 && (x+i) < len; x++) {
      if(b[i+x]<=0xf) Serial.print("0");
      Serial.print(b[i+x],HEX);
      Serial.print(" ");
    }
    Serial.print(" ");
    for(int x=0; x<16 && (x+i) < len; x++) {
      if (b[i+x]<=32||b[i+x] >= 126) {
          Serial.print(".");
      } else Serial.print((char)b[i+x]);
    }
    Serial.print("\n");
  }
  Serial.print("                   Length: ");
  Serial.println(len);
//  #endif
}

void falltosleep() {
  DEBUG("Sleep...\n");
  ESP.deepSleep(60e6); // 60 sec
  // RF_NO_CAL
  // ESP.deepSleepInstant(microseconds, mode); // mode WAKE_RF_DEFAULT, WAKE_RFCAL, WAKE_NO_RFCAL, WAKE_RF_DISABLED
}

// mqtt subscribe callback / command topic
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("\nMessage arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  if (strend(topic,"/txpow")) {
      Serial.println("TX power set");
      payload[length] = '\0'; // might be unsafe
      float txpow = atof((char *)payload);
      WiFi.setOutputPower(txpow);        // float 0 - 20.5 ->>> 4.0 * val (0-82)
  }

  if (strend(topic,"/reboot")) {
    Serial.println("Topic Reboot...");
    Serial.flush();
    delay(1000);
    ESP.reset();
    delay(5000);
  }

  #define TOKEN_SEP   " "

  // set util charge current to 2 a
  //  '/iot/node/powmr/c/modbus_write_single' -m "5024 2"
  //  
  // set solar only charger: 
  // func 6, (5017), 0003
  // /iot/node/powmr/s/rec 0506139900031ce4
  //
  // set util and solar charger:
  // func 6, (5017), 0002

  if (strend(topic,"/modbus_write_single")) {
    uint16_t reg_n; 
    uint16_t reg_val;
    char* ptr;
    ptr = (char *)payload;
    static char* seq_tok_last; // last char in tokenizer

    *(payload + length) = '\0';
    ptr = strtok_r(ptr, TOKEN_SEP, &seq_tok_last);

    if (ptr != NULL) {
      reg_n = atoi(ptr);
      ptr = strtok_r(NULL, TOKEN_SEP, &seq_tok_last);
      if (ptr != NULL) {
        reg_val = atoi(ptr);
        uint8_t res = node.writeSingleRegister(reg_n, reg_val);
        char buf[10];
        snprintf(buf, 9,  "%u", res);
        client.publish(publishTopicLog, buf);
      }
    } 
  }


  if (strend(topic,"/modbus_read_single")) {
    *(payload + length) = '\0';
    uint16_t reg_n = atoi((char *)payload);

    uint8_t res = node.readHoldingRegisters(reg_n, 1);
    uint16_t response;
    if (res == node.ku8MBSuccess) {
      response = node.getResponseBuffer(0);
      char buf[10];
      char topic_buf[50];
      snprintf(buf, 9, "%u", response);
      snprintf(topic_buf, 49, "%s/%u", publishTopic, reg_n);
      client.publish(topic_buf, buf);
    } else {
      client.publish(publishTopicLog, "error reading from holding register");
    }
  }

  if (strend(topic,"/set_charge_current")) {
    *(payload + length) = '\0';
    uint16_t cc = atoi((char *)payload);
    set_util_charge_current(cc);
  }

  if (strend(topic,"/set_charge_current_limit")) {
    *(payload + length) = '\0';
    batt_max_charge_current = atoi((char *)payload);
  }

  if (strend(topic,"/set_batt_voltage_charged")) {
    *(payload + length) = '\0';
    batt_voltage_charged = atof((char *)payload);
  }
}

void reconnect() {
  // Loop until we're reconnected
  if (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      client.subscribe(mqtt_topic_cmd);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" ");
      // should we retry ???
      // Wait 5 seconds before retrying
      //delay(5000);
      //timerId = timer.setTimeout(5000,reconnect);
    }
  }
}

int getRSSI() {
  // print the received signal strength:
  int rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI): ");
  Serial.print(rssi);
  Serial.println(" dBm");
  return(rssi);
}

void publish_float(const char *topic, float var) {
    char buf[11];
    snprintf(buf, 10,"%3.1f", var);
    client.publish(topic, buf);
}

void publish_float4(const char *topic, float var) {
    char buf[11];
    snprintf(buf, 10,"%3.4f", var);
    client.publish(topic, buf);
}


// requests data from Slave device
void send_request() {
  uint8_t j, result;
  #define MAX_MBUS_WORDS  50
  uint16_t data[MAX_MBUS_WORDS];

  #define MAX_MBUS_PKT_S (MAX_MBUS_WORDS*2)

  static uint8_t charArr[2*MAX_MBUS_PKT_S + 1]; //Note there needs to be 1 extra space for this to work as snprintf null terminates.
  uint8_t *myPtr;
  myPtr = charArr; 

      // variables
  // read 45 registers (func 3) from slave 5 starting from address 4501 (Decimal) 
  // 05031195002d9143
      // state
  // read 16 registers (func 3) from slave 5 starting from address 4546 (Decimal) 11C2
  // 050311c20010e142

  uint16_t nregisters = 45;

  result = node.readHoldingRegisters(4501, nregisters);

  // do something with data if read is successful
  if (result == node.ku8MBSuccess)
  {
    for (j = 0; j < 45; j++)
    {
      data[j] = node.getResponseBuffer(j);
    }
    // dump data to mqtt topic

    myPtr = charArr; 
    for (uint16_t i = 0; i < nregisters; i++){
      snprintf((char *)myPtr, 5,"%04x", data[i]); 
      myPtr += 4; 
    }

    client.publish(publishTopic_raw1, charArr, nregisters*2*2);


    // convert and publish variables

    op_mode = htons(data[0]);
    myPtr = charArr; 
    snprintf((char *)myPtr, 5, "%04x", op_mode);
    client.publish("/iot/node/powmr/s/mode", charArr, strlen((char *)charArr)); 


    ac_voltage = htons(data[1]) / 10.0;
    publish_float("/iot/node/powmr/s/ac_voltage", ac_voltage);

    ac_freq = htons(data[2]) / 10.0;
    publish_float("/iot/node/powmr/s/ac_freq", ac_freq);


    pv_voltage = htons(data[3]) / 10.0;
    publish_float("/iot/node/powmr/s/pv_voltage", pv_voltage);

    pv_power = (float) htons(data[4]);
    publish_float("/iot/node/powmr/s/pv_power", pv_power);


    batt_voltage = htons(data[5]) / 10.0;
    publish_float("/iot/node/powmr/s/batt_voltage", batt_voltage);

    batt_charge_current = htons(data[7]);
    publish_float("/iot/node/powmr/s/batt_charge_current", batt_charge_current);

    batt_discharge_current = htons(data[8]);
    publish_float("/iot/node/powmr/s/batt_discharge_current", batt_discharge_current);

    output_power = htons(data[11]);
    publish_float("/iot/node/powmr/s/output_power", output_power);

    output_load = htons(data[12]) / 20.0;
    publish_float("/iot/node/powmr/s/output_load", output_load);

    // update wires resistanse coeff
    float new_k;
    //uint8_t update_k = 0;
    /*
    if (i_state_available && batt_discharge_current_ - batt_discharge_current > 10) {
      new_k = abs(batt_voltage_ - batt_voltage) / abs(batt_discharge_current_ - batt_discharge_current);
      update_k = 1;
    }
    if (i_state_available && batt_charge_current - batt_charge_current_ > 10) {
      new_k = abs(batt_voltage - batt_voltage_) / abs(batt_charge_current - batt_discharge_current_);
      update_k = 1;
    }
    */
    float charge_current_change = -(batt_discharge_current - batt_discharge_current_) 
                                    + (batt_charge_current - batt_charge_current_);
    if (i_state_available 
          &&  abs(charge_current_change) > 5.0) {
        new_k = (batt_voltage - batt_voltage_) / charge_current_change;

        batt_v_compensation_k = batt_v_compensation_k + (new_k - batt_v_compensation_k) * 0.1;

        myPtr = charArr; 
        snprintf((char *)myPtr, 100,"New voltage coeff: %3.5f, updated batt_v_compensation_k %3.5f", new_k, batt_v_compensation_k); 
        client.publish(publishTopicLog, (char *) charArr);
    }

    batt_voltage_ = batt_voltage;
    batt_charge_current_ = batt_charge_current;
    batt_discharge_current_ = batt_discharge_current;

    i_state_available = 1; 
  }

  nregisters = 16;
  result = node.readHoldingRegisters(4546, nregisters);

  if (result == node.ku8MBSuccess)
  {
    for (j = 0; j < 45; j++)
    {
      data[j] = node.getResponseBuffer(j);
    }
    // dump data to mqtt topic

    myPtr = charArr; 

    for (uint16_t i = 0; i < nregisters; i++){
      snprintf((char *)myPtr, 5,"%04x", data[i]); 
      myPtr += 4;
    }

    client.publish(publishTopic_raw2, charArr, nregisters*2*2);
  }


}

// runs when we are waiting for modbus data
void idle() {
    ArduinoOTA.handle();

    if ( !client.connected() ) {
        reconnect();
    }

    client.loop();

    yield();
}

uint8_t allowed_charger_currents[] = {0, 2, 10, 20, 30, 40, 50, 60};

// set utility charge current
// 0 - switch to solar only
// 2, 10, 20, 30, 40, 50, 60
void set_util_charge_current(uint8_t cc) {
  uint8_t res;
  uint8_t try_cnt = 0;
  char buf[60];

    if (cc == 0) {
      charger_active = 0;
      while ( // solar only
              (res = node.writeSingleRegister(5017, 3)) != 0 
              && try_cnt++ < 5
      );
      if (res != 0) { charger_active = 1; }
      snprintf(buf, 50,  "Charger disconnected, solar only, res = %u", res);
      client.publish(publishTopicLog, buf);
    } else {
      // set utility charge current
      while ( 
              (res = node.writeSingleRegister(5024, cc)) != 0 
              && try_cnt++ < 5 
      );
      snprintf(buf, 50,  "Charger current set to %u A, res = %u", cc, res);
      client.publish(publishTopicLog, buf);

      if (!charger_active) {
        charger_active = 1;
        try_cnt = 0;
        // set util and solar charge mode
        while (
                (res = node.writeSingleRegister(5017, 2)) != 0
                && try_cnt++ < 5
        );
        if (res != 0) { charger_active = 0; }
        snprintf(buf, 50,  "Charger enabled, res = %u", res);
        client.publish(publishTopicLog, buf);
      } 
    }
}



// runs periodically to monitor and control battery voltage 
void controller() {
  if (i_state_available) {
    batt_v_corrected = batt_voltage - (batt_v_compensation_k * batt_charge_current)
                                    + (batt_v_compensation_k * batt_discharge_current);
    publish_float4("/iot/node/powmr/s/batt_voltage_corrected", batt_v_corrected);
    publish_float4("/iot/node/powmr/s/batt_v_compensation_k", batt_v_compensation_k);

    batt_soc = 100.0 * (batt_voltage - 20.0) / (25.0 - 20.0);
    publish_float("/iot/node/powmr/s/batt_soc", batt_soc);

/*
    uint16_t charge_current = 2;
    if (batt_voltage_charged - batt_v_corrected > 0.4) {
      // set max allowed charge current
      charge_current = batt_max_charge_current;
    } else if (batt_voltage_charged - batt_v_corrected > 0.2) {
      charge_current = batt_max_charge_current / 2;
      if (charge_current < 10) charge_current = 2;
      if (charge_current == 25) charge_current = 20;
      if (charge_current == 15) charge_current = 10;
    } else if (batt_voltage_charged - batt_v_corrected >= 0.1) {
      charge_current = 2;
    } else if (batt_voltage_charged <= batt_v_corrected) {
      charge_current = 0;
    } 
    set_util_charge_current(charge_current);
*/
    if (batt_voltage == batt_voltage_charged) {
      // do nothing, keep the same charge current
    } else {
      // battery voltage is out of desired state
      
      if (batt_voltage >= batt_voltage_charged) {
        // decrease charger current
        if (charge_current == 2) {
          charge_current = 0;
        } else if (charge_current == 10) {
          charge_current = 2;
        } else if (charge_current > 10) {
          charge_current = charge_current - 10;
        }
      } else {
        // increase charger current
        if (charge_current < batt_max_charge_current) {
            if (charge_current == 0) {
              charge_current = 2;
            } else if (charge_current == 2) {
              charge_current = 10;
            } else if (charge_current < 60) {
              charge_current = charge_current + 10;
            }
        }
      }

      /*
        if (batt_voltage_charged - batt_voltage >= 0.3) {
          // set max allowed charge current
          charge_current = batt_max_charge_current;
        } else if (batt_voltage_charged - batt_voltage >= 0.2) {
          if (batt_max_charge_current < 10) {
            charge_current = 2;
          } else {
            charge_current = 20;
          }
        } else if (batt_voltage_charged - batt_voltage >= 0.1) {
          if (charge_current == 0) {
            charge_current = 2;
          }
        }
        */
    }

    if (charge_current > batt_max_charge_current) {
      charge_current = batt_max_charge_current;
    }

    set_util_charge_current(charge_current); 
  }
}


void setup() {
  //unsigned long StartTime = millis();

  //pinMode(holdPin, OUTPUT);  // sets GPIO 0 to output
  //digitalWrite(holdPin, HIGH);  // sets GPIO 0 to high (this holds CH_PD high even if the PIR output goes low)

  //Serial.begin(2400); // looks like it's a main speed
  Serial.begin(250000);
  Ser1.begin(2400);
  Ser1.enableIntTx(false);
  //Ser1.enableRxGPIOPullUp(true);

  delay(10);
  Serial.printf("\n\nSDK version:%s\n\r", system_get_sdk_version());
  Serial.print("Firmware version: ");
  Serial.println(VERSION);

  Serial.println("Booting...");

  pinMode(LED_BUILTIN, OUTPUT);     // Initialize the BUILTIN_L
  digitalWrite(LED_BUILTIN, HIGH);  // turn off

  //clean FS, for testing
  //SPIFFS.format();
  //read configuration from FS json
  Serial.println("mounting FS");

  if (LittleFS.begin()) {
    Serial.println("mounted file system");
    if (LittleFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = LittleFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          Serial.println("\nparsed json");
          if (json["server_ip"]) {
            strcpy(conf_server_ip, json["server_ip"]);
            strcpy(conf_server_port, json["server_port"]);
          }
        } else {
          Serial.println("failed to load json config");
        }
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }
  //end read

  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  WiFiManagerParameter c_conf_server_ip("server_ip", "MQTT server ip", conf_server_ip, 32);
  WiFiManagerParameter c_conf_server_port("port", "MQTT server port", conf_server_port, 6);

  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  //add all your parameters here
  wifiManager.addParameter(&c_conf_server_ip);
  wifiManager.addParameter(&c_conf_server_port);

  //reset settings - for testing
  //wifiManager.resetSettings();

  //set minimu quality of signal so it ignores AP's under that quality
  //defaults to 8%
  //wifiManager.setMinimumSignalQuality();

  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep
  //in seconds
  wifiManager.setTimeout(180);

  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  if (!wifiManager.autoConnect("ESP-POWMR","XXXXXXXX")) {
    Serial.println("failed to connect and hit timeout...");
  }

  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");

  //read updated parameters
  strcpy(conf_server_ip, c_conf_server_ip.getValue());
  strcpy(conf_server_port, c_conf_server_port.getValue());
  conf_server_port_int = atoi(conf_server_port);

  //save the custom parameters to FS
  if (shouldSaveConfig) {
    Serial.println("saving config");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();

    json["server_ip"] = conf_server_ip;
    json["server_port"] = conf_server_port;

    json["ip"] = WiFi.localIP().toString();
    json["gateway"] = WiFi.gatewayIP().toString();
    json["subnet"] = WiFi.subnetMask().toString();

    File configFile = LittleFS.open("/config.json", "w");
    if (!configFile) {
        Serial.println("failed to open config file for writing");
    } else {
        json.printTo(Serial);
        json.printTo(configFile);
        configFile.close();
        //end save
    }
  }

  //Serial.print("\nlocal ip: ");
  //Serial.println(WiFi.localIP());

  //WiFi.printDiag(Serial);

  Serial.print("DNS Lookup ...");

  IPAddress mqttServerIP;

  if (!WiFi.hostByName(conf_server_ip, mqttServerIP)) { // Get the IP address of the NTP server
    Serial.println("DNS lookup failed. Rebooting...");
    //falltosleep();
    delay(1000);
    //ESP.reset();
  }
  Serial.print("MQTT server IP:\t");
  Serial.println(mqttServerIP);

/// end Of WifiManager portal

  //WiFi.mode(WIFI_STA);
  //WiFi.begin(ssid, password);
  //while (WiFi.waitForConnectResult() != WL_CONNECTED) {
  //  Serial.println("Connection Failed! Rebooting...");
  //  delay(5000);
  //  ESP.restart();
  //}

  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  // ArduinoOTA.setHostname("myesp8266");

  // No authentication by default
  // ArduinoOTA.setPassword("foxyfox3");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("\nOTA: Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });

  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());


  client.setServer((const char *)conf_server_ip, conf_server_port_int);
  client.setCallback(callback);
  //client.setBufferSize(MQTT_MAX_PACKET_SIZE);

  DEBUG(conf_server_ip);
  DEBUG(conf_server_port_int);

  if ( !client.connected() ) {
    reconnect();
  }

  Serial.println("Publishing to mqtt");
  vdd = ESP.getVcc();
  snprintf (msg, MAX_MSG, "%ld", vdd);
  client.publish(publishTopicLog, msg);

  // communicate with Modbus slave ID 5 over Serial
  node.begin(5, Ser1);
  node.idle(idle);
  // private method
  //node.ku16MBResponseTimeout = 20; // ms of response timeout

  //timer.setInterval(2000,alivePrint);

  timer.setInterval(2000, send_request);
  timer.setInterval(15000, controller);

  Serial.println("To the main loop...");

}

#define TIME_OUT  15
// 512
#define MAX_MBUS_PKT  500

void loop() {

    ArduinoOTA.handle();

    if ( !client.connected() ) {
        reconnect();
    }

    client.loop();
    timer.run();

    static unsigned long timeLastInput = 0;
    unsigned long now = millis();
    static char buffer[MAX_MBUS_PKT + 1];
    static int index = 0;

    // this part also used to sniff communication with the original dongle

    if (Serial.available() > 0) {
        char x = Serial.read();
        timeLastInput = now;

        if (index < MAX_MBUS_PKT) {
            buffer[index++] = x;
        }
    }

    if (now - timeLastInput > TIME_OUT || index >= MAX_MBUS_PKT) {
      if (index > 0) {
        // Serial.println("Time out");
        // hexDump((uint8_t *) buffer, index);

        static uint8_t charArr[2*MAX_MBUS_PKT + 1]; //Note there needs to be 1 extra space for this to work as snprintf null terminates.
        uint8_t *myPtr;
        myPtr = charArr; 

        for (uint16_t i = 0; i < index; i++){
          snprintf((char *)myPtr, 3,"%02x", buffer[i]); //convert a byte to character string, and save 2 characters (+null) to charArr;
          myPtr += 2; //increment the pointer by two characters in charArr so that next time the null from the previous go is overwritten.
        }

        client.publish(publishTopic, charArr, index*2);

        index = 0;
        timeLastInput = now;
      } else {
        //Serial.print(".");
      }
    }
}


/*

defult mode
RTU reply

slave 5 reply, func 3, 32 bytes, 
on line
/iot/node/powmr/s/rec 050320fa00fa00c800fa003c0078001e0062d9e90e0000000045000000000000000000f1ca
/iot/node/powmr/s/rec 050320fa00fa00c800fa003c0078001e0062d9e90e0000000045000000000000000000f1ca
/iot/node/powmr/s/rec 050320fa00fa00c800fa003c0078001e0062d9e90e0000000045000000000000000000f1ca
/iot/node/powmr/s/rec 050320fa00fa00c800fa003c0078001e0062d9e90e0000000045000000000000000000f1ca
/iot/node/powmr/s/rec 050320fa00fa00c800fa003c0078001e0062d9e90e0000000045000000000000000000f1ca
/iot/node/powmr/s/rec 050320fa00fa00c800fa003c0078001e0062d9e90e0000000045000000000000000000f1ca
/iot/node/powmr/s/rec 050320fa00fa00c800fa003c0078001e0062d9e90e0000000045000000000000000000f1ca
                      050320fa00fa00c800fa003c0078001e0062d9e90e020000004b00000000000000000093fd

on battery:
050320fa00fa00c800fa003c0078001e0041d9680f00000000440000000000000000003ab1
050320fa00fa00c800fa003c0078001e0041d9680f00000000440000000000000000003ab1

050320fa00fa00c800fa003c0078001e0041d9680f00000000450000000000000000006b74

05035a03001e00000014000000e2001b0000001700fa08f4013402f4011900190028610000c1a20000020060096009e6000a00f000e600f4010a0000000000f923d200d60f0208e501020000000100020000005000e6000200f000f5004f3d
05035a03001f00000016000000de00140000002100fc08f4011703ab022400240028610000c1a20000020060096009e6000a00f000e600f4010a0000000000f923d200d60f0208e501020000000100020000005000e6000200f000f500dc76
05035a03002200000015000000dd00130000002000fd08f40111039e022400240028610000c1a20000020060096009e6000a00f000e600f4010a0000000000f923d200d60f0208e501020000000100020000005000e6000200f000f5009a6c
05035a03001e00000014000000dd00130000001b00fa08f4017c022d021d001d0028610000c1a20000020060096009e6000a00f000e600f4010a0000000000f923d200d60f0208e501020000000100020000005000e6000200f000f500ce1c
05035a03000000000015000000dc00110000001400f908f401f701a8011700170028610000c1a20000020060096009e6000a00f000e600f4010a0000000000f923d200d60f0208e501020000000100020000005000e6000200f000f500fc4b
05035a03000000000013000000db000f0000001500fc08f4011002bb011800180028610000c1a20000020060096009e6000a00f000e600f4010a0000000000f923d200d60f0208e501020000000100020000005000e6000200f000f5002d19
05035a03000000000014000000d9000c0000001500fd08f401f801ac011800180028610000c1a20000020060096009e6000a00f000e600f4010a0000000000f923d200d60f0208e501020000000100020000005000e6000200f000f5002ada

charging online

050320fa00fa00c800fa003c0078001e0062d9e90e02000000460000000000000000000207
    Data: Row Format: Data Type, Signed, Unsigned

        (0)   (char)       -6       250  
            
        (1)   (char)        0         0  
            
        (2)   (char)       -6       250  
            
        (3)   (char)        0         0  
            
        (4)   (char)      -56       200  
            
        (5)   (char)        0         0  
            
        (6)   (char)       -6       250  
            
        (7)   (char)        0         0  
            
        (8)   (char)       60        60  
            
        (9)   (char)        0         0  
            
        (10)   (char)      120       120  
            
        (11)   (char)        0         0  
            
        (12)   (char)       30        30  
            
        (13)   (char)        0         0  
            
        (14)   (char)       98        98  
            
        (15)   (char)      -39       217  
            
        (16)   (char)      -23       233  
            
        (17)   (char)       14        14  
            
        (18)   (char)        2         2  
            
        (19)   (char)        0         0  
            
        (20)   (char)        0         0  
            
        (21)   (char)        0         0  
            
        (22)   (char)       70        70  
            
        (23)   (char)        0         0  
            
        (24)   (char)        0         0  
            
        (25)   (char)        0         0  
            
        (26)   (char)        0         0  
            
        (27)   (char)        0         0  
            
        (28)   (char)        0         0  
            
        (29)   (char)        0         0  
            
        (30)   (char)        0         0  
            
        (31)   (char)        0         0  




05035a0400ef08f301c7000100dd00120001000000ef08f30196011a011000100028610000c1a20000020060096009e6000a00f000e600f4010a0000000000f923d200d60f0208e501020000000100020000005000e6000200f000f5000abd
05035a0400ae08f401c3000100e3001d003c000000ae08f401c4013a011200120028610000c1a20000020060096009e6000a00f000e600f4010a0000000000f923d200d60f0208e501020000000100020000005000e6003c00f000f500e4d1


   Data: Row Format: Data Type, Signed, Unsigned
 ? mode ?
        (0)   (char)        3         3     4 online charging
        (1)   (char)        0         0  
 ? ac volt         
        (2)   (char)       34        34      239
        (3)   (char)        0         0        8
 ? ac freq           
        (4)   (char)        0         0      243
        (5)   (char)        0         0        1

??? PV imput voltage ?            
        (6)   (char)       21        21      199
        (7)   (char)        0         0        0

?? PV input power            
        (8)   (char)        0         0        1
        (9)   (char)        0         0        0 

  Battery voltage  * 10        
        (10)   (char)      -35       221      221
        (11)   (char)        0         0       0 

  Battery internal percantage          
        (12)   (char)       19        19  
        (13)   (char)        0         0  

   ?? Battery charge current ?         
        (14)   (char)        0         0        60
        (15)   (char)        0         0         0
            
  Battery discharge current
        (16)   (char)       32        32    
        (17)   (char)        0         0  

  ? output voltage * 10 ?          
        (18)   (char)       -3       253  
        (19)   (char)        8         8  
  ?? Output freq * 10         
        (20)   (char)      -12       244  
        (21)   (char)        1         1  
  ?? Output power          
        (22)   (char)       17        17  
        (23)   (char)        3         3  
  ?? output load           
        (24)   (char)      -98       158  
        (25)   (char)        2         2  
            
        (26)   (char)       36        36  
        (27)   (char)        0         0  
            
        (28)   (char)       36        36  
        (29)   (char)        0         0  
    ? steady        
        (30)   (char)       40        40  
        (31)   (char)       97        97  
            
        (32)   (char)        0         0  
        (33)   (char)        0         0  
    ? steady        
        (34)   (char)      -63       193  
        (35)   (char)      -94       162  
            
        (36)   (char)        0         0  
        (37)   (char)        0         0  
            
        (38)   (char)        2         2  
        (39)   (char)        0         0  
            
        (40)   (char)       96        96  
        (41)   (char)        9         9  
            
        (42)   (char)       96        96  
        (43)   (char)        9         9 

 ??? Output Voltage            
        (44)   (char)      -26       230  
        (45)   (char)        0         0  
            
        (46)   (char)       10        10  
        (47)   (char)        0         0  
            
        (48)   (char)      -16       240  
        (49)   (char)        0         0  
            
        (50)   (char)      -26       230  
        (51)   (char)        0         0  
            
        (52)   (char)      -12       244  
        (53)   (char)        1         1  
 ?? nominal ac current            
        (54)   (char)       10        10  
        (55)   (char)        0         0  
            
        (56)   (char)        0         0  
        (57)   (char)        0         0  
            
        (58)   (char)        0         0  
        (59)   (char)        0         0  
            
        (60)   (char)       -7       249  
        (61)   (char)       35        35  
            
        (62)   (char)      -46       210  
        (63)   (char)        0         0  
            
        (64)   (char)      -42       214  
        (65)   (char)       15        15  
            
        (66)   (char)        2         2  
        (67)   (char)        8         8  
            
        (68)   (char)      -27       229  
        (69)   (char)        1         1  
            
        (70)   (char)        2         2  
        (71)   (char)        0         0  
            
        (72)   (char)        0         0  
        (73)   (char)        0         0  
            
        (74)   (char)        1         1  
        (75)   (char)        0         0  
            
        (76)   (char)        2         2  
        (77)   (char)        0         0  
            
        (78)   (char)        0         0  
        (79)   (char)        0         0  
  ??   Max total curret       
        (80)   (char)       80        80  
        (81)   (char)        0         0  
            
        (82)   (char)      -26       230  
        (83)   (char)        0         0

  ?? Utility charge current          
        (84)   (char)        2         2  
        (85)   (char)        0         0  
  ?? BATTERY comeback volt          
        (86)   (char)      -16       240  
        (87)   (char)        0         0  
  ?? Battery comeback volt2          
        (88)   (char)      -11       245  
        (89)   (char)        0         0  

???
050382e150
059181cc31



Requests:

read 45 registers (func 3) from slave 5 starting from address 4501 (Decimal) 
/iot/node/powmr/s/rec 05031195002d9143
read 16 registers (func 3) from slave 5 starting from address 4546 (Decimal) 11C2
/iot/node/powmr/s/rec 050311c20010e142

/iot/node/powmr/s/rec 05031195002d9143
/iot/node/powmr/s/rec 050311c20010e142
/iot/node/powmr/s/rec 05031195002d9143
/iot/node/powmr/s/rec 050311c20010e142
/iot/node/powmr/s/rec 05031195002d9143


read charger source priority (5017) , 1 register
/iot/node/powmr/s/rec 0503139900015125

set solar only charger: 
func 6, (5017), 0003
/iot/node/powmr/s/rec 0506139900031ce4

set util and solar charger:
func 6, (5017), 0002
/iot/node/powmr/s/rec 050613990002dd24
read func 3 (5017), 0001 register
/iot/node/powmr/s/rec 0503139900015125

get until charge current:
/iot/node/powmr/s/rec 050313a000018128

set util charge current to 2a:
func 6, (5024), 0002 decimal 
/iot/node/powmr/s/rec 050613a000020d29
read
/iot/node/powmr/s/rec 050313a000018128

set util charge current to 50a:
func 6, (5024), 50 decimal (32 hex)
/iot/node/powmr/s/rec 050613a000320d3d
read
/iot/node/powmr/s/rec 050313a000018128



from:

https://github.com/Shotgun167/simplesolarmon/blob/main/simplesolarmon.py

registers = {
    "Grid Voltage": {'register':531, 'multiplier':0.1, 'unit':'V'},
    "Grid Current": {'register':532, 'multiplier':0.1, 'unit':'A'},
    
    "Ambient Temperature": {'register':546, 'multiplier':0.1, 'unit':'C'},

    "PV Temperature": {'register':544, 'multiplier':0.1, 'unit':'C'},
    "Solar Voltage": {'register':263, 'multiplier':0.1, 'unit':'V'},
    "Solar Current": {'register':548, 'multiplier':0.1, 'unit':'A'},
    "Solar Power": {'register':265, 'multiplier':1, 'unit':'W'},
    
    "Battery Voltage": {'register':257, 'multiplier':0.1, 'unit':'V'},
    "Battery Current In": {'register':0, 'multiplier':1, 'unit':'A'},
    "Battery Power In": {'register':270, 'multiplier':0.1, 'unit':'W'},
    "Battery Current Out": {'register':0, 'multiplier':0.1, 'unit':'A'},
    "Battery Power Out": {'register':0, 'multiplier':1, 'unit':'W'},
    
    "Inv Temperature": {'register':545, 'multiplier':0.1, 'unit':'C'},
    "Inv Current": {'register':537, 'multiplier':0.1, 'unit':'A'},
    "Inv Voltage": {'register':534, 'multiplier':0.1, 'unit':'V'},
    "Inv VA": {'register':540, 'multiplier':1, 'unit':'VA'},
    "Inv Power": {'register':539, 'multiplier':1, 'unit':'W'}
}

https://github.com/Paxy/SmartESS-proxy/blob/main/ProcessInverterData.java

    private short modeIdx = 14;
    private short acVoltageIdx = 16;
    private short acFrequencyIdx = 18;
    private short pvVoltageIdx = 20;
    private short pvPowerIdx = 22;
    private short batteryVoltageIdx = 24;
    private short batteryChargedIdx = 26;
    private short batteryChargingCurrIdx = 28;
    private short batteryDisChargingCurrIdx = 30;
    private short outputVoltageIdx = 32;
    private short outputFrequencyIdx = 34;
    private short outputPowerIdx = 38;
    private short outputLoadIdx = 40;
    private short chargeStateIdx = 84;
    private short loadStateIdx = 86;

    public static final String chargeSolarOnly = "40630001000A05040506139900031CE4";
    public static final String chargeSolarUtility = "48E30001000A0504050613990002DD24";
    public static final String loadUtility = "490A0001000A05040506139A0000ACE5";
    public static final String loadSBU = "490D0001000A05040506139A00022D24";


Another useful project:
https://github.com/Paxy/SmartESS-proxy

Main website that collects info:
https://www.dessmonitor.com/


*/
