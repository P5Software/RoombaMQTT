/*
 * Roomba MQTT Status and Control
 * 
 *  MQTT Topics:
 *    Device Management Requests:
 *      /manage/[Device Name]/[Operation]
 * 
 *      Supported management topic operations:
 *        status (commands controller to send a health status)
 *        restart (commands controller to reboot, requires "request" as payload)
 *        firmwareUpdate (commands controller to check for a firmware update)
 *        bootstrap (commands controller to use an alternate bootstrap, erasing current EEPROM and replacing with the URL provided, requires "request" as payload)
 *        provisionReset (commands controller to erase EEPROM and wait for manual provisioning, requires "request" in payload)
 *        clean (commands Roomba to perform a normal cleaning routine or temporarily stops an existing cleaning in progress)
 *        stop (commands Roomba to shutdown and sleep immediately)
 *        dock (commands Roomba to seek the dock)
 *        setTime (commands Roomba to set its time in particular format)
 *        resetSchedule (commands Roomba to delete its internal schedule)
 *        rebootRoomba (commands Roomba to reboot, essentially a battery removal)
 *        wake (commands Roomba to wake when the processor has gone to sleep, which is unnecessary when docked)
 *        
 *    Device Status Responses:
 *      /client/[Device Name]/[Response]
 *
 *      Supported client topic responses:
 *        status (conditions on the controller)
 *        firmware (controller firmware version)
 *        uptime (controller uptime in ms)
 *        ip (controller IP address)
 *        deviceName (controller device name)
 *
 *    Sensor Reading Responses:
 *      /sensor/[Device Name]/[Sensor Name]
 *      
 *      Supported sensor topic responses:
 *        chargingState (battery charging state, as string [NOT_CHARGING, CHARGING_RECONDITIONING, CHARGING_FULL, CHARGING_TRICKLE, WAITING, FAULT, UNKNOWN]
 *        batteryVoltage (battery voltage in mV)
 *        batteryTemperature (battery temperature in degrees celcius)
 *        batteryCapacity (battery capacity in mAh)
 *        batteryCharge (battery charge in mAh)
 *        batteryPercentage (percent of the battery remaining as an integer)
 *        batteryCurrent (battery current charge/draw in mA)
 *        runStatus (Roomba's current running mode, as string [SLEEPING, CLEANING, IDLING])
 *        runTime (amount of time the last cleaing mode ran, in ms)
 *        odometer (distance Roomba traveled in last cleaning mode, in mm)
 * 
 *    Device Name is the MAC address with no spaces, dashes, or colons.  MAC AA:BB:CC:DD:EE:00 = AABBCCDDEE00.
 *    
 *    Inspired by https://github.com/thehookup/MQTT-Roomba-ESP01.  Go subscribe.
 * 
 * (C) 2019, P5 Software, LLC.
 */


#define SLEEPING 0 //Roomba will not respond unless manual intervention or the wake pin is activated; Off dock and processor in low power;
#define CLEANING 1 //Roomba is actively cleaning
#define IDLING 2 //Roomba will respond if sent data; Off dock but not sleeping; On dock;


#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <PubSubClient.h>
#include <EEPROM.h>
#include <ArduinoJson.h> //https://arduinojson.org/
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>
#include <Math.h>


const int firmwareVersion = X;


typedef struct structSettings{
  bool deviceIsProvisioned;
  String ssidName;
  String wpaKey;
  String machineMacAddress;
  String deviceName;
  
  struct {
    String serverName;
    int port;
    String username;
    String password;
    String manageTopic;
    String clientTopic;
    String sensorTopic;
   } mqttServer;

  struct {
    String url;
    unsigned long updateFrequency;
    unsigned long lastRetrieved;
  } firmware;

  struct {
     unsigned long updateFrequency;
     unsigned long lastRetrieved;
  } sensors;

  struct {
     unsigned long updateFrequency;
     unsigned long lastRetrieved;
  } health;

  struct {
    int voltage;
    int current;
    int temperature;
    int charge;
    int percentRemaining;
  } minimumReportingDelta;

};


typedef struct structRoombaData{
  
  int runStatus;
  unsigned long odometer;
  int stasis;

  struct {
    unsigned long startTime;
    unsigned long endTime;
    unsigned long runTime;
  } cleaningCycle;
 
  struct {
    uint16_t voltage;
    int16_t current;
    int temperature;
    uint16_t charge;
    uint16_t capacity;
    String chargingState;
    String percentRemaining;
  } battery;

  struct {
    uint16_t leftWheel;
    uint16_t rightWheel;
    uint16_t mainBrush;
    uint16_t sideBrush;
  } motorCurrent;

  struct {
    uint16_t leftEncoder;
    uint16_t rightEncoder;
  } wheels;
 
};


const int wakePin = D3;
const int EEPROMLength = 512;
const int EEPROMDataBegin = 10; //Reserving the first 10 addresses for provision statuses
const int wheelRadius = 69; //mm, measured tire
const float wheelEncodersPerRevolution = 508.8; //Provided by iRobot
const float pi = 3.14159265359;
struct structSettings settings;
struct structRoombaData roombaDataObserved;
struct structRoombaData roombaDataReported;
WiFiClient espClient;
ESP8266WebServer webServer(80);
ESP8266HTTPUpdateServer httpUpdater;
PubSubClient mqttClient(espClient);
WiFiServer telnetServer(23);
WiFiClient telnetClient;
WiFiClient tmpTelnetClient;


void setup() {
  
  //Use different UARTS.  TX is required to be on UART 1, RX on UART 0
  Serial.begin(115200);
  Serial1.begin(115200);

  //Set the wakeup pin for output
  pinMode(wakePin, OUTPUT);

  //Give the WiFi chip time to start
  delay(1000);

  broadcastLine("\n");
  broadcastLine(padRight("", 23, ">") + " System Restarted " + padRight("", 23, "<"));

  //Get the MAC address
  settings.machineMacAddress = WiFi.macAddress();
  settings.machineMacAddress.toUpperCase();

  //Set the device name from the MAC address
  settings.deviceName = settings.machineMacAddress;
  settings.deviceName.replace(":","");

  //Set the hostname
  WiFi.hostname("Roomba-" + settings.deviceName);  
  broadcastLine("Hostname is " +  WiFi.hostname());

  //Configure the settings
  settings.health.updateFrequency = 1680000; //Every 28 minutes
  settings.firmware.updateFrequency = 3600000; //Every sixty minutes
  settings.sensors.updateFrequency = 2000; //Every two seconds
  settings.minimumReportingDelta.voltage = 200; //200mv
  settings.minimumReportingDelta.current = 512; //512mA
  settings.minimumReportingDelta.temperature = 2; //2 degrees Celcius
  settings.minimumReportingDelta.charge = 50; //50mAh
  settings.minimumReportingDelta.percentRemaining = 1; //One percent
  
  //Check the provisioning status
  checkDeviceProvisioned();
  
  //Set the device's current mode depending on if it is provisioned or not
  if(settings.deviceIsProvisioned == false){

    //Configure for provisioning mode
    setupProvisioningMode();

  }else{

    //The device is provisioned, get the settings from the EEPROM
    broadcastLine("Device has been provisioned.\n");

    //Read the data from EEPROM
    readEEPROMToRAM();
    
    //Setup the WiFi Connection
    //Attempt to connect to the WiFi Network as a client
    WiFi.mode(WIFI_STA);
  
    WiFi.begin(settings.ssidName.c_str(), settings.wpaKey.c_str());
  
    broadcastLine("Attempting to connect to network " + settings.ssidName); 
    
    //Print output while waiting for WiFi to connect
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
    }

    //Connected, begin sending to the broadcastLin function
    broadcastLine("IP address: " + WiFi.localIP().toString() + "\nMAC Address: " + WiFi.macAddress());

    //Start the telent server
    telnetServer.begin();
  
    //Configure the mqtt Client
    mqttClient.setServer(settings.mqttServer.serverName.c_str(), settings.mqttServer.port);
    mqttClient.setCallback(mqttCallback);

  }

}


void loop() {
  
  //See which loop we should execute
  if(settings.deviceIsProvisioned == false){

      //Setup a client handler
      webServer.handleClient();
    
  }else{


    //The device has been provisioned, run the normal processes
    if (!mqttClient.connected()) {
      reconnectMQTT();
    }
  
    mqttClient.loop();

    //See if there are telnet client connections waiting.  If there are not, this will be false.  If there are, an object is returned.
    tmpTelnetClient = telnetServer.available();
  
    if(tmpTelnetClient){
  
      broadcastLine("A new telnet client connected.  Dropping any existing connections.\n");
  
      //Copy the object over to the actual telnetClient, which will kill off any existing telnet clients
      telnetClient = tmpTelnetClient;
  
   }

   //Handle health updates if enough time has passed
    if((unsigned long)(millis() - settings.health.lastRetrieved) > (unsigned long)settings.health.updateFrequency){

      //We want to ignore the first attempt to run the health check
      if(settings.health.lastRetrieved > 0) {
        
        broadcastLine(padRight("", 64, "#"));
        broadcastLine("Health Update\n");

        //Time to send a health status update
        handleHealth();

        broadcastLine(padRight("", 64, "#"));

      }

      //Update the last handled time to be now
      settings.health.lastRetrieved = millis();
    }

    //Handle firmware updates if enough time has passed
    if((unsigned long)(millis() - settings.firmware.lastRetrieved) > (unsigned long)settings.firmware.updateFrequency){
      
      broadcastLine("Checking firmware");

      checkFirmwareUpgrade();

      //Update the last handled time to be now
      settings.firmware.lastRetrieved = millis();
      
    }

    if((unsigned long)(millis() - settings.sensors.lastRetrieved) > settings.sensors.updateFrequency){

      //Get Roomba's data
      getRoombaData();
    
    }
    
  }

}


void checkDeviceProvisioned(){
  /*
   * This function will check the EEPROM to see if the device has been provisioned.  If the EEPROM is not "true", it will assume the device has not been provisioned and will
   * set the setting accordingly.
   */

   //Read the EEPROM starting at memory position 0
   if(EEPROMRead(0) == "true"){

      //The device has been provisioned, set the setting to true
      settings.deviceIsProvisioned = true;
      
   }else{

      //The device has not been provisioned, set the setting to false
      settings.deviceIsProvisioned = false;
    
   }
  
}


void setupProvisioningMode(){
  /*
   * Configures necessary items for when the device has not been provisioned.
   */

  //Set the default IP address, gateway, and subnet to use when in AP mode
  IPAddress ip(192, 168, 1, 1);
  IPAddress gateway(192, 168, 1, 1);
  IPAddress subnet(255, 255, 255, 0);
  
  //Create an access point
  WiFi.softAPConfig(ip, gateway, subnet);
  
  //Set the AP name Roomba-.....
  WiFi.softAP(("Roomba-" + settings.deviceName).c_str());

  //Listen for web server requests
  webServer.on("/", wwwHandleRoot);
  webServer.onNotFound(wwwHandleNotFound);

  MDNS.begin(settings.deviceName.c_str());

  httpUpdater.setup(&webServer);

  MDNS.addService("http", "tcp", 80);

  webServer.begin();
  
}


void attemptProvision(String SSID, String wpaKey, String bootstrapURL){

  //Kill the soft AP
  WiFi.softAPdisconnect(true);

  //Connect to the requested SSID
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID.c_str(), wpaKey.c_str());

  //Remember the start time
  unsigned long timeStartConnect = millis();
  
  //Print output while waiting for WiFi to connect
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);

    //Set a timer for 30 seconds to connect
    if(millis() - timeStartConnect > 30000){

      //Disconnect the WiFi
      WiFi.disconnect();

      //Restart the provisioning server
      setupProvisioningMode();

      //Exit this
      return;
      
    }
  }

  broadcastLine("\n");
  broadcastLine("IP address: " + WiFi.localIP().toString() + "\nMAC Address: " + WiFi.macAddress());

  //Attempt to retrieve the bootstrap
  if(retrieveBootstrap(bootstrapURL) == true){
    broadcastLine("Retrieved configuration successfully.  Restarting.\n");

    //Ensure the EEPROM has time to finish writing and closing the client
    delay(1000);

    //Restart
    ESP.restart();
      
  } else {
      
    //Restart on failure
    ESP.restart();

  }

}


boolean retrieveBootstrap(String bootstrapURL){
  /*
    Retrieves the bootstrap JSON document from the provided URL.  On success, writes "TRUE" to the EEPROM and the provisioning information.
  */
  
  HTTPClient httpClient;

  httpClient.begin(bootstrapURL);

  //See if we were successful in retrieving the bootstrap
  if (httpClient.GET() == HTTP_CODE_OK) {

    //Write the response to the EEPROM
    EEPROMWrite(httpClient.getString(), EEPROMDataBegin);

    //Set the provision status
    EEPROMWrite("true", 0);

    httpClient.end();

    //Re-Read the EEPROM into RAM
    readEEPROMToRAM();

    return true;
    
  }else{

    broadcastLine("Failed retrieving bootstrap from " + bootstrapURL + ": " + (String)httpClient.errorToString(httpClient.GET()) + ".\n");
    
    return false;
  }

}


void reconnectMQTT() {

  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    
    broadcastLine("Attempting to connect to MQTT server " + settings.mqttServer.serverName + ": ");
        
    // Attempt to connect
    if (mqttClient.connect(settings.machineMacAddress.c_str(), settings.mqttServer.username.c_str(), settings.mqttServer.password.c_str())) {
      
      broadcastLine("connected.\n");
      
      // Publish an announcement on the client topic
      publishMQTT(settings.mqttServer.clientTopic + "/status", "Started");

      broadcastLine(padRight("", 64, "#"));
      
      //Publish the health
      handleHealth();

      broadcastLine(padRight("", 64, "#"));
      
      // Subscribe to the management topic wildcard
      mqttClient.subscribe((settings.mqttServer.manageTopic + "/#").c_str());
      
    } else {
      
      broadcastLine("failed, rc=" + (String)mqttClient.state());
      broadcastLine("Try again in 5 seconds.\n");
      
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


void mqttCallback(char* topic, byte* payload, unsigned int length) {
  /* This function will handle all incoming requests for subscribed topics */

  String receivedPayload = "";
 
  //Assemble the payload data
  for (int i = 0; i < length; i++) {
    receivedPayload = String(receivedPayload + (char)payload[i]);
  }

  //Clean the payload received
  receivedPayload.trim();

  //Pass the payload off for further processing
  handleMQTTManagementMessage(String(topic), receivedPayload);

}


void publishMQTT(String topic, String payload){
  /*
   * Sends the requested payload to the requested topic, and also broadcasts the message on the serial line.
   */

  //Clean the strings
  topic.trim();
  payload.trim();

  mqttClient.publish(String(topic).c_str(), String(payload).c_str());
  broadcastLine("<=> " + padRight(topic, 40, " ") + payload);
  
}


void handleMQTTManagementMessage(String topic, String payload){
  /* 
    Handles incoming management requests from MQTT.  Payloads must exactly match.
    Output is echoed on telnet and MQTT /client/# topic. 
  */

  //Copy the payload so we can modify it
  String payloadToLowerCase = payload;

  //Make the payload lower
  payloadToLowerCase.toLowerCase();

  //See if the payload requests a restart
  if(topic == settings.mqttServer.manageTopic + "/restart" && payloadToLowerCase == "request"){

    publishMQTT(settings.mqttServer.clientTopic + "/status", "Restarting");
    
    delay(500);
    
    ESP.restart();

    return;
    
  }

  //See if the payload requests a status message
  if(topic == settings.mqttServer.manageTopic + "/status"){
    handleHealth();

    return;
  }

  //Force the system to attempt a firmware update
  if(topic == settings.mqttServer.manageTopic + "/firmwareUpdate"){
    checkFirmwareUpgrade();

    return;
  }

  //See if the payload requests an updated bootstrap
  if(topic == settings.mqttServer.manageTopic + "/bootstrap" && payload != ""){

    if(retrieveBootstrap(payload) == true){

      broadcastLine("Bootstrapped from " + payload);
      publishMQTT(settings.mqttServer.clientTopic + "/status", "Bootstrap Requested");
      broadcastLine(EEPROMRead(EEPROMDataBegin));

      return;
      
    }else{
      
      publishMQTT(settings.mqttServer.clientTopic + "/status", "Bootstrap Failed");

      return;
      
    }

  }

  //Clear the EEPROM and reset provisioning
  if(topic == settings.mqttServer.manageTopic + "/provisionReset" && payloadToLowerCase == "request"){

    //Erase the EEPROM flag and replace it with FALSE, which will force a provisioning
    EEPROMWrite("false", 0);

    publishMQTT(settings.mqttServer.clientTopic + "/status", "Provision Reset");

    //Ensure the EEPROM has time to finish writing and closing the client
    delay(1000);

    //Restart
    ESP.restart();

    return;
  }

  //See if the payload requests roomba to start cleaning
  if(topic == settings.mqttServer.manageTopic + "/clean"){
    commandRoombaClean();

    return;
  }

  //See if the payload requests roomba to return to dock
  if(topic == settings.mqttServer.manageTopic + "/dock"){
    commandRoombaDock();

    return;
  }

  //See if the payload requests roomba to stop
  if(topic == settings.mqttServer.manageTopic + "/stop"){
    commandRoombaStop();

    return;
  }


  //See if the payload requests roomba to set its time.  Note the payload must be DD,hh:mm
  if(topic == settings.mqttServer.manageTopic + "/setTime"){

    commandRoombaSetDayTime(payloadToLowerCase);

    return;
  }

  //See if the payload requests roomba to reset its schedule
  if(topic == settings.mqttServer.manageTopic + "/resetSchedule"){

    commandRoombaResetSchedule();

    return;
  }

   //See if the payload requests roomba to reboot
  if(topic == settings.mqttServer.manageTopic + "/rebootRoomba"){

    commandRoombaReboot();

    return;
  }

  //See if the payload requests to wake up Roomba
  if(topic == settings.mqttServer.manageTopic + "/wake"){

    commandRoombaWake();

    return;
  }

}


void EEPROMWrite(String stringToWrite, int startPosition){
  /*
   * This function writes a string of data to the EEPROM beginning at a specified address.
   * It is sandwiched between two non-printable ASCII characters (2/STX and 3/ETX)
   * to deliniate where the data begins and ends, which is used when reading the data.
   */

  //Sandwich the data with STX and ETX so we can keep track of the data
  stringToWrite = char(02) + stringToWrite + char(03);

  //Set the EEPROM size
  EEPROM.begin(EEPROMLength);

  //Write a start of text and end of text around the actual text
  for (int i = 0; i < stringToWrite.length(); ++i)
  {
    //Write the data to the EEPROM buffer
    EEPROM.write((startPosition + i), stringToWrite[i]);
  }

  //Commit the EEPROM
  EEPROM.commit();
  
}


String EEPROMRead(int startPosition){
  /*
   * This function reads EEPROM data.  It is sandwiched between two non-printable ASCII characters (2/STX and 3/ETX) to deliniate where the data begins and ends.
   * Becasuse the EEPROM length is longer than the data contained, we can only reliably read the data between the two non-printable characters, as there could be junk data left
   * after the data string in the EEPROM.  After the end of the data feed is found, we automatically advance to the end to force the function to stop reading (we could have another
   * STX later on that we don't want to pick up).
   */

  byte value;
  String strReturnValue = "";
  bool isProcessingData = false;

  EEPROM.begin(EEPROMLength);

  //Start reading the EEPROM where the data begins until the EEPROM length
  for(int i=startPosition; i<EEPROMLength; i++){

    // read a byte from the current address of the EEPROM
    value = EEPROM.read(i);

    //See what the value of this byte is
    switch(value){

      case 2:
        //This byte is the beginning of the data, start saving information at this point
        isProcessingData = true;
        break;

      case 3:
         //This byte is the end of the data, stop saving information at this point
        isProcessingData = false;

        //Set the address to the end of the EEPROM so we will break out of the WHILE loop
        i = EEPROMLength;
        break;

      default:

        //If the character is printable AND we are processing a word, store the character to the string
        if(value >= 32 && value <= 127 && isProcessingData == true){
          strReturnValue = strReturnValue + String(char(value));
        }   
      }
    }

  return strReturnValue;

}


void readEEPROMToRAM(){

  //Read the data from EEPROM
    DynamicJsonBuffer jsonBuffer(1500);

    //Read the EEPROM
    JsonObject& root = jsonBuffer.parseObject(EEPROMRead(EEPROMDataBegin));
    
    if (!root.success()) {
      broadcastLine("parseObject() failed\n");
      EEPROMWrite("false",0);
      return;
    }

    //Store the settings
    settings.ssidName = root["SSID"].asString();
    settings.wpaKey = root["wpaKey"].asString();

    JsonObject& mqtt = root["mqtt"];
    settings.mqttServer.serverName = mqtt["server"].asString();
    settings.mqttServer.port = mqtt["port"];
    settings.mqttServer.username = settings.deviceName;
    settings.mqttServer.password = settings.deviceName;
    settings.mqttServer.manageTopic = "/manage/" + settings.deviceName;
    settings.mqttServer.clientTopic = "/client/" + settings.deviceName;
    settings.mqttServer.sensorTopic = "/sensor/" + settings.deviceName;

    JsonObject& firmware = root["firmware"];
    settings.firmware.url = firmware["url"].asString();
  
}


void broadcastLine(String data){

  /*
   * Prints the data input on the telnet client.
   */

  telnetClient.print(data + "\n");
  
}


String padRight(String data, int toLength, String padValue){
  /* 
  * Makes output pretty on the telnet client.
  */

  while(data.length() < toLength){
    data = data + padValue;    
  }

  return data;
    
}


void handleHealth(){

  //Publish the status of OK
  publishMQTT(settings.mqttServer.clientTopic + "/status", "OK");
  
  //Publish the uptime
  publishMQTT(settings.mqttServer.clientTopic + "/uptime", String(millis()));

  //Publish the IP Address
  publishMQTT(settings.mqttServer.clientTopic + "/ip", WiFi.localIP().toString());

  //Publish the Firmware version
  publishMQTT(settings.mqttServer.clientTopic + "/firmware", (String)firmwareVersion);

  //Publish the device name
  publishMQTT(settings.mqttServer.clientTopic + "/deviceName", settings.deviceName);

  //Get the latest data from roomba
  getRoombaData();

  //Publish Roomba's Charge State
  publishMQTT(settings.mqttServer.sensorTopic + "/chargingState", roombaDataReported.battery.chargingState);

  //Publish Roomba's Battery Voltage
  publishMQTT(settings.mqttServer.sensorTopic + "/batteryVoltage", (String)roombaDataReported.battery.voltage);

  //Publish Roomba's Battery Temperature
  publishMQTT(settings.mqttServer.sensorTopic + "/batteryTemperature", (String)roombaDataReported.battery.temperature);

  //Publish Roomba's Battery Capacity
  publishMQTT(settings.mqttServer.sensorTopic + "/batteryCapacity", (String)roombaDataReported.battery.capacity);

  //Publish Roomba's Battery Charge
  publishMQTT(settings.mqttServer.sensorTopic + "/batteryCharge", (String)roombaDataReported.battery.charge);

  //Publish Roomba's Battery Percentage
  publishMQTT(settings.mqttServer.sensorTopic + "/batteryPercentage", (String)roombaDataReported.battery.percentRemaining);

  //Publish Roomba's Battery Current
  publishMQTT(settings.mqttServer.sensorTopic + "/batteryCurrent", (String)roombaDataReported.battery.current);

  //Publish Roomba's current status
  publishMQTT(settings.mqttServer.sensorTopic + "/runStatus", runStatusToString());

}


String runStatusToString(){
  /*
  * Returns the Roomba run status as a string.
  */

  switch(roombaDataObserved.runStatus){

    case 0:
      return "SLEEPING";
      break;

    case 1:
      return "CLEANING";
      break;

    case 2:
      return "IDLING";
      break;

    default:
      return "UNKNOWN";
  }

}

void setRunStatus(int runStatus){
  /*
  * Sets Roomba's current runStatus and sends MQTT message if there is a change in state.
  */

  if(runStatus != roombaDataReported.runStatus){

    //Set the updated status
    roombaDataReported.runStatus = runStatus;
    roombaDataObserved.runStatus = runStatus;

    //Publish Roomba's current status
    publishMQTT(settings.mqttServer.sensorTopic + "/runStatus", runStatusToString());
  }

}


void getRoombaData(){
  /*
  * Retrieves packets of data from Roomba and saves them to the associated roombaData elements.
  * When appropriate, broadcasts sensor data via MQTT.
  */

  Serial1.write(128);
  delay(50);
  Serial1.write(149); // Query List
  delay(50);
  Serial1.write(15); //15 packets
  delay(50);
  Serial1.write(15); //Dirt detect, 1 byte, 0-255
  delay(50);
  Serial1.write(19); //Distance, 2 bytes, signed, -32768-32767
  delay(50);
  Serial1.write(21); //Battery charging state, 1 byte, unsigned, 0-5
  delay(50);
  Serial1.write(22); //Battery voltage, 2 bytes unsigned, 0-65535
  delay(50);
  Serial1.write(23); //Battery current, 2 bytes signed, -32768-32767
  delay(50);
  Serial1.write(24); //Battery temperature, 1 byte signed, -128-127
  delay(50);
  Serial1.write(25); //Battery charge, 2 bytes unsigned, 0-65535
  delay(50);
  Serial1.write(26); //Battery capacity, 2 bytes unsigned, 0-65535
  delay(50);
  Serial1.write(43); //Right encoder count, 2 bytes unsigned, 0-65535
  delay(50);
  Serial1.write(44); //Left encoder count, 2 bytes unsigned, 0-65535
  delay(50);
  Serial1.write(54); //Left motor current, 2 bytes signed, -32768-32767
  delay(50);
  Serial1.write(55); //Right motor current, 2 bytes signed, -32768-32767
  delay(50);
  Serial1.write(56); //Main brush motor current, 2 bytes signed, -32768-32767
  delay(50);
  Serial1.write(57); //Side brush motor current, 2 bytes signed, -32768-32767
  delay(50);
  Serial1.write(58); //Stasis caster, 1 byte, 0-2
  delay(250);

  char tmpBuffer[26];
  int i=0;

  //Ensure we have exactly 26 bytes to read
  if(Serial.available() != 26){

    //We did not get exactly 26 bytes back; skip this iteration
    broadcastLine("Received " + (String)Serial.available() + " bytes on buffer, expected 26.  Skipping this iteration.");
    
    settings.sensors.lastRetrieved = millis();

    //Roomba will usually do this when it is sleeping, so set the mode to sleeping
    setRunStatus(SLEEPING);

    return;
    
  }

  //We should have 26 bytes to read
  while(Serial.available()){

    
    //Read the data into the buffer
    tmpBuffer[i] = Serial.read();

    i++;
    
  }

  broadcastLine(padRight("", 64, "*"));

  String debug = "";

  for(i=0; i<sizeof(tmpBuffer); i++){

    debug = debug + (String)(int)tmpBuffer[i];
    
  }

  //If Roomba responded, set the mode to IDLING if not cleaning
  if(roombaDataObserved.runStatus != CLEANING){
    setRunStatus(IDLING);
  }

  //Dirt detect, 1 byte, 0-255, element 0; Increment the value at the position by 1
  //This function does not work on a Roomba 770 and has been removed

  //Distance, 2 bytes, signed, -32768-32767, element 1-2
  //This function does not work on a Roomba 770 and has been removed

  //Battery charging state, 1 byte, unsigned, 0-5, element 3
  switch(tmpBuffer[3]){

    case 0:
      roombaDataObserved.battery.chargingState = "NOT_CHARGING";
      break;
    
    case 1:
      roombaDataObserved.battery.chargingState = "CHARGING_RECONDITIONING";
      break;
    
    case 2:
      roombaDataObserved.battery.chargingState = "CHARGING_FULL";
      break;
    
    case 3:
      roombaDataObserved.battery.chargingState = "CHARGING_TRICKLE";
      break;
    
    case 4:
      roombaDataObserved.battery.chargingState = "WAITING";
      break;
    
    case 5:
      roombaDataObserved.battery.chargingState = "FAULT";
      break;
    
    default:
      roombaDataObserved.battery.chargingState = "UNKNOWN";
  }

  broadcastLine("roombaDataObserved.battery.chargingState: " + roombaDataObserved.battery.chargingState);
  
  //Battery voltage, 2 bytes unsigned, 0-65535, element 4-5
  roombaDataObserved.battery.voltage = combineBytesToUnsignedInt(tmpBuffer[4], tmpBuffer[5]);
  broadcastLine("roombaDataObserved.battery.voltage: " + (String)roombaDataObserved.battery.voltage);

  //Battery current, 2 bytes signed, -32768-32767, element 6-7
  roombaDataObserved.battery.current = combineBytesToSignedInt(tmpBuffer[6], tmpBuffer[7]);
  broadcastLine("roombaDataObserved.battery.current: " + (String)roombaDataObserved.battery.current);

  //Battery temperature, 1 byte signed, -128-127, element 8
  roombaDataObserved.battery.temperature = tmpBuffer[8];
  broadcastLine("roombaDataObserved.battery.temperature: " + (String)roombaDataObserved.battery.temperature);

  //Battery charge, 2 bytes unsigned, 0-65535, element 9-10
  roombaDataObserved.battery.charge = combineBytesToUnsignedInt(tmpBuffer[9], tmpBuffer[10]);
  broadcastLine("roombaDataObserved.battery.charge: " + (String)roombaDataObserved.battery.charge);

  //Battery capacity, 2 bytes unsigned, 0-65535, element 11-12
  roombaDataObserved.battery.capacity = combineBytesToUnsignedInt(tmpBuffer[11], tmpBuffer[12]);
  broadcastLine("roombaDataObserved.battery.capacity: " + (String)roombaDataObserved.battery.capacity);

  //Right encoder count, 2 bytes unsigned, 0-65535, element 13-14
  roombaDataObserved.wheels.rightEncoder = roombaDataObserved.wheels.rightEncoder + combineBytesToUnsignedInt(tmpBuffer[13], tmpBuffer[14]);
  //broadcastLine("roombaDataObserved.wheels.rightEncoder: " + (String)roombaDataObserved.wheels.rightEncoder);
  broadcastLine("Raw right encoder: " + (String)combineBytesToUnsignedInt(tmpBuffer[13], tmpBuffer[14]));

  //Left encoder count, 2 bytes unsigned, 0-65535, element 15-16
  roombaDataObserved.wheels.leftEncoder = roombaDataObserved.wheels.leftEncoder + combineBytesToUnsignedInt(tmpBuffer[15], tmpBuffer[16]);
  //broadcastLine("roombaDataObserved.wheels.leftEncoder: " + (String)roombaDataObserved.wheels.leftEncoder);
  broadcastLine("Raw left encoder: " + (String)combineBytesToUnsignedInt(tmpBuffer[15], tmpBuffer[16]));

  //Left motor current, 2 bytes signed, -32768-32767, elemnet 17-18
  roombaDataObserved.motorCurrent.leftWheel = combineBytesToSignedInt(tmpBuffer[17], tmpBuffer[18]);
  broadcastLine("roombaDataObserved.motorCurrent.leftWheel: " + (String)roombaDataObserved.motorCurrent.leftWheel);

  //Right motor current, 2 bytes signed, -32768-32767, elemnet 19-20
  roombaDataObserved.motorCurrent.rightWheel = combineBytesToSignedInt(tmpBuffer[19], tmpBuffer[20]);
  broadcastLine("roombaDataObserved.motorCurrent.rightWheel: " + (String)roombaDataObserved.motorCurrent.rightWheel);

  //Main brush motor current, 2 bytes signed, -32768-32767, element 21-22
  roombaDataObserved.motorCurrent.mainBrush = combineBytesToSignedInt(tmpBuffer[21], tmpBuffer[22]);
  broadcastLine("roombaDataObserved.motorCurrent.mainBrush: " + (String)roombaDataObserved.motorCurrent.mainBrush);

  //Side brush motor current, 2 bytes signed, -32768-32767, 23-24
  roombaDataObserved.motorCurrent.sideBrush = combineBytesToSignedInt(tmpBuffer[23], tmpBuffer[24]);
  broadcastLine("roombaDataObserved.motorCurrent.sideBrush: " + (String)roombaDataObserved.motorCurrent.sideBrush);

  //Stasis caster, 1 byte, 0-2, element 25
  roombaDataObserved.stasis = tmpBuffer[25];
  broadcastLine("roombaDataObserved.stasis: " + (String)roombaDataObserved.stasis);

  broadcastLine(padRight("", 64, "*"));

  settings.sensors.lastRetrieved = millis();

  //Report any sensor changes necessary
  reportRoombaSensorChanges();

  //Determine if Roomba's cleaning mode has changed
  toggleCleaningMode();

}


void reportRoombaSensorChanges(){
  /* 
  * Reports any changes greater than the minimumReportingDelta to MQTT
  */

  //Voltage
  if(abs(roombaDataObserved.battery.voltage - roombaDataReported.battery.voltage) > settings.minimumReportingDelta.voltage) {

    //Set the new reported value
    roombaDataReported.battery.voltage = roombaDataObserved.battery.voltage;

    //Publish an update
    publishMQTT(settings.mqttServer.sensorTopic + "/batteryVoltage", (String)roombaDataReported.battery.voltage);
  }

  //Current
  if(abs(roombaDataObserved.battery.current - roombaDataReported.battery.current) > settings.minimumReportingDelta.current) {

    //Set the new reported value
    roombaDataReported.battery.current = roombaDataObserved.battery.current;

    //Publish an update
    publishMQTT(settings.mqttServer.sensorTopic + "/batteryCurrent", (String)roombaDataReported.battery.current);
  }

  //Temperature
  if(abs(roombaDataObserved.battery.temperature - roombaDataReported.battery.temperature) > settings.minimumReportingDelta.temperature) {

    //Set the new reported value
    roombaDataReported.battery.temperature = roombaDataObserved.battery.temperature;

    //Publish an update
    publishMQTT(settings.mqttServer.sensorTopic + "/batteryTemperature", (String)roombaDataReported.battery.temperature);
  }

  //Charge
  if(abs(roombaDataObserved.battery.charge - roombaDataReported.battery.charge) > settings.minimumReportingDelta.charge) {

    //Set the new reported value
    roombaDataReported.battery.charge = roombaDataObserved.battery.charge;

    //Publish an update
    publishMQTT(settings.mqttServer.sensorTopic + "/batteryCharge", (String)roombaDataReported.battery.charge);
  }

  //Percent Remaining
  if(abs(roombaDataObserved.battery.percentRemaining.toInt() - roombaDataReported.battery.percentRemaining.toInt()) > settings.minimumReportingDelta.percentRemaining) {

    //Set the new reported value
    roombaDataReported.battery.percentRemaining = roombaDataObserved.battery.percentRemaining;

    //Publish an update
    publishMQTT(settings.mqttServer.sensorTopic + "/batteryPercentage", (String)roombaDataReported.battery.percentRemaining);
  }

  //Charging State
  if(roombaDataObserved.battery.chargingState != roombaDataReported.battery.chargingState){

    //Set the new reported value
    roombaDataReported.battery.chargingState = roombaDataObserved.battery.chargingState;

    //Publish an update
    publishMQTT(settings.mqttServer.sensorTopic + "/chargingState", (String)roombaDataReported.battery.chargingState);
 
  }
}


void toggleCleaningMode(){
  /*
  * Looks at the received data to determine if Roomba's run status has changed
  */

  int voteForCleaningMode = 0;

  //Roomba will always report NOT CHARGING or WAITING while cleaning
  if(roombaDataObserved.battery.chargingState == "NOT_CHARGING" || roombaDataObserved.battery.chargingState == "WAITING" ){
    voteForCleaningMode++;
  }

  //Roomba will always consume > 500mA while cleaning.  Controller consumes ~130mA at idle
  if(roombaDataObserved.battery.current < -500){
    voteForCleaningMode++;
  }

  //Check the motor currents.  We'll give them 100mA buffer just for erroneous readings
  if(roombaDataObserved.motorCurrent.leftWheel > 100){
    voteForCleaningMode++;
  }

  if(roombaDataObserved.motorCurrent.rightWheel > 100){
    voteForCleaningMode++;
  }

  if(roombaDataObserved.motorCurrent.mainBrush > 100){
    voteForCleaningMode++;
  }

  if(roombaDataObserved.motorCurrent.sideBrush > 100){
    voteForCleaningMode++;
  }

  //If the stasis reports 1 then Roomba is moving forward
  if(roombaDataObserved.stasis == 1){
    voteForCleaningMode++;
  }

  //There are 7 potential votes for cleaning, though not all of them will always be triggered.  Majority will rule.
  if(voteForCleaningMode >= 4 && roombaDataObserved.runStatus != CLEANING){

    //We have now begun cleaning
    beginCleaningMode();

  }

  //If we have 0 or 1 votes and our last observation as that we were cleaning, we need to end the cleaning cycle.
  if(voteForCleaningMode <= 1 && roombaDataObserved.runStatus == CLEANING){

    //We are no longer cleaning
    endCleaningMode();

  }

  //If we have 2 or 3 votes, just assume the same condition we were previously in since it could be ambiguous

}


void beginCleaningMode(){
  /*
  * Changes the value of Roomba cleaning, which resets certain counters.
  * This is automatically triggered when the controller notices changes in the motors, regardless if triggered from the controller or directly
  * on Roomba's control panel.  This function's compliment is endCleaningMode();
  */

  //Set the start time to now
  roombaDataObserved.cleaningCycle.startTime = millis();

  //Clear the end time and runTime
  roombaDataObserved.cleaningCycle.endTime = roombaDataObserved.cleaningCycle.startTime;
  roombaDataObserved.cleaningCycle.runTime = 0;

  //Set the run status
  setRunStatus(CLEANING);

  //Clear the odometer
  roombaDataObserved.odometer = 0;

}


void endCleaningMode() {
  /*
  * Changes the value of Roomba cleaning, which resets certain counters.
  * This is automatically triggered when the controller notices changes in the motors, regardless if triggered from the controller or directly
  * on Roomba's control panel.  This function's compliment is beginCleaningMode();
  */

  //Set the end time to now
  roombaDataObserved.cleaningCycle.endTime = millis();

  //Set the runTime
  roombaDataObserved.cleaningCycle.runTime = roombaDataObserved.cleaningCycle.endTime - roombaDataObserved.cleaningCycle.startTime;

  //Set the run status

  setRunStatus(IDLING);

  //Send MQTT messages
  publishMQTT(settings.mqttServer.sensorTopic + "/runTime", (String)roombaDataObserved.cleaningCycle.runTime);
  publishMQTT(settings.mqttServer.sensorTopic + "/odometer", (String)roombaDataObserved.odometer);

}


void observeEncoders(uint16_t leftEncoder, uint16_t rightEncoder){

  //Determine the distance traveled since last Retrieved
  uint16_t distanceLeftEncoder = computeEncoderDistance(leftEncoder, roombaDataObserved.wheels.leftEncoder);
  broadcastLine("distanceLeftEncoder: " + (String)distanceLeftEncoder);
  
  uint16_t distanceRightEncoder = computeEncoderDistance(rightEncoder, roombaDataObserved.wheels.rightEncoder);
  broadcastLine("distanceRightEncoder: " + (String)distanceRightEncoder);

  //Determine the average number of revolutions between the two encoders
  float averageDistance = (distanceLeftEncoder + distanceRightEncoder) / 2;

  broadcastLine("Average Distance (encoders): " + (String)averageDistance);

  //Convert the average distance to millimeters
  float distanceTraveled = averageDistance * ((float)pi * (float)wheelRadius / (float)wheelEncodersPerRevolution);

  //Broadcast the distance
  broadcastLine("Distance traveled this iteration: " + (String)round(distanceTraveled));

  //Add the distance traveled to the odometer
  roombaDataObserved.odometer = roombaDataObserved.odometer + round(distanceTraveled);

  //Update the observed data
  roombaDataObserved.wheels.leftEncoder = leftEncoder;
  roombaDataObserved.wheels.rightEncoder = rightEncoder;

}


uint16_t computeEncoderDistance(uint16_t currentValue, uint16_t previousValue){
  /*
  * Determines the absolute distance traveled by the encoder, including passing over the minimum or maximum values.
  */ 
  const uint16_t ceilingValue = 65535;
  uint16_t returnValue = 0;

  //Use the ceiling and determine if it was rolled over
  if(previousValue + currentValue > ceilingValue){ 

    //We rolled over the ceiling, so count from the previous value to the ceiling and also the current value
    returnValue = (ceilingValue - previousValue) + currentValue;
  }
  else{
    
    //We did not roll over the ceiling, do a simple computation
    returnValue = currentValue - previousValue;
  }

  return returnValue;
}


void commandRoombaClean(){

  //Ensure Roomba is awake
  if(roombaDataObserved.runStatus == SLEEPING){
    commandRoombaWake();
  }
  
  broadcastLine("Commanding Roomba to Clean\n");

  //Send the command to Roomba
  Serial1.write(128);
  delay(50);
  Serial1.write(131);
  delay(50);
  Serial1.write(135);
  delay(50);

}


void commandRoombaDock(){

  //Ensure Roomba is awake
  if(roombaDataObserved.runStatus == SLEEPING){
    commandRoombaWake();
  }

  broadcastLine("Commanding Roomba to Return to Dock\n");

  //Send the command to Roomba
  Serial1.write(128);
  delay(50);
  Serial1.write(131);
  delay(50);
  Serial1.write(143);
  delay(50);
  
}


void commandRoombaStop(){

  //Only allow Roomba to stop if it is currently cleaning
  if(roombaDataObserved.runStatus != CLEANING){

    broadcastLine("Roomba is not cleaning; No need to Stop\n");
    return;
  }

  broadcastLine("Commanding Roomba to Stop\n");

  //Send the command to Roomba
  Serial1.write(128);
  delay(50);
  Serial1.write(131);
  delay(50);
  Serial1.write(133);
  delay(50);
  
}


void commandRoombaResetSchedule(){
  /*
  * Clears Roomba's built-in schedule so it never runs.  Future runtimes will be defined by MQTT.
  */

  broadcastLine("Resetting Roomba's Schedule to Never Clean\n");

  //Send the commands to Roomba
  Serial1.write(128);
  delay(50);
  Serial1.write(167);
  delay(10);

  //Send value 0, 15 times
  for(int i = 1; i <= 15; i++){
    Serial1.write(0);
    delay(10);
  }

  broadcastLine("Done.\n");
  
}


void commandRoombaSetDayTime(String roombaTimeFormattedTime){
  /*
  * Sets Roomba's day and time.  Requires time in "WEEKDAYNAME,hh:mm" format, inclusive of leading zero's
  */

  //Force the input string to lowercase
  roombaTimeFormattedTime.toLowerCase();
  
  int dayCode = -1;
  String weekdayName = roombaTimeFormattedTime.substring(0,roombaTimeFormattedTime.indexOf(","));
  String hour = roombaTimeFormattedTime.substring(roombaTimeFormattedTime.indexOf(",") + 1,roombaTimeFormattedTime.indexOf(":"));
  String minute = roombaTimeFormattedTime.substring(roombaTimeFormattedTime.indexOf(":") + 1, roombaTimeFormattedTime.indexOf(":") + 3);

  weekdayName.trim();
  hour.trim();
  minute.trim();

  if(weekdayName == "sunday") {
    dayCode = 0;
  }

  if(weekdayName == "monday") {
    dayCode = 1;
  }

  if(weekdayName == "tuesday") {
    dayCode = 2;
  }

  if(weekdayName == "wednesday") {
    dayCode = 3;
  }

  if(weekdayName == "thursday") {
    dayCode = 4;
  }

  if(weekdayName == "friday") {
    dayCode = 5;
  }

  if(weekdayName == "saturday") {
    dayCode = 6;
  }

  //Validate we have valid data
  if(dayCode == -1){
    
    broadcastLine("Failed to set day time.  DayCode = " + (String)dayCode + ".  Passed string: [" + roombaTimeFormattedTime + "]");
    return;
  }

  if((hour.toInt() < 0) || (hour.toInt() >24)){
    broadcastLine("Failed to set day time.  Hour = " + (String)hour.toInt() + ".  Passed string: [" + roombaTimeFormattedTime + "]");
    return;
  }

  if((minute.toInt() < 0) || (minute.toInt() >60)){
    broadcastLine("Failed to set day time.  Minute = " + (String)minute.toInt() + ".  Passed string: [" + roombaTimeFormattedTime + "]");
    return;
  }

  broadcastLine("Setting Roomba's Time to " + weekdayName + " [" + (String)dayCode + "] Hour: [" + (String)hour.toInt() + "] Minute: [" + (String)minute.toInt() + "]\n");

  //Send the commands to Roomba
  Serial1.write(128);
  delay(50);
  Serial1.write(168);
  delay(50);
  Serial1.write(dayCode);
  delay(50);
  Serial1.write(hour.toInt());
  delay(50);
  Serial1.write(minute.toInt());
  delay(50);

  broadcastLine("Done.\n");

}


void commandRoombaReboot(){
  /*
  * Forces Roomba to reboot, essentially a battery remove/insert.  This resets the clock and schedules.
  */

  broadcastLine("Rebooting Roomba.\n");
  
  Serial1.write(128);
  delay(50);
  Serial1.write(7);
  delay(2000);
  
  broadcastLine("Done.\n");

}


void commandRoombaWake(){


  //Only attempt to wake Roomba when it is sleeping
  if(roombaDataObserved.runStatus != SLEEPING){

    return;
  }

  broadcastLine("Waking Roomba.\n");

  //Pulse the pin to low then back to high, which wakes Roomba
  digitalWrite(wakePin, LOW);
  delay(100);
  digitalWrite(wakePin, HIGH);

  //Set the status to idling
  setRunStatus(IDLING);

  broadcastLine("Done.\n");
 
}


uint16_t combineBytesToUnsignedInt(char byteHigh, char byteLow){
  /* 
  * Converts a 2-byte number into an unsigned integer
  */

  uint16_t returnValue;

  returnValue = byteHigh;

  //Store the high byte on the first 8 bytes
  returnValue = returnValue << 8;

  //Shift the low byte to the right 8 bytes
  returnValue |= byteLow & 0x00FF;

  //Return the new value
  return returnValue;  
  
}


int16_t combineBytesToSignedInt(char byteHigh, char byteLow){
  /* 
  * Converts a 2-byte number into a signed integer
  */

  int16_t returnValue;

  returnValue = byteHigh;

  //Store the high byte on the first 8 bytes
  returnValue = returnValue << 8;

  //Shift the low byte to the right 8 bytes
  returnValue |= byteLow & 0x00FF;

  //Return the new value
  return returnValue;  
  
}


void wwwHandleRoot(){

  if (webServer.hasArg("cmdProvision")) {
    wwwHandleSubmit();
  }
  else {

    char temp[2000];

    snprintf(temp, 2000,
    "<!DOCTYPE html>\
    <html>\
      <head>\
        <meta http-equiv=\"content-type\" content=\"text/html; charset=UTF-8\">\
        <title>Device Provisioning</title>\
        <style>\
          * {font-family: \"Lucida Sans Unicode\", \"Lucida Grande\", sans-serif; }\
          body {font-size: 1em; color: #81D8D0; background-color: #363636;}\
          a {color: #81D8D0;}\
          input {width: 100%%; height: 30px; font-size: 1em; color: #363636;}\
          #firmwareVersion {font-size: 0.5em; position: fixed; bottom: 0; right: 0; }\
        </style>\
      </head>\
      <body>\
        <p style=\"text-align: center; font-size: 1.5em;\">Roomba MQTT Interface<p><p style=\"font-size: 1em; text-align: center; color: #d881b1;\">Device Name: %s<br>MAC Address: %s</p>\
        <form name=\"frmMain\" action=\"/\" method=\"POST\">\
        <label>SSID</label><input name=\"txtSSID\" type=\"text\"><br>\
        <label>WPA Key</label><input name=\"txtWpaKey\" type=\"text\"><br>\
        <label>Bootstrap URL (use FQDN)</label><input name=\"txtBootstrapURL\" value=\"http://server/%s.json\" type=\"text\"><br>\
        <br>\
        <center><input name=\"cmdProvision\" value=\"Provision\" type=\"submit\" style=\"width: 50%%; background-color: #81D8D0;\"></center>\
        </form>\
        <div id=\"firmwareVersion\"><a href=\"./update\">Firmware Version: %d</a></div>\
      </body>\
    </html>", settings.deviceName.c_str(), settings.machineMacAddress.c_str(), settings.deviceName.c_str(), firmwareVersion);

    webServer.send(200, "text/html", temp);
  }
}


void wwwHandleSubmit(){

  //See if we're trying to submit for the provisioning page
  if(webServer.hasArg("cmdProvision")){

    String SSID = webServer.arg("txtSSID");
    String wpaKey = webServer.arg("txtWpaKey");
    String bootstrapURL = webServer.arg("txtBootstrapURL");

    //Trim each of the inputs
    SSID.trim();
    wpaKey.trim();
    bootstrapURL.trim();
      
    broadcastLine("Attempting SSID " + SSID + " with WPA Key " + wpaKey + ".\nBootstrap URL: " + bootstrapURL);
    
    webServer.sendHeader("Connection", "close");
    webServer.sendHeader("Access-Control-Allow-Origin", "*");
    webServer.send(200, "text/plain", "OK\r\n");

    //Give the system time to send the data back to the caller before attempting to provision
    delay(1000);
    
    //Attempt to bootstrap
    attemptProvision(SSID, wpaKey, bootstrapURL);
    
    return;
  }

  //if...some other post page


  //If we made it here, we have an unhandled error
  webServer.sendHeader("Connection", "close");
  webServer.sendHeader("Access-Control-Allow-Origin", "*");
  webServer.send(400, "text/plain", "Unknown Request\r\n");
    
}


void wwwHandleNotFound(){
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += webServer.uri();
  message += "\nMethod: ";
  message += (webServer.method() == HTTP_GET)?"GET":"POST";
  message += "\nArguments: ";
  message += webServer.args();
  message += "\n";
  for (uint8_t i=0; i<webServer.args(); i++){
    message += " " + webServer.argName(i) + ": " + webServer.arg(i);
  }
  webServer.send(404, "text/plain", message);
}


void checkFirmwareUpgrade(){

  //Build the firmware URL
  String firmwareVersionURL = settings.firmware.url;

  //Replace $DEVICENAME$ with the actual device name
  firmwareVersionURL.replace("$DEVICENAME$", settings.deviceName);

  //Create an http client to connect to the firmware server
  HTTPClient httpClient;
  httpClient.begin(firmwareVersionURL);

  int httpCode = httpClient.GET();

  //See if we get an HTTP 200/OK response
  if( httpCode == 200 ) {

    String jsonServerResponse = httpClient.getString();

    //Read the data from the client response
    DynamicJsonBuffer jsonBuffer(1500);

    //Read the EEPROM
    JsonObject& root = jsonBuffer.parseObject(jsonServerResponse);
    
    if (!root.success()) {
      broadcastLine("Unable to parse firmware server response.\n");
      broadcastLine(jsonServerResponse);
      return;
    }

    //Get the firmware version
    String newFirmwareVersion = root["version"].asString();
    String firmwareBinURL = root["url"].asString();

    broadcastLine(padRight("", 64, "%"));
    broadcastLine("Current firmware version: " + (String)firmwareVersion);
    broadcastLine( "Available firmware version: " + (String)newFirmwareVersion);

    //Check if the version requested is not the same as the one loaded (yes, we allow back flashing)
    if( (String)newFirmwareVersion != (String)firmwareVersion ) {
      broadcastLine( "Preparing to update firmware using " + firmwareBinURL);
      publishMQTT(settings.mqttServer.clientTopic + "/status", "Updating Firmware");

      //Wait 1 second for the message to be sent
      delay(1000);

      //Attempt to retrieve the firmware and perform the update
      t_httpUpdate_return firmwareUpdateResponse = ESPhttpUpdate.update(firmwareBinURL);

      switch(firmwareUpdateResponse) {
        case HTTP_UPDATE_FAILED:
          broadcastLine("HTTP Firmware Update Error: " + (String)ESPhttpUpdate.getLastErrorString().c_str());
          publishMQTT(settings.mqttServer.clientTopic + "/status", "Firmware Update Failed");
          publishMQTT(settings.mqttServer.clientTopic + "/firmware", (String)firmwareVersion);
          break;

        case HTTP_UPDATE_NO_UPDATES:
          broadcastLine("HTTP_UPDATE_NO_UPDATES\n");
          break;
      }
    }
    else {
      broadcastLine("Firmware is up-to-date.\n");
      publishMQTT(settings.mqttServer.clientTopic + "/firmware", (String)firmwareVersion);
    }
  }
  else {
    broadcastLine("Firmware version check failed, got HTTP response code " + (String)httpCode + " while trying to retrieve firmware version from " + (String)firmwareVersionURL);
    publishMQTT(settings.mqttServer.clientTopic + "/status", "Firmware Update Failed");
    publishMQTT(settings.mqttServer.clientTopic + "/firmware", (String)firmwareVersion);
  }
  
  broadcastLine(padRight("", 64, "%"));
  
  httpClient.end();
}
