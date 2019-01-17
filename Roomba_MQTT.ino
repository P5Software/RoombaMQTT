/*
 * Roomba MQTT Status and Control
 * 
 *  MQTT Topics:
 *    Device Management Requests:
 *      /manage/[Device Name]/[Operation]
 *  
 *    Device Status Responses:
 *      /client/[Device Name]/[Response]
 *      
 *    Note, MAC addresses have no spaces, dashes, or colons.  MAC AA:BB:CC:DD:EE:00 = AABBCCDDEE00.
 *    
 *    Inspired by https://github.com/thehookup/MQTT-Roomba-ESP01.
 * 
 * (C) 2019, P5 SOftware, LLC.
 */

#define SLEEPING 0
#define CLEANING 1
#define IDLING 2
 
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


const int firmwareVersion = 53;


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

};


typedef struct structRoombaData{
  
  int interfaceMode;
  int currentStatus;

  struct {
    unsigned long startTime;
    unsigned long endTime;
    unsigned long runTime;
  } cleaningCycle;
 
  struct {
    unsigned long lastRetrieved;
    uint16_t voltage;
    int16_t current;
    int temperature;
    uint16_t charge;
    uint16_t capacity;
    String chargingState;
    String percentRemaining;
  } battery;

  struct {
    unsigned long lastRetrieved;
    long priorDistance;
    long odometer;
  } distance;

  struct {
    unsigned long lastRetrieved;
    int dirtiness;
  } dirt;

  struct {
    unsigned long lastRetrieved;
    int degrees;    
  } angle;

  struct {
    unsigned long lastRetrieved;
    int current;
  } leftWheelMotor;
    
  struct {
    unsigned long lastRetrieved;
    int current;
  } rightWheelMotor;
  
  struct {
    unsigned long lastRetrieved;
    int current;
  } mainBrushMotor;

  struct {
    unsigned long lastRetrieved;
    int current;
  } sideBrushMotor;
 
};
  
const int wakePin = D3;
const int EEPROMLength = 512;
const int EEPROMDataBegin = 10; //Reserving the first 10 addresses for provision statuses
struct structSettings settings;
struct structRoombaData roombaData;
WiFiClient espClient;
ESP8266WebServer webServer(80);
ESP8266HTTPUpdateServer httpUpdater;
PubSubClient mqttClient(espClient);
WiFiServer telnetServer(23);
WiFiClient telnetClient;
WiFiClient tmpTelnetClient;


void setup() {
  
  Serial.begin(115200);
  Serial1.begin(115200);

  pinMode(wakePin, OUTPUT);

  delay(2000);

  broadcastLine("\n");
  broadcastLine(padRight("", 23, ">") + " System Restarted " + padRight("", 23, "<") + "\n");

  //Get the MAC address
  settings.machineMacAddress = WiFi.macAddress();
  settings.machineMacAddress.toUpperCase();

  //Set the device name from the MAC address
  settings.deviceName = settings.machineMacAddress;
  settings.deviceName.replace(":","");

  //Set the hostname
  WiFi.hostname("Roomba-" + settings.deviceName);  
  broadcastLine("Hostname is " +  WiFi.hostname() + "\n");

  //Configure the settings
  settings.health.updateFrequency = 1680000; //Every 28 minutes
  settings.firmware.updateFrequency = 3600000; //Every sixty minutes
  settings.sensors.updateFrequency = 1000; //Every second


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
  
    broadcastLine("Attempting to connect to network " + settings.ssidName + "\n"); 
    
    //Print output while waiting for WiFi to connect
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
    }

    //Connected, begin sending to the broadcastLin function
    broadcastLine("IP address: " + WiFi.localIP().toString() + "\nMAC Address: " + WiFi.macAddress() + "\n");

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

      //We want to ignore the first attempt to run the health check, otherwise it will fire on startup, which is abusive
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
    
  }

  int inByte = 0;

  //Watch the serial line for unsolicited communications
  if(Serial.available()){

    inByte = Serial.read();
  
    broadcastLine((String)(char)inByte);
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
  
  WiFi.softAPConfig(ip, gateway, subnet);
  
  WiFi.softAP(("Roomba-" + settings.deviceName).c_str());

  webServer.on("/", wwwHandleRoot);
  webServer.onNotFound(wwwHandleNotFound);

  MDNS.begin(settings.deviceName.c_str());

  httpUpdater.setup(&webServer);

  MDNS.addService("http", "tcp", 80);

  webServer.begin();
  
  broadcastLine("Device requires provisioning.\n");
  broadcastLine("Connect to Access Point Roomba-" + settings.deviceName + " and point your browser to http://192.168.1.1 to begin.  To update firmware, go to http://192.168.1.1/update.\n");
  
}


void attemptProvision(String SSID, String wpaKey, String bootstrapURL){

  //Kill the soft AP
  WiFi.softAPdisconnect(true);

  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID.c_str(), wpaKey.c_str());

  //Remember the start time
  unsigned long timeStartConnect = millis();

  broadcastLine("Attempting to connect to network " + SSID + "\n");
  
  //Print output while waiting for WiFi to connect
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    //Serial.print(".");

    //Set a timer for 30 seconds to connect
    if(millis() - timeStartConnect > 30000){

      //Disconnect the WiFi
      WiFi.disconnect();

      //Write an error
      broadcastLine("\nAttempting to provision failed!  Unable to connect to AP specified.\n");

      //Restart the provisioning server
      setupProvisioningMode();

      //Exit this
      return;
      
    }
  }

  //Serial.print("connected.");
  broadcastLine("\n");
  broadcastLine("IP address: " + WiFi.localIP().toString() + "\nMAC Address: " + WiFi.macAddress() + "\n");

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
      
      broadcastLine("failed, rc=" + (String)mqttClient.state() + "\n");
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

  receivedPayload.trim();

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
  broadcastLine("<=> " + padRight(topic, 40, " ") + payload + "\n");
  
}


void handleMQTTManagementMessage(String topic, String payload){
  /* Handles incoming management requests from MQTT.  Payloads must exactly match.  Output is echoed on serial and MQTT /client/# topic. */

  //Copy the payload so we can modify it
  String payloadToLowerCase = payload;

  //Make the payload lower
  payloadToLowerCase.toLowerCase();

  //See if the payload requests a restart
  if(topic == settings.mqttServer.manageTopic + "/restart" && payloadToLowerCase == "request"){

    publishMQTT(settings.mqttServer.clientTopic + "/restart", "OK");
    
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

      broadcastLine("Bootstrapped from " + payload + "\n");
      publishMQTT(settings.mqttServer.clientTopic + "/bootstrap", "OK");
      broadcastLine(EEPROMRead(EEPROMDataBegin));

      return;
      
    }else{
      
      publishMQTT(settings.mqttServer.clientTopic + "/bootstrap", "Failed");

      return;
      
    }

  }

  //Clear the EEPROM and reset provisioning
  if(topic == settings.mqttServer.manageTopic + "/provisionReset" && payloadToLowerCase == "request"){

    EEPROMWrite("false", 0);

    publishMQTT(settings.mqttServer.clientTopic + "/status", "Provision Reset");
    publishMQTT(settings.mqttServer.clientTopic + "/restart", "OK");

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
   * This function writes a string of data to the EEPROM beginning at a specified address.  It is sandwiched between two non-printable ASCII characters (2/STX and 3/ETX)
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

  telnetClient.print(data);
  
}


String padRight(String data, int toLength, String padValue){

  while(data.length() < toLength){
    data = data + padValue;    
  }

  return data;
    
}


void handleHealth(){

  broadcastLine("void handleHealth()");

  //Publish the uptime
  publishMQTT(settings.mqttServer.clientTopic + "/uptime", String(millis()));

  //Publish the IP Address
  publishMQTT(settings.mqttServer.clientTopic + "/ip", WiFi.localIP().toString());

  //Publish the Firmware version
  publishMQTT(settings.mqttServer.clientTopic + "/firmware", (String)firmwareVersion);

  //Publish the device name
  publishMQTT(settings.mqttServer.clientTopic + "/deviceName", settings.deviceName);

  //Retrieve the battery info if Roomba isn't sleeping
  //if(roombaData.currentStatus != SLEEPING){
    
  getBatteryInfo();

  //Publish Roomba's Charge State
  publishMQTT(settings.mqttServer.sensorTopic + "/chargingState", roombaData.battery.chargingState);

  //Publish Roomba's Battery Voltage
  publishMQTT(settings.mqttServer.sensorTopic + "/batteryVoltage", (String)roombaData.battery.voltage);

  //Publish Roomba's Battery Temperature
  publishMQTT(settings.mqttServer.sensorTopic + "/batteryTemperature", (String)roombaData.battery.temperature);

  //Publish Roomba's Battery Capacity
  publishMQTT(settings.mqttServer.sensorTopic + "/batteryCapacity", (String)roombaData.battery.capacity);

  //Publish Roomba's Battery Charge
  publishMQTT(settings.mqttServer.sensorTopic + "/batteryCharge", (String)roombaData.battery.charge);

  //Publish Roomba's Battery Percentage
  publishMQTT(settings.mqttServer.sensorTopic + "/batteryPercentage", (String)roombaData.battery.percentRemaining);

  //Publish Roomba's Battery Current
  publishMQTT(settings.mqttServer.sensorTopic + "/batteryCurrent", (String)roombaData.battery.current);

}


void getBatteryInfo(){
    broadcastLine("void getBatteryInfo()");

  //Make sure we are respecting the minimum battery sensor read rate
  if((unsigned long)(millis() - roombaData.battery.lastRetrieved) < (unsigned long)settings.sensors.updateFrequency){

    broadcastLine("Ignoring getBatteryInfo; Abusive.\n");

    //Not enough milliseconds have elapsed
    return;

  }

  broadcastLine("Attempting to read Roomba's battery.\n");

  Serial1.write(128);
  delay(50);
  Serial1.write(142);
  delay(50);
  Serial1.write(3); //Retrieve packets 21-26, length of 10
  delay(50);

  int32_t tmpBuffer[10];
  int i = 0;

  while(Serial.available()){

    tmpBuffer[i] = Serial.read();
    
    i++;
  }

   //Get the charging state (0, 1)
  switch(tmpBuffer[0]){

    case 0:
      roombaData.battery.chargingState = "NOT_CHARGING";
      break;
    
    case 1:
      roombaData.battery.chargingState = "CHARGING_RECONDITIONING";
      break;
    
    case 2:
      roombaData.battery.chargingState = "CHARGING_FULL";
      break;
    
    case 3:
      roombaData.battery.chargingState = "CHARGING_TRICKLE";
      break;
    
    case 4:
      roombaData.battery.chargingState = "WAITING";
      break;
    
    case 5:
      roombaData.battery.chargingState = "FAULT";
      break;
    
    default:
      roombaData.battery.chargingState = "UNKNOWN";
  }
  
  //Get the voltage (1, 2)
  roombaData.battery.voltage = combineBytesToInt((uint8_t)tmpBuffer[1], (uint8_t)tmpBuffer[2]);

  //Get the current (3,2)
  roombaData.battery.current = combineBytesToInt(tmpBuffer[3], tmpBuffer[4]);

  broadcastLine("tmpBuffer[3]: [" + (String)tmpBuffer[3] + "] tmpBuffer[4]: [" + (String)tmpBuffer[4] + "]\n");

  //Get the temperature (5, 1)
  roombaData.battery.temperature = tmpBuffer[5];

  //Get the charge (6, 2)
  roombaData.battery.charge = combineBytesToInt((uint8_t)tmpBuffer[6], (uint8_t)tmpBuffer[7]);

  //Get the capacity (8, 2)
  roombaData.battery.capacity = combineBytesToInt((uint8_t)tmpBuffer[8], (uint8_t)tmpBuffer[9]);

  //Get the percent remaining
  roombaData.battery.percentRemaining = round(((float)roombaData.battery.charge / (float)roombaData.battery.capacity) * 100);

  //Set the last retrieved time
  roombaData.battery.lastRetrieved = millis();
  
}


void commandRoombaClean(){
      broadcastLine("void commandRoombaClean()");

  //Ensure Roomba is awake
  commandRoombaWake();
  
  broadcastLine("Commanding Roomba to Clean\n");

  //Send the command to Roomba
  Serial1.write(128);
  delay(50);
  Serial1.write(131);
  delay(50);
  Serial1.write(135);
  delay(50);

  //Set Roomba's status to cleaning
  roombaData.currentStatus == CLEANING;

}


void commandRoombaDock(){
        broadcastLine("void commandRoombaDock()");

  //Ensure Roomba is awake
  commandRoombaWake();

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
        broadcastLine("void commandRoombaStop()");
  broadcastLine("Commanding Roomba to Stop\n");

  //Send the command to Roomba
  Serial1.write(128);
  delay(50);
  Serial1.write(131);
  delay(50);
  Serial1.write(133);
  delay(50);

  //Set Roomba's status to SLEEPING
  roombaData.currentStatus == SLEEPING;
  
}


void commandRoombaResetSchedule(){
  broadcastLine("void commandRoombaResetSchedule()");
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
    broadcastLine("void commandRoombaSetDayTime(String roombaTimeFormattedTime)");
  //Requires time in "WEEKDAYNAME,hh:mm" format, inclusive of leading zero's

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

  //Validate we have a correct day
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
    broadcastLine("void commandRoombaReboot()");
  broadcastLine("Rebooting Roomba.\n");
  
  Serial1.write(128);
  delay(50);
  Serial1.write(7);
  delay(2000);
  
  broadcastLine("Done.\n");

  //Set Roomba's status to SLEEPING
  roombaData.currentStatus == SLEEPING;

}


void commandRoombaWake(){
broadcastLine("void commandRoombaWake()");
  //debug
  roombaData.currentStatus = IDLING;

  return;


  //Only attempt to wake Roomba when it is sleeping
  if(roombaData.currentStatus != SLEEPING){

    return;
  }

  broadcastLine("Waking Roomba.\n");

  //Pulse the pin to low then back to high, which wakes Roomba
  digitalWrite(wakePin, LOW);
  delay(100);
  digitalWrite(wakePin, HIGH);

  //Set the status to idling
  roombaData.currentStatus = IDLING;

  broadcastLine("Done.\n");
 
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
      
    broadcastLine("Attempting SSID " + SSID + " with WPA Key " + wpaKey + ".\nBootstrap URL: " + bootstrapURL + "\n");
    
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


int combineBytesToInt(int byteHigh, int byteLow){

  int16_t returnValue = 0;
  int tmpByteHigh = 0;
  int tmpByteLow = 0;

  //Store the temporary values and left-justify them
  tmpByteHigh = byteHigh << 8;
  tmpByteLow = byteLow >> 8;

  //Store the combined result
  returnValue = tmpByteHigh;

  //Pad the last 8 bytes
  returnValue |= tmpByteLow;

  //Return the new value
  return returnValue;  
  
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
    message += " " + webServer.argName(i) + ": " + webServer.arg(i) + "\n";
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
    broadcastLine("Current firmware version: " + (String)firmwareVersion + "\n");
    broadcastLine( "Available firmware version: " + (String)newFirmwareVersion + "\n");

    //Check if the version requested is not the same as the one loaded (yes, we allow back flashing)
    if( (String)newFirmwareVersion != (String)firmwareVersion ) {
      broadcastLine( "Preparing to update firmware using " + firmwareBinURL + "\n");
      publishMQTT(settings.mqttServer.clientTopic + "/firmwareUpdate", "Updating");
      publishMQTT(settings.mqttServer.clientTopic + "/status", "Updating Firmware");

      //Attempt to retrieve the firmware and perform the update
      t_httpUpdate_return firmwareUpdateResponse = ESPhttpUpdate.update(firmwareBinURL);

      switch(firmwareUpdateResponse) {
        case HTTP_UPDATE_FAILED:
          broadcastLine("HTTP Firmware Update Error: " + (String)ESPhttpUpdate.getLastErrorString().c_str() + "\n");
          publishMQTT(settings.mqttServer.clientTopic + "/firmwareUpdate", "Failed");
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
    broadcastLine("Firmware version check failed, got HTTP response code " + (String)httpCode + " while trying to retrieve firmware version from " + (String)firmwareVersionURL + "\n");
    publishMQTT(settings.mqttServer.clientTopic + "/firmwareUpdate", "Failed");
    publishMQTT(settings.mqttServer.clientTopic + "/status", "Firmware Update Failed");
    publishMQTT(settings.mqttServer.clientTopic + "/firmware", (String)firmwareVersion);
  }
  
  broadcastLine(padRight("", 64, "%"));
  
  httpClient.end();
}
