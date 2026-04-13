/***********************************************************************/ /**
   \file   n2k.ino
   \brief  NMEA2000 to signalk over zenoh



   This base file is a bidirectional NMEA2000 to signalk over zenoh

   Does not fullfill all NMEA2000 requirements.
 */

#define NODENAME "n2k"
// #define N2k_SPI_CS_PIN 53    // If you use mcp_can and CS pin is not 53, uncomment this and modify definition to match your CS pin.
// #define N2k_CAN_INT_PIN 21   // If you use mcp_can and interrupt pin is not 21, uncomment this and modify definition to match your interrupt pin.
// #define USE_MCP_CAN_CLOCK_SET 8  // If you use mcp_can and your mcp_can shield has 8MHz chrystal, uncomment this.
#define ESP32_CAN_TX_PIN GPIO_NUM_21 // If you use ESP32 and do not have TX on default IO 16, uncomment this and and modify definition to match your CAN TX pin.
#define ESP32_CAN_RX_PIN GPIO_NUM_20 // If you use ESP32 and do not have RX on default IO 4, uncomment this and and modify definition to match your CAN RX pin.

#define LED_BLUE 8 // blue LED pin

#include <n2k.h>

// modbus
HardwareSerial modSerial(1);

NMEA2000Node nmea2000Node;

// Define schedulers for messages. They are declared in base but keep local reference for clarity.
tN2kSyncScheduler n2kScheduler(false, 100, 500);

// *****************************************************************************
// Call back for NMEA2000 open. This will be called, when library starts bus communication.
// See NMEA2000.SetOnOpen(OnN2kOpen); on setup()

void OnN2kOpen()
{
  // Start schedulers now.
  n2kScheduler.UpdateNextTime();
}

void setData(char* key, double value){
  webServerNode.setSensorData(key, value);
  zenoh.publish(key, value);
  readings[key] = value;
}

//*****************************************************************************
void handleHeading(const tN2kMsg &N2kMsg)
{
  unsigned char SID;
  tN2kHeadingReference ref;
  double deviation = 0;
  double variation;
  double heading;

  if (ParseN2kHeading(N2kMsg, SID, heading, deviation, variation, ref))
  {
    setData(KEY_NAVIGATION_HEADINGTRUE, heading + variation + deviation);
  }
}

//*****************************************************************************
void handleBoatSpeed(const tN2kMsg &N2kMsg)
{
  unsigned char SID;
  double waterReferenced;
  double groundReferenced;
  tN2kSpeedWaterReferenceType SWRT;

  if (ParseN2kBoatSpeed(N2kMsg, SID, waterReferenced, groundReferenced, SWRT))
  {
    setData(KEY_NAVIGATION_SPEEDTHROUGHWATER, waterReferenced);
    setData(KEY_NAVIGATION_SPEEDOVERGROUND, groundReferenced);
  }
}

//*****************************************************************************
void handleDepth(const tN2kMsg &N2kMsg)
{
  unsigned char SID;
  double depthBelowTransducer;
  double offset;
  double range;
  double waterDepth;

  if (ParseN2kWaterDepth(N2kMsg, SID, depthBelowTransducer, offset, range))
  {
    waterDepth = depthBelowTransducer + offset;
    setData(KEY_ENVIRONMENT_DEPTH_BELOWTRANSDUCER, depthBelowTransducer);
    setData(KEY_ENVIRONMENT_DEPTH_BELOWSURFACE, waterDepth);
  }
}

//*****************************************************************************
void handlePosition(const tN2kMsg &N2kMsg)
{
  double latitude;
  double longitude;
  char buf[100];

  if (ParseN2kPGN129025(N2kMsg, latitude, longitude))
  {
   
    setData(KEY_NAVIGATION_POSITION_ALTITUDE, 0.0);
    setData(KEY_NAVIGATION_POSITION_LATITUDE, latitude);
    setData(KEY_NAVIGATION_POSITION_LONGITUDE, longitude);
  }
}

//*****************************************************************************
void handleCOG_SOG(const tN2kMsg &N2kMsg)
{
  unsigned char SID;
  tN2kHeadingReference ref;
  double cog;
  double sog;

  if (ParseN2kPGN129026(N2kMsg, SID, ref, cog, sog))
  {
  
    setData(KEY_NAVIGATION_COURSEOVERGROUNDTRUE, cog);
    setData(KEY_NAVIGATION_SPEEDOVERGROUND, sog);

  }
}

//*****************************************************************************
void handleWind(const tN2kMsg &N2kMsg)
{
  syslog.debug.println("Handle n2k wind");
  unsigned char SID;
  tN2kWindReference windReference;

  double windAngle, windSpeed;

  if (ParseN2kWindSpeed(N2kMsg, SID, windSpeed, windAngle, windReference))
  {
    syslog.debug.printf("Handle n2k wind - parsed speed: %f, angle %f \n",windSpeed, windAngle);
    if( windReference == N2kWind_Apparent)
    {
   
      setData(KEY_ENVIRONMENT_WIND_ANGLEAPPARENT, windAngle);
      setData(KEY_ENVIRONMENT_WIND_SPEEDAPPARENT, windSpeed);

    }
    else if( windReference == N2kWind_True_boat)
    {
    
      setData(KEY_ENVIRONMENT_WIND_ANGLETRUEGROUND, windAngle);
      setData(KEY_ENVIRONMENT_WIND_SPEEDTRUE, windSpeed);

    }
    else if( windReference == N2kWind_True_water)
    {
    
      setData(KEY_ENVIRONMENT_WIND_ANGLETRUEWATER, windAngle);
      setData(KEY_ENVIRONMENT_WIND_SPEEDTRUE, windSpeed);

    }
  }
}

//*****************************************************************************
void handleLog(const tN2kMsg &N2kMsg)
{

  uint16_t daysSince1970;
  double secondsSinceMidnight;
  uint32_t log;
  uint32_t triplog;

  if (ParseN2kDistanceLog(N2kMsg, daysSince1970, secondsSinceMidnight, log, triplog))
  {
   
    setData(KEY_NAVIGATION_TRIP_LOG, (int)triplog);
    setData(KEY_NAVIGATION_LOG, (int)log);

  }
}

//*****************************************************************************
void handleWaterTemp(const tN2kMsg &N2kMsg)
{

  unsigned char SID;
  double outsideAmbientAirTemperature;
  double atmosphericPressure;
  double waterTemperature;

  if (ParseN2kPGN130310(N2kMsg, SID, waterTemperature, outsideAmbientAirTemperature, atmosphericPressure))
  {
  
    setData(KEY_ENVIRONMENT_WATER_TEMPERATURE, waterTemperature);
    setData(KEY_ENVIRONMENT_OUTSIDE_TEMPERATURE, outsideAmbientAirTemperature);
    setData(KEY_ENVIRONMENT_OUTSIDE_PRESSURE, atmosphericPressure);

  }
}

//*****************************************************************************
void handleRudder(const tN2kMsg &N2kMsg)
{

  double rudderPosition;
  unsigned char instance;
  tN2kRudderDirectionOrder rudderDirectionOrder;
  double angleOrder;

  if (ParseN2kRudder(N2kMsg, rudderPosition, instance, rudderDirectionOrder, angleOrder))
  {
  
    setData(KEY_STEERING_RUDDERANGLE, rudderPosition);
   
  }
}

//*****************************************************************************
void handleGNSS(const tN2kMsg &N2kMsg)
{

  unsigned char SID;
  uint16_t daysSince1970;
  double secondsSinceMidnight;
  double latitude;
  double longitude;
  double altitude;
  tN2kGNSStype GNSStype;
  tN2kGNSSmethod GNSSmethod;
  unsigned char nSatellites;
  double HDOP;
  double PDOP;
  double geoidalSeparation;
  unsigned char nReferenceStations;
  tN2kGNSStype referenceStationType;
  uint16_t referenceSationID;
  double ageOfCorrection;
  char buf[100];

  if (ParseN2kGNSS(N2kMsg, SID, daysSince1970, secondsSinceMidnight, latitude, longitude, altitude, GNSStype, GNSSmethod,
                   nSatellites, HDOP, PDOP, geoidalSeparation,
                   nReferenceStations, referenceStationType, referenceSationID, ageOfCorrection))
  {

  
    setData(KEY_NAVIGATION_GNSS_TYPE, GNSStype);
    setData(KEY_NAVIGATION_GNSS_HORIZONTALDILUTION, HDOP);
    setData(KEY_NAVIGATION_GNSS_POSITIONDILUTION, PDOP);

   
    setData(KEY_NAVIGATION_GNSS_SATELLITES, nSatellites);
    setData(KEY_NAVIGATION_GNSS_GEOIDALSEPARATION, geoidalSeparation);
    setData(KEY_NAVIGATION_GNSS_DIFFERENTIALAGE, ageOfCorrection);

    
    setData(KEY_NAVIGATION_GNSS_DIFFERENTIALREFERENCE, referenceSationID);
    setData(KEY_NAVIGATION_POSITION_ALTITUDE, altitude);
    setData(KEY_NAVIGATION_POSITION_LATITUDE, latitude);
    setData(KEY_NAVIGATION_POSITION_LONGITUDE, longitude);

  }
}

//
// Largely based on https://github.com/AK-Homberger/NMEA2000-SignalK-Gateway/blob/main/NMEA2000-SignalK-Gateway/NMEA2000-SignalK-Gateway.ino
// Modified to send zenoh flavour of signalk
//
void handleNMEA2000Msg(const tN2kMsg &N2kMsg)
{

  switch (N2kMsg.PGN)
  {
  case 127250L:
    handleHeading(N2kMsg);
  case 128259L:
    handleBoatSpeed(N2kMsg);
  case 128267L:
    handleDepth(N2kMsg);
  case 129025L:
    handlePosition(N2kMsg);
  case 129026L:
    handleCOG_SOG(N2kMsg);
  case 129029L:
    handleGNSS(N2kMsg);
  case 130306L:
    handleWind(N2kMsg);
  case 128275L:
    handleLog(N2kMsg);
  case 130310L:
    handleWaterTemp(N2kMsg);
  case 127245L:
    handleRudder(N2kMsg);
  }
}

// ZenohMessageCallback
void handleZenohWind(const char *topic, const char *payload, size_t len)
{
  // what data is this?
  char value[len+1] {'\0'};
  strncpy(value,payload,len);
  syslog.debug.printf("Zenoh message: %s = %s\n", topic, value);
  syslog.debug.printf("Zenoh message payload: %.*s\n", len, payload);
  if( strcmp(KEY_ENVIRONMENT_WIND_ANGLEAPPARENT , topic ) == 0)
  {
    readings[KEY_ENVIRONMENT_WIND_ANGLEAPPARENT] = strtod(value,NULL);
    //syslog.printf("    readings[KEY_ENVIRONMENT_WIND_ANGLEAPPARENT] = %f\n",readings[KEY_ENVIRONMENT_WIND_ANGLEAPPARENT].as<double>());
  }
  if( strcmp(KEY_ENVIRONMENT_WIND_SPEEDAPPARENT , topic ) == 0)
  {
    readings[KEY_ENVIRONMENT_WIND_SPEEDAPPARENT] = strtod(value,NULL);
  }
  if( strcmp(KEY_ENVIRONMENT_WIND_ANGLETRUEGROUND , topic ) == 0)
  {
    readings[KEY_ENVIRONMENT_WIND_ANGLETRUEGROUND] = strtod(value,NULL);
  }
  if( strcmp(KEY_ENVIRONMENT_WIND_SPEEDTRUE , topic ) == 0)
  {
    readings[KEY_ENVIRONMENT_WIND_SPEEDTRUE] = strtod(value,NULL);
  }

  //have we got data?
  if(!readings[KEY_ENVIRONMENT_WIND_ANGLEAPPARENT].isNull() && !readings[KEY_ENVIRONMENT_WIND_SPEEDAPPARENT].isNull()){
    nmea2000Node.sendWindApparent(readings[KEY_ENVIRONMENT_WIND_ANGLEAPPARENT].as<double>(), readings[KEY_ENVIRONMENT_WIND_SPEEDAPPARENT].as<double>(), false);
  }

  if(!readings[KEY_ENVIRONMENT_WIND_ANGLETRUEGROUND].isNull() && !readings[KEY_ENVIRONMENT_WIND_SPEEDTRUE].isNull()){
    nmea2000Node.sendWindTrue(readings[KEY_ENVIRONMENT_WIND_ANGLETRUEGROUND].as<double>(), readings[KEY_ENVIRONMENT_WIND_SPEEDTRUE].as<double>(), false);
  }
  
}

// int seq = 1;
void test(){
  tN2kMsg N2kMsg;
  SetN2kLatLonRapid(N2kMsg, -41.7034, 170.223);
  handleNMEA2000Msg(N2kMsg);
  // seq++;
  // if (seq == 255) seq = 1;
    
}

// *****************************************************************************
void setup()
{
  // Initialize base subsystems (WiFi, OTA, WebServer, Zenoh, Syslog)
  
  
  syslog.app = NODENAME;
  baseInit(NODENAME);
  zenoh.setHostname(NODENAME);

  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_BLUE, LOW);

  // zenoh key that is published.
  zenoh.declarePublisher(KEY_ENVIRONMENT_DEPTH_BELOWSURFACE);
  zenoh.declarePublisher(KEY_ENVIRONMENT_DEPTH_BELOWTRANSDUCER);
  zenoh.declarePublisher(KEY_ENVIRONMENT_OUTSIDE_PRESSURE);
  zenoh.declarePublisher(KEY_ENVIRONMENT_OUTSIDE_TEMPERATURE);
  zenoh.declarePublisher(KEY_ENVIRONMENT_WATER_TEMPERATURE);
  // zenoh.declarePublisher(KEY_ENVIRONMENT_WIND_ANGLEAPPARENT);
  //zenoh.declarePublisher(KEY_ENVIRONMENT_WIND_ANGLETRUEGROUND);
  //zenoh.declarePublisher(KEY_ENVIRONMENT_WIND_ANGLETRUEWATER);
  // zenoh.declarePublisher(KEY_ENVIRONMENT_WIND_SPEEDAPPARENT);
  //zenoh.declarePublisher(KEY_ENVIRONMENT_WIND_SPEEDTRUE);
  zenoh.declarePublisher(KEY_NAVIGATION_COURSEOVERGROUNDTRUE);
  zenoh.declarePublisher(KEY_NAVIGATION_GNSS_DIFFERENTIALAGE);
  zenoh.declarePublisher(KEY_NAVIGATION_GNSS_DIFFERENTIALREFERENCE);
  zenoh.declarePublisher(KEY_NAVIGATION_GNSS_GEOIDALSEPARATION);
  zenoh.declarePublisher(KEY_NAVIGATION_GNSS_HORIZONTALDILUTION);
  zenoh.declarePublisher(KEY_NAVIGATION_GNSS_POSITIONDILUTION);
  zenoh.declarePublisher(KEY_NAVIGATION_GNSS_SATELLITES);
  zenoh.declarePublisher(KEY_NAVIGATION_GNSS_TYPE);
  zenoh.declarePublisher(KEY_NAVIGATION_HEADINGTRUE);
  zenoh.declarePublisher(KEY_NAVIGATION_LOG);
  zenoh.declarePublisher(KEY_NAVIGATION_POSITION_ALTITUDE);
  zenoh.declarePublisher(KEY_NAVIGATION_POSITION_LATITUDE);
  zenoh.declarePublisher(KEY_NAVIGATION_POSITION_LONGITUDE);
  zenoh.declarePublisher(KEY_NAVIGATION_SPEEDOVERGROUND);
  zenoh.declarePublisher(KEY_NAVIGATION_SPEEDTHROUGHWATER);
  zenoh.declarePublisher(KEY_NAVIGATION_TRIP_LOG);
  zenoh.declarePublisher(KEY_STEERING_RUDDERANGLE);

  const long unsigned receiveMessages[] = {
      127250L, // Heading
      128259L, // Boat speed
      128267L, // Depth
      129025L, // Position
      129026L, // COG and SOG
      129029L, // GNSS
      // 130306L, // Wind
      128275L, // log
      130310L, // Water temperature
      127245L, // Rudder
      0};
  nmea2000Node.setReceiveMessages(receiveMessages);
  nmea2000Node.setReceiveMsgHandler(handleNMEA2000Msg);

  const long unsigned transmitMessages[] = {
      // 127250L, // Heading
      // 128259L, // Boat speed
      // 128267L, // Depth
      // 129025L, // Position
      // 129026L, // COG and SOG
      // 129029L, // GNSS
      130306L, // Wind
      // 128275L, // log
      // 130310L, // Water temperature
      // 127245L, // Rudder
      0};
  nmea2000Node.setTransmitMessages(transmitMessages);
  zenoh.subscribe(KEY_ENVIRONMENT_WIND_ANGLEAPPARENT, handleZenohWind);
  zenoh.subscribe(KEY_ENVIRONMENT_WIND_SPEEDAPPARENT, handleZenohWind);
  zenoh.subscribe(KEY_ENVIRONMENT_WIND_ANGLETRUEGROUND, handleZenohWind);
  zenoh.subscribe(KEY_ENVIRONMENT_WIND_SPEEDTRUE, handleZenohWind);
  nmea2000Node.init();
  nmea2000Node.setOnOpen(OnN2kOpen);
  nmea2000Node.open();
  
}

long last = millis();
bool blink = LOW;
// *****************************************************************************
void loop()
{

  // if (n2kScheduler.IsTime())
  // {
  //   n2kScheduler.UpdateNextTime();
   //test();
  if( (millis() - last)>1000){
    
    //JsonArray arr = JsonArray();
    //zenoh.getZenohPeers(arr);
    zenoh.getPeerHostnames();
    //getMDNShosts();
    last = millis();
    blink=!blink;
    digitalWrite(LED_BLUE, blink);
  }

  nmea2000Node.parseMessages();
  nmea2000Node.checkNodeAddress();

  // run base periodic tasks (Zenoh publish, OTA, web updates)
  baseLoopTasks();
}
