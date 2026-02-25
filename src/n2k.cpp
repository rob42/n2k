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
    webServerNode.setSensorData("navigation.headingTrue", heading + variation + deviation);
    // setup values for zenoh
    zenoh.publish(KEY_NAVIGATION_HEADINGTRUE, heading + variation + deviation);

    readings[KEY_NAVIGATION_HEADINGTRUE] = heading + variation + deviation;
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
    webServerNode.setSensorData("navigation.speedThroughWater", waterReferenced);
    webServerNode.setSensorData("navigation.speedOverGround", groundReferenced);
    // setup values for zenoh
    zenoh.publish(KEY_NAVIGATION_SPEEDTHROUGHWATER, waterReferenced);
    zenoh.publish(KEY_NAVIGATION_SPEEDOVERGROUND, groundReferenced);

    readings[KEY_NAVIGATION_SPEEDTHROUGHWATER] = waterReferenced;
    readings[KEY_NAVIGATION_SPEEDOVERGROUND] = groundReferenced;
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
    webServerNode.setSensorData("environment.depth.belowTransducer", depthBelowTransducer);
    webServerNode.setSensorData("environment.depth.belowSurface", waterDepth);
    // setup values for zenoh
    zenoh.publish(KEY_ENVIRONMENT_DEPTH_BELOWTRANSDUCER, depthBelowTransducer);
    zenoh.publish(KEY_ENVIRONMENT_DEPTH_BELOWSURFACE, waterDepth);

    readings[KEY_ENVIRONMENT_DEPTH_BELOWTRANSDUCER] = depthBelowTransducer;
    readings[KEY_ENVIRONMENT_DEPTH_BELOWSURFACE] = waterDepth;
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
    // snprintf(buf, sizeof(buf), "{\"altitude\":%f,\"latitude\":%f,\"longitude\":%f}", 0.0 , latitude, longitude);
    webServerNode.setSensorData("navigation.position.altitude", 0.0);
    webServerNode.setSensorData("navigation.position.latitude", latitude);
    webServerNode.setSensorData("navigation.position.longitude", longitude);
    // setup values for zenoh
    zenoh.publish(KEY_NAVIGATION_POSITION_ALTITUDE, 0.0);
    zenoh.publish(KEY_NAVIGATION_POSITION_LATITUDE, latitude);
    zenoh.publish(KEY_NAVIGATION_POSITION_LONGITUDE, longitude);

    readings[KEY_NAVIGATION_POSITION_ALTITUDE] = 0.0;
    readings[KEY_NAVIGATION_POSITION_LATITUDE] = latitude;
    readings[KEY_NAVIGATION_POSITION_LONGITUDE] = longitude;
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
    webServerNode.setSensorData("navigation.courseOverGroundTrue", cog);
    webServerNode.setSensorData("navigation.speedOverGround", sog);
    // setup values for zenoh
    zenoh.publish(KEY_NAVIGATION_COURSEOVERGROUNDTRUE, cog);
    zenoh.publish(KEY_NAVIGATION_SPEEDOVERGROUND, sog);

    readings[KEY_NAVIGATION_COURSEOVERGROUNDTRUE] = cog;
    readings[KEY_NAVIGATION_SPEEDOVERGROUND] = sog;
  }
}

//*****************************************************************************
void handleWind(const tN2kMsg &N2kMsg)
{
  unsigned char SID;
  tN2kWindReference windReference;

  double windAngle, windSpeed;

  if (ParseN2kWindSpeed(N2kMsg, SID, windSpeed, windAngle, windReference))
  {
    if( windReference == N2kWind_Apparent)
    {
      webServerNode.setSensorData("environment.wind.angleApparent", windAngle);
      webServerNode.setSensorData("environment.wind.speedApparent", windSpeed);
      // setup values for zenoh
      zenoh.publish(KEY_ENVIRONMENT_WIND_ANGLEAPPARENT, windAngle);
      zenoh.publish(KEY_ENVIRONMENT_WIND_SPEEDAPPARENT, windSpeed);

      readings[KEY_ENVIRONMENT_WIND_ANGLEAPPARENT] = windAngle;
      readings[KEY_ENVIRONMENT_WIND_SPEEDAPPARENT] = windSpeed;
    }
    else if( windReference == N2kWind_True_boat)
    {
      webServerNode.setSensorData("environment.wind.angleTrueGround", windAngle);
      webServerNode.setSensorData("environment.wind.speedTrue", windSpeed);
      // setup values for zenoh
      zenoh.publish(KEY_ENVIRONMENT_WIND_ANGLETRUEGROUND, windAngle);
      zenoh.publish(KEY_ENVIRONMENT_WIND_SPEEDTRUE, windSpeed);

      readings[KEY_ENVIRONMENT_WIND_ANGLETRUEGROUND] = windAngle;
      readings[KEY_ENVIRONMENT_WIND_SPEEDTRUE] = windSpeed;
    }
    else if( windReference == N2kWind_True_water)
    {
      webServerNode.setSensorData("environment.wind.angleTrueWater", windAngle);
      webServerNode.setSensorData("environment.wind.speedTrue", windSpeed);
      // setup values for zenoh
      zenoh.publish(KEY_ENVIRONMENT_WIND_ANGLETRUEWATER, windAngle);
      zenoh.publish(KEY_ENVIRONMENT_WIND_SPEEDTRUE, windSpeed);

      readings[KEY_ENVIRONMENT_WIND_ANGLETRUEWATER] = windAngle;
      readings[KEY_ENVIRONMENT_WIND_SPEEDTRUE] = windSpeed;
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
    webServerNode.setSensorData("navigation.trip.log", (int)triplog);
    webServerNode.setSensorData("navigation.log", (int)log);
    // setup values for zenoh
    zenoh.publish(KEY_NAVIGATION_TRIP_LOG, (int)triplog);
    zenoh.publish(KEY_NAVIGATION_LOG, (int)log);

    readings[KEY_NAVIGATION_TRIP_LOG] = (int)triplog;
    readings[KEY_NAVIGATION_LOG] = (int)log;
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
    webServerNode.setSensorData("environment.outside.temperature", outsideAmbientAirTemperature);
    webServerNode.setSensorData("environment.outside.pressure", atmosphericPressure);
    webServerNode.setSensorData("environment.water.temperature", waterTemperature);
    // setup values for zenoh
    zenoh.publish(KEY_ENVIRONMENT_WATER_TEMPERATURE, waterTemperature);
    zenoh.publish(KEY_ENVIRONMENT_OUTSIDE_TEMPERATURE, outsideAmbientAirTemperature);
    zenoh.publish(KEY_ENVIRONMENT_OUTSIDE_PRESSURE, atmosphericPressure);

    readings[KEY_ENVIRONMENT_WATER_TEMPERATURE] = waterTemperature;
    readings[KEY_ENVIRONMENT_OUTSIDE_TEMPERATURE] = outsideAmbientAirTemperature;
    readings[KEY_ENVIRONMENT_OUTSIDE_PRESSURE] = atmosphericPressure;
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
    webServerNode.setSensorData("steering.rudderAngle", rudderPosition);
    // setup values for zenoh
    zenoh.publish(KEY_STEERING_RUDDERANGLE, rudderPosition);
    readings[KEY_STEERING_RUDDERANGLE] = rudderPosition;
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

    webServerNode.setSensorData("navigation.gnss.type", GNSStype);
    webServerNode.setSensorData("navigation.gnss.horizontalDilution", HDOP);
    webServerNode.setSensorData("navigation.gnss.positionDilution", PDOP);

    // setup values for zenoh
    zenoh.publish(KEY_NAVIGATION_GNSS_TYPE, GNSStype);
    zenoh.publish(KEY_NAVIGATION_GNSS_HORIZONTALDILUTION, HDOP);
    zenoh.publish(KEY_NAVIGATION_GNSS_POSITIONDILUTION, PDOP);

    readings[KEY_NAVIGATION_GNSS_TYPE] = GNSStype;
    readings[KEY_NAVIGATION_GNSS_HORIZONTALDILUTION] = HDOP;
    readings[KEY_NAVIGATION_GNSS_POSITIONDILUTION] = PDOP;

    webServerNode.setSensorData("navigation.gnss.satellites", nSatellites);
    webServerNode.setSensorData("navigation.gnss.geoidalSeparation", geoidalSeparation);
    webServerNode.setSensorData("navigation.gnss.differentialAge", ageOfCorrection);

    // setup values for zenoh
    zenoh.publish(KEY_NAVIGATION_GNSS_SATELLITES, nSatellites);
    zenoh.publish(KEY_NAVIGATION_GNSS_GEOIDALSEPARATION, geoidalSeparation);
    zenoh.publish(KEY_NAVIGATION_GNSS_DIFFERENTIALAGE, ageOfCorrection);

    readings[KEY_NAVIGATION_GNSS_SATELLITES] = nSatellites;
    readings[KEY_NAVIGATION_GNSS_GEOIDALSEPARATION] = geoidalSeparation;
    readings[KEY_NAVIGATION_GNSS_DIFFERENTIALAGE] = ageOfCorrection;

    webServerNode.setSensorData("navigation.gnss.differentialReference", referenceSationID);
    // snprintf(buf, sizeof(buf), "{\"altitude\":%f,\"latitude\":%f,\"longitude\":%f}", altitude , latitude, longitude);
    webServerNode.setSensorData("navigation.position.altitude", altitude);
    webServerNode.setSensorData("navigation.position.latitude", latitude);
    webServerNode.setSensorData("navigation.position.longitude", longitude);

    // setup values for zenoh
    zenoh.publish(KEY_NAVIGATION_GNSS_DIFFERENTIALREFERENCE, referenceSationID);
    zenoh.publish(KEY_NAVIGATION_POSITION_ALTITUDE, altitude);
    zenoh.publish(KEY_NAVIGATION_POSITION_LATITUDE, latitude);
    zenoh.publish(KEY_NAVIGATION_POSITION_LONGITUDE, longitude);

    readings[KEY_NAVIGATION_GNSS_DIFFERENTIALREFERENCE] = referenceSationID;
    readings[KEY_NAVIGATION_POSITION_ALTITUDE] = altitude;
    readings[KEY_NAVIGATION_POSITION_LATITUDE] = latitude;
    readings[KEY_NAVIGATION_POSITION_LONGITUDE] = longitude;
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
  if( strcmp(KEY_ENVIRONMENT_WIND_ANGLEAPPARENT , topic ))
  {
    readings[KEY_ENVIRONMENT_WIND_ANGLEAPPARENT] = strtod(payload, NULL);
  }
  if( strcmp(KEY_ENVIRONMENT_WIND_SPEEDAPPARENT , topic ))
  {
    readings[KEY_ENVIRONMENT_WIND_SPEEDAPPARENT] = strtod(payload, NULL);
  }
  if( strcmp(KEY_ENVIRONMENT_WIND_ANGLETRUEGROUND , topic ))
  {
    readings[KEY_ENVIRONMENT_WIND_ANGLETRUEGROUND] = strtod(payload, NULL);
  }
  if( strcmp(KEY_ENVIRONMENT_WIND_SPEEDTRUE , topic ))
  {
    readings[KEY_ENVIRONMENT_WIND_SPEEDTRUE] = strtod(payload, NULL);
  }

  //have we got data?
  if(!readings[KEY_ENVIRONMENT_WIND_ANGLEAPPARENT].isNull() && !readings[KEY_ENVIRONMENT_WIND_ANGLEAPPARENT].isNull()){
    nmea2000Node.sendWindApparent(readings[KEY_ENVIRONMENT_WIND_ANGLEAPPARENT], readings[KEY_ENVIRONMENT_WIND_ANGLEAPPARENT], false);
  }

  if(!readings[KEY_ENVIRONMENT_WIND_ANGLETRUEGROUND].isNull() && !readings[KEY_ENVIRONMENT_WIND_SPEEDTRUE].isNull()){
    nmea2000Node.sendWindTrue(readings[KEY_ENVIRONMENT_WIND_ANGLETRUEGROUND], readings[KEY_ENVIRONMENT_WIND_SPEEDTRUE], false);
  }
  
}

// *****************************************************************************
void setup()
{
  // Initialize base subsystems (WiFi, OTA, WebServer, Zenoh, Syslog)
  ArduinoOTA.setHostname(NODENAME);
  syslog.app = NODENAME;
  baseInit();
  // zenoh key that is published.
  zenoh.declarePublisher(KEY_ENVIRONMENT_DEPTH_BELOWSURFACE);
  zenoh.declarePublisher(KEY_ENVIRONMENT_DEPTH_BELOWTRANSDUCER);
  zenoh.declarePublisher(KEY_ENVIRONMENT_OUTSIDE_PRESSURE);
  zenoh.declarePublisher(KEY_ENVIRONMENT_OUTSIDE_TEMPERATURE);
  zenoh.declarePublisher(KEY_ENVIRONMENT_WATER_TEMPERATURE);
  // zenoh.declarePublisher(KEY_ENVIRONMENT_WIND_ANGLEAPPARENT);
  zenoh.declarePublisher(KEY_ENVIRONMENT_WIND_ANGLETRUEGROUND);
  zenoh.declarePublisher(KEY_ENVIRONMENT_WIND_ANGLETRUEWATER);
  // zenoh.declarePublisher(KEY_ENVIRONMENT_WIND_SPEEDAPPARENT);
  zenoh.declarePublisher(KEY_ENVIRONMENT_WIND_SPEEDTRUE);
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

// *****************************************************************************
void loop()
{

  if (n2kScheduler.IsTime())
  {
    n2kScheduler.UpdateNextTime();
   
    // webServerNode.setSensorData("navigation.gnss.differentialReference", random());
    // webServerNode.setSensorData("navigation.position.altitude", random());
    // webServerNode.setSensorData("navigation.position.latitude", random());
    // webServerNode.setSensorData("navigation.position.longitude", random());
    
  }

  nmea2000Node.parseMessages();
  nmea2000Node.checkNodeAddress();

  // run base periodic tasks (Zenoh publish, OTA, web updates)
  baseLoopTasks();
}
