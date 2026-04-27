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

WMM_Tinier declination;

// *****************************************************************************
// Call back for NMEA2000 open. This will be called, when library starts bus communication.
// See NMEA2000.SetOnOpen(OnN2kOpen); on setup()

void OnN2kOpen()
{
  // Start schedulers now.
  n2kScheduler.UpdateNextTime();
}


void setDatafromZenoh(const char* key, double value, unsigned long millis){
  webServerNode.setSensorData(key, value);
  readings[key][KEY_VALUE] = value;
  readings[key][KEY_TIMEOUT] = millis;
}


void setDatafromN2k(const char* key, double value){
  setDatafromZenoh(key,value,millis());
  zenoh.publish(key, value);
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
     //value is in degrees 0-360?
    //if(variation>360 || variation < -360) variation = 0; //can be -1000000000!!

   /// syslog.debug.printf("Heading: %f, variation: %f, deviation: %f\n",heading,variation, deviation);

    if(ref == tN2kHeadingReference::N2khr_true) {
      // true heading
      setDatafromN2k(KEY_NAVIGATION_HEADINGTRUE, heading );
      if(!readings[KEY_NAVIGATION_MAGNETICDEVIATION].isNull()){
        double decl = readings[KEY_NAVIGATION_MAGNETICDEVIATION][KEY_VALUE].as<double>();
        setDatafromN2k(KEY_NAVIGATION_HEADINGMAGNETIC, heading + decl);
      }
    } else if(ref == tN2kHeadingReference::N2khr_magnetic) {
      // mag heading
      setDatafromN2k(KEY_NAVIGATION_HEADINGMAGNETIC, heading );
      if(!readings[KEY_NAVIGATION_MAGNETICDEVIATION].isNull()){
        double decl = readings[KEY_NAVIGATION_MAGNETICDEVIATION][KEY_VALUE].as<double>();
        setDatafromN2k(KEY_NAVIGATION_HEADINGTRUE, heading - decl);
      }
    }
   
    
    
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
    setDatafromN2k(KEY_NAVIGATION_SPEEDTHROUGHWATER, waterReferenced);
    setDatafromN2k(KEY_NAVIGATION_SPEEDOVERGROUND, groundReferenced);
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
    setDatafromN2k(KEY_ENVIRONMENT_DEPTH_BELOWTRANSDUCER, depthBelowTransducer);
    setDatafromN2k(KEY_ENVIRONMENT_DEPTH_BELOWSURFACE, waterDepth);
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
   
    setDatafromN2k(KEY_NAVIGATION_POSITION_ALTITUDE, 0.0);
    setDatafromN2k(KEY_NAVIGATION_POSITION_LATITUDE, latitude);
    setDatafromN2k(KEY_NAVIGATION_POSITION_LONGITUDE, longitude);
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
  
    setDatafromN2k(KEY_NAVIGATION_COURSEOVERGROUNDTRUE, cog);
    setDatafromN2k(KEY_NAVIGATION_SPEEDOVERGROUND, sog);

  }
}

//*****************************************************************************
void handleWind(const tN2kMsg &N2kMsg)
{
  //syslog.debug.println("Handle n2k wind");
  unsigned char SID;
  tN2kWindReference windReference;

  double windAngle, windSpeed;

  if (ParseN2kWindSpeed(N2kMsg, SID, windSpeed, windAngle, windReference))
  {
    //syslog.debug.printf("Handle n2k wind - parsed speed: %f, angle %f \n",windSpeed, windAngle);
    if( windReference == N2kWind_Apparent)
    {
   
      setDatafromN2k(KEY_ENVIRONMENT_WIND_ANGLEAPPARENT, windAngle);
      setDatafromN2k(KEY_ENVIRONMENT_WIND_SPEEDAPPARENT, windSpeed);

    }
    else if( windReference == N2kWind_True_boat)
    {
    
      setDatafromN2k(KEY_ENVIRONMENT_WIND_ANGLETRUEGROUND, windAngle);
      setDatafromN2k(KEY_ENVIRONMENT_WIND_SPEEDTRUE, windSpeed);

    }
    else if( windReference == N2kWind_True_water)
    {
    
      setDatafromN2k(KEY_ENVIRONMENT_WIND_ANGLETRUEWATER, windAngle);
      setDatafromN2k(KEY_ENVIRONMENT_WIND_SPEEDTRUE, windSpeed);

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
   
    setDatafromN2k(KEY_NAVIGATION_TRIP_LOG, (int)triplog);
    setDatafromN2k(KEY_NAVIGATION_LOG, (int)log);

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
  
    setDatafromN2k(KEY_ENVIRONMENT_WATER_TEMPERATURE, waterTemperature);
    setDatafromN2k(KEY_ENVIRONMENT_OUTSIDE_TEMPERATURE, outsideAmbientAirTemperature);
    setDatafromN2k(KEY_ENVIRONMENT_OUTSIDE_PRESSURE, atmosphericPressure);

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
  
    setDatafromN2k(KEY_STEERING_RUDDERANGLE, rudderPosition);
   
  }
}

//*****************************************************************************
// void handleDeclination(const tN2kMsg &N2kMsg)
// {
//   uint16_t daysSince1970;
//   double variation;
//   tN2kMagneticVariation source;
//   unsigned char SID;
//   if(ParseN2kMagneticVariation(N2kMsg, SID, source, daysSince1970, variation))
//   {
//     setDatafromN2k(KEY_NAVIGATION_MAGNETICDEVIATION, variation);
//   }

// }
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

  
    setDatafromN2k(KEY_NAVIGATION_GNSS_TYPE, GNSStype);
    setDatafromN2k(KEY_NAVIGATION_GNSS_HORIZONTALDILUTION, HDOP);
    setDatafromN2k(KEY_NAVIGATION_GNSS_POSITIONDILUTION, PDOP);

   
    setDatafromN2k(KEY_NAVIGATION_GNSS_SATELLITES, nSatellites);
    setDatafromN2k(KEY_NAVIGATION_GNSS_GEOIDALSEPARATION, geoidalSeparation);
    setDatafromN2k(KEY_NAVIGATION_GNSS_DIFFERENTIALAGE, ageOfCorrection);

    
    setDatafromN2k(KEY_NAVIGATION_GNSS_DIFFERENTIALREFERENCE, referenceSationID);
    setDatafromN2k(KEY_NAVIGATION_POSITION_ALTITUDE, altitude);
    setDatafromN2k(KEY_NAVIGATION_POSITION_LATITUDE, latitude);
    setDatafromN2k(KEY_NAVIGATION_POSITION_LONGITUDE, longitude);

  }
}

//
// Largely based on https://github.com/AK-Homberger/NMEA2000-SignalK-Gateway/blob/main/NMEA2000-SignalK-Gateway/NMEA2000-SignalK-Gateway.ino
// Modified to send zenoh flavour of signalk
//
void handleNMEA2000Msg(const tN2kMsg &N2kMsg)
{
  //syslog.debug.printf("N2K message received: %d\n", N2kMsg.PGN);
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
  // case 127258L:
  //   handleDeclination(N2kMsg);
  }
}

// ZenohMessageCallback
void handleZenohWind(const char *topic, const char *payload, size_t len)
{
  // what data is this?
  char value[len+1] {'\0'};
  strncpy(value,payload,len);
  //syslog.debug.printf("Zenoh message: %s = %s\n", topic, value);
  //syslog.debug.printf("Zenoh message payload: %.*s\n", len, payload);
  if( strcmp(KEY_ENVIRONMENT_WIND_ANGLEAPPARENT , topic ) == 0
      || strcmp(KEY_ENVIRONMENT_WIND_SPEEDAPPARENT , topic ) == 0 
      || strcmp(KEY_ENVIRONMENT_WIND_ANGLETRUEGROUND , topic ) == 0
      || strcmp(KEY_ENVIRONMENT_WIND_SPEEDTRUE , topic ) == 0)

  {
    setDatafromZenoh(topic,strtod(value,NULL), millis());
  }

  //have we got data?
  if(!readings[KEY_ENVIRONMENT_WIND_ANGLEAPPARENT].isNull() && !readings[KEY_ENVIRONMENT_WIND_SPEEDAPPARENT].isNull()){
    double angle = readings[KEY_ENVIRONMENT_WIND_ANGLEAPPARENT][KEY_VALUE].as<double>();
    double speed = readings[KEY_ENVIRONMENT_WIND_SPEEDAPPARENT][KEY_VALUE].as<double>();
   // syslog.debug.printf("Sending windapparent, angle = %f, speed = %f \n",angle,speed);
    nmea2000Node.sendWindApparent(angle,speed , false);
  }

  if(!readings[KEY_ENVIRONMENT_WIND_ANGLETRUEGROUND].isNull() && !readings[KEY_ENVIRONMENT_WIND_SPEEDTRUE].isNull()){
    nmea2000Node.sendWindTrue(readings[KEY_ENVIRONMENT_WIND_ANGLETRUEGROUND][KEY_VALUE].as<double>(), readings[KEY_ENVIRONMENT_WIND_SPEEDTRUE][KEY_VALUE].as<double>(), false);
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

const unsigned long  receiveMessages[] = {
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
      //127258L,
      0};

const unsigned long  transmitMessages[] = {
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
      // 127258L, // Declination
      0};
  
// *****************************************************************************
void setup()
{
  // Initialize base subsystems (WiFi, OTA, WebServer, Zenoh, Syslog)
  
  
  syslog.app = NODENAME;
  baseInit(NODENAME, RSYSLOG_IP, PicoSyslog::LogLevel::debug );
  zenoh.setHostname(NODENAME);

  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_BLUE, LOW);

  declination.begin();

  // zenoh key that is published.
  zenoh.declarePublisher(KEY_ENVIRONMENT_DEPTH_BELOWSURFACE);
  zenoh.declarePublisher(KEY_ENVIRONMENT_DEPTH_BELOWTRANSDUCER);
  zenoh.declarePublisher(KEY_ENVIRONMENT_OUTSIDE_PRESSURE);
  zenoh.declarePublisher(KEY_ENVIRONMENT_OUTSIDE_TEMPERATURE);
  zenoh.declarePublisher(KEY_ENVIRONMENT_WATER_TEMPERATURE);
  zenoh.declarePublisher(KEY_ENVIRONMENT_WIND_ANGLEAPPARENT);
  zenoh.declarePublisher(KEY_ENVIRONMENT_WIND_ANGLETRUEGROUND);
  zenoh.declarePublisher(KEY_ENVIRONMENT_WIND_ANGLETRUEWATER);
  zenoh.declarePublisher(KEY_ENVIRONMENT_WIND_SPEEDAPPARENT);
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
  zenoh.declarePublisher(KEY_NAVIGATION_HEADINGMAGNETIC);
  zenoh.declarePublisher(KEY_NAVIGATION_MAGNETICDEVIATION);
  zenoh.declarePublisher(KEY_NAVIGATION_LOG);
  zenoh.declarePublisher(KEY_NAVIGATION_POSITION_ALTITUDE);
  zenoh.declarePublisher(KEY_NAVIGATION_POSITION_LATITUDE);
  zenoh.declarePublisher(KEY_NAVIGATION_POSITION_LONGITUDE);
  zenoh.declarePublisher(KEY_NAVIGATION_SPEEDOVERGROUND);
  zenoh.declarePublisher(KEY_NAVIGATION_SPEEDTHROUGHWATER);
  zenoh.declarePublisher(KEY_NAVIGATION_TRIP_LOG);
  zenoh.declarePublisher(KEY_STEERING_RUDDERANGLE);

  
  nmea2000Node.setReceiveMessages(receiveMessages, 10);
  nmea2000Node.setReceiveMsgHandler(handleNMEA2000Msg);

  
  nmea2000Node.setTransmitMessages(transmitMessages, 2);
  zenoh.subscribe(KEY_ENVIRONMENT_WIND_ANGLEAPPARENT, handleZenohWind);
  zenoh.subscribe(KEY_ENVIRONMENT_WIND_SPEEDAPPARENT, handleZenohWind);
  zenoh.subscribe(KEY_ENVIRONMENT_WIND_ANGLETRUEGROUND, handleZenohWind);
  zenoh.subscribe(KEY_ENVIRONMENT_WIND_SPEEDTRUE, handleZenohWind);
  nmea2000Node.init();
  nmea2000Node.setOnOpen(OnN2kOpen);
  nmea2000Node.open();
  
}

void publishDeclination(){
   if(!readings[KEY_NAVIGATION_POSITION_LATITUDE].isNull() 
      && !readings[KEY_NAVIGATION_POSITION_LONGITUDE].isNull()){

    float lat = readings[KEY_NAVIGATION_POSITION_LATITUDE][KEY_VALUE].as<float>();
    float lon = readings[KEY_NAVIGATION_POSITION_LONGITUDE][KEY_VALUE].as<float>();
    uint8_t day = rtc.getDay();
    uint8_t month = rtc.getMonth();
    uint8_t year = rtc.getYear()-2000; //just need last two digits
    float decl = declination.magneticDeclination(lat, lon, year, month, day); //in degrees
    decl = decl * (PI / 180); //radians
    setDatafromN2k(KEY_NAVIGATION_MAGNETICDEVIATION,decl);
    }
  }

long last = millis();
long declLast = 0;
bool blink = LOW;
// *****************************************************************************
void loop()
{

  // if (n2kScheduler.IsTime())
  // {
  //   n2kScheduler.UpdateNextTime();
   //test();
  if( (millis() - last)>1000){
    
    
    last = millis();
    blink=!blink;
    digitalWrite(LED_BLUE, blink);
    
  }
  //every 5 minutes
  if( (millis() - declLast)>300000){
    publishDeclination();
    declLast=millis();
  }
  

  nmea2000Node.parseMessages();
  nmea2000Node.checkNodeAddress();

  // run base periodic tasks (Zenoh publish, OTA, web updates)
  baseLoopTasks();
}
