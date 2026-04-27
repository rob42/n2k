#ifndef N2K_H
#define N2K_H

#include <NMEA2000Node.h>
#include <zenohBase.h>
#include <signalk.h>
#include <ArduinoJson.h>
#include <WMM_Tinier.h>

// remote syslog server for logs
#ifdef RSYSLOG_IP
#undef RSYSLOG_IP
#endif
//#define RSYSLOG_IP "cbr.local"
#define RSYSLOG_IP "192.168.1.125"
//#define RSYSLOG_IP ""
//zenoh

// Peer mode values (comment/uncomment as needed)
#define ZENOH_MODE "peer"
#define ZENOH_LOCATOR "udp/224.0.0.123:7447#iface=eth0" //in peer mode it MUST have #iface=eth0
//scout doesnt work in peer, flips to tcp and crashes.

// Client mode values (comment/uncomment as needed)
//#define ZENOH_MODE "client"
//#define ZENOH_LOCATOR "tcp/192.168.1.125:7447" 
//#define ZENOH_LOCATOR "" // If empty, it will scout
                   
                               
#endif