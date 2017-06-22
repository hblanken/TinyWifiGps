#include <TinyGPS++.h>
#include <SoftwareSerial.h> // I tried with "SoftwareSerial256.h" as well
#include <Streaming.h>
#include <PString.h>

#include <Adafruit_CC3000.h>
#include <ccspi.h>
#include <SPI.h>
#include <string.h>
#include "utility/debug.h"

char buffer[40];
PString pstr(buffer, sizeof(buffer));

/*Wifi*/
// These are the interrupt and control pins
#define ADAFRUIT_CC3000_IRQ   2  // MUST be an interrupt pin!
// These can be any two pins
#define ADAFRUIT_CC3000_VBAT  A3
#define ADAFRUIT_CC3000_CS    8
// Use hardware SPI for the remaining pins
// On an UNO, SCK = 13, MISO = 12, and MOSI = 11
Adafruit_CC3000 cc3000 = Adafruit_CC3000(ADAFRUIT_CC3000_CS, ADAFRUIT_CC3000_IRQ, ADAFRUIT_CC3000_VBAT,
                                         SPI_CLOCK_DIVIDER); // you can change this clock speed but DI

#define WLAN_SSID       "x"        // cannot be longer than 32 characters!
#define WLAN_PASS       "x"
// Security can be WLAN_SEC_UNSEC, WLAN_SEC_WEP, WLAN_SEC_WPA or WLAN_SEC_WPA2
#define WLAN_SECURITY   WLAN_SEC_WPA2

Adafruit_CC3000_Client client;

const unsigned long
  dhcpTimeout     = 60L * 1000L, // Max time to wait for address from DHCP
  connectTimeout  = 15L * 1000L, // Max time to wait for server connection
  responseTimeout = 15L * 1000L; // Max time to wait for data from server

// The Arduino pins used by the GPS module
const int GPS_ONOFFPin = A4; // I tried with A3 as well
const int GPS_SYSONPin = A2;
const int GPS_RXPin = A1;
const int GPS_TXPin = A0;
//const int chipSelect = 10;
const int GPSBaud = 9600;

// The GPS connection is attached with a software serial port
SoftwareSerial Gps_serial(GPS_RXPin, GPS_TXPin);

TinyGPSPlus gps; // create gps object

/*-----( Declare Variables )-----*/

void setup()   /****** SETUP: RUNS ONCE ******/
{
  // GPS
  Gps_serial.begin(GPSBaud);
  delay(1000);
  Serial.begin(115200);
  // Init the GPS Module to wake mode
  pinMode(GPS_SYSONPin, INPUT);
  pinMode(GPS_ONOFFPin, OUTPUT);
  digitalWrite(GPS_ONOFFPin, LOW);
  delay(5);
  Serial.print(F("Attempting to wake GPS module.. ")); //http://forum.tinycircuits.com/index.php?topic=328.msg868#msg868
  if( digitalRead( GPS_SYSONPin ) == HIGH )
  {
    // Need to wake the module
    digitalWrite( GPS_ONOFFPin, HIGH );
    delay(5);
    digitalWrite( GPS_ONOFFPin, LOW );
    delay(5);
  }
  Serial.println(F("done."));
  delay(100);

// WiFi
  uint32_t ip = 0L, t;

  Serial.print("Free RAM: "); Serial.println(getFreeRam(), DEC);

  Serial.print(F("Initializing WiFi..."));
  if(!cc3000.begin()) {
    Serial.println(F("failed. Check?"));
    return;
  }

  Serial.print(F("OK.\r\nConnecting..."));
  if(!cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY)) {
    Serial.println(F("Failed!"));
    return;
  }
  Serial.println(F("connected!"));

  Serial.print(F("Requesting address from server..."));
  for(t=millis(); !cc3000.checkDHCP() && ((millis() - t) < dhcpTimeout); delay(1000));
  if(cc3000.checkDHCP()) {
    Serial.println(F("OK"));
  } else {
    Serial.println(F("failed"));
    return;
  }
  /* Display the IP address DNS, Gateway, etc. */  
  while (! displayConnectionDetails()) {
    delay(1000);
  }
  pstr.begin();     // Empty the buffer
  /* You need to make sure to clean up after yourself or the CC3000 can freak out */
  /* the next time you try to connect ... */
  //Serial.println(F("\n\nClosing the connection"));
  //cc3000.disconnect();
}
void loop()
{
  // This sketch displays information every time a new sentence is correctly encoded.
  while (Gps_serial.available() > 0)
    if (gps.encode(Gps_serial.read()))
      displayInfo();

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while(true);
  }
}

void displayInfo()
{
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID Location"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID Date"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
    Serial.println(F("Sending data"));
    }
  else
  {
    Serial.print(F("INVALID DATA"));
  }
  Serial.println();
  sendData();
//  flush();
}
void sendData() {
  float latitude = gps.location.lat();
  float longitude = gps.location.lng();
  uint32_t ip_Address = cc3000.IP2U32(192, 168, 0, 4); // Your local computer, for testing - to read UDP go to terminal on your machine and start netcat: nc -l -u 5001
  Adafruit_CC3000_Client client = cc3000.connectUDP(ip_Address, 5001);  

// Send data 
  Serial.println(F("Sending data"));
  if (client.connected()) 
    {
    pstr.begin();     // Empty the buffer
    pstr << millis();
    pstr += " lat: ";
    pstr << _FLOAT(latitude,6) << endl;
    pstr += " lng: ";
    pstr << _FLOAT(longitude,6) << endl;
    Serial << pstr;
    client.fastrprintln( pstr );
    delay(1000);
    Serial.println();
    } 
  else {
    Serial.print(F("Error, could not send data"));
  }
}
//void flush() {
//  while(Serial.available())
//  Serial.read();
//}
// Wifi
bool displayConnectionDetails(void) {
  uint32_t ipAddress, netmask, gateway, dhcpserv, dnsserv;
  if(!cc3000.getIPAddress(&ipAddress, &
  netmask, &gateway, &dhcpserv, &dnsserv)) {
    Serial.println(F("Unable to retrieve the IP Address!\r\n"));
    return false;
  } else {
    Serial.print(F("\nIP Addr: ")); cc3000.printIPdotsRev(ipAddress);
//    Serial.print(F("\nNetmask: ")); cc3000.printIPdotsRev(netmask);
//    Serial.print(F("\nGateway: ")); cc3000.printIPdotsRev(gateway);
//    Serial.print(F("\nDHCPsrv: ")); cc3000.printIPdotsRev(dhcpserv);
//    Serial.print(F("\nDNSserv: ")); cc3000.printIPdotsRev(dnsserv);
    Serial.println();
    return true;
  }
}

