/*
  Web Server

 A simple web server that shows the value of the analog input pins.
 using an Arduino Wiznet Ethernet shield.

 Circuit:
 * Ethernet shield attached to pins 10, 11, 12, 13
 * LM35s attached to pins A0 through A4 A0-A3 are external, A4 is on the PCB.
 * DS18S20 on Pin 2

 created 18 Dec 2009
 by David A. Mellis
 modified 9 Apr 2012
 by Tom Igoe
 modified 02 Sept 2015
 by Arturo Guadalupi

 */

#include <SPI.h>
#include <Ethernet.h>
#include <OneWire.h>
#include "max6675.h"

double T_savukaasut, T_l0, T_l1, T_l2, T_l3, T_kotelo, T_cpu; 
float T_ds;

int thermoDO = 6;
int thermoCS = 7;
int thermoCLK = 8;
MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

OneWire  ds(2);  // on pin 10 (a 4.7K resistor is necessary)

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x00
};
IPAddress ip(192, 168, 200, 22);

// Initialize the Ethernet server library
// with the IP address and port you want to use
// (port 80 is default for HTTP):
EthernetServer server(80);

void setup() {
  // wait for MAX chip to stabilize
  delay(500);
    
  // start the Ethernet connection and the server:
  Ethernet.begin(mac, ip);
  server.begin();
  
  Serial.begin(9600);
}


void loop() {
  updateTemps();
  // listen for incoming clients
  EthernetClient client = server.available();
  if (client) {
    Serial.println("new client");
    // an http request ends with a blank line
    boolean currentLineIsBlank = true;
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank) {
          // send a standard http response header
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/html");
          client.println("Connection: close");  // the connection will be closed after completion of the response
          client.println("Refresh: 20");  // refresh the page automatically every 5 sec
          client.println();
          client.println("<!DOCTYPE HTML>");
          client.println("<html>");
          client.println("<head>");
          client.println("<title>Hakelämpökeskus lämpötilat</title>");
          client.println("<meta http-equiv='Content-Type' content='text/html; charset=utf-8' />");
          client.println("</head>");
          client.println("<html>");
          client.println("<body>");
          client.print("<h3>Hakelämpökeskus</h3>");
          client.print("<br />");

          // Data listing for human read
          client.print("<strong>Savukaasut:</strong> ");
          client.print(T_savukaasut);
          client.print("C");
          client.println("<br/>");
          client.print("<strong>Lisäanturi0:</strong> ");
          client.print(T_l0);
          client.print("C");
          client.println("<br/>");
          client.print("<strong>Lisäanturi1:</strong> ");
          client.print(T_l1);
          client.print("C");
          client.println("<br/>");
          client.print("<strong>Lisäanturi2:</strong> ");
          client.print(T_l2);
          client.print("C");
          client.println("<br/>");
          client.print("<strong>Lisäanturi3:</strong> ");
          client.print(T_l3);
          client.print("C");
          client.println("<br/>");
          client.print("<strong>Kotelo:</strong> ");
          client.print(T_kotelo);
          client.print("C");
          client.println("<br/>");
          client.print("<strong>DS18S20:</strong> ");
          client.print(T_ds);
          client.print("C");
          client.println("<br/>");
          client.print("<strong>CPU:</strong> ");
          client.print(T_cpu);
          client.print("C");
          client.println("<br/>");
          client.println("<br/>");
          client.println("<br/>");

          // Data for machine read
          client.print("[PARSE|");
          client.print("TC1:");
          client.print(T_savukaasut);
          client.print("|A0:");
          client.print(T_l0);
          client.print("|A1:");
          client.print(T_l1);
          client.print("|A2:");
          client.print(T_l2);
          client.print("|A3:");
          client.print(T_l3);
          client.print("|AMBIENT:");
          client.print(T_kotelo);
          client.print("|DS:");
          client.print(T_ds);
          client.print("|CPU:");
          client.print(T_cpu);
          client.print("|EOL]");
          
          client.println("</body>");
          client.println("</html>");
          break;
        }
        if (c == '\n') {
          // you're starting a new line
          currentLineIsBlank = true;
        } else if (c != '\r') {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
    }
    // give the web browser time to receive the data
    delay(1);
    // close the connection:
    client.stop();
    Ethernet.maintain();
  }
}

void updateTemps()
{
  T_savukaasut = thermocouple.readCelsius();
  T_l0 = GetAnalogTemp(0);
  T_l1 = GetAnalogTemp(1);
  T_l2 = GetAnalogTemp(2);
  T_l3 = GetAnalogTemp(3);
  T_kotelo = GetAnalogTemp(4);
  GetDS18S20Temp();
  T_cpu = GetTemp();
}

double GetTemp(void)
{
  unsigned int wADC;
  double t;

  // The internal temperature has to be used
  // with the internal reference of 1.1V.
  // Channel 8 can not be selected with
  // the analogRead function yet.

  // Set the internal reference and mux.
  ADMUX = (_BV(REFS1) | _BV(REFS0) | _BV(MUX3));
  ADCSRA |= _BV(ADEN);  // enable the ADC

  delay(20);            // wait for voltages to become stable.

  ADCSRA |= _BV(ADSC);  // Start the ADC

  // Detect end-of-conversion
  while (bit_is_set(ADCSRA,ADSC));

  // Reading register "ADCW" takes care of how to read ADCL and ADCH.
  wADC = ADCW;

  // The offset of 324.31 could be wrong. It is just an indication.
  t = (wADC - 334.31 ) / 1.22;

  // The returned temperature is in degrees Celcius.
  return (t);
}

double GetAnalogTemp(int analogpin)
{
  int sensorValue = 0;
  double outputValue = 0.0;

  // read the analog in value:
  sensorValue = analogRead(analogpin);

  // LM35 measures in range 2-150C and outputs voltage of 0-1V
  // With 5V ADC reference, 0-1V is 0-204.8
  
  
  // map it to the range of the analog out:
  //outputValue = map(sensorValue, 0, 1023, 0, 255);
  outputValue = map(sensorValue, 0, 1023, 0, 550);

  return outputValue;
}

void GetDS18S20Temp(void)
{
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];
  float celsius, fahrenheit;
  
  if ( !ds.search(addr)) {
    Serial.println("No more addresses.");
    Serial.println();
    ds.reset_search();
    delay(250);
    return;
  }
  
  Serial.print("ROM =");
  for( i = 0; i < 8; i++) {
    Serial.write(' ');
    Serial.print(addr[i], HEX);
  }

  if (OneWire::crc8(addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return;
  }
  Serial.println();
 
  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
      Serial.println("  Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
      Serial.println("  Chip = DS18B20");
      type_s = 0;
      break;
    case 0x22:
      Serial.println("  Chip = DS1822");
      type_s = 0;
      break;
    default:
      Serial.println("Device is not a DS18x20 family device.");
      return;
  } 

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end
  
  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.
  
  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad

  Serial.print("  Data = ");
  Serial.print(present, HEX);
  Serial.print(" ");
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.print(" CRC=");
  Serial.print(OneWire::crc8(data, 8), HEX);
  Serial.println();

  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  celsius = (float)raw / 16.0;
  fahrenheit = celsius * 1.8 + 32.0;
  Serial.print("  Temperature = ");
  Serial.print(celsius);
  Serial.print(" Celsius, ");
  Serial.print(fahrenheit);
  Serial.println(" Fahrenheit");

  T_ds = celsius;
}
