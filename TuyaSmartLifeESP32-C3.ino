#include <Arduino.h>
#include <WiFi.h>
#include "ESPTelnet.h"
#include <SoftwareSerial.h>

#define SERIAL_SPEED    115200
#define WIFI_SSID       "Zulu"
#define WIFI_PASSWORD   ""

// Telnet server
ESPTelnet telnet1;
IPAddress ip;
uint16_t port1 = 23;
String buffer1 = "";
String SerialBufferIn = "";
String SerialBufferReady = "";
String bufferReady = "";
String MCUstatus = "AA0201AD";
String MCUinit   = "AA030303B3";
String ByteID[] = {"Header1","Header1","Header2","Header2","StatusOrCommand","StatusOrCommand","Power","Power","Unknown","Unknown","SwingV","SwingV","SwingH","SwingH","Mode","Mode","Unknown","Unknown","FanSpeed","FanSpeed","SetTemp","SetTemp","CurrentTemp","CurrentTemp","Unknown","Unknown","Unknown","Unknown","Unknown","Unknown","Unknown","Unknown","Unknown","Unknown","Unknown","Unknown","Unknown","Unknown","Unknown","Unknown","Unknown","Unknown","Checksum","Checksum"};
unsigned long previousMillis = 0;
const unsigned long interval = 4000;

// SoftwareSerial instance
SoftwareSerial serial2(20, 21); // RX, TX

// Function prototypes
void setupSerial(long speed, String msg = "");
bool isConnected();
bool connectToWiFi(const char* ssid, const char* password, int max_tries = 20, int pause = 500);
void errorMsg(String error, bool restart = true);
void setupTelnet(ESPTelnet& telnet, uint16_t port);
void sendBufferedHexToSerial(String &buffer, SoftwareSerial &serial);
void ProcessSerial(String &buffer);

void setup() {
  setupSerial(SERIAL_SPEED, "Telnet Test");

  // Connect to WiFi
  connectToWiFi(WIFI_SSID, WIFI_PASSWORD);
  if (isConnected()) {
    ip = WiFi.localIP();
    setupTelnet(telnet1, port1);
  } else {
    errorMsg("Error connecting to WiFi");
  }

  // Initialize SoftwareSerial
  serial2.begin(9600);
  previousMillis = millis();
}

void loop() {
  telnet1.loop();
  if (serial2.available()) {
    int byteReceived = serial2.read();
      if (byteReceived == 170) {
        //telnet1.print("\r\nAA");
        SerialBufferReady = SerialBufferIn;
        //telnet1.print(SerialBufferReady);//Nope. We need to process it.
        ProcessSerial(SerialBufferReady);
        SerialBufferIn = "AA";
    } else {
        char hexString[4]; // buffer to hold the hex string
        snprintf(hexString, sizeof(hexString), "%02X", byteReceived); // convert byte to hex
        SerialBufferIn += hexString;
    }
  }

//  // Read from telnet1 and buffer the input bytes until end-of-line
//  if (telnet1.getClient().available()) {
//    char c = telnet1.getClient().read();
//    if (c == '\n' || c == '\r') {
//      // Convert buffered hex string to bytes and send to serial2
//      bufferReady = buffer1;
//      sendBufferedHexToSerial(bufferReady, serial2);
//      buffer1 = "";  // Clear the buffer
//    } else if (!isspace(c)) {
//      buffer1 += c;  // Add character to buffer if not a space
//    }
//  }

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    if (bufferReady != "") {
      sendBufferedHexToSerial(bufferReady, serial2);
      bufferReady = "";
    } else {
      sendBufferedHexToSerial(MCUstatus, serial2);
    }
    previousMillis = millis();
  }
}

void setupSerial(long speed, String msg) {
  Serial.begin(speed);
  while (!Serial) {
    ; // Wait for serial port to connect
  }
  delay(200);
}

bool isConnected() {
  return (WiFi.status() == WL_CONNECTED);
}

bool connectToWiFi(const char* ssid, const char* password, int max_tries, int pause) {
  int i = 0;
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  do {
    delay(pause);
    //serial.print(".");
  } while (!isConnected() && i++ < max_tries);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
  return isConnected();
}

void errorMsg(String error, bool restart) {
  //serial.println(error);
  if (restart) {
    //serial.println("Rebooting now...");
    delay(2000);
    ESP.restart();
    delay(2000);
  }
}

void setupTelnet(ESPTelnet& telnet, uint16_t port) {
  // Passing on functions for various telnet events
  telnet.onConnect(onTelnetConnect);
  telnet.onConnectionAttempt(onTelnetConnectionAttempt);
  telnet.onReconnect(onTelnetReconnect);
  telnet.onDisconnect(onTelnetDisconnect);
  telnet.onInputReceived(onTelnetInput);
  Serial.print("- Telnet: ");
  if (telnet.begin(port)) {
    Serial.print(ip);
    Serial.println(" available");
  } else {
    Serial.println("error.");
    errorMsg("Will reboot...");
  }
}

void onTelnetConnect(String ip) {
  //serial.print("- Telnet: ");
  //serial.print(ip);
  //serial.println(" connected");
}

void onTelnetDisconnect(String ip) {
  //serial.print("- Telnet: ");
  //serial.print(ip);
  //serial.println(" disconnected");
}

void onTelnetReconnect(String ip) {
  //serial.print("- Telnet: ");
  //serial.print(ip);
  //serial.println(" reconnected");
}

void onTelnetConnectionAttempt(String ip) {
  //serial.print("- Telnet: ");
  //serial.print(ip);
  //serial.println(" tried to connect");
}

void onTelnetInput(String str) {
  // Handle the input received from telnet
  str.trim();
  for (char c : str) {
    if (c == '\n' || c == '\r') {
      bufferReady = buffer1;
      //sendBufferedHexToSerial(buffer1, serial2);
      buffer1 = "";  // Clear the buffer
    } else if (!isspace(c)) {
      buffer1 += c;  // Add character to buffer if not a space
    }
  }
}

void sendBufferedHexToSerial(String &buffer, SoftwareSerial &serial) {
  int len = buffer.length();
  for (int i = 0; i < len; i += 2) {
    String hexPair = buffer.substring(i, i + 2);
    uint8_t byte = (uint8_t) strtol(hexPair.c_str(), NULL, 16);
    serial.write(byte);
  }
}

void ProcessSerial(String &buffer) {
    telnet1.print("\r\nPROCESSING:");
    telnet1.print(buffer);
    telnet1.print(":");    

    if (buffer == MCUinit) {
        telnet1.print("\r\n   SENDING:MCUinit");
        sendBufferedHexToSerial(MCUinit, serial2);
        buffer = "";
    } else {
        int len = buffer.length();
        for (int i = 0; i < len; i += 2) {
            String hexPair = buffer.substring(i, i + 2);
            uint8_t byte = (uint8_t) strtol(hexPair.c_str(), NULL, 16);
            telnet1.print("\r\n");
            telnet1.print(ByteID[i]);
            telnet1.print(" = ");
            if (ByteID[i] == "Mode") {
                switch (byte) { //Mode 0=Auto,1=AC,2=Dehumidification,3=Heat,4=FanOnly
                    case 0: telnet1.print("Auto"); break;
                    case 1: telnet1.print("AC"); break;
                    case 2: telnet1.print("Dehumidification"); break;
                    case 3: telnet1.print("Heat"); break;
                    case 4: telnet1.print("Fan Only"); break;
                    default: telnet1.print(byte); break;
                }
            } else if (ByteID[i] == "FanSpeed") {
                switch (byte) {
                    case 0: telnet1.print("Off"); break;
                    case 1: telnet1.print("Low"); break;
                    case 2: telnet1.print("Med"); break;
                    case 3: telnet1.print("High"); break;
                    case 4: telnet1.print("Auto"); break;
                    default: telnet1.print(byte); break;
                }
            } else if (ByteID[i] == "StatusOrCommand") {
                switch (byte) {
                    case 1: telnet1.print("Status"); break;
                    case 2: telnet1.print("Command"); break;
                    default: telnet1.print(byte); break;
                }
            } else if (ByteID[i] == "Power") {
                switch (byte) {
                    case 0: telnet1.print("Off"); break;
                    case 1: telnet1.print("On"); break;
                    default: telnet1.print(byte); break;
                }
            } else if (ByteID[i] == "Checksum") {
                if (len <= 2) {
                    // Handle error: buffer too short to contain a checksum
                    telnet1.print("No Checksum");
                    return;
                }

                // Truncate buffer to exclude the last 2 bytes
                String TruncBuffer = buffer.substring(0, len - 2);
                
                // Calculate checksum of truncated buffer
                int checksum = 0;
                for (int j = 0; j < TruncBuffer.length(); j += 2) {
                    String hexPair = TruncBuffer.substring(j, j + 2);
                    checksum += (int) strtol(hexPair.c_str(), NULL, 16);
                    checksum = checksum % 256;
                }

                // Extract the last 2 bytes from the original buffer
                String checksumStr = buffer.substring(len - 2);
                
                // Convert the last 2 bytes from hex to an integer
                int checksumFromBuffer = (int) strtol(checksumStr.c_str(), NULL, 16);
                
                // Compare calculated checksum with checksum from buffer
                telnet1.print(checksumFromBuffer);
                telnet1.print(":");
                telnet1.print(checksum);
                telnet1.print("   ");
                if (checksum == checksumFromBuffer) {
                    telnet1.print("Checksum is valid.");
                } else {
                    telnet1.print("Checksum is invalid.");
                }
            } else {
                telnet1.print(byte);
            }
        }
    }
}

