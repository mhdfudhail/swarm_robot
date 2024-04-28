#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

WiFiUDP udp;

char packetBuffer[255];
unsigned int localPort = 9696;
char *serverip = "192.168.100.16";
unsigned int serverport = 8888;

const char *ssid = "HUAWEI-FTTH";
const char *password = "66554433";
// int val_3=0;
void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(F("."));
  }

  udp.begin(localPort);
  Serial.printf("UDP Client : %s:%i \n", WiFi.localIP().toString().c_str(), localPort);
}

void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    Serial.print(" Received packet from : "); Serial.println(udp.remoteIP());
    int len = udp.read(packetBuffer, 255);
    // Serial.printf("Data : %s\n", packetBuffer);
    float val_1,val_2, val_3,val_4;
    int status=0;
    

    char* ptr = packetBuffer;
    char* comma = strchr(ptr, ',');
    if (comma != nullptr){
      *comma = '\0';
      val_1 = atof(ptr);
      *comma = ',';
      ptr = comma + 1;
      comma = strchr(ptr, ',');
      if(comma!=nullptr){
        *comma = '\0';
        val_2 = atof(ptr);
        *comma = ',';
        ptr = comma + 1;
        comma = strchr(ptr, ',');
        if(comma!=nullptr){
          *comma = '\0';
          val_3 = atof(ptr);
          *comma = ',';
          ptr = comma + 1;
        }
        val_4 = atof(ptr);
      }
    }

    Serial.print("First value: ");
    Serial.println(val_1);
    Serial.print("Second value: ");
    Serial.println(val_2);
    Serial.print("Third value: ");
    Serial.println(val_3);
    Serial.print("Fourth value: ");
    Serial.println(val_4);
  }
  delay(250);
  Serial.print("[Client Connected] "); Serial.println(WiFi.localIP());
  udp.beginPacket(serverip, serverport);
  char buf[30];
  unsigned long testID = millis();
  sprintf(buf, "ESP8266 send millis: %lu", testID);
  udp.printf(buf);
  udp.endPacket();

}
