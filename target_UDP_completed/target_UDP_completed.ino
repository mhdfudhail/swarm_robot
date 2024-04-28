#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

// wifi udp packet configuration
WiFiUDP udp;

char packetBuffer[255];
unsigned int localPort = 9696;
char *serverip = "192.168.100.16";
unsigned int serverport = 8888;

const char *ssid = "HUAWEI-FTTH";
const char *password = "66554433";

// motor controller
const int En1 = D8;
const int En2 = D7;
const int In1 = D6;
const int In2 = D0;
const int In3 = D5;
const int In4 = D1;

float left=0;
float right=0;

float linearVelocity=0;
float angularVelocity=0;
float rightLeftVelocity = 0;
int right_duty=0;
int left_duty=0;
float rightVel=0;
float leftVel=0;

void setup() {
  Serial.begin(115200);

  pinMode(En1, OUTPUT);
  pinMode(En2, OUTPUT);
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  pinMode(In3, OUTPUT);
  pinMode(In4, OUTPUT);

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
    
    char* ptr = packetBuffer;
    char* comma = strchr(ptr, ',');
    if (comma != nullptr){
      *comma = '\0';
      linearVelocity = atof(ptr);
      *comma = ',';
      ptr = comma + 1;
      rightLeftVelocity = atof(ptr);
    }

    Serial.print("linear vel: ");
    Serial.println(linearVelocity);
    Serial.print("right left: ");
    Serial.println(rightLeftVelocity);
    
    moveToTarget(linearVelocity,rightLeftVelocity);

  }
  delay(100);
  // motorControl(linearVelocity, angularVelocity);
  


}


void moveToTarget(float linearVel, float rightLeftDuty){
  
  if(rightLeftDuty>=4 && linearVel<0){
    Serial.println("--Rotating Right--");
    analogWrite(En1, 105);
    analogWrite(En2, 105);
    digitalWrite(In1, HIGH);
    digitalWrite(In2, LOW);
    digitalWrite(In3, LOW);
    digitalWrite(In4, HIGH);
  }else if(rightLeftDuty<=-4 && linearVel<0){
    Serial.println("--Rotating Left--");
    analogWrite(En1, 105);
    analogWrite(En2, 105);
    digitalWrite(In1, LOW);
    digitalWrite(In2, HIGH);
    digitalWrite(In3, HIGH);
    digitalWrite(In4, LOW);
}else if(rightLeftDuty>-4 && rightLeftDuty<4 && linearVel<0){
  Serial.println("--Moving Forward--");
  int forwardVel = -1*linearVel;
    analogWrite(En1, map(forwardVel,0,255,45,90));
    analogWrite(En2, map(forwardVel,0,255,45,90));
    digitalWrite(In1, HIGH);
    digitalWrite(In2, LOW);
    digitalWrite(In3, HIGH);
    digitalWrite(In4, LOW);
}else{
  Serial.println("--Speed Rotation--");
  // turn Left
  analogWrite(En1, 120);
  analogWrite(En2, 120);
  digitalWrite(In1, LOW);
  digitalWrite(In2, HIGH);
  digitalWrite(In3, HIGH);
  digitalWrite(In4, LOW);
  delay(150);

}
}

