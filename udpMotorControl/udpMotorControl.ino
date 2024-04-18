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
    float linearVelocity, angularVelocity;
    char* ptr = packetBuffer;
    char* comma = strchr(ptr, ',');
    if (comma != nullptr){
      *comma = '\0';
      linearVelocity = atof(ptr);
      *comma = ',';
      ptr = comma + 1;
      angularVelocity = atof(ptr);
    }

    // Serial.print("First value: ");
    // Serial.println(linearVelocity);
    // Serial.print("Second value: ");
    // Serial.println(angularVelocity);
    
    float rightVel = rightWheelVelocity(linearVelocity, angularVelocity);
    float leftVel = leftWheelVelocity(linearVelocity, angularVelocity);
    motorControl(rightVel, leftVel);
  }
  delay(250);


}

float rightWheelVelocity(float linearVelocity, float angularVelocity){
  float wheelRadius = 0.035;
  float rightVelocity = linearVelocity + (angularVelocity*wheelRadius);
  return rightVelocity;
}
float leftWheelVelocity(float linearVelocity, float angularVelocity){
  float wheelRadius = 0.035;
  float leftVelocity = linearVelocity - (angularVelocity*wheelRadius);
  return leftVelocity;
}

void motorControl(float rightVelocity, float leftVelocity){
  int right_duty = (255*abs(rightVelocity)/500);
  int left_duty = (255*abs(leftVelocity)/500);
  Serial.println("right velocity: "+String(rightVelocity)+","+ "left velocity: "+ String(leftVelocity));
  Serial.println("right duty: "+String(right_duty)+","+ "left duty: "+ String(left_duty));

  // writing pwm values
  if (right_duty>left_duty){
    analogWrite(En1, 80+right_duty);
    analogWrite(En2, 50+left_duty);
  }else if (right_duty<left_duty){
    analogWrite(En1, 50+right_duty);
    analogWrite(En2, 80+left_duty);
  }else if(right_duty==left_duty){
    analogWrite(En1, 5+right_duty);
    analogWrite(En2, 5+left_duty);
  }
  


  if(rightVelocity>=0){
    digitalWrite(In1, HIGH);
    digitalWrite(In2, LOW);
    
  }else{
    digitalWrite(In1, LOW);
    digitalWrite(In2, HIGH);
  }

  if(leftVelocity>=0){
    digitalWrite(In3, HIGH);
    digitalWrite(In4, LOW);
  }else{
    digitalWrite(In1, LOW);
    digitalWrite(In2, HIGH);
  }
}

