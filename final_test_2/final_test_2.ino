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

// ultrasonic sensor
const int trig = D2;
const int echo = D3;

// robot-ID:90
// const int En1 = D7;
// const int En2 = D8;
// const int In1 = D1;
// const int In2 = D0;
// const int In3 = D5;
// const int In4 = D6;

// robot-ID:70
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
int status = 0;
float cache;
float rightLeftVelocity = 0;
int right_duty=0;
int left_duty=0;
float rightVel=0;
float leftVel=0;
bool flag = true;

void setup() {
  Serial.begin(115200);
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
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
    // float linearVelocity, angularVelocity
    int status=0;

    char* ptr = packetBuffer;
    char* comma = strchr(ptr, ',');
    if (comma != nullptr){
      *comma = '\0';
      linearVelocity = atof(ptr);
      *comma = ',';
      ptr = comma + 1;
      comma = strchr(ptr, ',');
      if(comma!=nullptr){
        *comma = '\0';
        angularVelocity = atof(ptr);
        *comma = ',';
        ptr = comma + 1;
        comma = strchr(ptr, ',');
        if(comma!=nullptr){
          *comma = '\0';
          status = atof(ptr);
          *comma = ',';
          ptr = comma + 1;
        }
        cache = atof(ptr);
      }
      
    }

    Serial.print("Linear: ");
    Serial.print(linearVelocity);Serial.print(" ");
    Serial.print("Angular: ");
    Serial.print(angularVelocity);Serial.print(" ");
    Serial.print("Status: ");
    Serial.println(status);

    if(linearVelocity>=0.00 && angularVelocity>=0.00 && status==0 ){
      flag = true;
      Serial.println("no target-roaming around!");
      // roamAround();
    }else if(linearVelocity>=0.00 && angularVelocity>=0.00 && status==1){
    Serial.println("--fast Right--");
    analogWrite(En1, 110);
    analogWrite(En2, 110);
    digitalWrite(In1, LOW);
    digitalWrite(In2, HIGH);
    digitalWrite(In3, HIGH);
    digitalWrite(In4, LOW);
    // delay(100);
    flag = false;
    }else if(linearVelocity>=0.00 && angularVelocity>=0.00 && status==-1){
    Serial.println("--fast Left--");
    analogWrite(En1, 110);
    analogWrite(En2, 110);
    digitalWrite(In1, HIGH);
    digitalWrite(In2, LOW);
    digitalWrite(In3, LOW);
    digitalWrite(In4, HIGH);
    // delay(100);
    flag = false;
    }else if(status>=10){
      Serial.println("target locked!");
    }else{
      Serial.println("--normal--");
      moveToTarget(linearVelocity,angularVelocity);
      flag = false;
    }
    
    

  }
    
  if(flag){
    roamAround();
    flag=true;
  }
  delay(100);
}






void moveToTarget(float linearVel, float rightLeftDuty){
  
  if(rightLeftDuty>=4 && linearVel<0 && status==0){
    Serial.println("--Rotating Right--");
    analogWrite(En1, 105);
    analogWrite(En2, 105);
    digitalWrite(In1, LOW);
    digitalWrite(In2, HIGH);
    digitalWrite(In3, HIGH);
    digitalWrite(In4, LOW);
  }else if(rightLeftDuty<=-4 && linearVel<0 && status==0){
    Serial.println("--Rotating Left--");
    analogWrite(En1, 105);
    analogWrite(En2, 105);
    digitalWrite(In1, HIGH);
    digitalWrite(In2, LOW);
    digitalWrite(In3, LOW);
    digitalWrite(In4, HIGH);
}else if(rightLeftDuty>-4 && rightLeftDuty<4 && linearVel<0){
  Serial.print("Linear: ");
  Serial.print(linearVel);Serial.print(" ");
  Serial.print("duty: ");
  Serial.print(rightLeftDuty);Serial.print(" ");
  Serial.print("Status: ");
  Serial.println(status);

  Serial.println("--Moving Forward--");
  int distance = obstacleAvoidance();
  // int distance = 30;
  if(distance<20){
    Serial.println("obstacle ahead!");
    Backward();
    delay(100);
    TurnRight();
    delay(500);
  }else{
    int forwardVel = -1*linearVel;
    analogWrite(En1, map(forwardVel,0,255,45,90));
    analogWrite(En2, map(forwardVel,0,255,45,90));
    digitalWrite(In1, HIGH);
    digitalWrite(In2, LOW);
    digitalWrite(In3, HIGH);
    digitalWrite(In4, LOW);

  }

}
// else{
//   Serial.println("--Speed Rotation--");
//   // turn Left
//   analogWrite(En1, 80);//120
//   analogWrite(En2, 80);//120
//   digitalWrite(In1, LOW);
//   digitalWrite(In2, HIGH);
//   digitalWrite(In3, HIGH);
//   digitalWrite(In4, LOW);
//   delay(150);

// }
}


int obstacleAvoidance(){
  long duration, distance;
  digitalWrite(trig, LOW);  
  delayMicroseconds(2); 
  
  digitalWrite(trig, HIGH);
  delayMicroseconds(10); 
  
  digitalWrite(trig, LOW);
  duration = pulseIn(echo, HIGH);
  distance = (duration/2) / 29.1;

  Serial.print("Centimeter:");
  Serial.println(distance);

  return distance;
}

void roamAround(){
  int dist=obstacleAvoidance();
    delay(100);
  if (dist<30){
    Serial.println("obstacle ahead!");
    Backward();
    delay(500);
    TurnRight();
    delay(500);
  }else{
    Serial.println("normal");
    Forward();
  } 
}

 


void Backward() 
{ analogWrite(En1, 90);
  analogWrite(En2, 90);
  digitalWrite(In1,LOW);
  digitalWrite(In2,HIGH);                       
  digitalWrite(In3,LOW);
  digitalWrite(In4,HIGH); 
}

void Forward() 
{
  analogWrite(En1, 100);
  analogWrite(En2, 100);
  digitalWrite(In1,HIGH);
  digitalWrite(In2,LOW);                         
  digitalWrite(In3,HIGH);
  digitalWrite(In4,LOW);
} 
void TurnRight() 
{ analogWrite(En1, 120);
  analogWrite(En2, 120);
  digitalWrite(In1, LOW);
  digitalWrite(In2, HIGH);
  digitalWrite(In3, HIGH);
  digitalWrite(In4, LOW);
}