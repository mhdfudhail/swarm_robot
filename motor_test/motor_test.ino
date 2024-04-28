const int trig = D2;
const int echo = D3;

// robot-ID:90
const int En1 = D7;
const int En2 = D8;
const int In1 = D1;
const int In2 = D0;
const int In3 = D5;
const int In4 = D6;

// robot-ID:70
// const int En1 = D8;
// const int En2 = D7;
// const int In1 = D6;
// const int In2 = D0;
// const int In3 = D5;
// const int In4 = D1;


void setup() {
	
  Serial.begin(9600);
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  pinMode(En1, OUTPUT);
  pinMode(En2, OUTPUT);
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  pinMode(In3, OUTPUT);
  pinMode(In4, OUTPUT);
}

void loop() {
  delay(1000);
  Forward();
  delay(500);
  Backward();
  delay(500);
  TurnLeft(); //right
  delay(500);
  TurnRight();//left
  delay(500);
  TurnRight();//left
  delay(500);
  TurnLeft();//right


	
  // long duration, distance;
  // digitalWrite(trig, LOW);  
  // delayMicroseconds(2); 
  
  // digitalWrite(trig, HIGH);
  // delayMicroseconds(10); 
  
  // digitalWrite(trig, LOW);
  // duration = pulseIn(echo, HIGH);
  // distance = (duration/2) / 29.1;
  
  
  // Serial.print("Centimeter:");
  // Serial.println(distance);
  // delay(500);
  // if (distance<30){
  //   Serial.println("obstacle ahead!");
  //   Backward();
  //   delay(500);
  //   TurnRight();
  //   delay(500);
  // }else{
  //   Serial.println("normal");
  //   Forward();
  // }

}

void Forward() 
{ analogWrite(En1, 250);
  analogWrite(En2, 250);
  digitalWrite(In1, HIGH);
  digitalWrite(In2, LOW);
  digitalWrite(In3, HIGH);
  digitalWrite(In4, LOW);
}

void Backward() 
{
  analogWrite(En1, 250);
  analogWrite(En2, 250);
  digitalWrite(In1,LOW);
  digitalWrite(In2,HIGH);                         
  digitalWrite(In3,LOW);
  digitalWrite(In4,HIGH);
} 
void TurnRight() 
{ analogWrite(En1, 230);
  analogWrite(En2, 230);
  digitalWrite(In1, HIGH);
  digitalWrite(In2, LOW);
  digitalWrite(In3, LOW);
  digitalWrite(In4, HIGH);
}
void TurnLeft() 
{ analogWrite(En1, 230);
  analogWrite(En2, 230);
  digitalWrite(In1, LOW);
  digitalWrite(In2, HIGH);
  digitalWrite(In3, HIGH);
  digitalWrite(In4, LOW);
}  
void Stop()
{analogWrite(En1, 0);
  analogWrite(En2, 0);
 digitalWrite(In1, LOW);
 digitalWrite(In2, LOW);
 digitalWrite(In3, LOW);
 digitalWrite(In4, LOW); 
}