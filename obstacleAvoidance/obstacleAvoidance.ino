const int trig = D2;
const int echo = D3;

const int En1 = D8;
const int En2 = D7;
const int In1 = D6;
const int In2 = D0;
const int In3 = D5;
const int In4 = D1;


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
  delay(500);
  if (distance<30){
    Serial.println("obstacle ahead!");
    Forward();
    delay(500);
    TurnRight();
    delay(500);
  }else{
    Serial.println("normal");
    Backward();
  }

}

void Forward() 
{ analogWrite(En1, 100);
  analogWrite(En2, 100);
  digitalWrite(In1,LOW);
  digitalWrite(In2,HIGH);                       
  digitalWrite(In3,LOW);
  digitalWrite(In4,HIGH); 
}

void Backward() 
{
  analogWrite(En1, 80);
  analogWrite(En2, 80);
  digitalWrite(In1,HIGH);
  digitalWrite(In2,LOW);                         
  digitalWrite(In3,HIGH);
  digitalWrite(In4,LOW);
} 
void TurnRight() 
{ analogWrite(En1, 130);
  analogWrite(En2, 130);
  digitalWrite(In1, LOW);
  digitalWrite(In2, HIGH);
  digitalWrite(In3, HIGH);
  digitalWrite(In4, LOW);
}
void TurnLeft() 
{analogWrite(En1, 100);
  analogWrite(En2, 100);
 digitalWrite(In1, HIGH);
 digitalWrite(In2, LOW);
 digitalWrite(In3, LOW);
 digitalWrite(In4, HIGH); 
}  
void Stop()
{analogWrite(En1, 0);
  analogWrite(En2, 0);
 digitalWrite(In1, LOW);
 digitalWrite(In2, LOW);
 digitalWrite(In3, LOW);
 digitalWrite(In4, LOW); 
}