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

}

void loop() {
  // put your main code here, to run repeatedly:
  for(int i=0; i<10; i++){
    right= rightWheelVelocity(i, 0);
    left = leftWheelVelocity(i, 0);
    Serial.println("right velocity: "+String(right)+","+ "left velocity: "+ String(left));
    motorControl(right, left);
    delay(1000);
  }
  for(int i=10; i>0;i--){
    right= rightWheelVelocity(i, 1);
    left = leftWheelVelocity(i, 0);
    Serial.println("right velocity: "+String(right)+","+ "left velocity: "+ String(left));
    motorControl(right, left);
    delay(1000);
  }

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
  Serial.println("right duty: "+String(right_duty)+","+ "left duty: "+ String(left_duty));

  // writing pwm values
  analogWrite(En1, right_duty);
  analogWrite(En2, left_duty);


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
