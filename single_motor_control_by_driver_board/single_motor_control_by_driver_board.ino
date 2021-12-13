int A_PWM = 18 ;
int A_1 = 14;
int A_2 = 15; 


void setup() {

  pinMode(A_PWM, OUTPUT);
  pinMode(A_1, OUTPUT);
  pinMode(A_2, OUTPUT);
}

void loop() {
  digitalWrite(A_1,LOW);
  digitalWrite(A_2,HIGH);
  analogWrite(A_PWM,20);
}
