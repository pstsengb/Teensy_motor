int B_PWM = 9;
int B_1 = 10;
int B_2 = 11;
int A_PWM = 18 ;
int A_1 = 14;
int A_2 = 15; 


void setup() {
  pinMode(B_PWM, OUTPUT);
  pinMode(B_1, OUTPUT);
  pinMode(B_2, OUTPUT);
  pinMode(A_PWM, OUTPUT);
  pinMode(A_1, OUTPUT);
  pinMode(A_2, OUTPUT);
}

void loop() {
  digitalWrite(B_1,LOW);
  digitalWrite(B_2,HIGH);
  analogWrite(B_PWM,30);
  digitalWrite(A_1,LOW);
  digitalWrite(A_2,HIGH);
  analogWrite(A_PWM,20);
}
