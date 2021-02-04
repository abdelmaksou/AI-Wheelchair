//define output pins
#define motor1_f 2 //first motor forward
#define motor1_b 3 //first motor backward
#define motor2_f 4 //second motor forward
#define motor2_b 5 //second motor backward
#define buzzer 6   //buzzer for on and off

char x; //serial reciever variable

bool s;

int on = 1;
int off = 1;
/*
int right = 1;
int left = 1;
int forward = 1;
int backward = 1;
*/

void setup() {
  // put your setup code here, to run once:
  pinMode(motor1_f, OUTPUT);
  pinMode(motor1_b, OUTPUT);
  pinMode(motor2_f, OUTPUT);
  pinMode(motor2_b, OUTPUT);
  pinMode(buzzer, OUTPUT);
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()>0){
    x = Serial.read();
  }
  if(x == 'q') //on
  {
    s = true;
    if (on == 1)
    {
      digitalWrite(buzzer,HIGH);
      delay(600);
      digitalWrite(buzzer,LOW);
      on = on + 1;
      off = 1;
    }
    Serial.print(x);
  }
  else if (x=='w'){ //off
    s = false;
    if (off == 1)
    {
      digitalWrite(buzzer,HIGH);
      delay(600);
      digitalWrite(buzzer,LOW);
      off = off + 1;
      on = 1;
    }
    Serial.print(x);
  }

  if (s == true)
  {
    if (x=='f'){ //forward
      digitalWrite(motor1_f,HIGH);
      digitalWrite(motor1_b,LOW);
      digitalWrite(motor2_f,HIGH);
      digitalWrite(motor2_b,LOW);
      /* 
      // Test
      if (forward == 1)
      {
        digitalWrite(buzzer,HIGH);
        delay(300);
        digitalWrite(buzzer,LOW);
        forward = forward + 1;
        backward = 1;
        right = 1;
        left = 1;
      }
      */
      Serial.print(x);
    }
    else if (x=='b'){ //backward
      digitalWrite(motor1_f,LOW);
      digitalWrite(motor1_b,HIGH);
      digitalWrite(motor2_f,LOW);
      digitalWrite(motor2_b,HIGH);
      /* 
      // Test
      if (backward == 1)
      {
        digitalWrite(buzzer,HIGH);
        delay(300);
        digitalWrite(buzzer,LOW);
        backward = backward + 1;
        forward = 1;
        right = 1;
        left = 1;
      }
      */
      Serial.print(x);
    }
    else if (x=='l'){ //left
      digitalWrite(motor1_f,HIGH);
      digitalWrite(motor1_b,LOW);
      digitalWrite(motor2_f,LOW);
      digitalWrite(motor2_b,HIGH);
      /* 
      // Test
      if (left == 1)
      {
        digitalWrite(buzzer,HIGH);
        delay(300);
        digitalWrite(buzzer,LOW);
        left = left + 1;
        forward = 1;
        right = 1;
        backward = 1;
      }
      */
      Serial.print(x);
    }
    else if (x=='r'){ //right
      digitalWrite(motor1_f,LOW);
      digitalWrite(motor1_b,HIGH);
      digitalWrite(motor2_f,HIGH);
      digitalWrite(motor2_b,LOW);
      /* 
      // Test
      if (right == 1)
      {
        digitalWrite(buzzer,HIGH);
        delay(300);
        digitalWrite(buzzer,LOW);
        right = right + 1;
        forward = 1;
        left = 1;
        backward = 1;
      }
      */
      Serial.print(x);
    }
    else if (x=='n'){ //nothing
      digitalWrite(motor1_f,LOW);
      digitalWrite(motor1_b,LOW);
      digitalWrite(motor2_f,LOW);
      digitalWrite(motor2_b,LOW);
      digitalWrite(buzzer,LOW);
      Serial.print(x);
    }
  }
  
}
