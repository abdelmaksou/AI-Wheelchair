//define output pins
#define motor1_f 2 //first motor forward
#define motor1_b 3 //first motor backward
#define motor2_f 4 //second motor forward
#define motor2_b 5 //second motor backward
#define buzzer 6   //buzzer for on and off

char x; //serial reciever variable

void setup() {
  // put your setup code here, to run once:
  pinMode(motor1_f, OUTPUT);
  pinMode(motor1_b, OUTPUT);
  pinMode(motor2_f, OUTPUT);
  pinMode(motor2_b, OUTPUT);
  pinMode(buzzer, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()>0){
    x = Serial.read();
  }
  if(x == 'q') //on
  {
    digitalWrite(buzzer,HIGH);
    if (x=='f'){ //forward
      digitalWrite(motor1_f,HIGH);
      digitalWrite(motor1_b,LOW);
      digitalWrite(motor2_f,HIGH);
      digitalWrite(motor2_b,LOW);
    }
    else if (x=='b'){ //backward
      digitalWrite(motor1_f,LOW);
      digitalWrite(motor1_b,HIGH);
      digitalWrite(motor2_f,LOW);
      digitalWrite(motor2_b,HIGH);
    }
    else if (x=='l'){ //left
      digitalWrite(motor1_f,HIGH);
      digitalWrite(motor1_b,LOW);
      digitalWrite(motor2_f,LOW);
      digitalWrite(motor2_b,HIGH);
    }
    else if (x=='r'){ //right
      digitalWrite(motor1_f,LOW);
      digitalWrite(motor1_b,HIGH);
      digitalWrite(motor2_f,HIGH);
      digitalWrite(motor2_b,LOW);
    }
    else if (x=='n'){ //nothing
      digitalWrite(motor1_f,LOW);
      digitalWrite(motor1_b,LOW);
      digitalWrite(motor2_f,LOW);
      digitalWrite(motor2_b,LOW);
      digitalWrite(buzzer,LOW);
    }
  }
  else if (x=='w'){ //off
     digitalWrite(buzzer,HIGH);
  }
}
