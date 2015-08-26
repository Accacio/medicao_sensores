#include<Servo.h>
Servo servooldg;

int servo_min=1000;
int servo_max=2000;
int angle=0;
int flag;

void setup()
{

  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);
}
void loop() {

  servooldg.attach(2,servo_min,servo_max); //2065 , 974
  // put your main code here, to run repeatedly:

    servooldg.write(angle);


}

void serialEvent()
{
  while(Serial.available())
  {

        servo_min = Serial.parseInt();
        servo_max = Serial.parseInt();
        angle = Serial.parseInt();
        Serial.println(angle);
        if (Serial.read()=='\n')
        {
          flag=1;
        }
   }
}
