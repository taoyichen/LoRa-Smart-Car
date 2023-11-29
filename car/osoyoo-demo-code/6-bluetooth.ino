/*  ___   ___  ___  _   _  ___   ___   ____ ___  ____  
 * / _ \ /___)/ _ \| | | |/ _ \ / _ \ / ___) _ \|    \ 
 *| |_| |___ | |_| | |_| | |_| | |_| ( (__| |_| | | | |
 * \___/(___/ \___/ \__  |\___/ \___(_)____)___/|_|_|_|
 *                  (____/ 
 * OSOYOO SG90 servo steering robot car Lesson 2
 * Tutorial URL https://osoyoo.com/?p=37134
 * CopyRight www.osoyoo.com

 *   
 * 
 */
#include <PWMServo.h>
#include <SoftwareSerial.h>
SoftwareSerial BLTSerial(11, 4);  //RX,TX
int buttonState;
 
#define IN1 7
#define IN2 8
#define ENA 5        //  ENA pin
 
 
#define FRONT 95        // steering to front 
int  SHARP_RIGHT=FRONT+35;
int  RIGHT=FRONT+25;
int  LEFT=FRONT-25;
int  SHARP_LEFT=FRONT-35;

#define MAX_PACKETSIZE 32    //Serial receive buffer
char buffUART[MAX_PACKETSIZE];
unsigned int buffUARTIndex = 0;
unsigned long preUARTTick = 0;
int SPEED =150;   
int TURNSPEED=50  ;

#define SERVO_PIN 9  //servo connect to D7
 

struct car_status{
  int speed;
  int angle;
  int direct;
};
 

PWMServo head;
/*motor control*/
void go_Back()  //Forward
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2,LOW); 
 
}
 
void  go_Advance()  //Forward
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2,HIGH); 
   
 
}
 
void turn(int angle)
{

  head.write(angle);

}
 
void stop_Stop()    //Stop
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2,LOW);
  analogWrite(ENA,0);
 
}

  
void setup() {

 pinMode(ENA, OUTPUT); 
 pinMode(IN1, OUTPUT); 
 pinMode(IN2, OUTPUT); 


head.attach(SERVO_PIN);
 turn(FRONT);

 Serial.begin(9600);
 BLTSerial.begin(9600);
}

 
void loop() {
 do_Uart_Tick();
}
void do_Uart_Tick()
{

  char Uart_Date=0;
  if(BLTSerial.available()) 
  {
    size_t len = BLTSerial.available();
    uint8_t sbuf[len + 1];
    sbuf[len] = 0x00;
    BLTSerial.readBytes(sbuf, len);
    //parseUartPackage((char*)sbuf);
    memcpy(buffUART + buffUARTIndex, sbuf, len);//ensure that the serial port can read the entire frame of data
    buffUARTIndex += len;
    preUARTTick = millis();
    if(buffUARTIndex >= MAX_PACKETSIZE - 1) 
    {
      buffUARTIndex = MAX_PACKETSIZE - 2;
      preUARTTick = preUARTTick - 200;
    }
  }
  car_status cs;
  if(buffUARTIndex > 0 && (millis() - preUARTTick >= 100))//APP send flag to modify the obstacle avoidance parameters
  { //data ready
    buffUART[buffUARTIndex] = 0x00;
    Uart_Date=buffUART[0];
    cs=get_status(buffUART);
    buffUARTIndex = 0;
  }
   SPEED=cs.speed+50;
   TURNSPEED=SPEED*3/2;
   if (SPEED>250) SPEED=250;
   if (TURNSPEED>250) TURNSPEED=250;
  switch (Uart_Date)    //serial control instructions
  {
    case 'M':  
      go_Advance();
      turn(FRONT);
      analogWrite(ENA,SPEED);
    break;
    case 'L':  
      if(cs.angle=1)  turn(LEFT);
      if(cs.angle=2)  turn(SHARP_LEFT); 
      analogWrite(ENA,SPEED);
    break;
    case 'R': 
      if(cs.angle=-1) turn(RIGHT);
      if(cs.angle=-2) turn(SHARP_RIGHT);  
      analogWrite(ENA,SPEED);
    break;
    case 'B':  
      go_Back(); 
      analogWrite(ENA,SPEED);
    break;
    case 'X': 
      go_Back();
      if(cs.angle=1)  turn(LEFT);
      if(cs.angle=2)  turn(SHARP_LEFT); 
      analogWrite(ENA,SPEED);
    break;
    case 'Y':  
      go_Back();
      if(cs.angle=-1) turn(RIGHT);
      if(cs.angle=-2) turn(SHARP_RIGHT);  
      analogWrite(ENA,SPEED);
    break;
 
    case 'E': stop_Stop() ;break;
    case 'J': stop_Stop() ;break;
    default:break;
  }
}

car_status get_status( char buffUART[])
{
  car_status cstatus;
  int index=2;
  if (buffUART[index]=='-'){
    cstatus.angle=-buffUART[index+1]+'0';
    index=index+3;
    
  } else {
   
    cstatus.angle=buffUART[index]-'0';
     index=index+2;
  }
  int currentvalue;
  int spd=0;
  while (buffUART[index]!=',')
  {
    currentvalue=buffUART[index]-'0';
    spd=spd*10+currentvalue;
    index++;
  }
  cstatus.speed=spd;
  index++;
  cstatus.direct=buffUART[index]-'0';
  return cstatus;
}
