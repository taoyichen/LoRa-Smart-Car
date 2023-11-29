/*  ___   ___  ___  _   _  ___   ___   ____ ___  ____  
 * / _ \ /___)/ _ \| | | |/ _ \ / _ \ / ___) _ \|    \ 
 *| |_| |___ | |_| | |_| | |_| | |_| ( (__| |_| | | | |
 * \___/(___/ \___/ \__  |\___/ \___(_)____)___/|_|_|_|
 *                  (____/ 
 * OSOYOO Servo Steering Robot Lesson 5 Wifi Control STA mode
 * Tutorial link: https://osoyoo.com/?p=37202
 * CopyRIGHT www.osoyoo.com
 */
#include <PWMServo.h>
#ifndef HAVE_HWSERIAL1
#include "SoftwareSerial.h"
SoftwareSerial Serial1(11, 4); // E_TX， E_RX 
#endif

#define LFSensor_0 A0  //OLD D3
#define LFSensor_1 A1
#define LFSensor_2 A2
#define LFSensor_3 A3
#define LFSensor_4 A4  //OLD D10

#define SPEED 160
#define HI_SPEED 190
#define TURN_SPEED 170
 
#define forwardtime 800  //Time the robot spends turning (miliseconds)
#define turntime    900  //Time the robot spends turning (miliseconds)
#define backtime    800  //Time the robot spends turning (miliseconds)

//define Model X pins
#define IN1 7
#define IN2 8
#define ENA 5        //  ENA pin
 
//define steer directing
 
#define FRONT 90        // adjust this value if your steer is not facing front at beginning
#define SERVO_PIN 9  //servo connect to D7
#define SERVO_SENSOR  10  //Ultrasonic sensor servo connect to D8
int LEFT=FRONT-30;
int SLIGHT_LEFT=FRONT-20;
int RIGHT=FRONT+30;
int SLIGHT_RIGHT=FRONT+20;
int angle =FRONT;
int turn_flag=0;
#define SENSOR_FRONT 90
int SENSOR_LEFT=SENSOR_FRONT+25;
int SENSOR_RIGHT=SENSOR_FRONT-25;
int SENSOR_FAR_LEFT=SENSOR_FRONT+60;
int SENSOR_FAR_RIGHT=SENSOR_FRONT-60;
PWMServo head_sensor;
PWMServo head;
#define Echo_PIN    2 // Ultrasonic Echo pin connect to D11
#define Trig_PIN    3  // Ultrasonic Trig pin connect to D12
int distance;
int numcycles = 0;
int leftscanval, centerscanval, rightscanval, ldiagonalscanval, rdiagonalscanval;
const int distancelimit = 30; //distance limit for obstacles in front           
const int sidedistancelimit = 30;
#define LPT 2 // scan loop coumter
int thereis;
/*motor control*/
#include "WiFiEsp.h"
#include "WiFiEspUDP.h"
char ssid[] = "osoyoo_robot";  
int status = WL_IDLE_STATUS;
// use a ring buffer to increase speed and reduce memory allocation
 char packetBuffer[5]; 
WiFiEspUDP Udp;
unsigned int localPort = 8888;  // local port to listen on

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

 
 
void steer(int angle)
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
 pinMode(LFSensor_0, INPUT); 
 pinMode(LFSensor_1, INPUT); 
 pinMode(LFSensor_2, INPUT); 
 pinMode(LFSensor_3, INPUT); 
 pinMode(LFSensor_4, INPUT); 
pinMode(Trig_PIN, OUTPUT); 
 pinMode(Echo_PIN,INPUT); 
 
 Serial.begin(9600);
 head.attach(SERVO_PIN);

 head_sensor.attach(SERVO_SENSOR); 
 
 head_sensor.write(SENSOR_FAR_LEFT);
 delay(1000);
 head_sensor.write(SENSOR_FAR_RIGHT);
 delay(1000);
 head_sensor.write(SENSOR_FRONT);
 delay(1000);


  Serial.begin(9600);   // initialize serial for debugging
    Serial1.begin(115200);
    Serial1.write("AT+UART_DEF=9600,8,1,0,0\r\n");
  delay(200);
  Serial1.write("AT+RST\r\n");
  delay(200);
  Serial1.begin(9600);    // initialize serial for ESP module
  WiFi.init(&Serial1);    // initialize ESP module

  // check for the presence of the shield
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue
    while (true);
  }

  status = WiFi.beginAP(ssid, 10, "", 0);

  Serial.println("Please connect your APP to ssid OSOYOO_ROBOT, set target IP address 192.168.4.1");
  
  Udp.begin(localPort);
  
  Serial.print("Listening on port ");
  Serial.println(localPort);

     
}

 int action=0;

void loop() {
     int packetSize = Udp.parsePacket();
  if (packetSize) {                               // if you get a client,
     Serial.print("Received packet of size ");
    Serial.println(packetSize);
    int len = Udp.read(packetBuffer, 255);
     Udp.flush();
    if (len > 0) {
      packetBuffer[len] = 0;
    }
      char c=packetBuffer[0];
            switch (c)    //serial control instructions
            {   
  
               case 'A': action=0; turn_flag=0;steer(FRONT); go_Advance(); analogWrite(ENA,SPEED); break;
               case 'R': action=0;
                         if(turn_flag==0) 
                         {
                           steer(FRONT);
                           analogWrite(ENA,SPEED); 
                           delay(200);
                           turn_flag=1;
                         }
                         steer(RIGHT);  
                         analogWrite(ENA,TURN_SPEED);   
                         break;
               case 'L': action=0;
                         if(turn_flag==0) 
                         {
                           steer(FRONT);
                           analogWrite(ENA,SPEED); 
                           delay(200);
                           turn_flag=1;
                         }
                         steer(LEFT);  
                         analogWrite(ENA,TURN_SPEED); 
                         break;
               case 'B': action=0;turn_flag=0; steer(FRONT);  go_Back();analogWrite(ENA,SPEED); break;
               case 'E':action=0;turn_flag=0; analogWrite(ENA,0); break;
               case 'O':action=2;turn_flag=0;break;
               case 'T':turn_flag=0; action=1; break;
               case 'S':if(packetBuffer[1] == 'T')
                        {
                          int angle = get_value(packetBuffer);
                          angle = map(angle, 0, 99,  LEFT,RIGHT);
                        
                          steer(angle);
                        }
                        break;
               
               default:break;
               
              } //END OF ACTION SWITCH
  
  }
  if (action==1) auto_tracking();
  if (action==2) auto_avoidance();
}
void printWifiStatus()
{
  // print your WiFi shield's IP address
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print where to go in the browser
  Serial.println();
  Serial.print("To see this page in action, connect to ");
  Serial.print(ssid);
  Serial.print(" and open a browser to http://");
  Serial.println(ip);
  Serial.println();
}
char sensor[5];
void auto_tracking(){
 int sensorvalue=32;
  sensor[0]= !digitalRead(LFSensor_0);
 
  sensor[1]=!digitalRead(LFSensor_1);
 
  sensor[2]=!digitalRead(LFSensor_2);
 
  sensor[3]=!digitalRead(LFSensor_3);
 
  sensor[4]=!digitalRead(LFSensor_4);
  sensorvalue +=sensor[0]*16+sensor[1]*8+sensor[2]*4+sensor[3]*2+sensor[4];
  
  String sensorval= String(sensorvalue,BIN).substring(1,6);
  
 
  Serial.print (sensorval);
  if (   sensorval=="01000" || sensorval=="10000"  || sensorval=="11000" )
  { 
    
    go_Advance();
    analogWrite(ENA,HI_SPEED); 
    steer(LEFT);
     Serial.println(" LEFT");
  }
  if ( sensorval=="01100" || sensorval=="11100"  || sensorval=="11110" )
  {
    
    go_Advance();
    analogWrite(ENA,  SPEED);
    steer(SLIGHT_LEFT);
    Serial.println("S LEFT");
  }
  if ( sensorval=="00010"  ||  sensorval=="00011" ||  sensorval=="00001"  ){ //The black line is  on the right of the car, need  right turn 
   
    go_Advance();
    analogWrite(ENA,HI_SPEED);
     steer(RIGHT);
     Serial.println("RIRGHT");
  }
  if ( sensorval=="00110" || sensorval=="00111" || sensorval=="01111")
  {
    
    go_Advance();
     analogWrite(ENA, SPEED);
     steer(SLIGHT_RIGHT);
     Serial.println("S RIRGHT");
  }
   if (sensorval=="00100"    || sensorval=="01110")
  {
    
    go_Advance();
    analogWrite(ENA, SPEED);
    steer(FRONT);
     Serial.println("FRONT");
  }
  if (sensorval=="11111"){
    stop_Stop();   //The car front touch stop line, need stop
    steer(FRONT);
     Serial.println("STOP");
  }

}
/*detection of ultrasonic distance*/
int watch(){
  long echo_distance;
  digitalWrite(Trig_PIN,LOW);
  delayMicroseconds(5);                                                                              
  digitalWrite(Trig_PIN,HIGH);
  delayMicroseconds(15);
  digitalWrite(Trig_PIN,LOW);
  echo_distance=pulseIn(Echo_PIN,HIGH);
  echo_distance=echo_distance*0.01657; //how far away is the object in cm
  //Serial.println((int)echo_distance);
  return round(echo_distance);
}
//Meassures distances to the right, left, front, left diagonal, right diagonal and asign them in cm to the variables rightscanval, 
//leftscanval, centerscanval, ldiagonalscanval and rdiagonalscanval (there are 5 points for distance testing)
String watchsurrounding(){
/*  obstacle_status is a binary integer, its last 5 digits stands for if there is any obstacles in 5 directions,
 *   for example B101000 last 5 digits is 01000, which stands for Left front has obstacle, B100111 means front, right front and right ha
 */
 
int obstacle_status =B100000;
  centerscanval = watch();
  if(centerscanval<distancelimit){
    stop_Stop();
    
    obstacle_status  =obstacle_status | B100;
    }
  head_sensor.write(SENSOR_LEFT);
  delay(100);
  ldiagonalscanval = watch();
  if(ldiagonalscanval<distancelimit){
    stop_Stop();
    
     obstacle_status  =obstacle_status | B1000;
    }
  head_sensor.write(SENSOR_FAR_LEFT); //Didn't use 180 degrees because my servo is not able to take this angle
  delay(300);
  leftscanval = watch();
  if(leftscanval<sidedistancelimit){
    stop_Stop();
    
     obstacle_status  =obstacle_status | B10000;
    }

  head_sensor.write(SENSOR_FRONT); //use 90 degrees if you are moving your servo through the whole 180 degrees
  delay(100);
  centerscanval = watch();
  if(centerscanval<distancelimit){
    stop_Stop();
    
    obstacle_status  =obstacle_status | B100;
    }
  head_sensor.write(SENSOR_RIGHT);
  delay(100);
  rdiagonalscanval = watch();
  if(rdiagonalscanval<distancelimit){
    stop_Stop();
    
    obstacle_status  =obstacle_status | B10;
    }
  head_sensor.write(SENSOR_FAR_RIGHT);
  delay(100);
  rightscanval = watch();
  if(rightscanval<sidedistancelimit){
    stop_Stop();
  
    obstacle_status  =obstacle_status | 1;
    }
  head_sensor.write(SENSOR_FRONT); //Finish looking around (look forward again)
  delay(300);
   String obstacle_str= String(obstacle_status,BIN);
  obstacle_str= obstacle_str.substring(1,6);
  
  return obstacle_str; //return 5-character string standing for 5 direction obstacle status
}

void auto_avoidance(){

  ++numcycles;
  if(numcycles>=LPT){ //Watch if something is around every LPT loops while moving forward 
     stop_Stop();
    String obstacle_sign=watchsurrounding(); // 5 digits of obstacle_sign binary value means the 5 direction obstacle status
      Serial.print("begin str=");
        Serial.println(obstacle_sign);
                    if( obstacle_sign=="00000"){
     Serial.println("FORWARD");
     go_Advance();
     steer(FRONT);
     analogWrite(ENA,SPEED); 
     delay(forwardtime);
     stop_Stop();
    }
 
    else if( obstacle_sign=="01000" || obstacle_sign=="11000" || obstacle_sign=="10000"  ){
     Serial.println("hand right");   
     go_Advance();
     steer(RIGHT);
     analogWrite(ENA,SPEED); 
     delay(turntime);
     stop_Stop();
     steer(FRONT);
    } 
    
    else if( obstacle_sign=="00010" ||obstacle_sign=="00011" ||obstacle_sign=="00001"  ){
    Serial.println("hand left");
      
     go_Advance();
     steer(LEFT);
     analogWrite(ENA,HI_SPEED); 
     delay(turntime);
     stop_Stop();
     steer(FRONT);
    }
 
    else if( obstacle_sign=="00111" ||   obstacle_sign=="01111" ||  obstacle_sign=="10111" || obstacle_sign=="11111"  || obstacle_sign=="00110" || obstacle_sign=="01010"  || obstacle_sign=="00101" ){
    Serial.println("hand back right");
     
     go_Back();
     steer(RIGHT);
     analogWrite(ENA,HI_SPEED); 
     delay(backtime);
     stop_Stop();
     steer(FRONT);
   } 
     else if(obstacle_sign=="00100"  || obstacle_sign=="10100"  || obstacle_sign=="01100" || obstacle_sign=="11100" || obstacle_sign=="11011"  ||    obstacle_sign=="11101"  ||  obstacle_sign=="11110"  || obstacle_sign=="01110"  ){
     Serial.println("hand back left");
     
     go_Back();
     steer(LEFT);
     analogWrite(ENA,HI_SPEED); 
     delay(backtime);
     stop_Stop();
     steer(FRONT);
    }    
  
      else Serial.println("no handle");
    numcycles=0; //Restart count of cycles
    } else {
   
     steer(FRONT);
     analogWrite(ENA,SPEED); 
     go_Advance();
     delay(backtime);
     stop_Stop();
         
  }
  
  //else  Serial.println(numcycles);
  
  distance = watch(); // use the watch() function to see if anything is ahead (when the robot is just moving forward and not looking around it will test the distance in front)
  if (distance<distancelimit){ // The robot will just stop if it is completely sure there's an obstacle ahead (must test 25 times) (needed to ignore ultrasonic sensor's false signals)
  Serial.println("final go back");
  steer(RIGHT);
  analogWrite(ENA,HI_SPEED); 
  go_Back();
  delay(backtime*3/2);
  steer(FRONT);
  
      ++thereis;}
  if (distance>distancelimit){
      thereis=0;} //Count is restarted
  if (thereis > 25){
  Serial.println("final stop");
    stop_Stop(); // Since something is ahead, stop moving.
    thereis=0;
  }
}
int get_value(char bf[])
{
  int value = bf[3]- '0';
  
  for(int i = 4; i < 6; i++)
  {
    if(bf[i] < '0' || bf[i] > '9')
      break;
    else
      value = value * 10 + bf[i] - '0';
  }
  
  return value;
}
