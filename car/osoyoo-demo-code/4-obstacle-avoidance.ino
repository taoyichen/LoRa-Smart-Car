/*  ___   ___  ___  _   _  ___   ___   ____ ___  ____  
 * / _ \ /___)/ _ \| | | |/ _ \ / _ \ / ___) _ \|    \ 
 *| |_| |___ | |_| | |_| | |_| | |_| ( (__| |_| | | | |
 * \___/(___/ \___/ \__  |\___/ \___(_)____)___/|_|_|_|
 *                  (____/ 
 * OSOYOO SG90 steering car Lesson 6 : Obstacle Avoidance
 * Tutorial URL http://osoyoo.com/2018/12/19/osoyoo-robot-car-kit-lesson-4-obstacle-avoidance-robot-car/
 * CopyRight www.osoyoo.com

 * This project will show you how to make Osoyoo robot car in auto drive mode and avoid obstacles
 *   
 * 
 */
#include <PWMServo.h>
#define IN1 7
#define IN2 8
#define ENA 5       //  ENA pin

#define FRONT 90        //degree when steering facing straight forward
int SHARP_RIGHT=FRONT+33;
int SHARP_LEFT=FRONT-40;
int  RIGHT=FRONT+17;
int  LEFT=FRONT-20;

#define SENSOR_FRONT 90
int SENSOR_LEFT=SENSOR_FRONT+25;
int SENSOR_RIGHT=SENSOR_FRONT-25;
int SENSOR_FAR_LEFT=SENSOR_FRONT+60;
int SENSOR_FAR_RIGHT=SENSOR_FRONT-60;

#define DELAY_TIME    1000   

#define SPEED         170
#define FAST_SPEED    200 
#define MID_SPEED     170
#define SERVO_STEER   9  //steer servo connect to D9
#define SERVO_SENSOR  10  //Ultrasonic sensor servo connect to D10

#define Echo_PIN    2 // Ultrasonic Echo pin connect to D11
#define Trig_PIN    3  // Ultrasonic Trig pin connect to D12

PWMServo head;
PWMServo head_steer;
int leftscanval, centerscanval, rightscanval, ldiagonalscanval, rdiagonalscanval;
const int distancelimit = 30; //distance limit for obstacles in front           
const int sidedistancelimit = 30; //minimum distance in cm to obstacles at both sides (the car will allow a shorter distance sideways)
int distance;
int numcycles = 0;
const int forwardtime = 600; //Time the robot spends turning (miliseconds)
const int turntime = 900; //Time the robot spends turning (miliseconds)
const int backtime = 900; //Time the robot spends turning (miliseconds)
#define LPT 2 // scan loop coumter
int thereis;
/*motor control*/
void  go_Back(int speed)  //Forward
{

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2,LOW);
  analogWrite(ENA,speed);
}
 
void go_Advance(int speed)  //Forward
{

  digitalWrite(IN1, LOW);
  digitalWrite(IN2,HIGH);
  analogWrite(ENA,speed);
}
 
void turn(int angle)
{

  head_steer.write(angle);

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
 pinMode(Trig_PIN, OUTPUT); 
 pinMode(Echo_PIN,INPUT); 

 head_steer.attach(SERVO_STEER);
 head.attach(SERVO_SENSOR); 
 head.write(90);
 turn(FRONT);
 stop_Stop();
 delay(2000);

 Serial.begin(9600);
 
}

 
void loop() {
 auto_avoidance();
 
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
  head.write(SENSOR_LEFT);
  delay(100);
  ldiagonalscanval = watch();
  if(ldiagonalscanval<distancelimit){
    stop_Stop();
    
     obstacle_status  =obstacle_status | B1000;
    }
  head.write(SENSOR_FAR_LEFT); //Didn't use 180 degrees because my servo is not able to take this angle
  delay(300);
  leftscanval = watch();
  if(leftscanval<sidedistancelimit){
    stop_Stop();
    
     obstacle_status  =obstacle_status | B10000;
    }

  head.write(SENSOR_FRONT); //use 90 degrees if you are moving your servo through the whole 180 degrees
  delay(100);
  centerscanval = watch();
  if(centerscanval<distancelimit){
    stop_Stop();
    
    obstacle_status  =obstacle_status | B100;
    }
  head.write(SENSOR_RIGHT);
  delay(100);
  rdiagonalscanval = watch();
  if(rdiagonalscanval<distancelimit){
    stop_Stop();
    
    obstacle_status  =obstacle_status | B10;
    }
  head.write(SENSOR_FAR_RIGHT);
  delay(100);
  rightscanval = watch();
  if(rightscanval<sidedistancelimit){
    stop_Stop();
  
    obstacle_status  =obstacle_status | 1;
    }
  head.write(SENSOR_FRONT); //Finish looking around (look forward again)
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
     turn(FRONT);
     go_Advance(SPEED);
 
      delay(forwardtime);
      stop_Stop();
    }
 
    else if( obstacle_sign=="01000" || obstacle_sign=="11000" || obstacle_sign=="10000"  ){
     Serial.println("hand right");
           turn(SHARP_RIGHT);
     go_Advance(FAST_SPEED);
      delay(turntime);
      stop_Stop();
      turn(FRONT);
    } 
    
    else if( obstacle_sign=="00010" ||obstacle_sign=="00011" ||obstacle_sign=="00001"  ){
    Serial.println("hand left");
          turn(SHARP_LEFT);
     go_Advance(FAST_SPEED);
      delay(turntime);
      stop_Stop();
      turn(FRONT);
    }
 
    else if( obstacle_sign=="00111" ||   obstacle_sign=="01111" ||  obstacle_sign=="10111" || obstacle_sign=="11111"  || obstacle_sign=="00110" || obstacle_sign=="01010"  || obstacle_sign=="00101" ){
    Serial.println("hand back right");
     turn(RIGHT);
     go_Back(FAST_SPEED);
       delay(backtime);
          stop_Stop();
          turn(FRONT);
     } 
         else if(obstacle_sign=="00100"  || obstacle_sign=="10100"  || obstacle_sign=="01100" || obstacle_sign=="11100" || obstacle_sign=="11011"  ||    obstacle_sign=="11101"  ||  obstacle_sign=="11110"  || obstacle_sign=="01110"  ){
    Serial.println("hand back left");
      turn(LEFT);
     go_Back(FAST_SPEED);
       delay(backtime);
          stop_Stop();
          turn(FRONT);
     }    
  
      else Serial.println("no handle");
    numcycles=0; //Restart count of cycles
    } else {
             turn(FRONT);
     go_Advance(SPEED);
        delay(backtime);
          stop_Stop();
         
  }
  
  //else  Serial.println(numcycles);
  
  distance = watch(); // use the watch() function to see if anything is ahead (when the robot is just moving forward and not looking around it will test the distance in front)
  if (distance<distancelimit){ // The robot will just stop if it is completely sure there's an obstacle ahead (must test 25 times) (needed to ignore ultrasonic sensor's false signals)
 Serial.println("final go back");
       turn(SHARP_RIGHT);
     go_Back(FAST_SPEED);
  delay(backtime*3/2);
  turn(FRONT);
      ++thereis;}
  if (distance>distancelimit){
      thereis=0;} //Count is restarted
  if (thereis > 25){
  Serial.println("final stop");
    stop_Stop(); // Since something is ahead, stop moving.
    thereis=0;
  }
}
 
