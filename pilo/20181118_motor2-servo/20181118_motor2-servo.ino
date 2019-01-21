//EXAMPLE OF SERVO WITH ROS.
// do
// roscore &
// for raspberry pi : rosrun rosserial_python serial_node.py /dev/ttyUSB0 &
// or for windows on COM4 : 
//        sudo chmod 666 /dev/ttyS4 if COM4
//        sudo chmod 666 /dev/ttyS3 if COM3
//        rosrun rosserial_python serial_node.py /dev/ttyS4 & 
//        rosrun rosserial_python serial_node.py /dev/ttyS3   _baud:=57600 & 
// rostopic pub motor2/A std_msgs/Int16 --once    10 
// rostopic pub motor2/B std_msgs/Int16 --once    180
// rostopic pub motor2/A std_msgs/Int16 --once -- -180
// rostopic pub motor2/B std_msgs/Int16 --once     10 


// NOTE : WINDOWS 10 with ubuntu 16.04 as subsystem, after ros install do this :
//      sudo apt-get install ros-kinetic-rosserial-windows
//     sudo apt-get install ros-kinetic-rosserial-server

// validated on Arduino Mega
// validated on Pilo

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif
#include <Zmotor2.h> 

#include <ZServoPCA9685.h> 
#include <ZPCA9685.h>


#if defined(BOARD_ID_Pilo)
#include <Wire.h>
#include <SPI.h>
#include <variant.h>

#include <WireUtility.h>


#define ROS_SERIAL (P_COM3.serial2)
#define ROS_BAUDRATE 57600
#include "ros.h"
ros::NodeHandle  nh;

#elif defined(BOARD_ID_Captor)
#include <Wire.h>
#include <SPI.h>
#include <variant.h>

#include <WireUtility.h>


#define ROS_SERIAL (P_COM3.serial2)
#define ROS_BAUDRATE 57600
#include "ros.h"
ros::NodeHandle  nh;

#else
//#include <Servo.h> 
#include "ros.h"
ros::NodeHandle  nh;
#endif


Zmotor2 card_motor2 = Zmotor2();


ZPCA9685 card_servos = ZPCA9685();
ZServoPCA9685 servo_cmd0= ZServoPCA9685(&card_servos);
ZServoPCA9685 servo_cmd1= ZServoPCA9685(&card_servos);
ZServoPCA9685 servo_cmd2= ZServoPCA9685(&card_servos);
ZServoPCA9685 servo_cmd3= ZServoPCA9685(&card_servos);
ZServoPCA9685 servo_cmd4= ZServoPCA9685(&card_servos);
ZServoPCA9685 servo_cmd5= ZServoPCA9685(&card_servos);
ZServoPCA9685 servo_cmd6= ZServoPCA9685(&card_servos);
ZServoPCA9685 servo_cmd7= ZServoPCA9685(&card_servos);
ZServoPCA9685 servo_cmd8= ZServoPCA9685(&card_servos);
ZServoPCA9685 servo_cmd9= ZServoPCA9685(&card_servos);
ZServoPCA9685 servo_cmdA= ZServoPCA9685(&card_servos);
ZServoPCA9685 servo_cmdB= ZServoPCA9685(&card_servos);
ZServoPCA9685 servo_cmdC= ZServoPCA9685(&card_servos);
ZServoPCA9685 servo_cmdD= ZServoPCA9685(&card_servos);
ZServoPCA9685 servo_cmdE= ZServoPCA9685(&card_servos);
ZServoPCA9685 servo_cmdF= ZServoPCA9685(&card_servos);
ZServoPCA9685 servo_cmd[16]={servo_cmd0,servo_cmd1,servo_cmd2,servo_cmd3,
servo_cmd4,servo_cmd5,servo_cmd6,servo_cmd7,
servo_cmd8,servo_cmd9,servo_cmdA,servo_cmdB,
servo_cmdC,servo_cmdD,servo_cmdE,servo_cmdF 
};
int pin=9;// digital  9

#define MySerial P_COM3.serial2 //usb
#define WireMotor2 (P_COM0_BIS.wire)
#define WirePCA9685 (P_COM0_BIS.wire)
void 	test_servo()
{

   for(int i=0;i<16;i+=1)  
    servo_cmd[i].write(90);
      delay(1000); 
       for(int i=0;i<16;i+=1)  
    servo_cmd[i].write(180);
 delay(1000); 
    for(int i=0;i<16;i+=1)  
    servo_cmd[i].write(0);
  delay(1000); 
 for(int i=0;i<16;i+=1)  
    servo_cmd[i].write(90);
}
#define I2CADDR_MOTOR2_1 0x20
#define I2CADDR_MOTOR2_2 0x40
#define I2CADDR_SERVO    0x43

void setup() {

if(!( WireTest( WireMotor2, I2CADDR_MOTOR2_2) && WireTest( WireMotor2, I2CADDR_SERVO) && WireTest( WireMotor2, I2CADDR_MOTOR2_2)))
{while(1);};//skip if wire issue

WireMotor2.begin();
	WireMotor2.setClock(10000);

if(!( WireTest( WireMotor2, I2CADDR_MOTOR2_2) && WireTest( WireMotor2, I2CADDR_SERVO) && WireTest( WireMotor2, I2CADDR_MOTOR2_2)))
{while(1);};//skip if wire issue
        
if (0)
{
MySerial.begin(57600);  //115200 //9600
	MySerial.println("Setup");

	volatile int ip = scan(MySerial, WireMotor2);
	while (ip = scanNext(MySerial, WireMotor2) != 0);
}

wireResetAllDevices(WireMotor2);

    //card_motor2.begin(&Wire,0x43);
	card_servos.begin(&(WirePCA9685),I2CADDR_SERVO);
    card_motor2.begin(&(WireMotor2),I2CADDR_MOTOR2_1,I2CADDR_MOTOR2_2);
        
  nh.initNode(); 
  card_motor2.setup(&nh,"motor2/0",0);
  card_motor2.setup(&nh,"motor2/2",2);
  card_motor2.setup(&nh,"motor2/4",4);
  card_motor2.setup(&nh,"motor2/6",6);
  card_motor2.setup(&nh,"motor2/8",8);
  card_motor2.setup(&nh,"motor2/A",10);
  card_motor2.setup(&nh,"motor2/C",12);
  card_motor2.setup(&nh,"motor2/E",14);

  servo_cmd0.setup(&nh,"servo/0",0);
  servo_cmd1.setup(&nh,"servo/1",1);
  servo_cmd2.setup(&nh,"servo/2",2);
  servo_cmd3.setup(&nh,"servo/3",3);
  servo_cmd4.setup(&nh,"servo/4",4);
  servo_cmd5.setup(&nh,"servo/5",5);
  servo_cmd6.setup(&nh,"servo/6",6);
  servo_cmd7.setup(&nh,"servo/7",7);
  servo_cmd8.setup(&nh,"servo/8",8);
  servo_cmd9.setup(&nh,"servo/9",9);
  servo_cmdA.setup(&nh,"servo/A",10);
  servo_cmdB.setup(&nh,"servo/B",11);
  servo_cmdC.setup(&nh,"servo/C",12);
  servo_cmdD.setup(&nh,"servo/D",13);
  servo_cmdE.setup(&nh,"servo/E",14);
  servo_cmdF.setup(&nh,"servo/F",15);
 // test_servo();
  nh.logdebug("Debug Statement");
  nh.logdebug("Debug Statement");

   //wait until you are actually connected
    while (!nh.connected())
    {
      nh.spinOnce();
      delay(10);
    }
    
    
if( !WireMotor2.testLine( ))
{
  nh.logerror( "Pilo : WireMotor2  not working" );
}
if (card_servos.test())
        {nh.logerror( "Pilo : card_servos  not working" );}
if (card_motor2.test())
        {nh.logerror( "Pilo : card_motor2  not working" );}
        
    nh.logdebug("Debug Statement");
    nh.loginfo("Program info");
    nh.logwarn("Warnings.");
    nh.logerror("Errors..");
    nh.logfatal("Fatalities!");

	/*
        for(int i=0;i<16;i++)
    card_motor2.cmd(i,000); 

        for(int i=0;i<16;i+=2)
    card_motor2.cmd(i,2000); 
*/
#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)

 	card_servos.begin(&(WirePCA9685),I2CADDR_SERVO);
 
card_servos.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
(card_servos.test());

 for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
 for (uint16_t servonum = 0; servonum < 16; servonum++) {
    card_servos.setPWM(servonum, 0, pulselen);
  }
  delay(10);
}
 for (uint16_t servonum = 0; servonum < 16; servonum++) {
    card_servos.setPWM(servonum, 0, SERVOMAX/2);
}
/*
while(1)
{
for(int i=0;i<16;i++)
{
servo_cmd[i].writeMicroseconds(600);
}
    delay(1000);

for(int i=0;i<16;i++)
{
servo_cmd[i].writeMicroseconds(1400);
}
    delay(1000);

for(int i=0;i<16;i++)
{
servo_cmd[i].writeMicroseconds(2300);
}
    delay(1000);

}	// play with servo to see it moving
*/
/*
while(1)
{
for(int i=0;i<16;i++)
{
servo_cmd[i].write(10);
}
    delay(1000);
for(int i=0;i<16;i++)
{
servo_cmd[i].write(90);
}
    delay(1000);
for(int i=0;i<16;i++)
{
servo_cmd[i].write(120);
}
    delay(1000);

}*/

servo_cmd0.write(10);
    delay(1000);
     servo_cmd0.write(150);
    delay(1000);
     servo_cmdE.write(90);
    
/*
servo_cmdE.write(10);
    servo_cmdF.write(10);
    delay(1000);
     servo_cmdE.write(180);
    servo_cmdF.write(180);
    delay(1000);
     servo_cmdE.write(90);
    servo_cmdF.write(90);
        */
}

void loopservos() {
  
  servo_cmd0.loop();
  servo_cmd1.loop();
  servo_cmd2.loop();
  servo_cmd3.loop();
  servo_cmd4.loop();
  servo_cmd5.loop();
  servo_cmd6.loop();
  servo_cmd7.loop();
  servo_cmd8.loop();
  servo_cmd9.loop();
  servo_cmdA.loop();
  servo_cmdB.loop();
  servo_cmdC.loop();
  servo_cmdD.loop();
  servo_cmdE.loop();
  servo_cmdF.loop();

}
void loopnh() {
  loopservos();
  card_motor2.loop();

  nh.spinOnce();
}
void loop()
{// put your main code here, to run repeatedly:
  loopnh();
    nh.loginfo("loop()");
   delay(5);
  
}
