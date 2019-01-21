//EXAMPLE OF SERVO WITH ROS.
// do
// roscore &
// for raspberry pi : rosrun rosserial_python serial_node.py /dev/ttyUSB0 &
// or for windows on COM4 : 
//        sudo chmod 666 /dev/ttyS4 if COM4
//        sudo chmod 666 /dev/ttyS24 if COM24
//        rosrun rosserial_python serial_node.py /dev/ttyS4 & 
//        rosrun rosserial_python serial_node.py /dev/ttyS24 & 
// rostopic pub servo/A std_msgs/Int16 --once    10 
// rostopic pub servo/B std_msgs/Int16 --once    180
// rostopic pub servo/A std_msgs/Int16 --once -- -180
// rostopic pub servo/B std_msgs/Int16 --once     10 


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




#if defined(BOARD_ID_Pilo)
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

#define MySerial P_COM3.serial2 //usb
#define WireMotor2 (P_COM0_BIS.wire)

void sczni2c() {
  // put your setup code here, to run once:
  MySerial.begin(115200);
 
  MySerial.println("begin end");
  WireMotor2.begin();
  WireMotor2.setClock(1000);

  volatile int ip = scan(MySerial, WireMotor2);
  while (ip = scanNext(MySerial, WireMotor2) != 0)
    ;
  MySerial.println("setup end");
  delay(2000);
}

void setup() {
  WireMotor2.begin();
 //sczni2c() ;
    //card_motor2.begin(&Wire,0x43);
    card_motor2.begin(&(WireMotor2),0x21,0x41);
    
  nh.initNode(); 
  card_motor2.setup(&nh,"motor2/0",0);
  card_motor2.setup(&nh,"motor2/2",2);
  card_motor2.setup(&nh,"motor2/4",4);
  card_motor2.setup(&nh,"motor2/6",6);
  card_motor2.setup(&nh,"motor2/8",8);
  card_motor2.setup(&nh,"motor2/A",10);
  card_motor2.setup(&nh,"motor2/C",12);
  card_motor2.setup(&nh,"motor2/E",14);

  nh.logdebug("Debug Statement");

   //wait until you are actually connected
    while (!nh.connected())
    {
      nh.spinOnce();
   //   delay(10);
    }
    
    nh.logdebug("Debug Statement");
    nh.loginfo("Program info");
    nh.logwarn("Warnings.");
    nh.logerror("Errors..");
    nh.logfatal("Fatalities!");

}
void loopnh() {
  
  card_motor2.loop();

  nh.spinOnce();
}
void loop()
{// put your main code here, to run repeatedly:
  loopnh();
    nh.loginfo("loop()");
   delay(5);
  
}
