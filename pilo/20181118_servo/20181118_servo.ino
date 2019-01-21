//EXAMPLE OF SERVO WITH ROS.
// do
// roscore &
// for raspberry pi : rosrun rosserial_python serial_node.py /dev/ttyUSB0 &
// or for windows on COM4 : 
//        sudo chmod 666 /dev/ttyS4 if COM4
//        sudo chmod 666 /dev/ttyS24 if COM24
//        rosrun rosserial_python serial_node.py /dev/ttyS4 & 
//        rosrun rosserial_python serial_node.py /dev/ttyS24 & 
// rostopic pub servo/A std_msgs/UInt16 10 --once
// rostopic pub servo/B std_msgs/UInt16 180 --once
// rostopic pub servo/A std_msgs/UInt16 180 --once
// rostopic pub servo/B std_msgs/UInt16 10 --once


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
#else
//#include <Servo.h> 
#include "ros.h"
ros::NodeHandle  nh;
#endif


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

int pin=9;// digital  9
// example of call back to be define in case of 2 card ZPCA9685.
/*
void callbackinstance00( const std_msgs::UInt16& cmd_msg)
{  
  //nh.loginfo("callbackinstance00()");
servo_cmdA.write(cmd_msg.data); //set servo angle, should be from 0-180   

}


ros::Subscriber<std_msgs::UInt16> sub("servo/A", callbackinstance00);
*/
#define MySerial P_COM3.serial2 //usb
#define WirePCA9685 (P_COM0_BIS.wire)
void sczni2c() {
  // put your setup code here, to run once:
  MySerial.begin(115200);
 
  MySerial.println("begin end");
  WirePCA9685.begin();
  WirePCA9685.setClock(1000);

  volatile int ip = scan(MySerial, WirePCA9685);
  while (ip = scanNext(MySerial, WirePCA9685) != 0)
    ;
  MySerial.println("setup end");
  delay(2000);
}

void setup() {
// sczni2c() ;
    //card_servos.begin(&Wire,0x43);
    card_servos.begin(&(WirePCA9685),0x43);
    
  nh.initNode();   
  //wait until you are actually connected
    while (!nh.connected())
    {
      nh.spinOnce();
      delay(10);
    }
 //servo_cmdA.setup(&nh,"servo/A",callbackinstance00,pin);
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
  
  nh.logdebug("Debug Statement");


    
    nh.logdebug("Debug Statement");
    nh.loginfo("Program info");
    nh.logwarn("Warnings.");
    nh.logerror("Errors..");
    nh.logfatal("Fatalities!");
    
// play with servo to see it moving
    servo_cmdA.write(10);
    servo_cmdB.write(10);
    delay(1000);
     servo_cmdA.write(180);
    servo_cmdB.write(180);
    delay(1000);
     servo_cmdA.write(90);
    servo_cmdB.write(90);
}
void loopnh() {
  
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

  nh.spinOnce();
}
void loop()
{// put your main code here, to run repeatedly:
  loopnh();
    nh.loginfo("loop()");
   delay(5);
  
}
