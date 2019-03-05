/*
EXAMPLE OF MOTOR WITH ROS.
 do
 roscore &
 for raspberry pi : rosrun rosserial_python serial_node.py /dev/ttyUSB0 &
 or for windows on COM4 : 
        sudo chmod 666 /dev/ttyS4 if COM4
        sudo chmod 666 /dev/ttyS24 if COM24
        rosrun rosserial_python serial_node.py /dev/ttyS4 & 
        rosrun rosserial_python serial_node.py /dev/ttyS24    _baud:=1000000 & 
        rosrun rosserial_server  serial_node _port:=/dev/ttyS24    _baud:=1000000
        
         rosrun rosserial_python serial_node.py /dev/ttyUSB0    _baud:=1000000 
         
        rostopic list
        rostopic echo /1/counter 

        rostopic echo -p /2/counter
/*
 rostopic pub /r1/pwm/left std_msgs/Int16 --once     500 & 
 rostopic pub /r1/pwm/right std_msgs/Int16 --once     800 & 
 rostopic pub /r1/pwm/back std_msgs/Int16 --once -- -800 &
 
  rostopic pub /r1/pwm/back/ std_msgs/Int16 --once -- 00 &
  rostopic pub /r1/pwm/left std_msgs/Int16 --once -- 00 &
  rostopic pub /r1/pwm/right std_msgs/Int16 --once -- 00 &
  
 rostopic pub /r1/pwm/left std_msgs/Int16  --once --  4096 & 
 rostopic pub /r1/pwm/right std_msgs/Int16 --once --  4096 & 
 rostopic pub /r1/pwm/back std_msgs/Int16  --once --  4096 &
 
 
  rostopic pub /r1/pwm/back/ std_msgs/Int16 -r 60 -- 00 &
  rostopic pub /r1/pwm/left std_msgs/Int16   -r 60 -- 00 &
  rostopic pub /r1/pwm/right std_msgs/Int16  -r 60 -- 00 &
  
  rostopic echo /r1/encoder/back
  rostopic echo /r1/encoder/left
  rostopic echo /r1/encoder/right
  
  
  

  rostopic pub /r1/pwm/pompe std_msgs/Int16  --once -- 00 &

  rostopic echo /sensor/0
  rostopic echo /sensor/1
  rostopic echo /sensor/2
  rostopic echo /sensor/3
  rostopic echo /sensor/4
  rostopic echo /sensor/5
  rostopic echo /sensor/6
  rostopic echo /sensor/7
  xming
 rosrun plotjuggler PlotJuggler
*/

// NOTE : WINDOWS 10 with ubuntu 16.04 as subsystem, after ros install do this :
//      sudo apt-get install ros-kinetic-rosserial-windows
//     sudo apt-get install ros-kinetic-rosserial-server

// validated on Captor


#include <Wire.h>
#include <SPI.h>
#include <variant.h>
#include <bootloaders/boot.h>

#if defined(BOARD_ID_Pilo)
      #include <Wire.h>
      #include <SPI.h>
      #include <variant.h>

      #include <WireUtility.h>


      #include "ros.h"
      ros::NodeHandle  nh;

#elif defined(BOARD_ID_Captor)
    #include <Wire.h>
    #include <SPI.h>
    #include <variant.h>

    #include <WireUtility.h>


    #define ROS_SERIAL (P_COM0.serial2)


    #include "ros.h"
    ros::NodeHandle  nh;

#else
//#include <Servo.h> 
#include "ros.h"
ros::NodeHandle  nh;
#endif

//import the library in the sketch
#include <ZOpticalSensor.h>

#include <ZCmdMotor.h>
#include <Zmotor3.h>
#define MySerial P_COM0.serial2
#define PcomSerial MySerial





void setupIntPriority()
{
  NVIC_SetPriority (EIC_IRQn, ((1<<__NVIC_PRIO_BITS) - 2));  /* set Priority low */
  /*
  NVIC_SetPriority (SERCOM0_IRQn, 0);  // set Priority high 
  NVIC_SetPriority (SERCOM3_IRQn,0);  // set Priority high 
  NVIC_SetPriority (SERCOM2_IRQn, 0);  //set Priority low 
  NVIC_SetPriority (SERCOM5_IRQn, 0); // set Priority low 
  */
  
  
}


#define HP1 P_Encoder[0].Pin.IA
#define HP2 P_Encoder[0].Pin.IB
        
        
#define BIP_FATAL       0xFFFFFFFF              //!< 4s bip
#define BIP_ERROR       0xFF00FF00              //!< 2*1s bip
#define BIP_WARNING     0xF0F0F0F0              //!< 4*0.5s
#define BIP_LONG_OK     0x33333333              //!< 8*0.25s
#define BIP_MEDIUM_OK   0x5555                  //!< 8*0.125s
#define BIP_OK          0x3333                  //!< 4*0.25s
#define BIP_FAST_OK     0x55                    //!< 4*0.125s
  
void bip(uint32_t bip)
{
  int s;
  for(s=32;(s>0) && ((((bip>>s)&1)==0));s--);
  s++;
  pinMode(HP1, OUTPUT);	
  pinMode(HP2, OUTPUT);        
  for(int i=0;i<s;i++)
  {
      if ((((bip>>i)&1)==1))
      {   
          digitalWrite(HP1, LOW); 
          digitalWrite(HP2, HIGH);
      }
      delay(124);
      {   
        digitalWrite(HP1, LOW); 
        digitalWrite(HP2, LOW);
      }
  }
  pinMode(HP1, INPUT);	pinMode(HP2, INPUT);        
}
void init_debug();
void setup()
{


ROS_SERIAL.begin(ROS_BAUDRATE);
while(1)
{
  ROS_SERIAL.print("temp: \r\n");
 delay(100);
} 
init_debug();
	pinMode(LED_BOTTOM, OUTPUT);
        digitalWrite(LED_BOTTOM, LOW);
	 // turn the LED on (HIGH is the voltage level)

//loopDEBUGMOTOR();

nh.initNode(); 
   //wait until you are actually connected
   uint8_t status=LOW;


setupIntPriority();

bip(BIP_MEDIUM_OK); 
/*
    while (!nh.connected())
    {
    
      nh.spinOnce();
      delay(50);
      status=(status==LOW)?HIGH:LOW;//toggle
      digitalWrite(LED_BOTTOM, status);
      
    }
*/
 digitalWrite(LED_BOTTOM, HIGH);
bip(BIP_OK);   
nh.loginfo("Captor Setup DoneK !");
}
int i=0;
void loop()
{


 
  //loopOpticalSensor();//<typ 1102 / max 9160µs


   
 if (i++%100==0)//1Hz
   nh.loginfo("loop()");//<240µs 
 
   // loop_time();
    
    delay(5);

    nh.spinOnce();//<20µs
 
 //   delay(1);//less than 500Hz less ros serial server fail //for python version 50Hz
    
}



