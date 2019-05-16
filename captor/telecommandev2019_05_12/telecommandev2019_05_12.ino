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
/*

https://www.youtube.com/watch?v=lujgeInpejY
http://wiki.ros.org/rosjava
https://github.com/rosjava/rosjava_core
http://rosjava.github.io/rosjava_core/0.1.6/getting_started.html

http://www.gurucoding.com/en/raspberry_pi_eclipse/raspberry_pi_cross_compilation_in_eclipse.php
http://www.raspberry-projects.com/pi/programming-in-c/compilers-and-ides/eclipse/create-new-eclipse-project-for-the-rpi

https://playground.arduino.cc/Code/Eclipse

https://www.materiel.net/pc-portable/acer-aspire-a315-21-97ja-144195.html

http://eclipse.baeyens.it/
*/


#define M1_CA P_Encoder[8-1].Pin.IB 
#define M1_CB P_Encoder[8-1].Pin.IA   
#define M1_MP motorBoard.getPin(MOTOR3_P11_PWM)
#define M1_MM motorBoard.getPin(MOTOR3_P11_IO)
#define M1_EN motorBoard.getPin(MOTOR3_P11_EN) 


/*
#define M2_CA  P_Encoder[2-1].Pin.IA  
#define M2_CB  P_Encoder[2-1].Pin.IB  
#define M2_MP  -1 
#define M2_MM  -1 

*/
#define M2_CA  P_Encoder[7-1].Pin.IB 
#define M2_CB  P_Encoder[7-1].Pin.IA  
#define M2_MP motorBoard.getPin(MOTOR3_P7_PWM)  
#define M2_MM  motorBoard.getPin(MOTOR3_P7_IO) 
#define M2_EN  motorBoard.getPin(MOTOR3_P7_EN)  

#define M3_CA  P_Encoder[6-1].Pin.IB  
#define M3_CB  P_Encoder[6-1].Pin.IA  
#define M3_MP motorBoard.getPin(MOTOR3_P12_PWM)  
#define M3_MM  motorBoard.getPin(MOTOR3_P12_IO) 
#define M3_EN  motorBoard.getPin(MOTOR3_P12_EN)  
#define  MyWireMotor WireB 

#define M1_TOPIC_PWM "/r1/pwm/right"
#define M1_TOPIC_ENC "/r1/encoder/right"
#define M1_TOPIC_SPEED "/r1/speed/right"
#define M1_TOPIC_ENC_SPEED "/r1/encoder/right/speed"

#define M2_TOPIC_PWM "/r1/pwm/left"
#define M2_TOPIC_ENC "/r1/encoder/left"
#define M2_TOPIC_SPEED "/r1/speed/left"
#define M2_TOPIC_ENC_SPEED "/r1/encoder/left/speed"
 

#define M3_TOPIC_PWM "/r1/pwm/back"
#define M3_TOPIC_ENC "/r1/encoder/back"
#define M3_TOPIC_SPEED "/r1/speed/back"
#define M3_TOPIC_ENC_SPEED "/r1/encoder/back/speed"

//206 impulsion/tour de roues


// pompe
#define M4_CA  -1 
#define M4_CB  -1 
#define M4_MP motorBoard.getPin(MOTOR3_P6_PWM)  
#define M4_MM  motorBoard.getPin(MOTOR3_P6_IO) 
#define M4_EN  motorBoard.getPin(MOTOR3_P6_EN) 

#define M4_TOPIC_PWM "/r1/pwm/pompe"
#define M4_TOPIC_ENC "/r1/encoder/pompe"
#define M4_TOPIC_SPEED "/r1/speed/pompe"


#define M5_CA  -1 
#define M5_CB  -1 
#define M5_MP motorBoard.getPin(MOTOR3_P13_PWM)  
#define M5_MM  motorBoard.getPin(MOTOR3_P13_IO) 
#define M5_EN  motorBoard.getPin(MOTOR3_P13_EN) 

#define M5_TOPIC_PWM "/r1/pwm/vanne1"
#define M5_TOPIC_ENC "/r1/encoder/vanne1"
#define M5_TOPIC_SPEED "/r1/speed/vanne1"


#define M6_CA  -1 
#define M6_CB  -1 
#define M6_MP motorBoard.getPin(MOTOR3_P9_PWM)  
#define M6_MM  motorBoard.getPin(MOTOR3_P9_IO) 
#define M6_EN  motorBoard.getPin(MOTOR3_P9_EN) 

#define M6_TOPIC_PWM "/r1/pwm/vanne2"
#define M6_TOPIC_ENC "/r1/encoder/vanne2"
#define M6_TOPIC_SPEED "/r1/speed/vanne2"



#define M7_CA  -1 
#define M7_CB  -1 
#define M7_MP motorBoard.getPin(MOTOR3_P8_PWM)  
#define M7_MM  motorBoard.getPin(MOTOR3_P8_IO) 
#define M7_EN  motorBoard.getPin(MOTOR3_P8_EN) 

#define M7_TOPIC_PWM "/r1/pwm/vanne3"
#define M7_TOPIC_ENC "/r1/encoder/vanne3"
#define M7_TOPIC_SPEED "/r1/speed/vanne3"


// called this way, it uses the default address 0x40
Zmotor3 motorBoard = Zmotor3();

//ZEncoder enc(A0,A2,FULL, NULL);
CMDMOTOR cmd1(M1_CA, M1_CB, M1_MP, M1_MM);
CMDMOTOR cmd2(M2_CA, M2_CB, M2_MP, M2_MM);
CMDMOTOR cmd3(M3_CA, M3_CB, M3_MP, M3_MM);


CMDMOTOR cmd4(M4_CA, M4_CB, M4_MP, M4_MM);

CMDMOTOR cmd5(M5_CA, M5_CB, M5_MP, M5_MM);
CMDMOTOR cmd6(M6_CA, M6_CB, M6_MP, M6_MM);
CMDMOTOR cmd7(M7_CA, M7_CB, M7_MP, M7_MM);


void privateIntHandler1() {
  cmd1.getEncoder()->update();
}
void privateIntHandler2() {
  cmd2.getEncoder()->update();
}
void privateIntHandler3() {
  cmd3.getEncoder()->update();
}

ZOpticalSensor sensor0( P_IR[0].Pin.IRR,P_IR[0].Pin.IRE );
ZOpticalSensor sensor1( P_IR[1].Pin.IRR,P_IR[1].Pin.IRE );
ZOpticalSensor sensor2( P_IR[2].Pin.IRR,P_IR[2].Pin.IRE );
ZOpticalSensor sensor3( P_IR[3].Pin.IRR,P_IR[3].Pin.IRE );
ZOpticalSensor sensor4( P_IR[4].Pin.IRR,P_IR[4].Pin.IRE );
ZOpticalSensor sensor5( P_IR[5].Pin.IRR,P_IR[5].Pin.IRE );
ZOpticalSensor sensor6( P_IR[6].Pin.IRR,P_IR[6].Pin.IRE );
ZOpticalSensor sensor7( P_IR[7].Pin.IRR,P_IR[7].Pin.IRE );

void setupOpticalSensor()
{

analogReadResolution(12);
sensor0.setup(&nh,"/r1/sensor/0/");
 sensor1.setup(&nh,"/r1/sensor/1/");
 sensor2.setup(&nh,"/r1/sensor/2/");
 sensor3.setup(&nh,"/r1/sensor/3/");
 sensor4.setup(&nh,"/r1/sensor/4/");
 sensor5.setup(&nh,"/r1/sensor/5/");
 sensor6.setup(&nh,"/r1/sensor/6/");
 sensor7.setup(&nh,"/r1/sensor/7/");

}

void loopOpticalSensor()
{

 sensor0.loop();
 sensor1.loop();
 sensor2.loop();

 sensor3.loop();
 sensor4.loop();
 sensor5.loop();
 sensor6.loop();
 sensor7.loop();

   
 
}






#include <std_msgs/Int32.h>
std_msgs::Int32 temp_msg;
ros::Publisher pub_temp("/r1/captor/time_loop", &temp_msg);

#include <std_msgs/Int16.h>
std_msgs::Int16 usart_msg;
std_msgs::Int16 usarttx_msg;
ros::Publisher pub_usart("/r1/captor/usart/tx", &usart_msg);
ros::Publisher pub_usartrx("/r1/captor/usart/rx", &usarttx_msg);


std_msgs::Int16 debug1_msg;
std_msgs::Int16 debug2_msg;
std_msgs::Int16 debug3_msg;
ros::Publisher pub_debug1("/r1/captor/debug/1", &debug1_msg);
ros::Publisher pub_debug2("/r1/captor/debug/2", &debug2_msg);
ros::Publisher pub_debug3("/r1/captor/debug/3", &debug3_msg);


void setup_time()
{
nh.advertise(pub_temp);
nh.advertise(pub_usart);
nh.advertise(pub_usartrx);

nh.advertise(pub_debug1);
nh.advertise(pub_debug2);
nh.advertise(pub_debug3);
	
pinMode(LED_TOP, OUTPUT);
digitalWrite(LED_TOP, LOW);

}
 
 
 int timetime=0;// refresh rate
int time1=micros();//load
int dmaxros=0;//max load
char toggle=LOW;
void loop_time()
 {
int d=micros()-time1;
if (d>dmaxros)
dmaxros=d;
time1=micros();  
 
 if(micros()-timetime>100000)
 {
 toggle=(toggle==LOW)?HIGH:LOW;//toggle
      
digitalWrite(LED_TOP, toggle);



/*
  temp_msg.data = dmaxros;// an high value means saturation of cpu, it is use of cpu in 킪
  pub_temp.publish(&temp_msg);
  
   usart_msg.data = ROS_SERIAL.availableForWrite();// 0 means saturation of usart, 255 means low load on usart.
   pub_usart.publish(&usart_msg);
   usarttx_msg.data = ROS_SERIAL.available();// 0 means saturation of usart, 255 means low load on usart.
   pub_usartrx.publish(&usart_msg);
   timetime=micros();
   dmaxros=0;
   
   
    pub_debug1.publish(&debug1_msg);
     pub_debug2.publish(&debug2_msg);
      pub_debug3.publish(&debug3_msg);
 */
 }
  
  }











// the setup function runs once when you press reset or power the board
void setupCMD() {
  MySerial.print("setup CMD \r\n");
//  delay(500);
   cmd1.setPin(M1_CA, M1_CB, M1_MP, M1_MM,M1_EN);
 cmd2.setPin(M2_CA, M2_CB, M2_MP, M2_MM,M2_EN);
 cmd3.setPin(M3_CA, M3_CB, M3_MP, M3_MM,M3_EN);
  cmd1.setup();
  cmd2.setup();
  cmd3.setup();
  
  MySerial.print("setup CMD end \r\n");
//  delay(500);
  cmd1.getEncoder()->attachEncoderInt(privateIntHandler1);
  cmd2.getEncoder()->attachEncoderInt(privateIntHandler2);
cmd3.getEncoder()->attachEncoderInt(privateIntHandler3);

cmd3.setup(&nh,	M3_TOPIC_PWM,M3_TOPIC_SPEED);
cmd1.setup( &nh,	M1_TOPIC_PWM,M1_TOPIC_SPEED);
cmd2.setup(&nh,	M2_TOPIC_PWM,M2_TOPIC_SPEED);
#ifdef ENABLE_SPEED
 cmd1.getEncoder()->setup( &nh, M1_TOPIC_ENC, M1_TOPIC_ENC_SPEED);
 cmd2.getEncoder()->setup( &nh,	M2_TOPIC_ENC, M2_TOPIC_ENC_SPEED);
 cmd3.getEncoder()->setup( &nh,	M3_TOPIC_ENC, M3_TOPIC_ENC_SPEED);
#else
 cmd1.getEncoder()->setup( &nh, M1_TOPIC_ENC);
 cmd2.getEncoder()->setup( &nh,	M2_TOPIC_ENC);
 cmd3.getEncoder()->setup( &nh,	M3_TOPIC_ENC);
 #endif
 cmd1.setRefreshRateUs(20000);
 cmd2.setRefreshRateUs(20000);
 cmd3.setRefreshRateUs(20000);
 
 
  cmd4.setPin(M4_CA, M4_CB, M4_MP, M4_MM,M4_EN);
  cmd4.setup();
  cmd4.setup( &nh,	M4_TOPIC_PWM,M4_TOPIC_SPEED);
  
  
  
  
  cmd5.setPin(M5_CA, M5_CB, M5_MP, M5_MM,M5_EN);
  cmd5.setup();
  cmd5.setup( &nh,	M5_TOPIC_PWM,M5_TOPIC_SPEED);
  
  cmd6.setPin(M6_CA, M6_CB, M6_MP, M6_MM,M6_EN);
  cmd6.setup();
  cmd6.setup( &nh,	M6_TOPIC_PWM,M6_TOPIC_SPEED);
  
  cmd7.setPin(M7_CA, M7_CB, M7_MP, M7_MM,M7_EN);
  cmd7.setup();
  cmd7.setup( &nh,	M7_TOPIC_PWM,M7_TOPIC_SPEED);
  
}


bool setupMotorBoard() {
if(!( WireTest( MyWireMotor, 0x22) && WireTest( MyWireMotor, 0x42)))
return false;//skip if wire issue
MyWireMotor.begin();
	MyWireMotor.setClock(100000);
if (0)
{
MySerial.begin(57600);  //115200 //9600
	MySerial.println("Setup");

	volatile int ip = scan(MySerial, MyWireMotor);
	while (ip = scanNext(MySerial, MyWireMotor) != 0);
}

	motorBoard.begin(&MyWireMotor, 0x22, 0x42);

motorBoard.analogWriteResolution(12);
motorBoard.setPWMFreq(1600);
	setPinExtender(&motorBoard); // connect the board to arduino API.
	// define pin as output


//put defaul state : enabled low/low
	pinMode(motorBoard.getPin(PIN_MOTOR3_EN_0), OUTPUT);
	pinMode(motorBoard.getPin(PIN_MOTOR3_EN_1), OUTPUT);
	pinMode(motorBoard.getPin(PIN_MOTOR3_EN_2), OUTPUT);
	pinMode(motorBoard.getPin(PIN_MOTOR3_EN_3), OUTPUT);
	pinMode(motorBoard.getPin(PIN_MOTOR3_EN_4), OUTPUT);
	pinMode(motorBoard.getPin(PIN_MOTOR3_EN_5), OUTPUT);
	pinMode(motorBoard.getPin(PIN_MOTOR3_EN_6), OUTPUT);
	pinMode(motorBoard.getPin(PIN_MOTOR3_EN_7), OUTPUT);

	digitalWrite(motorBoard.getPin(PIN_MOTOR3_EN_0), LOW);
	digitalWrite(motorBoard.getPin(PIN_MOTOR3_EN_1), LOW);
	digitalWrite(motorBoard.getPin(PIN_MOTOR3_EN_2), LOW);
	digitalWrite(motorBoard.getPin(PIN_MOTOR3_EN_3), LOW);
	digitalWrite(motorBoard.getPin(PIN_MOTOR3_EN_4), LOW);
	digitalWrite(motorBoard.getPin(PIN_MOTOR3_EN_5), LOW);
	digitalWrite(motorBoard.getPin(PIN_MOTOR3_EN_6), LOW);
	digitalWrite(motorBoard.getPin(PIN_MOTOR3_EN_7), LOW);



	pinMode(motorBoard.getPin(PIN_MOTOR3_IO_0), OUTPUT);
	pinMode(motorBoard.getPin(PIN_MOTOR3_IO_1), OUTPUT);
	pinMode(motorBoard.getPin(PIN_MOTOR3_IO_2), OUTPUT);
	pinMode(motorBoard.getPin(PIN_MOTOR3_IO_3), OUTPUT);
	pinMode(motorBoard.getPin(PIN_MOTOR3_IO_4), OUTPUT);
	pinMode(motorBoard.getPin(PIN_MOTOR3_IO_5), OUTPUT);
	pinMode(motorBoard.getPin(PIN_MOTOR3_IO_6), OUTPUT);
	pinMode(motorBoard.getPin(PIN_MOTOR3_IO_7), OUTPUT);

	pinMode(motorBoard.getPin(PIN_MOTOR3_PWM_0), OUTPUT);
	pinMode(motorBoard.getPin(PIN_MOTOR3_PWM_1), OUTPUT);
	pinMode(motorBoard.getPin(PIN_MOTOR3_PWM_2), OUTPUT);
	pinMode(motorBoard.getPin(PIN_MOTOR3_PWM_3), OUTPUT);
	pinMode(motorBoard.getPin(PIN_MOTOR3_PWM_4), OUTPUT);
	pinMode(motorBoard.getPin(PIN_MOTOR3_PWM_5), OUTPUT);
	pinMode(motorBoard.getPin(PIN_MOTOR3_PWM_6), OUTPUT);
	pinMode(motorBoard.getPin(PIN_MOTOR3_PWM_7), OUTPUT);


	digitalWrite(motorBoard.getPin(PIN_MOTOR3_IO_0), HIGH);
	digitalWrite(motorBoard.getPin(PIN_MOTOR3_IO_1), HIGH);
	digitalWrite(motorBoard.getPin(PIN_MOTOR3_IO_2), HIGH);
	digitalWrite(motorBoard.getPin(PIN_MOTOR3_IO_3), HIGH);
	digitalWrite(motorBoard.getPin(PIN_MOTOR3_IO_4), HIGH);
	digitalWrite(motorBoard.getPin(PIN_MOTOR3_IO_5), HIGH);
	digitalWrite(motorBoard.getPin(PIN_MOTOR3_IO_6), HIGH);
	digitalWrite(motorBoard.getPin(PIN_MOTOR3_IO_7), HIGH);

	digitalWrite(motorBoard.getPin(PIN_MOTOR3_PWM_0), HIGH);
	digitalWrite(motorBoard.getPin(PIN_MOTOR3_PWM_1), HIGH);
	digitalWrite(motorBoard.getPin(PIN_MOTOR3_PWM_2), HIGH);
	digitalWrite(motorBoard.getPin(PIN_MOTOR3_PWM_3), HIGH);
	digitalWrite(motorBoard.getPin(PIN_MOTOR3_PWM_4), HIGH);
	digitalWrite(motorBoard.getPin(PIN_MOTOR3_PWM_5), HIGH);
	digitalWrite(motorBoard.getPin(PIN_MOTOR3_PWM_6), HIGH);
	digitalWrite(motorBoard.getPin(PIN_MOTOR3_PWM_7), HIGH);

//put defaul state : enabled high/high
	pinMode(motorBoard.getPin(PIN_MOTOR3_EN_0), OUTPUT);
	pinMode(motorBoard.getPin(PIN_MOTOR3_EN_1), OUTPUT);
	pinMode(motorBoard.getPin(PIN_MOTOR3_EN_2), OUTPUT);
	pinMode(motorBoard.getPin(PIN_MOTOR3_EN_3), OUTPUT);
	pinMode(motorBoard.getPin(PIN_MOTOR3_EN_4), OUTPUT);
	pinMode(motorBoard.getPin(PIN_MOTOR3_EN_5), OUTPUT);
	pinMode(motorBoard.getPin(PIN_MOTOR3_EN_6), OUTPUT);
	pinMode(motorBoard.getPin(PIN_MOTOR3_EN_7), OUTPUT);

	digitalWrite(motorBoard.getPin(PIN_MOTOR3_EN_0), HIGH);
	digitalWrite(motorBoard.getPin(PIN_MOTOR3_EN_1), HIGH);
	digitalWrite(motorBoard.getPin(PIN_MOTOR3_EN_2), HIGH);
	digitalWrite(motorBoard.getPin(PIN_MOTOR3_EN_3), HIGH);
	digitalWrite(motorBoard.getPin(PIN_MOTOR3_EN_4), HIGH);
	digitalWrite(motorBoard.getPin(PIN_MOTOR3_EN_5), HIGH);
	digitalWrite(motorBoard.getPin(PIN_MOTOR3_EN_6), HIGH);
	digitalWrite(motorBoard.getPin(PIN_MOTOR3_EN_7), HIGH);

/*
//===========TEST MOTOR ==========================
        digitalWrite(M1_EN, HIGH);
	digitalWrite(M2_EN, HIGH);
	digitalWrite(M3_EN, HIGH);
        
	digitalWrite((M1_MM), HIGH);
	digitalWrite((M1_MP), HIGH);

	digitalWrite((M2_MM), HIGH);
	digitalWrite((M2_MP), HIGH);

	digitalWrite((M3_MM), HIGH);
	digitalWrite((M3_MP), HIGH);

	digitalWrite((M4_MM), HIGH);
	digitalWrite((M4_MP), HIGH);
        digitalWrite(M4_EN, HIGH);*/
        
 return true;       
}
void loopDEBUGMOTOR();

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
  wdt_clr();
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
wdt_clr();
 WDT_on(  );

init_debug();
	pinMode(LED_BOTTOM, OUTPUT);
        digitalWrite(LED_BOTTOM, LOW);
	 // turn the LED on (HIGH is the voltage level)

        
/*
bip(BIP_FATAL);                  
bip(BIP_ERROR); 
bip(BIP_WARNING);
bip(BIP_LONG_OK); 
bip(BIP_MEDIUM_OK); 
bip(BIP_OK); 
bip(BIP_FAST_OK); 
*/

bool r=setupMotorBoard();


if (!r)
bip(BIP_FATAL); 
else
bip(BIP_LONG_OK); 

assert(r);

//loopDEBUGMOTOR();
SUPC->BODCORE.bit.ENABLE=0;
SUPC->BODCORE.bit.ACTION= SUPC_BODVDD_ACTION_RESET | SUPC_BODCORE_HYST |  SUPC_BODCORE_LEVEL(40);
SUPC->BODCORE.bit.ENABLE=1;
SUPC->BODVDD.bit.ENABLE=0;
SUPC->BODVDD.bit.ACTION= SUPC_BODVDD_ACTION_RESET | SUPC_BODVDD_HYST |  SUPC_BODVDD_LEVEL(7);
SUPC->BODVDD.bit.ENABLE=1;

nh.initNode(); 
   //wait until you are actually connected
   uint8_t status=LOW;
setup_time();
setupCMD();
setupOpticalSensor();
setupIntPriority();
/*
  pinMode(M5_MP, OUTPUT); digitalWrite(M5_MP, LOW); 
  pinMode(M5_MM, OUTPUT); digitalWrite(M5_MM, HIGH); 
  pinMode(M5_EN, OUTPUT); digitalWrite(M5_EN, HIGH); 

  pinMode(M6_MP, OUTPUT); digitalWrite(M6_MP, LOW); 
  pinMode(M6_MM, OUTPUT); digitalWrite(M6_MM, HIGH); 
  pinMode(M6_EN, OUTPUT); digitalWrite(M6_EN, HIGH); 

  pinMode(M7_MP, OUTPUT); digitalWrite(M7_MP, LOW); 
  pinMode(M7_MM, OUTPUT); digitalWrite(M7_MM, HIGH); 
  pinMode(M7_EN, OUTPUT); digitalWrite(M7_EN, HIGH); 

  pinMode(M4_MP, OUTPUT); digitalWrite(M4_MP, LOW); 
  pinMode(M4_MM, OUTPUT); digitalWrite(M4_MM, HIGH); 
  pinMode(M4_EN, OUTPUT); digitalWrite(M4_EN, HIGH); 

*/
wdt_clr();
bool test=false;
if (test)
  {
  cmd1.setPWMValue(0XFFFF);//vanne 1
  cmd2.setPWMValue(0XFFFF);//vanne 1
  cmd3.setPWMValue(0XFFFF);//vanne 1
  delay(200);wdt_clr();
  cmd1.stop();
  cmd2.stop();
  cmd3.stop();
  delay(200);wdt_clr();
  cmd1.setPWMValue(-0XFFFF);//vanne 1
  cmd2.setPWMValue(-0XFFFF);//vanne 1
  cmd3.setPWMValue(-0XFFFF);//vanne 1
  delay(200);wdt_clr();
  cmd1.stop();
  cmd2.stop();
  cmd3.stop();
  delay(500);wdt_clr();

/*while(1)
{*/
delay(500);wdt_clr();
cmd5.setPWMValue(-0XFFFF);//vanne 1
delay(500);wdt_clr();
cmd6.setPWMValue(-0XFFFF);//vanne 2
delay(500);wdt_clr();
cmd7.setPWMValue(-0XFFFF);//vanne 3
delay(500);wdt_clr();
cmd4.setPWMValue(-0XFFFF);//pompe
delay(500);wdt_clr();
cmd4.stop();delay(100);
cmd5.stop();delay(100);
cmd6.stop();delay(100);wdt_clr();
cmd7.stop();
delay(500);wdt_clr();
//}
}

bip(BIP_MEDIUM_OK); 
    while (!nh.connected())
    {    
      nh.spinOnce();
      delay(50);
      status=(status==LOW)?HIGH:LOW;//toggle
      digitalWrite(LED_BOTTOM, status);
      wdt_clr();
    }

if( !MyWireMotor.testLine( ))
{
  nh.logerror( "Captor : WireMotor  not working" );
}

if(!r)
{
  nh.logerror( "Captor : Motor3 Board missing" );
}
else
  nh.loginfo("Captor i2c OK !");

cmd1.stop();
	cmd2.stop();
	cmd3.stop();
        /*
         delay(1000);
    cmd1.setPWMValue(1000);
	cmd2.setPWMValue(1000);
	cmd3.setPWMValue(1000);
     delay(1000);
    
    	cmd1.setPoint(2000);
	cmd2.setPoint(2000);
	cmd3.setPoint(2000);
        int i=0;
 	while(i<1000)
	{
          i++;
          cmd1.loop();
          cmd2.loop();
          cmd3.loop();

          delay(10);
        }

        cmd1.setPoint(-2000);
	cmd2.setPoint(-2000);
	cmd3.setPoint(-2000);
        i=0;
 	while(i<100)
	{
            i++;
            cmd1.loop();
            cmd2.loop();
            cmd3.loop();

            delay(10);
        }*/
    	cmd1.stop();
	cmd2.stop();
	cmd3.stop();

 digitalWrite(LED_BOTTOM, HIGH);
bip(BIP_OK);   
nh.loginfo("Captor Setup DoneK !");
}

  int i=0;

void loopcmd()
{

 cmd1.loop();//<110탎
  cmd2.loop();//<110탎
  cmd3.loop();//<110탎
  cmd4.loop();//<110탎

 cmd5.loop();//<110탎
 cmd6.loop();//<110탎
 cmd7.loop();//<110탎
  
}
void loop()
{

 loopcmd();
 
  loopOpticalSensor();//<typ 1102 / max 9160탎


   /*
 if (i++%50==0)//5Hz
   nh.loginfo("loop()");//<240탎 
 */
    loop_time();
    
    delay(5);

 int result=nh.spinOnce();//<20탎
 //PV assert(result==ros::SPIN_OK);
 // if((result==ros::SPIN_OK))
 if(nh.connected())
    wdt_clr();
    
 //   delay(1);//less than 500Hz less ros serial server fail //for python version 50Hz

}




  
void loopDEBUGMOTOR() {
  // Drive each pin in a 'wave'
 
#define motor motorBoard  
/* this don't work :
      for (int i=0;i<16;i++)
  digitalWrite(motor.getPin(PIN_MOTOR3_EN_0+i), HIGH);
  put this :
  */
     pinMode(motor.getPin(PIN_MOTOR3_EN_0), OUTPUT);
     pinMode(motor.getPin(PIN_MOTOR3_EN_1), OUTPUT);
     pinMode(motor.getPin(PIN_MOTOR3_EN_2), OUTPUT);
     pinMode(motor.getPin(PIN_MOTOR3_EN_3), OUTPUT);
     pinMode(motor.getPin(PIN_MOTOR3_EN_4), OUTPUT);
     pinMode(motor.getPin(PIN_MOTOR3_EN_5), OUTPUT);
     pinMode(motor.getPin(PIN_MOTOR3_EN_6), OUTPUT);
     pinMode(motor.getPin(PIN_MOTOR3_EN_7), OUTPUT);
     
     pinMode(motor.getPin(PIN_MOTOR3_IO_0), OUTPUT);
     pinMode(motor.getPin(PIN_MOTOR3_IO_1), OUTPUT);
     pinMode(motor.getPin(PIN_MOTOR3_IO_2), OUTPUT);
     pinMode(motor.getPin(PIN_MOTOR3_IO_3), OUTPUT);
     pinMode(motor.getPin(PIN_MOTOR3_IO_4), OUTPUT);
     pinMode(motor.getPin(PIN_MOTOR3_IO_5), OUTPUT);
     pinMode(motor.getPin(PIN_MOTOR3_IO_6), OUTPUT);
     pinMode(motor.getPin(PIN_MOTOR3_IO_7), OUTPUT);
     
     pinMode(motor.getPin(PIN_MOTOR3_PWM_0), OUTPUT);
     pinMode(motor.getPin(PIN_MOTOR3_PWM_1), OUTPUT);
     pinMode(motor.getPin(PIN_MOTOR3_PWM_2), OUTPUT);
     pinMode(motor.getPin(PIN_MOTOR3_PWM_3), OUTPUT);
     pinMode(motor.getPin(PIN_MOTOR3_PWM_4), OUTPUT);
     pinMode(motor.getPin(PIN_MOTOR3_PWM_5), OUTPUT);
     pinMode(motor.getPin(PIN_MOTOR3_PWM_6), OUTPUT);
     pinMode(motor.getPin(PIN_MOTOR3_PWM_7), OUTPUT);
     /*
    digitalWrite(motor.getPin(PIN_MOTOR3_EN_0), LOW);
    digitalWrite(motor.getPin(PIN_MOTOR3_EN_1), LOW);
    digitalWrite(motor.getPin(PIN_MOTOR3_EN_2), LOW);
    digitalWrite(motor.getPin(PIN_MOTOR3_EN_3), LOW);
    digitalWrite(motor.getPin(PIN_MOTOR3_EN_4), LOW);
    digitalWrite(motor.getPin(PIN_MOTOR3_EN_5), LOW);
    digitalWrite(motor.getPin(PIN_MOTOR3_EN_6), LOW);
    digitalWrite(motor.getPin(PIN_MOTOR3_EN_7), LOW);
    
    
    
       for (int i=0;i<16;i++)
    digitalWrite(motor.getPin(MCP23017_ADDR_BASE<<16)+i, LOW);
   
    for (int i=0;i<16;i++)
    digitalWrite(motor.getPin(MCP23017_ADDR_BASE<<16)+i, HIGH);
  








    delay(1000);  
 
*/
  
    digitalWrite(motor.getPin(PIN_MOTOR3_IO_0), LOW);//4
    digitalWrite(motor.getPin(PIN_MOTOR3_IO_1), LOW);//5
    digitalWrite(motor.getPin(PIN_MOTOR3_IO_2), LOW);//7
    digitalWrite(motor.getPin(PIN_MOTOR3_IO_3), LOW);//6
    digitalWrite(motor.getPin(PIN_MOTOR3_IO_4), LOW);
    digitalWrite(motor.getPin(PIN_MOTOR3_IO_5), LOW);
    digitalWrite(motor.getPin(PIN_MOTOR3_IO_6), LOW);
    digitalWrite(motor.getPin(PIN_MOTOR3_IO_7), LOW);//M3

    digitalWrite(motor.getPin(PIN_MOTOR3_PWM_0), LOW);
    digitalWrite(motor.getPin(PIN_MOTOR3_PWM_1), LOW);
    digitalWrite(motor.getPin(PIN_MOTOR3_PWM_2), LOW);
    digitalWrite(motor.getPin(PIN_MOTOR3_PWM_3), LOW);
    digitalWrite(motor.getPin(PIN_MOTOR3_PWM_4), LOW);
    digitalWrite(motor.getPin(PIN_MOTOR3_PWM_5), LOW);
    digitalWrite(motor.getPin(PIN_MOTOR3_PWM_6), LOW);
    digitalWrite(motor.getPin(PIN_MOTOR3_PWM_7), LOW);
    
    digitalWrite(motor.getPin(PIN_MOTOR3_EN_0), HIGH);
    digitalWrite(motor.getPin(PIN_MOTOR3_EN_1), HIGH);
    digitalWrite(motor.getPin(PIN_MOTOR3_EN_2), HIGH);
    digitalWrite(motor.getPin(PIN_MOTOR3_EN_3), HIGH);
    digitalWrite(motor.getPin(PIN_MOTOR3_EN_4), HIGH);
    
    digitalWrite(motor.getPin(PIN_MOTOR3_EN_5), HIGH);
    digitalWrite(motor.getPin(PIN_MOTOR3_EN_6), HIGH);
    digitalWrite(motor.getPin(PIN_MOTOR3_EN_7), HIGH);
   
/*
    digitalWrite(motor.getPin(PIN_MOTOR3_PWM_0), HIGH);
    digitalWrite(motor.getPin(PIN_MOTOR3_PWM_1), HIGH);//c
    digitalWrite(motor.getPin(PIN_MOTOR3_PWM_2), HIGH);//b
    digitalWrite(motor.getPin(PIN_MOTOR3_PWM_3), HIGH);
    digitalWrite(motor.getPin(PIN_MOTOR3_PWM_4), HIGH);
    digitalWrite(motor.getPin(PIN_MOTOR3_PWM_5), HIGH);
    digitalWrite(motor.getPin(PIN_MOTOR3_PWM_6), HIGH);
    digitalWrite(motor.getPin(PIN_MOTOR3_PWM_7), HIGH);//M3



    digitalWrite(motor.getPin(PIN_MOTOR3_EN_0), LOW);
    digitalWrite(motor.getPin(PIN_MOTOR3_EN_1), LOW);
    digitalWrite(motor.getPin(PIN_MOTOR3_EN_2), LOW);
    digitalWrite(motor.getPin(PIN_MOTOR3_EN_3), LOW);
    digitalWrite(motor.getPin(PIN_MOTOR3_EN_5), LOW);
    digitalWrite(motor.getPin(PIN_MOTOR3_EN_6), LOW);
    digitalWrite(motor.getPin(PIN_MOTOR3_EN_7), LOW);//M3
    
    
    digitalWrite(motor.getPin(PIN_MOTOR3_IO_4), HIGH);
   
    delay(1000);  
    digitalWrite(motor.getPin(PIN_MOTOR3_IO_0), HIGH);
    digitalWrite(motor.getPin(PIN_MOTOR3_IO_1), HIGH);
    digitalWrite(motor.getPin(PIN_MOTOR3_IO_2), HIGH);
    digitalWrite(motor.getPin(PIN_MOTOR3_IO_3), HIGH);
    digitalWrite(motor.getPin(PIN_MOTOR3_IO_5), HIGH);
    digitalWrite(motor.getPin(PIN_MOTOR3_IO_6), HIGH);
    digitalWrite(motor.getPin(PIN_MOTOR3_IO_7), HIGH);

    digitalWrite(motor.getPin(PIN_MOTOR3_PWM_0), HIGH);
    digitalWrite(motor.getPin(PIN_MOTOR3_PWM_1), HIGH);
    digitalWrite(motor.getPin(PIN_MOTOR3_PWM_2), HIGH);
    digitalWrite(motor.getPin(PIN_MOTOR3_PWM_3), HIGH);
    digitalWrite(motor.getPin(PIN_MOTOR3_PWM_4), HIGH);
    digitalWrite(motor.getPin(PIN_MOTOR3_PWM_5), HIGH);
    digitalWrite(motor.getPin(PIN_MOTOR3_PWM_6), HIGH);
    digitalWrite(motor.getPin(PIN_MOTOR3_PWM_7), HIGH);
    delay(1000);
    
    analogWrite(motor.getPin(PIN_MOTOR3_PWM_0), 200);
    analogWrite(motor.getPin(PIN_MOTOR3_PWM_1), 500);
    analogWrite(motor.getPin(PIN_MOTOR3_PWM_2), 1000);
    analogWrite(motor.getPin(PIN_MOTOR3_PWM_3), 1500);
    analogWrite(motor.getPin(PIN_MOTOR3_PWM_4), 2000);
    analogWrite(motor.getPin(PIN_MOTOR3_PWM_5), 2500);
    analogWrite(motor.getPin(PIN_MOTOR3_PWM_6), 3000);
    analogWrite(motor.getPin(PIN_MOTOR3_PWM_7), 3500);  
  */  

}
 