/*
//EXAMPLE OF SERVO WITH ROS.
// do
// roscore &
// for raspberry pi : rosrun rosserial_python serial_node.py /dev/ttyUSB0 &
// or for windows on COM4 : 
//        sudo chmod 666 /dev/ttyS4 if COM4
//        sudo chmod 666 /dev/ttyS3 if COM3
//        rosrun rosserial_python serial_node.py /dev/ttyS4 & 
//        rosrun rosserial_python serial_node.py /dev/ttyS3   _baud:=57600 & 
rosrun rosserial_python serial_node.py /dev/ttyS23   _baud:=57600 &
rosrun rosserial_python serial_node.py /dev/ttyAMA0   _baud:=115200 &
rostopic list
rostopic info /servo/F
rostopic pub /r1/servo/F std_msgs/UInt16 --once -- 80

// rostopic pub motor2/A std_msgs/Int16 --once    10 
// rostopic pub motor2/B std_msgs/Int16 --once    180
// rostopic pub motor2/A std_msgs/Int16 --once -- -180
// rostopic pub motor2/B std_msgs/Int16 --once     10 

rostopic pub /servo/F std_msgs/UInt16 --once     220
rostopic pub /servo/E std_msgs/UInt16 --once     220
// NOTE : WINDOWS 10 with ubuntu 16.04 as subsystem, after ros install do this :
//      sudo apt-get install ros-kinetic-rosserial-windows
//     sudo apt-get install ros-kinetic-rosserial-server

// validated on Arduino Mega
// validated on Pilo
*/
#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <Arduino.h> 
#include <Zmotor2.h> 
#include <assert.h>
#include <ZServoPCA9685.h> 
#include <ZPCA9685.h>
#include <ZSwitchs.h>

#if defined(BOARD_ID_Pilo)
#include <Wire.h>
#include <SPI.h>
#include <variant.h>

#include <WireUtility.h>


#define ROS_SERIAL (P_COM3.serial2)
#define ROS_BAUDRATE 57600
//#define ROS_BAUDRATE 9600

//#define ROS_BAUDRATE 1000000
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

/*
#include <std_msgs/String.h>
std_msgs::String str_msg;
ros::Publisher rosdebug("debug/Pilo", &str_msg);
char debug_msg[13] = "hello world!";
*/  

Zmotor2 card_motor2 = Zmotor2();


ZPCA9685 card_servos = ZPCA9685();
/*
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
*/
ZServoPCA9685 servo_cmdC= ZServoPCA9685(&card_servos);
ZServoPCA9685 servo_cmdD= ZServoPCA9685(&card_servos);
ZServoPCA9685 servo_cmdE= ZServoPCA9685(&card_servos);
ZServoPCA9685 servo_cmdF= ZServoPCA9685(&card_servos);
/*
ZServoPCA9685 servo_cmd[16]={servo_cmd0,servo_cmd1,servo_cmd2,servo_cmd3,
servo_cmd4,servo_cmd5,servo_cmd6,servo_cmd7,
servo_cmd8,servo_cmd9,servo_cmdA,servo_cmdB,
servo_cmdC,servo_cmdD,servo_cmdE,servo_cmdF 
};*/
int pin=9;// digital  9

#define MySerial P_COM3.serial2 //usb
#define WireMotor2 (P_COM0_BIS.wire)
#define WirePCA9685 (P_COM0_BIS.wire)
void 	test_servo()
{
/*
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
*/
}
#define I2CADDR_MOTOR2_1 0x21
#define I2CADDR_MOTOR2_2 0x41
#define I2CADDR_SERVO    0x43



void setupIntPriority()
{

/// 0 highest priority
// (1<<__NVIC_PRIO_BITS) - 2) lowest priority


  NVIC_SetPriority (EIC_IRQn, NVIC_PRIO_MED);  /* set Priority low */
  
  NVIC_SetPriority (SERCOM0_IRQn, NVIC_PRIO_LOWEST);  // set Priority high 
  NVIC_SetPriority (SERCOM3_IRQn,NVIC_PRIO_HIGH);  // set Priority high 
  NVIC_SetPriority (SERCOM2_IRQn, NVIC_PRIO_HIGHEST);  //set Priority low 
  NVIC_SetPriority (SERCOM5_IRQn, NVIC_PRIO_LOWEST); // set Priority low 
  
  
  
}


///////////////////////////////////////////////// LCD ///////////////////////////////

/*
#define CLK P_COM1.Pin.PIN16
#define DIO P_COM1.Pin.PIN18  
#define GND P_COM1.Pin.PIN04
#define VCC P_COM1.Pin.PIN02


int num, bsend, i;
uint8_t WriteOK = 0xFB;
int incomingByte = 0;
byte buff_header[5];
byte data[250];
LED4x7_t led;


TM1637Display display(CLK, DIO);
*/

#define MyWireLcd P_COM1.wire
#include <LiquidCrystal_I2C.h>
#include <std_msgs/String.h>
#define LCD_ADDR 63
LiquidCrystal_I2C lcd(MyWireLcd,LCD_ADDR,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display

 
 void message1( const std_msgs::String& toggle_msg){
 
  lcd.setCursor(0,0);
  lcd.print(toggle_msg.data);    
   }
   
void message2( const std_msgs::String& toggle_msg){
 
  lcd.setCursor(0,1);
  lcd.print(toggle_msg.data);    
   }
   
 ros::Subscriber<std_msgs::String> msg1("/r1/lcd/line1", &message1 );
 ros::Subscriber<std_msgs::String> msg2("/r1/lcd/line2", &message2 );
 
void setupLcd()
{
  lcd.init();                      // initialize the lcd 
  // Print a message to the LCD.
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("  Hello, world! ");
  lcd.setCursor(0,1);
  lcd.print(" StartDust 3000 ");
  lcd.setCursor(0,0);
  nh.subscribe(msg1);
  nh.subscribe(msg2);
}

///////////////////////////////////////////////// LCD ///////////////////////////////


#include <std_msgs/Int32.h>
std_msgs::Int32 temp_msg;
ros::Publisher pub_temp("time", &temp_msg);

#include <std_msgs/Int16.h>
std_msgs::Int16 usart_msg;
std_msgs::Int16 usarttx_msg;
ros::Publisher pub_usart("usart/tx", &usart_msg);
ros::Publisher pub_usartrx("usart/rx", &usarttx_msg);


std_msgs::Int16 debug1_msg;
std_msgs::Int16 debug2_msg;
std_msgs::Int16 vbat_msg;
ros::Publisher pub_debug1("debug/1", &debug1_msg);
ros::Publisher pub_debug2("debug/2", &debug2_msg);
ros::Publisher pub_vbat("/r1/pilo/VbatmV", &vbat_msg);

void setup_time()
{

nh.advertise(pub_temp);
nh.advertise(pub_usart);
nh.advertise(pub_usartrx);

nh.advertise(pub_debug1);
nh.advertise(pub_debug2);
nh.advertise(pub_vbat);

pinMode(LED_TOP, OUTPUT);
digitalWrite(LED_TOP, HIGH);

}
 int timetime=0;// refresh rate
int time1=0;//load
int dmaxros=0;//max load
char toggle=LOW;
int n=0;
void loop_time()
 {
int d=micros()-time1;
if (d>dmaxros)
dmaxros=d;
time1=micros();  
 
 if(micros()-timetime>100000)
 {
 toggle=(toggle==LOW)?HIGH:LOW;//toggle
   if (n++%100==0) 
   nh.loginfo("loop()");    
digitalWrite(LED_TOP, toggle);
  temp_msg.data = dmaxros;// an high value means saturation of cpu, it is use of cpu in µS
  pub_temp.publish(&temp_msg);
  
   usart_msg.data = ROS_SERIAL.availableForWrite();// 0 means saturation of usart, 255 means low load on usart.
   pub_usart.publish(&usart_msg);
   usarttx_msg.data = ROS_SERIAL.available();// 0 means saturation of usart, 255 means low load on usart.
   pub_usartrx.publish(&usart_msg);
   timetime=micros();
   dmaxros=0;
   
   
    pub_debug1.publish(&debug1_msg);
     pub_debug2.publish(&debug2_msg);
     
     
     
     float Vbat=0;
for(int i=0;i<5;i++)
Vbat+=analogRead(P_ANA1.Pin.IO4);
Vbat=Vbat*5.0/4096/5*11.5;//vbat
vbat_msg.data=Vbat*1000;

      pub_vbat.publish(&vbat_msg);
 }
  
  }

ZSwitchs switches;


void setup() {




if (1)// I2C LCD
{
MySerial.begin(57600);  //115200 //9600
//	MySerial.println("MyWireLcd scan");

	volatile int ip = scan(MySerial, MyWireLcd);
	while (ip = scanNext(MySerial, MyWireLcd) != 0);
        
        
}

if (MyWireLcd.testLine( ) && WireTest( MyWireLcd, LCD_ADDR))// if LCD present use it.
{
MyWireLcd.begin();
MyWireLcd.setClock(100000);
  setupLcd();
}

	pinMode(LED_BOTTOM, OUTPUT);
        digitalWrite(LED_BOTTOM, HIGH);

setupIntPriority();

boolean i2cok=true;

WireMotor2.begin();
	WireMotor2.setClock(10000);

//0x21 41 43 70
if (0)// I2C SERVO/MOTOR
{
MySerial.begin(57600);  //115200 //9600
	MySerial.println("Setup");

	volatile int ip = scan(MySerial, WireMotor2);
	while (ip = scanNext(MySerial, WireMotor2) != 0);
        ip=ip;
}

  if( !WireMotor2.testLine( ))
{
     
    lcd.setCursor(0,1);
    lcd.print("I2C WireMotor2 FAIL");
 assert(1==0);
}

if(!( WireTest( WireMotor2, I2CADDR_MOTOR2_2) && WireTest( WireMotor2, I2CADDR_SERVO) && WireTest( WireMotor2, I2CADDR_MOTOR2_2)))
{
lcd.init();
  // Print a message to the LCD.
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("  I2C SERVO FAIL  ");
i2cok=false;
wireResetAllDevices(WireMotor2);
assert(1==0);


};//skip if wire issue
   
wireResetAllDevices(WireMotor2);

    //card_motor2.begin(&Wire,0x43);
	card_servos.begin(&(WirePCA9685),I2CADDR_SERVO);
    card_motor2.begin(&(WireMotor2),I2CADDR_MOTOR2_1,I2CADDR_MOTOR2_2);
        
  nh.initNode(); 

  if(i2cok  )
  {
    lcd.setCursor(0,0);
    lcd.print("  I2C SERVO OK  ");
  }
  else
  {
    lcd.setCursor(0,0);
    lcd.print("  I2C SERVO FAIL  ");
  }
  

if (!card_servos.test())
        {
         lcd.setCursor(0,0);
    lcd.print("     SERVO FAIL  ");
     assert(1==0);
        }
 // nh.advertise(rosdebug);
setup_time();

switches.setup(&nh,"/r1/pilo/switches");//Uint32
switches.attach(P_ANA1.Pin.IO0);//bit 0 : bleu/noir_rouge : ventouse gauche enforcé
switches.attach(P_ANA1.Pin.IO1);//bit 1 : noir_bleu : ventouse centre enforcé
switches.attach(P_ANA1.Pin.IO2);//bit 2 : noir_vert : ventouse droite enforcé
switches.attach(P_ANA1.Pin.IO3);//NC
switches.attach(-1);//bit 4 : NC (blanc? pond div Vbat)
switches.attach(P_ANA1.Pin.IO5);//bit 5 : bleu? = gnd_power
switches.attach(P_ANA1.Pin.IO6);//bit 6 : vert(gnd_bat): bouton arret d'urgence no percuté(robot alimenté)
switches.attach(P_ANA1.Pin.IO7);//bit 7 : jaune: tirette de demarrage presente


switches.attach(P_COM3.Pin.IO0);//bit 9  : noir/bleu_rouge: palet gauche present
switches.attach(P_COM3.Pin.IO1);//bit 10 : fil noir_bleu: palet centre present
switches.attach(P_COM3.Pin.IO5);//bit 11 : fil noir_vert: palet droite present
analogReadResolution(12);
/*
 rostopic echo /r1/pilo/switches
data: 1992
data: 1864
data: 1992
data: 1996

*/
 


  card_motor2.setup(&nh,"/r1/motor2/0",0);
  card_motor2.setup(&nh,"/r1/motor2/2",2);
  card_motor2.setup(&nh,"/r1/motor2/4",4);
  card_motor2.setup(&nh,"/r1/motor2/6",6);
  card_motor2.setup(&nh,"/r1/motor2/8",8);
  card_motor2.setup(&nh,"/r1/motor2/A",10);
  card_motor2.setup(&nh,"/r1/motor2/C",12);
  card_motor2.setup(&nh,"/r1/motor2/E",14);

/*
  servo_cmd0.setup(&nh,"/r1/servo/0",0);
  servo_cmd1.setup(&nh,"/r1/servo/1",1);
  servo_cmd2.setup(&nh,"/r1/servo/2",2);
  servo_cmd3.setup(&nh,"/r1/servo/3",3);
  servo_cmd4.setup(&nh,"/r1/servo/4",4);
  servo_cmd5.setup(&nh,"/r1/servo/5",5);
  servo_cmd6.setup(&nh,"/r1/servo/6",6);
  servo_cmd7.setup(&nh,"/r1/servo/7",7);
  servo_cmd8.setup(&nh,"/r1/servo/8",8);
  servo_cmd9.setup(&nh,"/r1/servo/9",9);
  servo_cmdA.setup(&nh,"/r1/servo/A",10);
  servo_cmdB.setup(&nh,"/r1/servo/B",11);
*/
  servo_cmdC.setup(&nh,"/r1/servo/C",12);
  servo_cmdD.setup(&nh,"/r1/servo/D",13);
  servo_cmdE.setup(&nh,"/r1/servo/E",14);
  servo_cmdF.setup(&nh,"/r1/servo/F",15);
 // test_servo();
  nh.logdebug("Debug Statement");
  nh.logdebug("Debug Statement");

card_servos.begin(&(WirePCA9685),I2CADDR_SERVO);
 
card_servos.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
//servo_cmdE.setPulseRange(  SERVO_MIN_PULSE_WIDTH, (SERVO_MAX_PULSE_WIDTH*6)/5) ;
// test servo :
    servo_cmdE.write(90);
    servo_cmdF.write(90);
    delay(800);
    servo_cmdE.write(20);
    delay(800);
    servo_cmdE.write(110);delay(200);
    servo_cmdE.write(120);delay(200);
    servo_cmdE.write(130);delay(200);
    servo_cmdE.write(140);delay(200);
    servo_cmdE.write(150);delay(200); 
    servo_cmdE.write(160);delay(200);
    servo_cmdE.write(170);delay(200);
    servo_cmdE.write(180);delay(200);
    
   
    
    
    servo_cmdE.write(90);
    
    delay(600);
    servo_cmdE.write(90);
    delay(600);
    
    servo_cmdF.write(10);
    
    delay(800);
   servo_cmdF.write(0);delay(200);
    servo_cmdF.write(10);delay(200);
    servo_cmdF.write(20);delay(200);
    servo_cmdF.write(30);delay(200);
    servo_cmdF.write(40);delay(200);
    servo_cmdF.write(50);delay(200);
    servo_cmdF.write(60);delay(200);
    servo_cmdF.write(70);delay(200);
    servo_cmdF.write(80);delay(200);
    servo_cmdF.write(90);delay(200);
    servo_cmdF.write(100);delay(200);
    servo_cmdF.write(110);delay(200);
    servo_cmdF.write(120);delay(200);
    servo_cmdF.write(130);delay(200);
    servo_cmdF.write(140);delay(200);
    servo_cmdF.write(150);delay(200);
    servo_cmdF.write(160);delay(200);
    servo_cmdF.write(170);delay(200);
    servo_cmdF.write(180);delay(200);
   
    servo_cmdF.write(30);delay(200);
    servo_cmdE.write(180);delay(200);
    
   uint8_t status=LOW;


servo_cmdE.write(90);servo_cmdE.write(90);
     lcd.setCursor(0,0);
    lcd.print("    WAIT ROS   ");
    
/*
if (!card_motor2.test())
        {nh.logerror( "Pilo : card_motor2  not working" );
        
            lcd.setCursor(0,1);
     assert(1==0);
    }
    */
   //wait until you are actually connected
    while (!nh.connected())
    {
      nh.spinOnce();
      delay(50);
          status=(status==LOW)?HIGH:LOW;//toggle
      digitalWrite(LED_BOTTOM, status);
   //    nh.loginfo( "Pilo : connecting" );
  
    }
    
    lcd.setCursor(0,0);
    lcd.print("    ROS OK    ");
    
    
if( !WireMotor2.testLine( ))
{
  nh.logerror( "Pilo : WireMotor2  not working" );
     
    lcd.setCursor(0,0);
    lcd.print("  I2C SERVO FAIL  ");
 assert(1==0);
}
/* to late to do check, ros has taken the hand on IT
if (!card_servos.test())
        {
        nh.logerror( "Pilo : card_servos  not working" );
         lcd.setCursor(0,0);
    lcd.print("     SERVO FAIL  ");
     assert(1==0);
        }
if (!card_motor2.test())
        {nh.logerror( "Pilo : card_motor2  not working" );
        
            lcd.setCursor(0,0);
    lcd.print("     MOTOR FAIL  ");
     assert(1==0);
    }
*/


/*
card_servos.begin(&(WirePCA9685),I2CADDR_SERVO);
 
card_servos.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

// test servo :
    servo_cmdE.write(90);
    servo_cmdF.write(90);
    delay(500);
    servo_cmdE.write(10);
    delay(500);
    servo_cmdE.write(150);
    delay(500);
    servo_cmdE.write(90);
    delay(500);
    
    servo_cmdF.write(10);
    delay(500);
    servo_cmdF.write(150);
    delay(500);
    servo_cmdF.write(90);


*/
    nh.logdebug("Debug Statement");
    nh.loginfo("Program info");
    nh.logwarn("Warnings.");
    nh.logerror("Errors..");
    nh.logfatal("Fatalities!");

 digitalWrite(LED_BOTTOM, HIGH);
/*
str_msg.data = "setup end";
rosdebug.publish( &str_msg );*/
 lcd.print("    READY    ");
}

void loopservos() {
  
/*  servo_cmd0.loop();
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
*/
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
  //  nh.loginfo("loop()");
   delay(15);
  loop_time();
  
  switches.loop();



}
