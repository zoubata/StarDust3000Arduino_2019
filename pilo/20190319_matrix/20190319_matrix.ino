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
rostopic list
rostopic info /servo/F
rostopic pub servo/F std_msgs/UInt16 --once -- 80

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
// testshapes demo for RGBmatrixPanel library.
// Demonstrates the drawing abilities of the RGBmatrixPanel library.
// For 32x64 RGB LED matrix.

// WILL NOT FIT on ARDUINO UNO -- requires a Mega, M0 or M4 board

#include <RGBmatrixPanel.h>

// Most of the signal pins are configurable, but the CLK pin has some
// special constraints.  On 8-bit AVR boards it must be on PORTB...
// Pin 8 works on the Arduino Uno & compatibles (e.g. Adafruit Metro),
// Pin 11 works on the Arduino Mega.  On 32-bit SAMD boards it must be
// on the same PORT as the RGB data pins (D2-D7)...
// Pin 8 works on the Adafruit Metro M0 or Arduino Zero,
// Pin A4 works on the Adafruit Metro M4 (if using the Adafruit RGB
// Matrix Shield, cut trace between CLK pads and run a wire to A4).

#define CLK  P_COM0_BIS.Pin.PIN10 //8   // USE THIS ON ARDUINO UNO, ADAFRUIT METRO M0, etc.
//#define CLK A4 // USE THIS ON METRO M4 (not M0)
//#define CLK 11 // USE THIS ON ARDUINO MEGA
#define OE   P_COM0_BIS.Pin.PIN06 //9
#define LAT P_COM0_BIS.Pin.PIN05 //10
#define A   P_COM0_BIS.Pin.PIN08 //A0
#define B   P_COM0_BIS.Pin.PIN09 //A1
#define C   P_COM0_BIS.Pin.PIN07 //A2
#define D   P_COM1.Pin.PIN06 //A3

  static const uint8_t defaultrgbpins[] = { 
    P_COM1.Pin.PIN10,
    P_COM1.Pin.PIN09,
    P_COM5.Pin.PIN08,
    P_COM5.Pin.PIN06,
    P_COM5.Pin.PIN05,
    P_COM5.Pin.PIN07
    };
  
  
    /*
    
    #define CLK  P_COM0_BIS.Pin.PIN08 //8   // USE THIS ON ARDUINO UNO, ADAFRUIT METRO M0, etc.
//#define CLK A4 // USE THIS ON METRO M4 (not M0)
//#define CLK 11 // USE THIS ON ARDUINO MEGA
#define OE   P_COM0.Pin.PIN06 //9
#define LAT P_COM0.Pin.PIN05 //10
#define A   P_COM0.Pin.PIN10 //A0
#define B   P_COM0.Pin.PIN09 //A1
#define C   P_COM0.Pin.PIN07 //A2
#define D   P_COM1.Pin.PIN06 //A3
  static const uint8_t defaultrgbpins[] = { 
    P_COM3.Pin.PIN10,
    P_COM3.Pin.PIN09,
    P_COM3.Pin.PIN08,
    P_COM3.Pin.PIN06,
    P_COM3.Pin.PIN05,
    P_COM3.Pin.PIN07};
    */
  #define DELAYTP -1
  
RGBmatrixPanel matrix(A, B, C, D, CLK, LAT, OE, false, 64);

// Input a value 0 to 24 to get a color value.
// The colours are a transition r - g - b - back to r.
uint16_t Wheel(byte WheelPos) {
  if(WheelPos < 8) {
   return matrix.Color333(7 - WheelPos, WheelPos, 0);
  } else if(WheelPos < 16) {
   WheelPos -= 8;
   return matrix.Color333(0, 7-WheelPos, WheelPos);
  } else {
   WheelPos -= 16;
   return matrix.Color333(0, WheelPos, 7 - WheelPos);
  }
}


    void writeD(int data)
    {    
     digitalWrite(A ,(((data>>0) & 1)==1)?HIGH:LOW);
     digitalWrite(B ,(((data>>1) & 1)==1)?HIGH:LOW);
     digitalWrite(C ,(((data>>2) & 1)==1)?HIGH:LOW);
     digitalWrite(D ,(((data>>3) & 1)==1)?HIGH:LOW);
     //digitalWrite(E ,(((data>>4) & 1)==1)?HIGH:LOW);
    }
    
    void writeC(int data)
    {    
  for(int i=0;i<6;i++)
     digitalWrite(defaultrgbpins[i] ,(((data>>i) & 1)==1)?HIGH:LOW);
    }
    
    
    
    void clock(int pin,int count)
    {
        volatile int t=0;

        for(int i=0;i<count;i++)
        {
              digitalWrite(pin ,LOW);
              for(t=0;t<DELAYTP;t++);
              digitalWrite(pin ,HIGH);
              for(t=0;t<DELAYTP;t++);
        }
        digitalWrite(pin ,LOW);
  }
  void clockN(int pin,int count)
    {
        volatile int t=0;

        for(int i=0;i<count;i++)
        {
              digitalWrite(pin ,HIGH);
              for(t=0;t<DELAYTP;t++);
              digitalWrite(pin ,LOW);
              for(t=0;t<DELAYTP;t++);
        }
        digitalWrite(pin ,HIGH);
  } 
  
  uint32_t table[32/2][128];// COLOR[4] =>    BIT  : R1, G1, B1, R2, G2, B2 
  uint32_t color[64];
 
  
  void test()
  {
    // cache clock : we shadow the register and the mask to do a direct access from toggle register
   uint32_t ulPin = CLK;   
   uint8_t pinPort = g_APinDescription[ulPin].ulPort;
   uint8_t pinNum = g_APinDescription[ulPin].ulPin;  
   volatile uint32_t * regclk= &(PORT->Group[pinPort].OUTTGL.reg);
   volatile uint32_t * regRbg= &(PORT->Group[g_APinDescription[defaultrgbpins[0]].ulPort].OUTTGL.reg);

   uint32_t  pinMsk=(1ul << pinNum);
   
   // cache color : we shadow the register and the mask to do a direct access from toggle register
   // so by write regRbg with the mask of changes we setup the new value.
   for(int i=0;i<64;i++)
     color[i]=0;
  for(int i=0;i<64;i++)// build the mask
    for(int p=0;p<6;p++)  
       color[i]|=((i>>p)&1)<<g_APinDescription[defaultrgbpins[p]].ulPin;
// build the picture :
  
  for(int y=0;y<32/2;y++)
  for(int x=0;x<128;x++)
table[y][x]=0;
for(int i=0;i<128;i++)
   table[32/2-1][i]=(i%7+1)<<3;
  for(int i=0;i<128;i++)
   table[0][i]=i%7+1;
  for(int i=0;i<32/2;i++)
   table[i][0]=(2)<<3|1;
  for(int i=0;i<32/2;i++)
   table[i][128-1]=(3)<<3|4;
   
// setup a black board :
    digitalWrite(OE ,HIGH);
    int c=0;
    int previouscolor=0;
    writeC(c);
    clock(CLK,128);
    digitalWrite(LAT ,HIGH);
  digitalWrite(LAT ,LOW);
  digitalWrite(OE ,LOW);
  digitalWrite(CLK ,HIGH);
  char COLOR_INDEX=0;
  /**
  NBCOLOR=1 : 8 color : R 0/1 ; B 0/1 ; G 0/1 ; refresh = 50*SCAN HZ
  NBCOLOR=2 : 64 color : R 0..3 ; B 0..3 ; G 0..3 ; refresh = 4*50*SCAN HZ
  NBCOLOR=3 : 512 color : R 0..7 ; B 0..7 ; G 0..7 ; refresh = 8*50*SCAN HZ
  NBCOLOR=4 : 4096 color : R 0..F ; B 0..F ; G 0..F ; refresh = 16*50*SCAN HZ
  
  */
  #define NBCOLOR 4
 // COLOR_INDEX=(COLOR_INDEX+1)%NBCOLOR;
  while(1)
  for(char y=0;y<32/2;y++)
  { 
  volatile int t=micros();
  //LOAD the shift register of panel.
 
  
  for(unsigned char x=0;x<128;x++)
  {
    // R1, G1, B1, R2, G2, B2 
    {  
    if (c!=table[y][x])
     { 
        c=table[y][x];
    //    writeC(c);
  
  *regRbg= color[(previouscolor^c)/*&0x63*/];
        previouscolor=c;
  
        
     }
     /*
     digitalWrite(CLK ,LOW);
     digitalWrite(CLK ,HIGH);*/
     
     *regclk= pinMsk ;//CLK TGL :L
     *regclk=  pinMsk ;//CLK TGL :H
     
      
    }
    }
 //disable the led
 digitalWrite(OE ,HIGH);
 // change the lines
 writeD(y);
 // latch
 digitalWrite(LAT ,HIGH); 
 digitalWrite(LAT ,LOW);
 // enable the led.
 digitalWrite(OE ,LOW);
    t-=micros();
    delayMicroseconds(20000/16-250);
    t=t+1;
   
}
  }
  
   
void setup() {

   volatile int t=0;
   int X=4;
   int Z=0;
  pinMode(CLK , OUTPUT); // Low
  pinMode(LAT , OUTPUT);   // Low
  pinMode(OE  , OUTPUT);     // High (disable output)
  pinMode(A   , OUTPUT);  // Low
  pinMode(B   , OUTPUT);  // Low
  pinMode(C   , OUTPUT);  // Low
    pinMode(D , OUTPUT);  // Low
 for(int i=0;i<6;i++)
  pinMode(defaultrgbpins[i], OUTPUT);  // Low
  
  digitalWrite(CLK ,LOW);
  digitalWrite(LAT ,LOW);
  digitalWrite(OE ,HIGH);
  digitalWrite(A ,LOW);
  digitalWrite(B ,LOW);
  digitalWrite(C ,LOW);
  digitalWrite(D ,LOW);
  digitalWrite(OE ,LOW);
  digitalWrite(CLK ,LOW);
  /*
  while(1)
test();
  while(1)
  {
  
 int Y=7;
  digitalWrite(OE ,LOW);
for( Y=0;Y<16;Y++)
 {
    digitalWrite(LAT ,LOW);
  writeD(Y);
   // for(int X=0;X<128;X++)
  
    X=0+(Z/20)%128;
    Z++;
    {  
      writeC(0x0);
      clock(CLK,X);
      writeC((1<<(Y%3))| (1<<((Y%3)+3)));
      clock(CLK,10);
      writeC(0);
      clock(CLK,128-X-10);
      
    }
    

  digitalWrite(LAT ,HIGH);
  
  }
  
//  digitalWrite(LAT ,LOW);
//  digitalWrite(LAT ,HIGH);

  
  
  digitalWrite(OE ,LOW);
  delay(1);

 }*/
 /**/
 /*
  while(1)
  {
   int r= 0;
  
  for(int i=0;i<6;i++)
  digitalWrite(defaultrgbpins[i] ,(((r>>i) &1==0))?HIGH:LOW);

clock(CLK,500);
clock(LAT,500);
clock(OE,500);
clock(A,500);
clock(B,500);
clock(C,500);
clock(D,500);
for(int i=0;i<6;i++)
clock(defaultrgbpins[i],500);
}
  for(int j=0;j<50000;j++)
  {
  for(int i=0;i<6;i++)
  digitalWrite(defaultrgbpins[i] ,LOW);
  int r= random(256);
  
  for(int i=0;i<6;i++)
  digitalWrite(defaultrgbpins[i] ,(((r>>i) &1))?HIGH:LOW);
  for(int i=0;i<6;i++)
  {
  digitalWrite(CLK ,LOW);
  digitalWrite(CLK ,HIGH);
  }
  digitalWrite(OE ,(((r>>6)&1)==0 )? HIGH:LOW);
  digitalWrite(LAT ,((j%128)==0) ? HIGH:LOW);
  
  }
    digitalWrite(LAT ,HIGH);
      digitalWrite(LAT ,LOW);
  */
  
  matrix.begin();

  // draw a pixel in solid white
  matrix.drawPixel(0, 0, matrix.Color333(7, 7, 7));
  delay(500);
  matrix.updateDisplay();
  // fix the screen with green
  matrix.fillRect(0, 0, matrix.width(), matrix.height(), matrix.Color333(0, 7, 0));
  delay(500);
  matrix.updateDisplay();
  // draw a box in yellow
  matrix.drawRect(0, 0, matrix.width(), matrix.height(), matrix.Color333(7, 7, 0));
  delay(500);

  // draw an 'X' in red
  matrix.drawLine(0, 0, matrix.width()-1, matrix.height()-1, matrix.Color333(7, 0, 0));
  matrix.drawLine(matrix.width()-1, 0, 0, matrix.height()-1, matrix.Color333(7, 0, 0));
  delay(500);

  // draw a blue circle
  matrix.drawCircle(10, 10, 10, matrix.Color333(0, 0, 7));
  delay(500);

  // fill a violet circle
  matrix.fillCircle(40, 21, 10, matrix.Color333(7, 0, 7));
  delay(500);

  // fill the screen with 'black'
  matrix.fillScreen(matrix.Color333(0, 0, 0));

  // draw some text!
  matrix.setTextSize(1);     // size 1 == 8 pixels high
  matrix.setTextWrap(false); // Don't wrap at end of line - will do ourselves

  matrix.setCursor(8, 0);    // start at top left, with 8 pixel of spacing
  uint8_t w = 0;
  char *str = "AdafruitIndustries";
  for (w=0; w<8; w++) {
    matrix.setTextColor(Wheel(w));
    matrix.print(str[w]);
  }
  matrix.setCursor(2, 8);    // next line
  for (w=8; w<18; w++) {
    matrix.setTextColor(Wheel(w));
    matrix.print(str[w]);
  }
  matrix.println();
  //matrix.setTextColor(matrix.Color333(4,4,4));
  //matrix.println("Industries");
  matrix.setTextColor(matrix.Color333(7,7,7));
  matrix.println("LED MATRIX!");

  // print each letter with a rainbow color
  matrix.setTextColor(matrix.Color333(7,0,0));
  matrix.print('3');
  matrix.setTextColor(matrix.Color333(7,4,0));
  matrix.print('2');
  matrix.setTextColor(matrix.Color333(7,7,0));
  matrix.print('x');
  matrix.setTextColor(matrix.Color333(4,7,0));
  matrix.print('6');
  matrix.setTextColor(matrix.Color333(0,7,0));
  matrix.print('4');
  matrix.setCursor(34, 24);
  matrix.setTextColor(matrix.Color333(0,7,7));
  matrix.print('*');
  matrix.setTextColor(matrix.Color333(0,4,7));
  matrix.print('R');
  matrix.setTextColor(matrix.Color333(0,0,7));
  matrix.print('G');
  matrix.setTextColor(matrix.Color333(4,0,7));
  matrix.print('B');
  matrix.setTextColor(matrix.Color333(7,0,4));
  matrix.print('*');

  // whew!
}

void loop() {
  // Do nothing -- image doesn't change
  matrix.updateDisplay();
}


