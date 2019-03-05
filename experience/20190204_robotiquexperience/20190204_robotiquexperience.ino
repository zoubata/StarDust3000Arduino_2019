
#include <Servo.h>

Servo myservo;  // create servo object to control a servo

const int sharp = 0;       // pin du sharp ( A0 )
const int moteur = 2;     // pin du moteur
int valeurdepart = 70;  
int valeursharp = 0; 
int led = 4;              // PIN LED
int inter = "30";             // temps d'attente entre 2 éclairage ( "grésillement")
 




/////////////////////////// PANNEAU ////////////////////////////////////

          //Pins des ampoules
int ampoule1 = 2;
int ampoule2 = 1;
int ampoule3 = 0;
int handUp = 160;             // l'angle du servo moteur en position haute
int handDown = 80;            // l'angle du servo moteur en position haute
short int delaybas = 100;     // temps d'attente en position basse en milisecondes
short int delayhaut = 2000;   // temps d'attente en position haute en milisecondes
short int delaytraget =500;   // temps d'attente pour laisser le servo bouger
void setuppanneau()
{


 pinMode(ampoule1,OUTPUT);        // initialise les ampoules
 pinMode(ampoule2,OUTPUT);        // initialise les ampoules
 pinMode(ampoule3,OUTPUT);        // initialise les ampoules

digitalWrite (ampoule1,HIGH);     //allume les ampoules
digitalWrite (ampoule2,HIGH);     //allume les ampoules
digitalWrite (ampoule3,HIGH);     //allume les ampoules
}

 void gresillement () {           //fonction définisant la séquance de "grésillement"
 analogWrite (ampoule1,255);      //100%
 analogWrite (ampoule2,255);
 analogWrite (ampoule3,255);
 delay(2*inter);                  //attend plus longtemps
 analogWrite (ampoule1,180);      //50%
 analogWrite (ampoule2,180);
 analogWrite (ampoule3,180);
 delay(inter);
 analogWrite (ampoule1,230);      //90%
 analogWrite (ampoule2,230);
 analogWrite (ampoule3,230);
 delay(inter);
 analogWrite (ampoule1,230);      //90%
 analogWrite (ampoule2,230);
 analogWrite (ampoule3,230);
 delay(inter);
 analogWrite (ampoule1,255);      //100%
 analogWrite (ampoule2,255);
 analogWrite (ampoule3,255);
 delay(inter);
for (int t=0;t<2;t=t+1) {           //répete 2 fois cette séquance
  analogWrite (ampoule1,0);         //0%
  analogWrite (ampoule2,0);
  analogWrite (ampoule3,0);
  delay(inter);
  analogWrite (ampoule1,102);       //40%
  analogWrite (ampoule2,102);
  analogWrite (ampoule3,102);
  delay(inter);
  analogWrite (ampoule1,26);        //10%
  analogWrite (ampoule2,26);
  analogWrite (ampoule3,26);
  delay(inter);
  analogWrite (ampoule1,255);       //100%
  analogWrite (ampoule2,255);
  analogWrite (ampoule3,255);
  delay(inter);
  }
 }
void looppanneau() { 
    Serial.println("looppanneau :");
   gresillement ();
  delay(delayhaut);
  delay(delaytraget);
  delay(delaybas);
  delay(delaytraget);
}



/////////////////////////////////////////////////

#define PINSERVO 2
void setup() {
  
   Serial.begin(9600);
   Serial.println("setup :");
   myservo.attach(PINSERVO);  // attaches the servo on pin 9 to the servo object
  
    valeursharp = analogRead(0)+analogRead(0)+analogRead(0)+analogRead(0);                              // lecture du capteur
   valeursharp/=4;

   
     valeurdepart=valeursharp;    
    Serial.print("servo :");Serial.println(130);
    myservo.write(130);  //arme le lance pierre
 }
 void lacher()
 {
  myservo.write(10); 
      Serial.print("servo :");Serial.println(10); 
  }
void loop() { 
//DETECT DEPAR ROBOT
  valeursharp = analogRead(0); 
      Serial.print("analogRead :");Serial.println(valeurdepart);  
  while((valeurdepart -20 < valeursharp) && (valeursharp < valeurdepart +20))
   {
    delay(200);
   valeursharp = analogRead(0)+analogRead(0)+analogRead(0)+analogRead(0);                              // lecture du capteur
   valeursharp/=4;
        Serial.print("analogRead :");Serial.println(valeursharp);  
    }  
//LACHER  
   {                                                 // si la valeur est differante (le robot est parti) allumer le moteur
        lacher();
        }
    delay(200);   
//LAMPE
    setuppanneau();
    while(1)
    looppanneau();
}
