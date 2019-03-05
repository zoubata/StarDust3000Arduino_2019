
const int sharp = 0;       // pin du sharp ( A0 )
const int moteur = 2;     // pin du moteur
int valeurdepart = 0;  
int valeursharp = 0; 
int led = 4;              // PIN LED

void setup() {
    valeurdepart = analogRead(sharp);       // initialisation du capteur lorsque le robot est pret
    valeursharp = valeurdepart;    
    digitalWrite(moteur, LOW);             //moteur eteint
    digitalWrite(led, HIGH);               // led temoin allumé
 }
void loop() { 
  valeurdepart = analogRead(0);                              // lecture du capteur
     if (valeurdepart -5 < valeursharp < valeurdepart +5)    // la valeur est-elle a peu pres egale à celle au depart ?
       {delay(200);
     }                                                       // si oui, attendre 0.2 segondes 
     else  {                                                 // si la valeur est differante (le robot est parti) allumer le moteur
        digitalWrite(moteur, HIGH);
        delay(1000);
        digitalWrite(moteur, LOW);                           // eteindre le moteur apres 1 segonde 
        }
    delay(200);
}
