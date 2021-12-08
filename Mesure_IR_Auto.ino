
#include "Timer.h"
#include <Wire.h>
#include <Adafruit_MLX90614.h>

Timer timer;
/////////////////////////////////////////////////////////////////////////////////////BLOC CIBLE//////////////////////////////////////////////////////////////////////////////////


//Initialisation des différentes variables pour la fonction pt100//
float fCapteurPin = A1;
float fValA1;
float fR0 = 100;
float fAlpha = 0.00385;
float fRx = 0;
float fTension = 0; // variable pour la valeur de la resistance
float fTemp = 0; //variable pour la valeur de la resistance
float fSommeConsigneCI;
float fMoyenneTO;

//PID constants//
double kp = 40;
double ki = 0.02;
double kd = 0;

//Niveau de la consigne
float fConsigne = 20; //TEMPERATURE DE LA CIBLE FIXEE AU PREALABLE

int state = 0;
int etatmontee = 1;
int compteur = 0;

//Initialisation des différentes varaibles pour la fonction computePID//
unsigned long currentTime, previousTime;
double elapsedTime;
float fErreur;
float fErreurprecedente = fConsigne;
float fSortie;
float fDeltaErreur, fRes, fPID;
float fSommeErreur = 0;

//Initialisation LED
int iLedPinR = 12;

/////////////////////////////////////////////////////////////////////////////////////BLOC CAPTEUR/////////////////////////////////////////////////////////////////////////////////

//Initialisation des différentes variables pour la fonction pt100//
float fCapteurPin2 = A0;
float fValA0;
float fR02 = 100;
float fAlpha2 = 0.00385;
float fRx2 = 0;
float fTension2 = 0; // variable pour la valeur de la resistance
float fTemp2 = 0; //variable pour la valeur de la resistance
float fSommeConsigneCA;
float fMoyenneTA;

//PID constants//
double kp2 = 25;
double ki2 = 0.02;
double kd2 = 0;

//Niveau de la consigne
float fConsigne2 = 20; //TEMPERATURE DU CAPTEUR MODIFIABLE DANS LE SERIAL MONITOR
int byte_read ;

//Initialisation des différentes varaibles pour la fonction computePID//
unsigned long currentTime2, previousTime2;
double elapsedTime2;
float fErreur2;
float fErreurprecedente2 = fConsigne2;
float fSortie2;
float fDeltaErreur2, fRes2, fPID2;
float fSommeErreur2 = 0;

//Initialisation LED
int iLedPinV = 10;

/////////////////////////////////////////////////////////////////////////////////////MLX & EXCEL/////////////////////////////////////////////////////////////////////////////////

//Initialisation valeur MLX
Adafruit_MLX90614 mlx = Adafruit_MLX90614();


//Initialisation valeur Excel
float fEmissivite = 0;

/////////////////////////////////////////////////////////////////////////////////////ELECTROVANNE/////////////////////////////////////////////////////////////////////////////////

int solenoidPompePin = 5;    //On associe la pin 4 Ã  l'Ã©lectrovanne de la pompe
int solenoidClochePin = 4;   //On associe la pin 5 Ã  l'Ã©lectrovanne de la pompe

/////////////////////////////////////////////////////////////////////////////////////Partie Code/////////////////////////////////////////////////////////////////////////////////


void setup() {

  Serial.begin(9600);

  pinMode(3, OUTPUT);  // on met la pin 3 (PWM) en lecture de sortie
  pinMode(11, OUTPUT); // on met la pin 11 (PWM) en lecture de sortie
  pinMode(iLedPinV, OUTPUT); // on met la pin 10 en lecture de sortie
  pinMode(iLedPinR, OUTPUT); // on met la pin 12 (PWM) en lecture de sortie

  pinMode(solenoidPompePin, OUTPUT);           //On met l'Ã©lectrovanne sur la pompe en sortie
  pinMode(solenoidClochePin, OUTPUT);          //On met l'Ã©lectrovanne sur la cloche en sortie

  
  digitalWrite(solenoidPompePin, LOW);    //On ouvre la vanne pour pouvoir pomper
  digitalWrite(solenoidClochePin, LOW);    //On ferme la vanne pour ne pas remonter en pression HIGH = OUVERT / LOW = FERME

  mlx.begin();
}

void loop() {
  //Bien mettre les elctrovannes dans le loop à chaque fois
  digitalWrite(solenoidPompePin, HIGH);    //On ouvre la vanne pour pouvoir pomper
  digitalWrite(solenoidClochePin, LOW);    //On ferme la vanne pour ne pas remonter en pression HIGH = OUVERT / LOW = FERME

  ValeurDepart();

  TempPt100();  //appel de la fonction qui converti la valeur de la tension au borne de la pt100 en température
  TempPt1002();

  fSortie = computePID();     //Récupère la valeur du PID qui est dans la fonction computePID, que l'on affecte a notre variable fSortie
  fSortie2 = computePID2();

  if ( fSortie > 255) {       //Convertion de la valeur final du PID en 8bit pour notre sortie PWM
    fSortie = 255;
  }
  else if ( fSortie < 0) {
    fSortie = 0;
  }

  if ( fSortie2 >= 255) {       //Convertion de la valeur final du PID en 8bit pour notre sortie PWM
    fSortie2 = 255;
    digitalWrite(iLedPinV, HIGH);// On allume la diode verte quand on est en phase de chauffe totale (suite Ã  un Ã©chelon par exemple)
  }
  else if ( fSortie2 <= 0) {
    fSortie2 = 0;
    digitalWrite(iLedPinR, HIGH);// On allume la diode rouge quand le PWM est Ã©teint afin de montrer que le systÃ¨me refroidit (lorsque l'on dÃ©passe la tempÃ©rature de consigne par exemple)
  }

  analogWrite(11, 0); //On Ã©crit en sortie PWM sur la pin 11
  analogWrite(3, 0); //On Ã©crit en sortie PWM sur la pin 3

  delay(1);
  Affichage();  //appel de la fonction Afffichage
  Affichage2();
  delay(1);

  analogWrite(3, fSortie);  //On écrit en sortie PWM sur la pin 3
  analogWrite(11, fSortie2);  //On écrit en sortie PWM sur la pin 11


  delay(1000);


  digitalWrite(iLedPinV, LOW);//On Ã©teint les leds afin d'afficher Ã  nouveau l'etat du pwm lors du prochain cycle du programme
  digitalWrite(iLedPinR, LOW);

  



}

/////////////////////////////////////////////////////////////////////////////////////BLOC CIBLE//////////////////////////////////////////////////////////////////////////////////

void TempPt100() {

  fValA1 = analogRead(fCapteurPin);  //On récupère la valeur de la tension de la résistance sur la PIN analogique A1

  fTension = (fValA1 * 5) / 1023.0; //calcul de la valeur de la tension

  // fRx = (fTension * 15) + 100;  // calcul de la valeur de la resistance

  //fTemp = ((fRx / fR0)-1)/fAlpha;  // calcul pour trouver la vlaeur de la temperature en fonction de la valeur de la résiatsnce obtenu précédement

  fTemp = (fTension / 5) * 200; // 5 est la tension qu'on a pour 200 degres
 // fTemp = 1.026 * fTemp + 1.33 ; // calcul de compensation de la resistance pt1000 bloc cible  carte arduino A1
    fTemp = 1.0244 *fTemp + 0.165; // après nouvelle mesure carte A1
 // fTemp = 1.0176 *fTemp - 5.83; // carte Arduino A3

}

float computePID() {

  currentTime = millis();                                         //on récupère le temps en milliseconde
  elapsedTime = (float)(currentTime - previousTime);              //Calcule le temps écoulé à partir du calcul précédent

 /* if (fTemp < (fConsigne - 2)) {

    fErreur = fConsigne - fTemp;                                   // determine l'erreur donc calcul du proportionnel
    fPID = kp * fErreur;
    //digitalWrite(iLedPinV, LOW);
  }
  else if (fTemp > (fConsigne + 2)) {

    fPID = 0;
    fSommeErreur = 0;
    //digitalWrite(iLedPinR, LOW);

  }*/
 // else {

    fErreur = fConsigne - fTemp;
    fSommeErreur += fErreur;  // * elapsedTime;                     // calcul de l'integral
    fPID = kp * fErreur + ki * fSommeErreur;                        //PID output
    //digitalWrite(iLedPinR, HIGH);
 // }

  //fDeltaErreur = (fErreur - fErreurprecedente)/elapsedTime;      // calcul du dérivé
  //fPID = kp*fErreur + ki*fSommeErreur + kd*fDeltaErreur;

  fErreurprecedente = fErreur;                                   //Rappele de l'erreur actuelle
  previousTime = currentTime;                                    //Rappele du temps actuelle

  return fPID;                                                 //retour du PID en sortie
}


void Affichage() {

  ////////////Affichage des diffrents paramètres////////////

  Serial.println("");
  Serial.print(currentTime / 1000);
  Serial.print(" Temps");
  Serial.print("   ");
  Serial.print("BLOC CIBLE");
  Serial.print("   ");
  Serial.print(fTemp);
  Serial.print(" degres");
  Serial.print("   ");
  Serial.print(fSortie);
  Serial.print(" Pwm");
  Serial.print("   ");
  //Serial.print(fSommeErreur);
  //Serial.print(" somme erreur");
  //Serial.print("   ");
  //Serial.print(fTension);
  //Serial.print(" volts");
  //Serial.print("   ");
  //Serial.print(fRx);
  //Serial.print(" val resistance");
  //Serial.print("   ");

  Serial.print("");

}


/////////////////////////////////////////////////////////////////////////////////////BLOC CAPTEUR/////////////////////////////////////////////////////////////////////////////////

void TempPt1002() {

  fValA0 = analogRead(fCapteurPin2);  //On récupère la valeur de la tension de la résistance sur la PIN analogique A1

  fTension2 = (fValA0 * 5) / 1023.0; //calcul de la valeur de la tension

  //fRx2 = (fTension2 * 15) + 100;  // calcul de la valeur de la resistance

  //fTemp2 = ((fRx2 / fR02)-1)/fAlpha2;  // calcul pour trouver la vlaeur de la temperature en fonction de la valeur de la résiatsnce obtenu précédement

  fTemp2 = (fTension2 / 5) * 200; // 5 est la tension qu'on a pour 200 degres
 // fTemp2 = 1.031 * fTemp2 ;  // calcul de compensation de la resistance pt100 bloc capteur      carte arduino A1
  fTemp2 = 1.0341 * fTemp2 - 0.483; // carte Arduino A1
 // fTemp2 = 1.0313 *fTemp + -5.65; // carte Arduino A3 

}

float computePID2() {

  currentTime2 = millis();                                         //on récupère le temps en milliseconde
  elapsedTime2 = (float)(currentTime - previousTime);              //Calcule le temps écoulé à partir du calcul précédent

  if ((fTemp2 < (fConsigne2 - 10)) || (fTemp2 > (fConsigne2 + 10))) {
    fErreur2 = fConsigne2 - fTemp2;                                   // determine l'erreur donc calcul du proportionnel
    fPID2 = kp2 * fErreur2;
    fSommeErreur2 = 0;
    //digitalWrite(iLedPinV, LOW);
  }
  //else if (fTemp2 > (fConsigne2 + 2)) {

    //fPID2 = 0;
   // fSommeErreur2 = 0;
    //digitalWrite(iLedPinV, LOW);

//  }
  
  else {

    fErreur2 = fConsigne2 - fTemp2;
    fSommeErreur2 += fErreur2;  // * elapsedTime;                     // calcul de l'integral
    fPID2 = kp2 * fErreur2 + ki2 * fSommeErreur2;                        //PID output
    //digitalWrite(iLedPinV, HIGH);
  }

  //fDeltaErreur = (fErreur - fErreurprecedente)/elapsedTime;      // calcul du dérivé
  //fPID = kp*fErreur + ki*fSommeErreur + kd*fDeltaErreur;

  fErreurprecedente2 = fErreur2;                                   //Rappele de l'erreur actuelle
  previousTime2 = currentTime2;                                    //Rappele du temps actuelle

  return fPID2;                                                 //retour du PID en sortie
}

/*  if ( (fTemp2 >= fConsigne2) && (fTemp >= fConsigne ) ) {

    for (int iBcl2; iBcl2 < 10; iBcl2 ++) {


      fSommeConsigneCA += fConsigne2;
      fSommeConsigneCI += fConsigne;
      AfficheConsigne();

      delay (500);
    }

    fSommeConsigneCA /= 10;
    fSommeConsigneCI /= 10;
    AfficheMoyenne();
    fSommeConsigneCA = 0;
    fSommeConsigneCI = 0;

        if  (fConsigne2 <= 110){

         Serial.print("CELL,SET,N1, ");
         fConsigne2 = fConsigne2 + 10;
         Serial.println(fConsigne2);
         }
    }


        if  (fConsigne2 > 120){
           Serial.print("CELL,SET,N1, ");
           fConsigne2 = 20;
            Serial.println(fConsigne2);
            fPID = 0;
            fPID2 = 0;
        }
*/




void Affichage2() {

  ////////////Affichage des diffrents paramètres////////////

  Serial.print("");
  Serial.print("BLOC CAPTEUR");
  Serial.print("   ");
  Serial.print(fTemp2);
  Serial.print(" degres");
  Serial.print("   ");
  Serial.print(fSortie2);
  Serial.print(" Pwm");
  Serial.print("   ");
  //Serial.print(fSommeErreur2);
  // Serial.print(" somme erreur");
  // Serial.print("   ");
  //Serial.print(fTension2);
  //Serial.print(" volts");
  //Serial.print("   ");
  //Serial.print(fRx2);
  //Serial.print(" val resistance");
  //Serial.print("   ");
  Serial.print("");
  Serial.print(" Valeur consigne : ");
  Serial.print((int)fConsigne2);
  Serial.print("  ");
  Serial.print("Object = "); Serial.print(mlx.readObjectTempC()); Serial.print(" degres");    //recupération de la température (°C) objet par le MLX
  Serial.print("  ");
  Serial.print("Ambient = "); Serial.print(mlx.readAmbientTempC()); Serial.print(" degres");           //recupération de la température (°C) ambiante par le MLX
  //Serial.print(" ");
  //Serial.print(fSommeErreur);
  //Serial.print(" ");
  //Serial.print(fSommeErreur2);
}

void ValeurDepart() {
  ///////Code de depart pour ecrit un valeur de la consigne sur le port serie de l'arduino et donc  de changer la valeur de la consigne sans upload a chaque fois////////

  if (Serial.available() > 0) {
    byte_read = Serial.readStringUntil(" ").toInt();
    fConsigne2 = byte_read;
  }

  if (Serial.available() < 0) {
    fConsigne2 = 0;
  }
  /*
    /////////code qui lit la consigne sur une case du tableau excel //////////

    /////////// Consigne Capteur//////////////

    Serial.println("CELL,GET,N1");   //lit la case L1 du tableau excel

    int CELL_num = Serial.parseInt();   // on met la valeur de la case dans la variable CELL_num

    fConsigne2 = (float)CELL_num;   //la consigne vaut la valeur que l'on rentre dans la case du tableau excel


    //////////Consigne Cible/////////////
    Serial.println("CELL,GET,N2");  //lit la case L2 du tableau excel

    int CELL2_num = Serial.parseInt();  // on met la valeur de la case dans la variable CELL2_num

    fConsigne = (float)CELL2_num;   //la consigne vaut la valeur que l'on rentre dans la case du tableau excel


    //////////Emissivite///////////////
    Serial.println("CELL,GET,N3");  //lit la case L3 du tableau excel

    int CELL3_num = Serial.parseInt();  // on met la valeur de la case dans la variable CELL3_num

    fEmissivite = (float)CELL3_num;   //l'emissivite vaut la valeur que l'on rentre dans la case du tableau excel
  */
}

/*
  void AffichageExcel() {

  Serial.print("Ambient = "); Serial.print(mlx.readAmbientTempC());                                //recupération de la température (°C) ambiante par le MLX
  Serial.print("*C\tObject = "); Serial.print(mlx.readObjectTempC()); Serial.println("*C");        //recupération de la température (°C) objet par le MLX
  //Serial.print("Ambient = "); Serial.print(mlx.readAmbientTempF());                              //recupération de la température (°F) ambiante par le MLX
  //Serial.print("*F\tObject = "); Serial.print(mlx.readObjectTempF()); Serial.println("*F");      //recupération de la température (°F) objet par le MLX

  Serial.println();
  //delay(500);

  float Bdata = mlx.readAmbientTempC();  // Valeur de la temperature ambiante donné par le MLX, qui est stocker dans la variable Adata
  float Adata = mlx.readObjectTempC();   // Valeur de la temperature objet donné par le MLX, qui est stocker dans la variable Bdata


  Serial.print("DATA,TIME,TIMER, ");    //ecrit l'heure dans la colonne A, le timer dans le colonne B ...
  //affichage des differentes variables sur un tableau excel//
  Serial.print(Adata);
  Serial.print(",");
  Serial.print(Bdata);
   Serial.print(",");
  Serial.print(fTemp);
  Serial.print(",");
  Serial.print(fTemp2);



  Serial.println("");   //être sur d'avoir afficher toute les valeurs sur une même ligne on reviens à la ligne juste apres
  delay(1000);
  }

  void AfficheConsigne() {

  Serial.print("Ambient = "); Serial.print(mlx.readAmbientTempC());                                //récuperation de la température (°C) ambiante par le MLX
  Serial.print("*C\tObject = "); Serial.print(mlx.readObjectTempC()); Serial.println("*C");        //recupération de la température (°C) objet par le MLX
  //Serial.print("Ambient = "); Serial.print(mlx.readAmbientTempF());                              //recupération de la température (°F) ambiante par le MLX
  //Serial.print("*F\tObject = "); Serial.print(mlx.readObjectTempF()); Serial.println("*F");      //recupération de la température (°F) objet par le MLX

  Serial.println();

  float Bdata = mlx.readAmbientTempC();  // Valeur de la temperature ambiante donné par le MLX, qui est stocker dans la variable Adata
  float Adata = mlx.readObjectTempC();   // Valeur de la temperature objet donné par le MLX, qui est stocker dans la variable Bdata


  Serial.print("DATA,TIME,TIMER, ");    //ecrit l'heure dans la colonne A, le timer dans le colonne B ...

  Serial.print(Adata);  //C    Tambiante
  Serial.print(",");
  Serial.print(Bdata);  //D    Tobjet
  Serial.print(",");
  Serial.print(fTemp);     //F Pt100cible
  Serial.print(",");
  Serial.print(fTemp2);    //E PT100capteur
  Serial.print(",");
  Serial.print((int)fConsigne);  //Consigne cible
  Serial.print(",");
  Serial.print((int)fConsigne2); //Consigne capteur
  Serial.print(",");

  Serial.println("");
  delay(1000);
  }

  void AfficheMoyenne() {

  Serial.print("Ambient = "); Serial.print(mlx.readAmbientTempC());                                //récuperation de la température (°C) ambiante par le MLX
  Serial.print("degres\tObject = "); Serial.print(mlx.readObjectTempC()); Serial.println("degres");        //recupération de la température (°C) objet par le MLX
  //Serial.print("Ambient = "); Serial.print(mlx.readAmbientTempF());                              //recupération de la température (°F) ambiante par le MLX
  //Serial.print("*F\tObject = "); Serial.print(mlx.readObjectTempF()); Serial.println("*F");      //recupération de la température (°F) objet par le MLX

  Serial.println();

  float Bdata = mlx.readAmbientTempC();  // Valeur de la temperature ambiante donné par le MLX, qui est stocker dans la variable Adata
  float Adata = mlx.readObjectTempC();   // Valeur de la temperature objet donné par le MLX, qui est stocker dans la variable Bdata

  Serial.print("DATA,TIME,TIMER, ");    //ecrit l'heure dans la colonne A, le timer dans le colonne B ...

  Serial.print(Adata);     //A     Heure
  Serial.print(",");
  Serial.print(Bdata);     //B     Timer
  Serial.print(",");


  Serial.print(fTemp);     //F Pt100cible
  Serial.print(",");
  Serial.print(fTemp2);    //E PT100capteur
  Serial.print(",");
  Serial.print((int)fConsigne);  //Consigne cible
  Serial.print(",");
  Serial.print((int)fConsigne2); //Consigne capteur
  Serial.print(",");


  Serial.print(fMoyenneTA);
  Serial.print(",");
  Serial.print(fMoyenneTO);
  Serial.print(",");
  Serial.print(fSommeConsigneCA);
  Serial.print(",");
  Serial.print(fSommeConsigneCI);




  Serial.println("");
  delay(1000);
  }
*/
