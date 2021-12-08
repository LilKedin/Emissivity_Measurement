
/*
 * 
 * 
 * 
 *  Ce programme fait des échelons de 10°C sur le bloc capteur et arrivé à 110°C fait un échelon sur le bloc cible
 * 
 * 
 * 
 */



#include "Timer.h"
#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <SparkFun_MLX90632_Arduino_Library.h>
MLX90632 capteurCMS;


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
int MemoireCible = 0;
int Refroidissement = 0;


//PID constants//
double kp = 80;
double ki = 0;
double kd = 0;

//Niveau de la consigne
float fConsigne = 90;

int state = 0;
int etatmontee = 1;
int compteur = 0;
const unsigned long SECOND = 1000;
const unsigned long HOUR = 3600 * SECOND;

//Initialisation des différentes variables pour la fonction computePID//
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



//PID constants//
double kp2 = 100;
double ki2 = 0;
double kd2 = 0;

//Niveau de la consigne
float fConsigne2 = 40;
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
//float Adata = mlx.readAmbientTempC();  // Valeur de la temperature ambiante donné par le MLX, qui est stocker dans la variable Adata
//float Bdata = mlx.readObjectTempC();   // Valeur de la temperature objet donné par le MLX, qui est stocker dans la variable Bdata

//Initialisation valeur Excel
float fEmissivite = 0;
int CELL_num = 0;
int CELL2_num = 0;
float EtatOldCA = 0;
float EtatOldCI = 0;

/////////////////////////////////////////////////////////////////////////////////////Partie Code/////////////////////////////////////////////////////////////////////////////////


void setup() {

  Serial.begin(9600);
  pinMode(3, OUTPUT);  // on met la pin 3 (PWM) en lecture de sortie
  pinMode(11, OUTPUT); // on met la pin 11 (PWM) en lecture de sortie
  pinMode(iLedPinV, OUTPUT); // on met la pin 12 en lecture de sortie
  pinMode(iLedPinR, OUTPUT); // on met la pin 10 (PWM) en lecture de sortie

  Serial.println("CLEARDATA"); // remet a jour le logiciel
  Serial.println("LABEL,Heure,Timer,Tambiante,Tobjet,MLX90632 ambiante,PT100capteur,PT100cible,ValeurCible,ValeurCapt,ValObjet,ValAmbiant"); //Intitulés de chaque colonne
  Serial.println("RESETTIMER"); //remet le timer à 0

  /////////code qui lit la consigne sur une case du tableau excel //////////

  Serial.print("CELL,SET,M1, ");
  Serial.println("Consigne Capteur");

  Serial.print("CELL,SET,O1, ");
  Serial.println("Consigne Cible");

  Wire.begin(),
  mlx.begin();
  capteurCMS.begin();
}



void loop() {

  ValeurDepart();
  //ValeurDepartCI();
  //ValeurDepartCA();

  TempPt100();  //appel de la fonction qui converti la valeur de la tension au borne de la pt100 en température
  TempPt1002();

  fSortie = computePID();     //Récupère la valeur du PID qui est dans la fonction computePID, que l'on affecte a notre variable fSortie
  fSortie2 = computePID2();

  if ( fSortie > 255) {       //Conversion de la valeur finale du PID en 8bit pour notre sortie PWM bloc cible
    fSortie = 255;
      }
      
  else if ( fSortie < 0) {
    fSortie = 0;
      }

  if ( fSortie2 > 255) {       //Conversion de la valeur finale du PID en 8bit pour notre sortie PWM bloc capteur
    fSortie2 = 255;
    digitalWrite(iLedPinV, HIGH);// On allume la diode verte quand on est en phase de chauffe totale (suite à un échelon par exemple)

  }
  else if ( fSortie2 < 0) {
    fSortie2 = 0;
    digitalWrite(iLedPinR, HIGH);// On allume la diode rouge quand le PWM est éteint afin de montrer que le système refroidit (lorsque l'on dépasse la température de consigne par exemple)
  }
  analogWrite(11, 0); //On écrit en sortie PWM sur la pin 11
  analogWrite(3, 0); //On écrit en sortie PWM sur la pin 3

  delay(1);
  AffichageExcel();
  delay(1);

  analogWrite(3, fSortie);  //On écrit en sortie PWM sur la pin 3
  analogWrite(11, fSortie2);  //On écrit en sortie PWM sur la pin 11

  delay(1000);

  TestConsigne(); //A commenter si on ne veut pas d'échelon en température
  //Affichage();  //appel de la fonction Afffichage
  //Affichage2();
  digitalWrite(iLedPinV, LOW);//On éteint les leds afin d'afficher à nouveau l'etat du pwm lors du prochain cycle du programme
  digitalWrite(iLedPinR, LOW);

}

///////////////////////////////////////////////////////////////////////////////////// BLOC CIBLE //////////////////////////////////////////////////////////////////////////////////


void TempPt100() {

  fValA1 = analogRead(fCapteurPin);  //On récupère la valeur de la tension de la résistance sur la PIN analogique A1

  fTension = (fValA1 * 5) / 1023.0; //calcul de la valeur de la tension

  // fRx = (fTension * 15) + 100;  // calcul de la valeur de la resistance

  //fTemp = ((fRx / fR0)-1)/fAlpha;  // calcul pour trouver la vlaeur de la temperature en fonction de la valeur de la résiatsnce obtenu précédement

  fTemp = (fTension / 5) * 200; // 5 est la tension qu'on a pour 200 degres
  fTemp = 1.026 * fTemp + 1.33 ; // calcul de compensation de la resistance pt100 bloc cible

}

float computePID() {

  currentTime = millis();                                         //on récupère le temps en milliseconde
  elapsedTime = (float)(currentTime - previousTime);              //Calcule le temps écoulé à partir du calcul précédent

  if ((fTemp < (fConsigne - 1)) || (fTemp > (fConsigne + 1))) {
    fErreur = fConsigne - fTemp;                                   // determine l'erreur donc calcul du proportionnel
    fPID = kp * fErreur;
    fSommeErreur = 0;

  }

  else  {

    fErreur = fConsigne - fTemp;
    fSommeErreur += fErreur;  // * elapsedTime;                     // calcul de l'integral
    //fDeltaErreur = (fErreur - fErreurprecedente)/elapsedTime;      // calcul du dérivé
    fPID = kp * fErreur + ki * fSommeErreur; // + kd*fDeltaErreur;                //PID output
    //digitalWrite(iLedPinV, HIGH);

  }


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
  Serial.print(fSommeErreur);
  Serial.print(" somme erreur");
  Serial.print("   ");
  Serial.print(fTension);
  Serial.print(" volts");
  Serial.print("   ");
  //Serial.print(fRx);
  //Serial.print(" val resistance");
  // Serial.print("   ");
  Serial.print(fConsigne2);
  Serial.print(" Valeurs consigne capteurs");
  Serial.print("   ");

  Serial.print("");



}


/////////////////////////////////////////////////////////////////////////////////////BLOC CAPTEUR/////////////////////////////////////////////////////////////////////////////////

void TempPt1002() {

  fValA0 = analogRead(fCapteurPin2);  //On récupère la valeur de la tension de la résistance sur la PIN analogique A1

  fTension2 = (fValA0 * 5) / 1023.0; //calcul de la valeur de la tension

  //fRx2 = (fTension2 * 15) + 100;  // calcul de la valeur de la resistance

  //fTemp2 = ((fRx2 / fR02)-1)/fAlpha2;  // calcul pour trouver la vlaeur de la temperature en fonction de la valeur de la résiatsnce obtenu précédement

  fTemp2 = (fTension2 / 5) * 200; // 5 est la tension qu'on a pour 200 degres
  fTemp2 = 1.031 * fTemp2 ;  // calcul de compensation de la resistance pt100 bloc capteur

}

float computePID2() {

  currentTime2 = millis();                                         //on récupère le temps en milliseconde
  elapsedTime2 = (float)(currentTime2 - previousTime2);              //Calcule le temps écoulé à partir du calcul précédent



  if ((fTemp2 < (fConsigne2 - 1)) || (fTemp2 > (fConsigne2 + 1))) {
    fErreur2 = fConsigne2 - fTemp2;                                   // determine l'erreur donc calcul du proportionnel
    fPID2 = kp2 * fErreur2;
    fSommeErreur2 = 0;
    //digitalWrite(iLedPinR, LOW);
    //AffichageExcel();
  }

  else  {

    fErreur2 = fConsigne2 - fTemp2;
    fSommeErreur2 += fErreur2;  // * elapsedTime2;                     // calcul de l'integral
    fPID2 = kp2 * fErreur2 + ki2 * fSommeErreur2;                        //PID output
    //digitalWrite(iLedPinR, HIGH);


  }

  //fDeltaErreur2 = (fErreur2 - fErreurprecedente2)/elapsedTime2;      // calcul du dérivé
  //fPID2 = kp2*fErreur2 + ki2*fSommeErreur2 + kd2*fDeltaErreur2;

  fErreurprecedente2 = fErreur2;                                   //Rappele de l'erreur actuelle
  previousTime2 = currentTime2;                                    //Rappele du temps actuelle

  return fPID2;                                                 //retour du PID en sortie

}

void Affichage2() {

  ////////////Affichage des diffrents paramètres////////////

  Serial.print("");
  Serial.print("   ");
  Serial.print("BLOC CAPTEUR");
  Serial.print("   ");
  Serial.print(fTemp2);
  Serial.print(" degres");
  Serial.print("   ");
  Serial.print(fSortie2);
  Serial.print(" Pwm");
  Serial.print("   ");
  Serial.print(fSommeErreur2);
  Serial.print(" somme erreur");
  Serial.print("   ");
  Serial.print(fTension2);
  Serial.print(" volts");
  Serial.print("   ");
  //Serial.print(fRx2);
  //Serial.print(" val resistance");
  //Serial.print("   ");
  Serial.print("");
  Serial.print((int)fConsigne);
  Serial.print(" Valeur consigne");
  Serial.print("  ");


}

void TestConsigne() {

  /*if ((fTemp2 >= (fConsigne2 - 1 )) && (fTemp >= (fConsigne - 1))) {

    delay (20000);
    for (int iBcl2; iBcl2 < 2; iBcl2++) {

      AfficheConsigne();
      delay (500);
    }*/
  //delay(1000);


  //digitalWrite(iLedPinR, HIGH);

  //if(compteur >= 18000)
  if (compteur >= 599)                 //On fixe un delai entre chaque changement de consigne en temperature = delay * ( ncompteur + 1 )
  {

    if (etatmontee == 1)                //1. Si on est a l'etat de montee 1 on augmente de 10°C jusqu'a 120°C
    {
      Serial.print("CELL,SET,N1, ");
      fConsigne2 += 10;
      Serial.println(fConsigne2);
    }
    /*if (fConsigne2 >= 130 && etatmontee == 1) //2. Ensuite quand on arrive à 120 °C l'etat de montee passe à 0
      {
      Serial.print("CELL,SET,M1, ");
      fConsigne2 -= 10;
      Serial.println(fConsigne2);
      etatmontee = 0;
      }
      if (fConsigne2 <= 70 && etatmontee == 0) //4. A 70°C on change l'etat de montee a 1 et on recommence la montee de temperature de 10 en 10
      {
      Serial.print("CELL,SET,M1, ");
      fConsigne2 += 10;
      Serial.println(fConsigne2);
      etatmontee = 1;
      }
      else  if (etatmontee == 0)                          //3. La consigne diminue de 10 en 10 jusqu'a 70°C
      {
      // digitalWrite(iLedPinV, HIGH);
      Serial.print("CELL,SET,M1, ");
      fConsigne2 -= 10;
      Serial.println(fConsigne2);

      }*/

    if (fConsigne2 == 100) {
      Serial.println("CELL,GET,P1");  //lit la case O1 du tableau excel

      MemoireCible = Serial.parseInt();  // on met la valeur de la case dans la variable CELL2_num, cela permet d'incrémenter température cible pour le test suivant(on les enchaîne)

    }
    compteur = 0;                       //reinitialisation du compteur
  }
  else
  {
    compteur += 1;                  //Incrementation du compteur
  }

  /*
    if (state == 0 && compteur == 0 ) {

      Serial.print("CELL,SET,O1, ");
      fConsigne = fConsigne - 60;
      Serial.println(fConsigne);
      compteur = compteur + 1;
      state=1;
    }
      if (state == 1 && compteur < 3600 ) {

      Serial.print("CELL,SET,O1, ");
      fConsigne = fConsigne;
      Serial.println(fConsigne);
      compteur = compteur +1;
    }


    if (compteur == 3600) {

      Serial.print("CELL,SET,O1, ");
      fConsigne = fConsigne + 60;
      Serial.println(fConsigne);
      compteur = compteur -1;
      state=0;
    }

    if (state == 0 && compteur < 3600 && compteur > 0 ) {

      Serial.print("CELL,SET,O1, ");
      fConsigne = fConsigne;
      Serial.println(fConsigne);
      compteur = compteur-1;
    }
  */

  if (fConsigne2 > 111 || Refroidissement == 1) {
    Serial.print("CELL,SET,N1, ");
    fConsigne2 = 20;
    Serial.println(fConsigne2);

    Serial.print("CELL,SET,P1, ");
    fConsigne = 20;
    Serial.println(fConsigne);

    etatmontee = 0;
    Refroidissement = 1;

    if (fTemp2 < 40 && fTemp < 40)
    {
      //On recupere les temperature en attendant d'avoir les deux blocs a moins de 40 °C

      fValA1 = analogRead(fCapteurPin);  //On récupère la valeur de la tension de la résistance sur la PIN analogique A1

      fTension = (fValA1 * 5) / 1023.0; //calcul de la valeur de la tension
      fTemp = (fTension / 5) * 200; // 5 est la tension qu'on a pour 200 degres
      fTemp = 1.026 * fTemp + 1.33 ; // calcul de compensation de la resistance pt100 bloc cible


      fValA0 = analogRead(fCapteurPin2);  //On récupère la valeur de la tension de la résistance sur la PIN analogique A0

      fTension2 = (fValA0 * 5) / 1023.0; //calcul de la valeur de la tension
      fTemp2 = (fTension2 / 5) * 200; // 5 est la tension qu'on a pour 200 degres
      fTemp2 = 1.031 * fTemp2 ;  // calcul de compensation de la resistance pt100 bloc capteur



      Serial.print("CELL,SET,N1, ");
      fConsigne2 = 30;
      Serial.println(fConsigne2);


      Serial.print("CELL,SET,P1, ");
      fConsigne = MemoireCible + 10;
      Serial.println(fConsigne);


      etatmontee = 1;//permet d'enchainer sur le prochaine test avec une consigne cible 10 degrés supérieure
      Refroidissement = 0;
      compteur = 0;//permet de recommencer une montée avec un pallier à 40 degrés pendant 10 minutes (sans la remise à 0 on passerait directement à 50°C)
    }

  }

  //else if (fConsigne > 151 || fTemp > 155)
  else if (fConsigne > 151 ) {
    Serial.print("CELL,SET,N1, ");
    fConsigne2 = 20;
    Serial.println(fConsigne2);

    Serial.print("CELL,SET,P1, ");
    fConsigne = 20;
    Serial.println(fConsigne);

    etatmontee = 0;//pour empecher code de recommencer à chauffer
  }

}

void ValeurDepart() {

  /////////// Consigne Capteur/////////////
  Serial.println("CELL,GET,N1");   //lit la case M1 du tableau excel

  CELL2_num = Serial.parseInt();   // on met la valeur de la case dans la variable CELL_num

  fConsigne2 = (float)CELL2_num;   //la consigne vaut la valeur que l'on rentre dans la case du tableau excel
  //Serial.println("DONE");       //Le done cause des problèmes sur la version de plxdaq que l'on utilise


  //////////Consigne Cible/////////////
  Serial.println("CELL,GET,P1");  //lit la case O1 du tableau excel

  CELL_num = Serial.parseInt();  // on met la valeur de la case dans la variable CELL2_num

  fConsigne = (float)CELL_num;   //la consigne vaut la valeur que l'on rentre dans la case du tableau excel

  //Serial.println("DONE");
  /*
    //////////Emissivite///////////////
    Serial.println("CELL,GET,M3");  //lit la case L3 du tableau excel

    int CELL3_num = Serial.parseInt();  // on met la valeur de la case dans la variable CELL3_num

    fEmissivite = (float)CELL3_num;   //l'emissivite vaut la valeur que l'on rentre dans la case du tableau excel
  */

}

void ValeurDepartCA() {

  ///////Code de depart pour ecrire une valeur de la consigne sur le port serie de l'arduino et changer la valeur de la consigne sans upload a chaque fois////////
  /*
    if (Serial.available()>0){
      byte_read= Serial.readStringUntil(" ").toInt();
      fConsigne2 = byte_read;
    }

     if (Serial.available()<0){
    fConsigne2 = 0;
    }
  */

  EtatOldCA = fConsigne2;

  if (EtatOldCA == fConsigne2) {

  }

  else {
    /////////// Consigne Capteur/////////////
    Serial.println("CELL,GET,N1");   //lit la case M1 du tableau excel

    CELL2_num = Serial.parseInt();   // on met la valeur de la case dans la variable CELL_num

    fConsigne2 = (float)CELL2_num;   //la consigne vaut la valeur que l'on rentre dans la case du tableau excel
    Serial.println("DONE");

  }
}


void ValeurDepartCI() {

  EtatOldCI = fConsigne;

  if (EtatOldCI == fConsigne) {

  }

  else {
    //////////Consigne Cible/////////////
    Serial.println("CELL,GET,P1");  //lit la case O1 du tableau excel

    CELL_num = Serial.parseInt();  // on met la valeur de la case dans la variable CELL2_num

    fConsigne = (float)CELL_num;   //la consigne vaut la valeur que l'on rentre dans la case du tableau excel
    Serial.println("DONE");

    /*
      //////////Emissivite///////////////
      Serial.println("CELL,GET,M3");  //lit la case L3 du tableau excel

      int CELL3_num = Serial.parseInt();  // on met la valeur de la case dans la variable CELL3_num

      fEmissivite = (float)CELL3_num;   //l'emissivite vaut la valeur que l'on rentre dans la case du tableau excel
    */
  }
}

void AffichageExcel() {

  //Serial.print("Ambient = "); Serial.print(mlx.readAmbientTempC());                                //recupération de la température (°C) ambiante par le MLX
  //Serial.print("*C\tObject = "); Serial.print(mlx.readObjectTempC()); Serial.println("*C");        //recupération de la température (°C) objet par le MLX
  //Serial.print("Ambient = "); Serial.print(mlx.readAmbientTempF());                              //recupération de la température (°F) ambiante par le MLX
  //Serial.print("*F\tObject = "); Serial.print(mlx.readObjectTempF()); Serial.println("*F");      //recupération de la température (°F) objet par le MLX

  // Serial.println();
  //delay(500);

  float Adata = mlx.readAmbientTempC();  // Valeur de la temperature ambiante donné par le MLX, qui est stocker dans la variable Adata
  float Bdata = mlx.readObjectTempC();   // Valeur de la temperature objet donné par le MLX, qui est stocker dans la variable Bdata

  float Cdata = capteurCMS.getSensorTemp();
  
  //if ( (Adata < 200) && (Bdata < 200) ) { //sécurité pour ne pas afficher des valeurs incohérentes


    Serial.print("DATA,TIME,TIMER, ");    //ecrit l'heure dans la colonne A, le timer dans le colonne B ...

    //affichage des differentes variables sur un tableau excel//
    Serial.print(Adata);
    Serial.print(",");
    Serial.print(Bdata);
    Serial.print(",");
    
    Serial.print(Cdata);
    Serial.print(",");
   
    
    Serial.print(fTemp2);
    Serial.print(",");
    Serial.print(fTemp);

    Serial.println("");   //être sur d'avoir afficher toute les valeurs sur une même ligne on reviens à la ligne juste apres

  //}
}

void AfficheConsigne() {

  //Serial.print("Ambient = "); Serial.print(mlx.readAmbientTempC());                                //recupération de la température (°C) ambiante par le MLX
  //Serial.print("*C\tObject = "); Serial.print(mlx.readObjectTempC()); Serial.println("*C");        //recupération de la température (°C) objet par le MLX
  //Serial.print("Ambient = "); Serial.print(mlx.readAmbientTempF());                              //recupération de la température (°F) ambiante par le MLX
  //Serial.print("*F\tObject = "); Serial.print(mlx.readObjectTempF()); Serial.println("*F");      //recupération de la température (°F) objet par le MLX

  // Serial.println();


  float Adata = mlx.readAmbientTempC();  // Valeur de la temperature ambiante donné par le MLX, qui est stocker dans la variable Adata
  float Bdata = mlx.readObjectTempC();   // Valeur de la temperature objet donné par le MLX, qui est stocker dans la variable Bdata






  if ( ( Adata < 200) && (Bdata < 200) ) {


    Serial.print("DATA,TIME,TIMER, ");    //ecrit l'heure dans la colonne A, le timer dans le colonne B ...

    //affichage des differentes variables sur un tableau excel//

    Serial.print(Adata);
    Serial.print(",");
    Serial.print(Bdata);
    Serial.print(",");
    Serial.print(fTemp2);
    Serial.print(",");
    Serial.print(fTemp);
    Serial.print(",");
    Serial.print(fTemp);
    Serial.print(",");
    Serial.print(fTemp2);
    Serial.print(",");
    Serial.print(Bdata);
    Serial.print(",");
    Serial.print(Adata);

    Serial.println("");

  }
}
