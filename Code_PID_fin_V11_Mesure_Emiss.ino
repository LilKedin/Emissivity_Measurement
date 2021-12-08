/*



   Ce programme fait des Ã©chelons de 10Â°C sur le bloc capteur et arrivÃ© Ã  110Â°C fait un Ã©chelon sur le bloc cible


   Il contrÃ´le en plus les deux Ã©lectrovannes du banc permettant de redescendre en tempÃ©rature Ã  chaque fin de sÃ©rie
   plus rapidement en remontant la cloche Ã  Patm

*/
//Ajout des librairies

#include "Timer.h"
#include <Wire.h>
#include <Adafruit_MLX90614.h>

//Timer timer;

///////////////////////////////////////////////////////////////////////////////////////PT1000//////////////////////////////////////////////////////////////////////////////////

float fR0 = 100;  //Resistance pt1000 pour T=0°C
float fAlpha = 0.00385;   //Coefficient de proportionnalité
float fRx = 0;

///////////////////////////////////////////////////////////////////////////////////////EMISSIVITE//////////////////////////////////////////////////////////////////////////////////

double Ke = 1;   //Nouveau par rapport à la V10

/////////////////////////////////////////////////////////////////////////////////////BLOC CIBLE//////////////////////////////////////////////////////////////////////////////////


//Initialisation des differentes variables pour la fonction pt1000//
float fPinCible = A1;
float fValA1;   //Pin Analogique A1



float fTensionCible = 0; // variable pour la valeur de la resistance
float fTempCible = 0; //variable pour la valeur de la resistance
float MemoireCible = 0;
int Refroidissement = 0;


//PID constants//
double KPcible = 40;
double KIcible = 0;    //Pas de Ki pour bloc cible(assez stable)
double KDcible = 0;

//Niveau de la consigne
float fConsigneCible = 90;

int state = 0;
int etatmontee = 1;
int compteur = 0;
const unsigned long SECOND = 1000;
const unsigned long HOUR = 3600 * SECOND;

//Initialisation des differentes variables pour la fonction ComputePID_Cible//
unsigned long CurrentTimeCible, PreviousTimeCible;
double ElapsedTimeCible;
float fErreurCible;
float fErreurPrecedenteCible = fConsigneCible;
float fSortieCible;
float fDeltaErreurCible, fPID_Cible;  //fRes,
float fSommeErreurCible = 0;

//Initialisation LED
int iLedPinR = 12;

/////////////////////////////////////////////////////////////////////////////////////BLOC CAPTEUR/////////////////////////////////////////////////////////////////////////////////

//Initialisation des diffÃ©rentes variables pour la fonction pt1000//

/*float fR02 = 100;
  float fAlpha2 = 0.00385;
  float fRx2 = 0;*/ //deja defini plus haut
float fPinCapteur = A0;
float fValA0;
float fTensionCapteur = 0; // variable pour la valeur de la resistance
float fTempCapteur = 0; //variable pour la valeur de la resistance



//PID constants//
double KPcapteur = 25;        //JE SUIS PASSE DE 100 A 80 A 50 A 25 COMME ON EST PASSES DE PT1000 A TA(MLX)
double KIcapteur = 0;         //JE SUIS PASSE DE 0 A 5 A 2 A 1 A 0.2 A 0.1 POUR LE KI
double KDcapteur = 2000;      //On met un KD pour le bloc capteur

//Niveau de la consigne
float fConsigneCapteur = 40;
int byte_read ;

//Initialisation des diffÃ©rentes varaibles pour la fonction ComputePID_Capteur//
unsigned long CurrentTimeCapteur, PreviousTimeCapteur;
double ElapsedTimeCapteur;
float fErreurCapteur;
float fErreurPrecedenteCapteur = fConsigneCapteur;
float fSortieCapteur;
float fDeltaerreurCapteur, fPID_Capteur;   //fRes2,
float fSommeErreurCapteur = 0;

//Initialisation LED
int iLedPinV = 10;

/////////////////////////////////////////////////////////////////////////////////////MLX & EXCEL/////////////////////////////////////////////////////////////////////////////////

//Initialisation valeur MLX
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
//float Ta = mlx.readAmbientTempC();  // Valeur de la temperature ambiante donnee par le MLX, qui est stockee dans la variable Ta
//float To = mlx.readObjectTempC();   // Valeur de la temperature objet donnee par le MLX, qui est stockee dans la variable To

//Initialisation valeur Excel
float fEmissivite = 0;
float CELL_num = 0;
float CELL2_num = 0;
float EtatOldCA = 0;
float EtatOldCI = 0;

/////////////////////////////////////////////////////////////////////////////////////ELECTROVANNE/////////////////////////////////////////////////////////////////////////////////

int solenoidPompePin = 5;    //On associe la pin 5 a  l'electrovanne cote pompe pour la mise a vide
int solenoidClochePin = 4;   //On associe la pin 4 a  l'electrovanne cote cloche pour la remise a Patm
int i = 0;

/////////////////////////////////////////////////////////////////////////////////////Partie Code/////////////////////////////////////////////////////////////////////////////////


void setup() {

  Serial.begin(9600);
  pinMode(3, OUTPUT);  // on met la pin 3 (PWM) en lecture de sortie
  pinMode(11, OUTPUT); // on met la pin 11 (PWM) en lecture de sortie
  pinMode(iLedPinV, OUTPUT); // on met la pin 12 en lecture de sortie
  pinMode(iLedPinR, OUTPUT); // on met la pin 10 (PWM) en lecture de sortie

  pinMode(solenoidPompePin, OUTPUT);           //On met l'electrovanne sur la pompe en sortie
  pinMode(solenoidClochePin, OUTPUT);          //On met l'electrovanne sur la cloche en sortie

  digitalWrite(solenoidPompePin, HIGH);    //On ouvre la vanne pour pouvoir pomper
  digitalWrite(solenoidClochePin, LOW);    //On ferme la vanne pour ne pas remonter en pression

//////////////////////////////////////////////////////////////////////////////INITIALISATION  EXCEL///////////////////////////////////////////////////////////////////////////////  


  Serial.println("CLEARDATA"); // remet a jour le logiciel d'acquisition
  Serial.println("LABEL,Heure,Timer,Tambiante,Tobjet,Pt1000capteur,Pt1000cible,PWMcapteur,PWMcible,Emissivite,Valeur du Proportionnel P,Valeur du Proportionnel D"); //IntitulÃ©s de chaque colonne
  Serial.println("RESETTIMER"); //remet le timer Ã  0

///////////code qui lit la consigne sur une case du tableau excel //////////

  Serial.print("CELL,SET,L1, ");
  Serial.println("Consigne Capteur");

  Serial.print("CELL,SET,N1, ");
  Serial.println("Consigne Cible");

  mlx.begin();    //Initialisation du capteur
}



void loop() {

  ValeurDepart();
  //ValeurDepartCI();
  //ValeurDepartCA();

  TempPt1000Cible();  //appel de la fonction qui convertit la valeur de la tension aux bornes de la pt1000 en temperature
  TempPt1000Capteur();

  fSortieCible = ComputePID_Cible();     //Recupere la valeur du PID qui est dans la fonction ComputePID_Cible, on affecte a notre variable fSortieCible
  analogWrite(11, 0); //On ecrit en sortie PWM sur la pin 11
  analogWrite(3, 0); //On ecrit en sortie PWM sur la pin 3

  delay(1);
  fSortieCapteur = ComputePID_Capteur(); //Recupere la valeur du PID qui est dans la fonction ComputePID_Capteur, on affecte a notre variable fSortieCapteur
  delay(1);

  //analogWrite(3, fSortieCible);  //On Ã©crit en sortie PWM sur la pin 3
  //analogWrite(11, fSortieCapteur);  //On Ã©crit en sortie PWM sur la pin 11


  // analogWrite(11, fSortieCapteur); //On Ã©crit en sortie PWM sur la pin 11
  // analogWrite(3, fSortieCible); //On Ã©crit en sortie PWM sur la pin 3

  if ( fSortieCible > 255) {       //Conversion de la valeur finale du PID en 8bit pour notre sortie PWM bloc cible
    fSortieCible = 255;
  }

  else if ( fSortieCible < 0) {
    fSortieCible = 0;
  }

  if ( fSortieCapteur > 255) {       //Conversion de la valeur finale du PID en 8bit pour notre sortie PWM bloc capteur
    fSortieCapteur = 255;
    digitalWrite(iLedPinV, HIGH);// On allume la diode verte quand on est en phase de chauffe totale (suite a  un echelon par exemple)

  }
  else if ( fSortieCapteur < 0) {
    fSortieCapteur = 0;
    digitalWrite(iLedPinR, HIGH);// On allume la diode rouge quand le PWM est eteint afin de montrer que le systeme refroidit (lorsque l'on depasse la temperature de consigne par exemple)
  }
  analogWrite(11, 0); //On ecrit en sortie PWM sur la pin 11
  analogWrite(3, 0); //On ecrit en sortie PWM sur la pin 3

  CalculEmissivite();

  delay(1);
  AffichageExcel();
  delay(1);

  //analogWrite(3, fSortieCible);     //On ecrit en sortie PWM sur la pin 3
  //analogWrite(11, fSortieCapteur);  //On ecrit en sortie PWM sur la pin 11



  analogWrite(11, 0); //On ecrit en sortie PWM sur la pin 11
  analogWrite(3, 0);  //On ecrit en sortie PWM sur la pin 3

  delay(1);
  TestConsigne(); //A commenter si on ne veut pas d'echelon en temperature
  delay(1);

  analogWrite(3, fSortieCible);  //On ecrit en sortie PWM sur la pin 3
  analogWrite(11, fSortieCapteur);  //On ecrit en sortie PWM sur la pin 11

  //Affichage();                  //appel de la fonction Affichage, affiche les donnees dans le port Serie Arduino
  //Affichage2();                 //appel de la fonction Affichage, affiche les donnees dans le port Serie Arduino
  digitalWrite(iLedPinV, LOW);    //On eteint les leds afin d'afficher a nouveau l'etat du PWM lors du prochain cycle du programme
  digitalWrite(iLedPinR, LOW);

  delay(4000);                //Cette valeur correspond au delai en ms

///// Code pour eviter de suralimenter l'électrovanne trop longtemps (On alimente en 30V au lieu de 24V)
  
  i++;
  if (i < 15){
    
  }
  else {
    digitalWrite(solenoidPompePin, LOW);    //On ferme la vanne pour éviter la surchauffe
  }
}

///////////////////////////////////////////////////////////////////////////////////// BLOC CIBLE //////////////////////////////////////////////////////////////////////////////////


void TempPt1000Cible() {

  fValA1 = analogRead(fPinCible);  //On recupere la valeur de la tension de la resistance sur la PIN analogique A1(cible)

  fTensionCible = (fValA1 * 5) / 1023.0; //calcul de la valeur de la tension

  // fRx = (fTensionCible * 15) + 100;  // calcul de la valeur de la resistance

  //fTempCible = ((fRx / fR0)-1)/fAlpha;  // calcul pour trouver la valeur de la temperature en fonction de la valeur de la resistance obtenue precedement

  fTempCible = (fTensionCible / 5) * 200; // 5 est la tension qu'on a pour 200 degres
  //fTempCible = 1.026 * fTempCible + 1.33 ;  // calcul de compensation de la resistance pt1000 pour bloc cible carte Arduino A1
  fTempCible = 1.0176 * fTempCible - 5.83;    // pour carte Arduino A3

}

float ComputePID_Cible() {

  CurrentTimeCible = millis();                                                    //On recupere le temps en milliseconde
  ElapsedTimeCible = (float)(CurrentTimeCible - PreviousTimeCible);               //Calcule le temps ecoule a partir du calcul precedent

  if ((fTempCible < (fConsigneCible - 1)) || (fTempCible > (fConsigneCible + 1))) {
    fErreurCible = fConsigneCible - fTempCible;                                   //Determine l'erreur avec calcul du proportionnel  (P)
    fPID_Cible = KPcible * fErreurCible;
    fSommeErreurCible = 0;

  }

  else  {

    fErreurCible = fConsigneCible - fTempCible;
    fSommeErreurCible += fErreurCible;  // * ElapsedTimeCible;                            // calcul de l'integrale  (I)
    //fDeltaErreurCible = (fErreurCible - fErreurPrecedenteCible)/ElapsedTimeCible;       // calcul de la derivee   (D)
    fPID_Cible = KPcible * fErreurCible + KIcible * fSommeErreurCible; // + KDcible*fDeltaErreurCible;                //PID output
    //digitalWrite(iLedPinV, HIGH);

  }


  fErreurPrecedenteCible = fErreurCible;                                   //Rappel de l'erreur actuelle
  PreviousTimeCible = CurrentTimeCible;                                    //Rappel du temps actuel

  return fPID_Cible;                                                 //retour du PID en sortie

}


void Affichage() {

  ////////////Affichage des differents parametres dans le port Serie////////////

  Serial.println("");
  Serial.print(CurrentTimeCible / 1000);
  Serial.print(" Temps");
  Serial.print("   ");
  Serial.print("BLOC CIBLE");
  Serial.print("   ");
  Serial.print(fTempCible);
  Serial.print(" degres");
  Serial.print("   ");
  Serial.print(fSortieCible);
  Serial.print(" Pwm");
  Serial.print("   ");
  Serial.print(fSommeErreurCible);
  Serial.print(" somme erreur");
  Serial.print("   ");
  Serial.print(fTensionCible);
  Serial.print(" volts");
  Serial.print("   ");
  //Serial.print(fRx);
  //Serial.print(" val resistance");
  // Serial.print("   ");
  Serial.print(fConsigneCapteur);
  Serial.print(" Valeurs consigne capteurs");
  Serial.print("   ");

  Serial.print("");



}


/////////////////////////////////////////////////////////////////////////////////////BLOC CAPTEUR/////////////////////////////////////////////////////////////////////////////////

void TempPt1000Capteur() {

  fValA0 = analogRead(fPinCapteur);  //On recupere la valeur de la tension de la resistance sur la PIN analogique A0(capteur)

  fTensionCapteur = (fValA0 * 5) / 1023.0; //calcul de la valeur de la tension

  //fRx = (fTensionCapteur * 15) + 100;  // calcul de la valeur de la resistance

  //fTempCapteur = ((fRx / fR0)-1)/fAlpha;  // calcul pour trouver la vlaeur de la temperature en fonction de la valeur de la rÃ©siatsnce obtenu prÃ©cÃ©dement

  fTempCapteur = (fTensionCapteur / 5) * 200; // 5 est la tension qu'on a pour 200 degres
  //fTempCapteur = 1.031 * fTempCapteur ;  // calcul de compensation de la resistance pt1000 bloc capteur carte Arduino A1
  fTempCapteur = 1.0313 * fTempCapteur - 5.65; // carte Arduino A3

}

float ComputePID_Capteur() {

  float Ta = mlx.readAmbientTempC();  // Valeur de la temperature ambiante par le MLX, qui est stockee dans la variable Ta

  CurrentTimeCapteur = millis();                                                //On recupere le temps en milliseconde
  ElapsedTimeCapteur = (float)(CurrentTimeCapteur - PreviousTimeCapteur);       //Calcule le temps ecoule a partir du calcul precedent
  ElapsedTimeCapteur /= 1000;   // On divise par 1000 car D trop petit


  if ((Ta < (fConsigneCapteur - 10)) || (Ta > (fConsigneCapteur + 10))) {
    fErreurCapteur = fConsigneCapteur - Ta;                                                     // Determine avec calcul du proportionnel  (P)
    fDeltaerreurCapteur = (fErreurCapteur - fErreurPrecedenteCapteur) / ElapsedTimeCapteur;     // Calcul de la derivee (D)
    fPID_Capteur = KPcapteur * fErreurCapteur + KDcapteur * fDeltaerreurCapteur;
    fSommeErreurCapteur = 0;

  }
  //On retire la partie if +-1 degrés car on va appliquer le calcul Ki pour toutes les températures, on veut réduire la marge statique

  else  {
    fErreurCapteur = fConsigneCapteur - Ta;
    fSommeErreurCapteur += fErreurCapteur;  // * ElapsedTimeCapteur;                             // Calcul de l'integrale (I)
   // fPID_Capteur = KPcapteur * fErreurCapteur + KIcapteur * fSommeErreurCapteur;               //PID output avec PI
    //digitalWrite(iLedPinR, HIGH);


    fDeltaerreurCapteur = (fErreurCapteur - fErreurPrecedenteCapteur) / ElapsedTimeCapteur;      // Calcul de la derivee (D)
    fPID_Capteur = KPcapteur * fErreurCapteur + KIcapteur * fSommeErreurCapteur + KDcapteur * fDeltaerreurCapteur; //PID output   avec PID
  }


  fErreurPrecedenteCapteur = fErreurCapteur;                                   //Rappee de l'erreur actuelle
  PreviousTimeCapteur = CurrentTimeCapteur;                                    //Rappel du temps actuelle

  return fPID_Capteur;                                                 //Retour du PID en sortie

}

void Affichage2() {

  ////////////Affichage des differents parametres dans le port Serie////////////

  Serial.print("");
  Serial.print("   ");
  Serial.print("BLOC CAPTEUR");
  Serial.print("   ");
  Serial.print(fTempCapteur);
  Serial.print(" degres");
  Serial.print("   ");
  Serial.print(fSortieCapteur);
  Serial.print(" Pwm");
  Serial.print("   ");
  Serial.print(fSommeErreurCapteur);
  Serial.print(" somme erreur");
  Serial.print("   ");
  Serial.print(fTensionCapteur);
  Serial.print(" volts");
  Serial.print("   ");
  //Serial.print(fRx);
  //Serial.print(" val resistance");
  //Serial.print("   ");
  Serial.print("");
  Serial.print((int)fConsigneCible);
  Serial.print(" Valeur consigne");
  Serial.print("  ");


}

void TestConsigne() {

  int cpt = 0;
  float Ta = mlx.readAmbientTempC();   // Valeur de la temperature ambiante par le MLX, qui est stockee dans la variable Ta

  /*if ((fTempCapteur >= (fConsigneCapteur - 1 )) && (fTempCible >= (fConsigneCible - 1))) {

    delay (20000);
    for (int iBcl2; iBcl2 < 2; iBcl2++) {

      AfficheConsigne();
      delay (500);
    }*/
  //delay(1000);

  //digitalWrite(iLedPinR, HIGH);



  //if(compteur >= 18000)
  if (compteur >= 149)                 //On fixe un delai entre chaque changement de consigne en temperature = delay * ( ncompteur + 1 )
  {

    if (etatmontee == 1)                //1. Si on est a l'etat de montee 1 on augmente de 10Â°C jusqu'a 120Â°C
    {
      Serial.print("CELL,SET,M1, ");          //Change M1 par O1
      fConsigneCapteur += 10;                 //Change capteur par cible jeudi 14/01/20
      // fSommeErreurCapteur = 0;             //PAS DANS LES ANCIENNES VERSIONS DE CODE
      Serial.println(fConsigneCapteur);       //Change capteur par cible jeudi 14/01/20
    }
    /*if (fConsigneCapteur >= 130 && etatmontee == 1) //2. Ensuite quand on arrive Ã  120 Â°C l'etat de montee passe Ã  0
      {
      Serial.print("CELL,SET,M1, ");
      fConsigneCapteur -= 10;
      Serial.println(fConsigneCapteur);
      etatmontee = 0;
      }
      if (fConsigneCapteur <= 70 && etatmontee == 0) //4. A 70Â°C on change l'etat de montee a 1 et on recommence la montee de temperature de 10 en 10
      {
      Serial.print("CELL,SET,M1, ");
      fConsigneCapteur += 10;
      Serial.println(fConsigneCapteur);
      etatmontee = 1;
      }
      else  if (etatmontee == 0)                          //3. La consigne diminue de 10 en 10 jusqu'a 70Â°C
      {
      // digitalWrite(iLedPinV, HIGH);
      Serial.print("CELL,SET,M1, ");
      fConsigneCapteur -= 10;
      Serial.println(fConsigneCapteur);

      }*/

    if (fConsigneCapteur == 100)
    {
      Serial.println("CELL,GET,O1");  //lit la case O1 du tableau excel

      MemoireCible = Serial.parseFloat();   //On met la valeur de la case dans la variable CELL2_num, cela permet d'incrementer la temperature cible pour le test suivant(on les enchaine)

    }
    compteur = 0;                           //Reinitialisation du compteur
  }
  else
  {
    compteur += 1;                          //Incrementation du compteur
  }

  /*
    if (state == 0 && compteur == 0 ) {

      Serial.print("CELL,SET,O1, ");
      fConsigneCible = fConsigneCible - 60;
      Serial.println(fConsigneCible);
      compteur = compteur + 1;
      state=1;
    }
      if (state == 1 && compteur < 3600 ) {

      Serial.print("CELL,SET,O1, ");
      fConsigneCible = fConsigneCible;
      Serial.println(fConsigneCible);
      compteur = compteur +1;
    }


    if (compteur == 3600) {

      Serial.print("CELL,SET,O1, ");
      fConsigneCible = fConsigneCible + 60;
      Serial.println(fConsigneCible);
      compteur = compteur -1;
      state=0;
    }

    if (state == 0 && compteur < 3600 && compteur > 0 ) {

      Serial.print("CELL,SET,O1, ");
      fConsigneCible = fConsigneCible;
      Serial.println(fConsigneCible);
      compteur = compteur-1;
    }
  */

  int i2 = 0;
  if (fConsigneCapteur > 121.00 || Refroidissement == 1) {

    float Ta = mlx.readAmbientTempC();   // Valeur de la temperature ambiante par le MLX, qui est stockee dans la variable Ta

    Serial.print("CELL,SET,M1, ");
    fConsigneCapteur = 20;
    Serial.println(fConsigneCapteur);

    Serial.print("CELL,SET,O1, ");
    fConsigneCible = 20;
    Serial.println(fConsigneCible);

    etatmontee = 0;
    Refroidissement = 1;

    digitalWrite(solenoidPompePin, LOW);            //On ferme la vanne pour ne plus pomper
    digitalWrite(solenoidClochePin, HIGH);          //On ouvre la vanne pour remonter en pression   MIS A LOW le 15/01/2021

    if (Ta < 40 && fTempCible < 40)
    {
      //On recupere les temperature en attendant d'avoir les deux blocs a moins de 40 Â°C

      fValA1 = analogRead(fPinCible);  //On recupere la valeur de la tension de la resistance sur la PIN analogique A1(cible)

      fTensionCible = (fValA1 * 5) / 1023.0;        //calcul de la valeur de la tension
      fTempCible = (fTensionCible / 5) * 200;       // 5 est la tension qu'on a pour 200 degres
      fTempCible = 1.026 * fTempCible + 1.33 ;      // calcul de compensation de la resistance pt1000 bloc cible


      fValA0 = analogRead(fPinCapteur);  //On recupere la valeur de la tension de la resistance sur la PIN analogique A0(capteur)

      fTensionCapteur = (fValA0 * 5) / 1023.0;      //calcul de la valeur de la tension
      fTempCapteur = (fTensionCapteur / 5) * 200;   // 5 est la tension qu'on a pour 200 degres
      fTempCapteur = 1.031 * fTempCapteur ;         // calcul de compensation de la resistance pt1000 bloc capteur



      Serial.print("CELL,SET,M1, ");
      fConsigneCapteur = 30;
      Serial.println(fConsigneCapteur);


      Serial.print("CELL,SET,O1, ");
      fConsigneCible = MemoireCible + 10;
      Serial.println(fConsigneCible);


      etatmontee = 1;//permet d'enchainer sur le prochaine test avec une consigne cible 10 degrÃ©s supÃ©rieure
      Refroidissement = 0;
      compteur = 0;//permet de recommencer une montÃ©e avec un pallier Ã  40 degrÃ©s pendant 10 minutes (sans la remise Ã  0 on passerait directement Ã  50Â°C)

      digitalWrite(solenoidPompePin, HIGH);         //On ouvre la vanne pour pouvoir pomper
      digitalWrite(solenoidClochePin, LOW);         //On ferme la vanne pour ne pas remonter en pression

      
      i2++;
    
      if (i2 < 15){
        
      }
      else {
        digitalWrite(solenoidPompePin, LOW);        //On ouvre la vanne pour pouvoir pomper
      }
  
    }

  }

  //else if (fConsigneCible > 151 || fTempCible > 155)      A CHANGER POUR 121 APRES TESTS EPOXY
  if (fConsigneCible > 121.00 ) //Temperature Cible MAx acceptee
  {
    Serial.print("CELL,SET,M1, ");
    fConsigneCapteur = 20;
    Serial.println(fConsigneCapteur);

    Serial.print("CELL,SET,O1, ");
    fConsigneCible = 20;
    Serial.println(fConsigneCible);

    etatmontee = 0;//pour empecher code de recommencer Ã  chauffer
    digitalWrite(solenoidPompePin, LOW);            //On ferme la vanne pour ne plus pomper
    digitalWrite(solenoidClochePin, HIGH);          //On ouvre la vanne pour remonter en pression
    delay(5000);                                    //On ferme la vanne apres 5 secondes pour eviter de faire fondre la vanne
    digitalWrite(solenoidClochePin, LOW);           //On ferme la vanne apres 5 secondes pour eviter de faire fondre la vanne
  }


  /*if ( To > fTempCible + 25) //10 fois d'affilÃ©e
    {
    {
      cpt++;
    }
    if (cpt == 10)
    {

      Serial.print("CELL,SET,M1, ");
      fConsigneCapteur = 20;
      Serial.println(fConsigneCapteur);

      Serial.print("CELL,SET,O1, ");
      fConsigneCible = 20;
      Serial.println(fConsigneCible);

      etatmontee = 0;//pour empecher code de recommencer Ã  chauffer
      digitalWrite(solenoidPompePin, LOW);    //On ferme la vanne pour ne plus pomper
      digitalWrite(solenoidClochePin, HIGH);    //On ouvre la vanne pour remonter en pression

      cpt == 0;
    }
    }

    else if ( To <= fTempCible + 25)
    {
    cpt == 0;
    }*/
}

void ValeurDepart() {

  /////////// Consigne Capteur/////////////
  Serial.println("CELL,GET,M1");   //lit la case M1 du tableau excel

  CELL2_num = Serial.parseFloat();   // on met la valeur de la case dans la variable CELL_num

  fConsigneCapteur = (float)CELL2_num;   //la consigne vaut la valeur que l'on rentre dans la case du tableau excel
  //Serial.println("DONE");       //Le done cause des problemes sur la version de plxdaq que l'on utilise


  //////////Consigne Cible/////////////
  Serial.println("CELL,GET,O1");  //lit la case O1 du tableau excel

  CELL_num = Serial.parseFloat();  // on met la valeur de la case dans la variable CELL2_num

  fConsigneCible = (float)CELL_num;   //la consigne vaut la valeur que l'on rentre dans la case du tableau excel

  //Serial.println("DONE");
  /*
    //////////Emissivite///////////////
    Serial.println("CELL,GET,M3");  //lit la case L3 du tableau excel

    int CELL3_num = Serial.parseFloat();  // on met la valeur de la case dans la variable CELL3_num

    fEmissivite = (float)CELL3_num;   //l'emissivite vaut la valeur que l'on rentre dans la case du tableau excel
  */

}

void ValeurDepartCA() {

  ///////Code de depart pour ecrire une valeur de la consigne sur le port serie de l'Arduino et changer la valeur de la consigne sans upload a chaque fois////////
  /*
    if (Serial.available()>0){
      byte_read= Serial.readStringUntil(" ").toInt();
      fConsigneCapteur = byte_read;
    }

     if (Serial.available()<0){
    fConsigneCapteur = 0;
    }
  */

  EtatOldCA = fConsigneCapteur;

  if (EtatOldCA == fConsigneCapteur) {

  }

  else {
    /////////// Consigne Capteur/////////////
    Serial.println("CELL,GET,M1");   //lit la case M1 du tableau excel

    CELL2_num = Serial.parseFloat();   // on met la valeur de la case dans la variable CELL_num

    fConsigneCapteur = (float)CELL2_num;   //la consigne vaut la valeur que l'on rentre dans la case du tableau excel
    Serial.println("DONE");

  }
}


void ValeurDepartCI() {

  EtatOldCI = fConsigneCible;

  if (EtatOldCI == fConsigneCible) {

  }

  else {
    //////////Consigne Cible/////////////
    Serial.println("CELL,GET,O1");  //lit la case O1 du tableau excel

    CELL_num = Serial.parseFloat();  // on met la valeur de la case dans la variable CELL2_num

    fConsigneCible = (float)CELL_num;   //la consigne vaut la valeur que l'on rentre dans la case du tableau excel
    Serial.println("DONE");

    /*
      //////////Emissivite///////////////
      Serial.println("CELL,GET,M3");  //lit la case L3 du tableau excel

      int CELL3_num = Serial.parseFloat();  // on met la valeur de la case dans la variable CELL3_num

      fEmissivite = (float)CELL3_num;   //l'emissivite vaut la valeur que l'on rentre dans la case du tableau excel
    */
  }
}

void CalculEmissivite() {

  float Ta = mlx.readAmbientTempC();  // Valeur de la temperature ambiante donnee par le MLX, qui est stockee dans la variable Ta
  float To = mlx.readObjectTempC();   // Valeur de la temperature objet donne par le MLX, qui est stockee dans la variable To

  Ke = ((pow(To+273.15, 4) - pow(Ta+273.15, 4)) / (pow(fTempCible+273.15, 4) - pow(Ta+273.15, 4)));
  //Ke = 1.23456;
}


void AffichageExcel() {

  //Serial.print("Ambient = "); Serial.print(mlx.readAmbientTempC());                                 //recuperation de la temperature (Â°C) ambiante par le MLX
  //Serial.print("*C\tObject = "); Serial.print(mlx.readObjectTempC()); Serial.println("*C");         //recuperation de la temperature(Â°C) objet par le MLX
  //Serial.print("Ambient = "); Serial.print(mlx.readAmbientTempF());                                 //recuperation de la temperature (Â°F) ambiante par le MLX
  //Serial.print("*F\tObject = "); Serial.print(mlx.readObjectTempF()); Serial.println("*F");         //recuperation de la temperature (Â°F) objet par le MLX

  // Serial.println();
  //delay(500);

  float Ta = mlx.readAmbientTempC();  // Valeur de la temperature ambiante donnee par le MLX, qui est stockee dans la variable Ta
  float To = mlx.readObjectTempC();   // Valeur de la temperature objet donnee par le MLX, qui est stockee dans la variable To

  //if ( (Ta < 200) && (To < 200) ) { //securite pour ne pas afficher des valeurs incohÃ©rentes


  Serial.print("DATA,TIME,TIMER, ");    //ecrit l'heure dans la colonne A, le timer dans le colonne B ...

  //affichage des differentes variables sur un tableau excel//
  Serial.print(Ta);
  Serial.print(",");
  Serial.print(To);
  Serial.print(",");
  Serial.print(fTempCapteur);
  Serial.print(",");
  Serial.print(fTempCible);
  Serial.print(",");
  Serial.print(fSortieCapteur);  //valeur PWM capteur
  Serial.print(",");
  Serial.print(fSortieCible);   //valeur PWM cible
  Serial.print(",");
  Serial.print(Ke, 3);   //valeur emissivité calculée Ke avec 3 decimales
  Serial.print(",");
  Serial.print(KPcapteur * fErreurCapteur); //Valeur du Proportionnel P
  Serial.print(",");
  Serial.print(KDcapteur * fDeltaerreurCapteur); //Valeur du Derive D



  Serial.println("");   //Etre sur d'avoir affiche toute les valeurs sur une meme ligne puis on revient a la ligne

  // }
}

void AfficheConsigne() {

  //Serial.print("Ambient = "); Serial.print(mlx.readAmbientTempC());                                //recupÃ©ration de la tempÃ©rature (Â°C) ambiante par le MLX
  //Serial.print("*C\tObject = "); Serial.print(mlx.readObjectTempC()); Serial.println("*C");        //recupÃ©ration de la tempÃ©rature (Â°C) objet par le MLX
  //Serial.print("Ambient = "); Serial.print(mlx.readAmbientTempF());                              //recupÃ©ration de la tempÃ©rature (Â°F) ambiante par le MLX
  //Serial.print("*F\tObject = "); Serial.print(mlx.readObjectTempF()); Serial.println("*F");      //recupÃ©ration de la tempÃ©rature (Â°F) objet par le MLX

  // Serial.println();


  float Ta = mlx.readAmbientTempC();  // Valeur de la temperature ambiante donnee par le MLX, qui est stocker dans la variable Ta
  float To = mlx.readObjectTempC();   // Valeur de la temperature objet donnee par le MLX, qui est stocker dans la variable To

  // if ( ( Ta < 200) && (To < 200) ) {


  Serial.print("DATA,TIME,TIMER, ");    //ecrit l'heure dans la colonne A, le timer dans le colonne B ...

  //affichage des differentes variables dans un tableau excel//

  Serial.print(Ta);
  Serial.print(",");
  Serial.print(To);
  Serial.print(",");
  Serial.print(fTempCapteur);
  Serial.print(",");
  Serial.print(fTempCible);
  Serial.print(",");
  Serial.print(fTempCible);
  Serial.print(",");
  Serial.print(fTempCapteur);
  Serial.print(",");
  Serial.print(To);
  Serial.print(",");
  Serial.print(Ta);

  Serial.println("");

  // }
}
