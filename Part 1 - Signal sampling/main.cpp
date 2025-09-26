#include <Arduino.h>
#define PinTimer 2
#define RC 0 //Todo: Définir la valeur trouver pour le registre de comparaison


// =================================Buffer===========================================
#define BUFFER_SIZE 45
#define OUTPUT_SIZE 8000

// const static int16_t filter_taps[BUFFER_SIZE];
static double filter_taps[BUFFER_SIZE] = {
  0.09653605376151034,
  -0.015660421368965574,
  -0.05010901129516925,
  -0.09818434263150769,
  -0.14734325760075245,
  -0.18390834549736335,
  0.802596527932073,
  -0.18390834549736335,
  -0.14734325760075245,
  -0.09818434263150769,
  -0.05010901129516925,
  -0.015660421368965574,
  0.09653605376151034
};
uint16_t filteredSignal[OUTPUT_SIZE];

void CircularBuffer() {
  // int tempoBuffer[BUFFER_SIZE];
  // int adcBufferIndex = 0;
  // for(int i=0; i < OUTPUT_SIZE; i++) {
  //   Serial.println("Avant !");
  //   while((ADC->ADC_ISR & 0x80)==0); // attente de la fin de la conversion
  //   Serial.println("Avant !");
  //   tempoBuffer[adcBufferIndex] = ADC->ADC_CDR[0]; // sauvegarde du signal dans le buffer 

  //   adcBufferIndex++;   // incrementation de l'index du buffer cirulaire
  //   uint16_t sumIndex = adcBufferIndex; // update de l'index pour le bon indexage des valeurs en fonction des coeffs 
  //   if(adcBufferIndex == BUFFER_SIZE) { // modulo pour remettre à 0 
  //     adcBufferIndex = 0;
  //   }
    
  //   // Calcul du filtrage sur les 15 valeurs du buffer 
  //   uint32_t acc = 0;
    
  //   for (int l = 0; l < BUFFER_SIZE; l++) { // multiplication des valeurs du buffer par les coeffs

  //     // gestion de l'index interne au buffer pour le calcul du filtre
  //     if( sumIndex > 0 ) {
  //       sumIndex--;
  //     } else {
  //       sumIndex=BUFFER_SIZE-1;
  //     }
  //     // calcule du filtre sur 32 bit
  //     acc += filter_taps[l] * tempoBuffer[sumIndex];
      
  //   }
  //   filteredSignal[i] =  acc >> 15; // shift de 15 bit pour repasser sur 16 bit
  // }
  // for (int i = 0; i < BUFFER_SIZE; i++)
  // {
  //   Serial.print(">tempoBuffer :");
  //   Serial.println(tempoBuffer[i]);
  // }
  
  // if(ADC->ADC_ISR & ADC_ISR_EOC7) { // Fin de conversion sur canal 7
    Serial.println("AAA");
    static int tempoBuffer[BUFFER_SIZE] = {0};
    static int adcBufferIndex = 0;
    static uint16_t i = 0;

    // Ajoute valeur dans le buffer circulaire
    tempoBuffer[adcBufferIndex] = ADC->ADC_CDR[7];
    int sumIndex = adcBufferIndex;

    Serial.print("rawSignal :");
    Serial.println(tempoBuffer[adcBufferIndex]);

    adcBufferIndex++;
    if(adcBufferIndex == BUFFER_SIZE)
      adcBufferIndex = 0;

    

    // Filtrage FIR
    int32_t acc = 0;
    for (int l = 0; l < BUFFER_SIZE; l++) {
      sumIndex = (sumIndex == 0) ? BUFFER_SIZE - 1 : sumIndex - 1;
      acc += filter_taps[l] * tempoBuffer[sumIndex];
    }

    uint16_t filteredVal = acc >> 15;
    // Serial.println(filteredVal);
    filteredSignal[i++] = filteredVal;
    if (i >= OUTPUT_SIZE) i = 0;

    // Envoi sur DAC
    DACC->DACC_CDR = DACC_CDR_DATA(filteredVal);
    while (!(DACC->DACC_ISR & DACC_ISR_TXRDY)); // attend que le DAC soit prêt
  // }
  


}


// =================================Timer===========================================
void setupTimer(){
  /*Todo:
    Partie 1:
    1 - Activer le périphérique Timer
    2 - Configurer le registre TC_CMR
    3 - Valeur du registre de comparaison
    4 - Activation de l'interruption par RC Compare
    5 - Activation de l'interruption dans le NVIC de la Due
  */

  PMC->PMC_PCER0 |= (1 << ID_TC0); // 1. Activer le périphérique Timer 0 (canal 0)

  TC0->TC_CHANNEL[0].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1 // 2. Clock = MCK/2 (42MHz)
                            | TC_CMR_WAVE                // Mode Waveform
                            | TC_CMR_WAVSEL_UP_RC        // Comptage jusqu'à RC
                            | TC_CMR_ACPA_SET            // SET sur A lors de comparaison RA
                            | TC_CMR_ACPC_CLEAR;         // CLEAR sur C (RC)

  TC0->TC_CHANNEL[0].TC_RC = RC; // 3. Définir la valeur de comparaison

  TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS; // 4. Interruption sur RC Compare
  NVIC_EnableIRQ(TC0_IRQn);                // 5. Activer l'interruption dans le NVIC

  TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG; // Démarre le timer

}

void TC0_Handler(){
  /*Todo:
    Partie 1:
    1 - Lire et effacer le drapeau d'interruption
    2 - Changer l'état du pin timer pour analyser la fréquence du timer sur oscilloscope

    Partie 2:
    1 - Lancer manuellement la conversion de l'ADC

  */
  TC0->TC_CHANNEL[0].TC_SR; // 1. Lire pour effacer le flag d’interruption
  
  digitalWrite(PinTimer, !digitalRead(PinTimer)); // 2. Toggle le pin pour visualiser la fréquence
  
  ADC->ADC_CR = ADC_CR_START; // 1. Lancer manuellement la conversion ADC
}

void setupADC(){
  /* Todo:
    Partie 2:
    1 - Activer le périphérique ADC
    2 - Configurer l'ADC dans ADC_MR
    3 - Selectionner le channel d'entrée de l'ADC 
  */
  PMC->PMC_PCER1 |= PMC_PCER1_PID37; // 1. Activer le périphérique ADC

  ADC->ADC_MR = ADC_MR_PRESCAL(10)   // 2. Prescaler: horloge ADC = MCK / ((PRESCAL+1)*2)
              | ADC_MR_STARTUP_SUT64 // Temps de démarrage
              | ADC_MR_TRACKTIM(15)  // Temps d’échantillonnage
              | ADC_MR_TRANSFER(2);  // Temps de transfert interne

  ADC->ADC_CHER = ADC_CHER_CH7;      // 3. Activer le canal 7 (A0)
}

void setupDAC(){
  PMC->PMC_PCER1 |= PMC_PCER1_PID38; // Activer le périphérique DAC

  DACC->DACC_MR = DACC_MR_TRGEN_DIS
                | DACC_MR_USER_SEL_CHANNEL1
                | DACC_MR_WORD_HALF
                | DACC_MR_REFRESH(1)
                | DACC_MR_STARTUP_8
                | DACC_MR_MAXS;

  DACC->DACC_CHER = DACC_CHER_CH1;
  DACC->DACC_IER |= DACC_IER_EOC;
  NVIC_EnableIRQ(DACC_IRQn);
}

void DACC_Handler() {
  DACC->DACC_ISR;  //  effacer le register d’état “status register”
}

void setup() {
  Serial.begin(9600); //Baud rate au maximum de l'Arduino Due
  pinMode(PinTimer, OUTPUT); //Pin pour analyser la freq du timer
  /* Todo:
    Partie 1:
    1 - setup le timer et le lancer
    Partie 2: 
    1 - setup l'ADC 
    2 - setup le DAC
  */
  setupTimer(); // Partie 1 : setup du timer
  setupADC();   // Partie 2 : setup ADC
  setupDAC();   // Partie 2 : setup DAC

  analogReadResolution(12);

  
  
}

void loop() {
  //Partie 2:
  /*if(ADC->ISR & XXXXXXXX){ //Masque binaire à compléter en fonction du channel de l'ADC
    //Todo: 
    //Partie 2:
    // 1 - Afficher des valeurs de l'ADC sur le moniteur série
    // 2 - Sortie sur le DAC des valeurs de l'ADC avec les lignes suivantes:
    // DACC->DACC_CDR = DACC_CDR_DATA(valeur_de_l'ADC);
    // while (!(DACC->DACC_ISR & DACC_ISR_TXRDY));
  }*/


  // if (ADC->ADC_ISR & ADC_ISR_EOC7) { // Fin de conversion sur le canal 7
  //   uint16_t val = ADC->ADC_CDR[7]; // Lecture du résultat ADC
  //   Serial.println(val);           // Affichage série
  //   DACC->DACC_CDR = DACC_CDR_DATA(val); // Écriture dans le DAC
  //   while (!(DACC->DACC_ISR & DACC_ISR_TXRDY)); // Attente du DAC prêt
  // }

  // Serial.println("Marche 1 !");

  CircularBuffer();

  // Serial.println("Marche 2 !");
  
  // Serial.println("Marche 3 !");
  // for (int i = 0; i < OUTPUT_SIZE; i++) {
  //   Serial.print(">filteredSignal :");
  //   Serial.print(i);
  //   Serial.println(filteredSignal[i]); // Affichage des données filtrées
  //   // DACC->DACC_CDR = DACC_CDR_DATA(filteredSignal[i]); // Sortie sur DAC
  //   // while (!(DACC->DACC_ISR & DACC_ISR_TXRDY));
  //   // delay(10); // Pause pour voir les points un à un (à ajuster)
  // }
}