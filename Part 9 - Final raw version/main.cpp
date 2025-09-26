// TP 2  - MFCC - //
// A complété

#include <Arduino.h>
#include <arduinoMFCC.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include "NeuralNetwork.h"
#include "cnn.h"

#define PinTimer 2
#define RC 0 //Todo: Définir la valeur trouver pour le registre de comparaison

const int16_t buttonPin = 7;
bool lastButtonState = LOW;  // État précédent du bouton
bool hasPrinted = false;     // Pour ne pas imprimer plusieurs fois

// =================================Buffer===========================================
#define BUFFER_SIZE 13 //45 avant
#define FILTER_SIZE 13
#define OUTPUT_SIZE 8000 //8000

// float rawSignal[OUTPUT_SIZE];
// float Signal[OUTPUT_SIZE]; //int16_t
int16_t Signal[OUTPUT_SIZE]; //int16_t
// float tempoBuffer[BUFFER_SIZE]; //int
int tempoBuffer[BUFFER_SIZE]; //int
int16_t adcBufferIndex = 0; //

// const static int16_t filter_taps[BUFFER_SIZE];
static double filter_taps[FILTER_SIZE] = {
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


// ==================  MFCC ================== // 


#define MFCC_SIZE 13 //13
#define DCT_MFCC_SIZE 13 //6

#define FRAME_SIZE 256
#define NB_FRAMES 48
#define FREQ_ECH 8000

// C'est la partie échantillonage avec enchevetrement
// #define OUTPUT_SIZE 8000
#define COUPURE 165
// int16_t filteredSignal [OUTPUT_SIZE];
float frames [NB_FRAMES][FRAME_SIZE];


// Déclaration de l'objet MFCC
arduinoMFCC mymfcc(MFCC_SIZE, DCT_MFCC_SIZE, FRAME_SIZE, FREQ_ECH);





// vecteur de données
double frame[FRAME_SIZE];
// vecteur de coefficient finaux ( pre-DCT)
float mfcc[MFCC_SIZE];

// vecteur de coefficient finaux ( post-DCT)
// float dct_mfcc[MFCC_SIZE];

float coefFinal[NB_FRAMES][MFCC_SIZE];

// ================== Découpage ================== //

// bool framesFilled = false;

void Decoupage () {
  

  for (int frame = 0; frame < NB_FRAMES; frame++) {
    int startIdx = frame * COUPURE;

    for (int i = 0; i < FRAME_SIZE; i++)
    {
      if (startIdx + i < OUTPUT_SIZE) {
        frames[frame][i] = (float) Signal [startIdx + i];
      } else {
        frames[frame][i] = 0.0;
      }
    }
  }
  // framesFilled = true;
}

// ======================== Neural Network =========================== //

#define NumberOf(arg) ((unsigned int)(sizeof(arg) / sizeof(arg[0])))  // calculates the number of layers (in this case 3)
#define _1_OPTIMIZE B00010000                                         // https://github.com/GiorgosXou/NeuralNetworks#define-macro-properties

//Todo layers
const unsigned int layers[] = {(DATA_ROW / (POOL_SIZE * LAYER)) * (DATA_COL / (POOL_SIZE * LAYER)), 2, 1};
float *output;

float biases[] = {1.0012149, 1.3830764};

float weights[] = {-0.8424503, 0.6789041, 0.4573612, 0.6115263, 0.5966545, 0.6494639, -0.6056939, 0.1727435, 0.6691052, 0.078972, 0.0953904, -0.2183257, 0.4183064, 0.0790708, 0.2415579, -0.6397283, -0.1191636, 0.8703101, 0.2136635, -0.3247533, 0.5650441, 0.792231, 0.4793751, 0.2256238, 0.5626417, 0.3542254, -0.1186426, 0.7630527, 0.5761577, 0.6571869, -0.4416831, -0.8030592, 0.0573736, -0.4825434, 0.6961388, -0.6508481, -0.6351978, 0.4843136, 0.3893938, 0.2770614, -0.3374139, -0.0755089, 0.6311243, -0.8378686, 0.608708, -0.8088326, -0.1110173, 0.2950578, 0.689309, -0.2031898, -0.2651769, -0.1183081, 0.8657114, 0.3780922, 0.3052422, 0.5347118, -0.578429, -0.0203217, 0.307137, -0.0315498, 0.0351762, 0.8411001, 0.394736, 0.1492628, 0.5993261, 0.0097716, 0.6514727, -0.5773077, 0.1281597, 0.2450357, 0.5692215, -0.3244119, 0.2686639, 0.0525192, -0.5810019, 0.8732333, 0.6397266, -0.2244099, 0.8379782, 0.7077902, 0.5889915, 0.0490798, -0.264861, 0.5236776, -0.676224, 0.3189426, 0.379722, -0.6658916, 0.3094375, 0.1588851, 0.2950954, 0.5232451, 0.4390546, 0.5093557, -0.0752525, -0.6210703, -0.602805, -0.2974074, -0.5070154, -0.4665609, 0.3045374, 0.1091266, 0.3899146, 0.3338381, -0.011778, 0.9034395, -0.7442977, -0.8700814, -0.0456503, -0.6256269, 0.5462858, 0.7133575, -0.0516147, 0.4136176, -0.7754036, -0.6606681, 0.7956724, -0.6855417, -0.1197289, 0.0863296, -0.4057371, -0.8110933, 0.6018751, -0.055939, 0.7697795, -0.3812491, -0.7957513, -0.2596615, -0.1442215, -0.3915473, -0.6767232, 0.1396021, -0.5289617, -0.8627716, 0.6913481, -0.0491106, 0.6747802, 0.6521997, -0.3262441, 0.2010442, 0.2969345, -0.2618064, -0.5679103, -0.1107107, -0.4849316, 0.0647546, 0.6938987, 0.4250054, 0.8063568, 1.4630396, -0.9121602, -0.7481966, 0.6094469, -0.9429922, -0.5793962, -1.1256325, -0.6764989, 2.4612277, -0.8993487, -1.3561051, -0.3865372, -0.3764644, 0.3533135, 0.6373041, 0.3367508, -1.498275, -1.9380705, 1.0067787, -1.6466769, 0.1286658, 0.0007619, 1.4317027, -0.0533622, 2.1398361, -0.2401163, 0.6268186, 0.6183061, 0.1172674, 1.0124724, 0.1919622, -2.6962113, -1.6235086, 0.678898, -0.0076748, -0.5895323, 1.8143389, -0.7942322, -1.5254045, -1.9389951, -0.5149593, -0.582566, 0.0579141, -0.3571512, -0.6678706, -2.0407331, 1.6610781, 1.037832, -1.0265502, 1.1451325, -1.8859386, -2.1550968, 2.9286842, 2.8347847, 0.600168, 0.3822345, -0.349911, -1.8510377, 2.2068799, 2.1722977, -1.0380708, 0.7920332, -4.1663976, -1.873116, 3.7331276, 1.8901918, -0.9803517, 0.2513645, -2.6854362, -4.2474442, 1.9356463, 0.72787, -1.0317315, 2.5249939, -2.0997329, -1.8700982, 1.2474079, 1.0432806, -0.2350738, 2.3247693, -1.141903, -1.463173, 2.7596676, 2.5824177, -0.5622712, 2.5014327, -0.948948, -2.8621962, 2.5309708, 0.5840374, 0.5613912, 2.7253163, -2.7068479, -2.8250444, 2.6874926, 2.0639997, -0.9443747, 2.3587182, -1.0751035, -2.6636646, 0.891142, -0.2058901, -1.1908352, 0.9304675, 0.3534585, -0.3995255, 1.935652, 1.105821, -1.7999007, 2.953649, 0.9648889, -1.4241556, 0.6244051, -0.0212277, -1.922374, 0.8736206, 1.6323081, 0.2355248, 0.6964942, -0.6060521, -0.4764319, 1.7179505, 1.5310353, 0.1373497, -0.6122729, -1.5275463, -1.7661119, 1.5679238, 0.7490159, -0.1393914, -0.5307595, -0.2097769, -0.733399, 0.6734338, 1.1529466, -0.5481636, 1.429827, 0.5294504, -0.3157386, 2.7489688, -13.5980797};


void NN_work() {
  float flat_test_input[624]; 

  // NeuralNetwork NN(layers,NumberOf(layers)); // Creating a NeuralNetwork with default learning-rates
  NeuralNetwork NN(layers, weights, biases, NumberOf(layers));

  float MSE_test = 0;
  // //Goes through all the input arrays
  // for (unsigned int i = 0; i < NumberOf(coefFinal); i++)
  // {
    cnn(coefFinal, flat_test_input);
    output = NN.FeedForward(flat_test_input); // FeedForwards the input[i]-array through the NN | returns the predicted output(s)
    Serial.print("Résultat : ");
    // Serial.println(output[0], 7);       // Prints the first 7 digits after the comma.
    if (output[0] > 0.1)
    {
      Serial.println("--------- Rouge ! ---------");
    }
    else {
      Serial.println("--------- Bleu ! ---------");
    }
    

  //   float error = flat_expected_test[i] - output[0];
  //   MSE_test += sqrt(error * error);
  //   Serial.println(MSE_test, 6);
  // }

  // MSE_test /= NumberOf(testInput);
  // Serial.print("Test MSE: ");
  // Serial.println(MSE_test, 6);
}

// ================================================ //

// Initialisation de l'ADC
void setupADC() {
  analogReadResolution(12);
  analogReference(AR_DEFAULT); // Utilise Vcc comme référence
  // Pas besoin de plus sauf si précision extrême voulue
}

// Fonction de remplissage du buffer
void fillBuffer() {

  unsigned long period = 1000000UL / FREQ_ECH; // µs entre les échantillons
  unsigned long microsNow = micros();

  for (int i = 0; i < OUTPUT_SIZE; i++) {
    //Chono à mettre
    Signal[i] = ((int16_t)analogRead(A0) - 512.0) ;  // Normalisation entre -1.0 et +1.0
    // Signal[i] = (float)analogRead(A0);
    // Serial.println("loop");
    // // Serial.println(i);
    // // Serial.print(" : ");
    // Serial.println(Signal[i]);
    while (micros() - microsNow < period); // Attendre la période d'échantillonnage
    microsNow += period; // S'assurer que la période reste constante
    
    // for(int i=0; i < OUTPUT_SIZE; i++) {
    // while((ADC->ADC_ISR & 0x80)==0); // attente de la fin de la conversion
    tempoBuffer[adcBufferIndex] = Signal[i]; // sauvegarde du signal dans le buffer 

    adcBufferIndex++;   // incrementation de l'index du buffer cirulaire
    uint16_t sumIndex = adcBufferIndex; // update de l'index pour le bon indexage des valeurs en fonction des coeffs 
    if(adcBufferIndex == BUFFER_SIZE) { // modulo pour remettre à 0 
      adcBufferIndex = 0;
    }
    
    // Calcul du filtrage sur les 15 valeurs du buffer 
    int16_t acc = 0;
    
    for (int l = 0; l < FILTER_SIZE; l++) { // multiplication des valeurs du buffer par les coeffs

      // gestion de l'index interne au buffer pour le calcul du filtre
      if( sumIndex > 0 ) {
        sumIndex--;
      } else {
        sumIndex=BUFFER_SIZE-1;
      }
      // calcule du filtre sur 32 bit
      acc += filter_taps[l] * tempoBuffer[sumIndex];
      
    }
    // Signal[i] =  acc >> 15; // shift de 15 bit pour repasser sur 16 bit
    Signal[i] = acc;
    //Chrono à mettre

    
  // }

    // CircularBuffer();

    // Serial.println("loop");
    // Serial.println(i);


    // Serial.print(">rawSignal :");
    // Serial.print(Signal[i]);

    // Serial.println("loop");
    // Serial.println(i);

    // Serial.print(">filtredSignal :");
    // Serial.println(filteredSignal[i]);

    
  }

 
}

void audioToMFCC () {
  Serial.println("Début fillBuffer !");
  fillBuffer();                             // remplissage du buffer audio
  Serial.println("Fin fillBuffer!");
  
  

  Serial.println("Debut Decoupage");
  Decoupage();
  Serial.println("Fin Decoupage");
  
  for (int i = 0; i < NB_FRAMES; i++) {
    // for (int j = 0; j < FRAME_SIZE; j++)
    // Serial.println("=======================");
    mymfcc.compute(frames[i], mfcc);
    for (int j = 0; j < DCT_MFCC_SIZE; j++)
    {
      // Serial.print(">Frames [");
      // Serial.print(i);
      // Serial.print("]");
      // Serial.print("[");
      // Serial.print(j);
      // Serial.print("] :");
      // Serial.println(frames[i][j]);
       //Peut être que on sort du tableau ?
      // Serial.println(mfcc[j]);

      coefFinal [i][j] = mfcc[j];
    }

  }

  // Normalisation du tableau
  float minVal = coefFinal[0][0];
  float maxVal = coefFinal[0][0];

  // Trouver le minimum et le maximum
  for (int i = 0; i < NB_FRAMES; i++) {
    for (int j = 0; j < MFCC_SIZE; j++) {
      if (coefFinal[i][j] < minVal) {
        minVal = coefFinal[i][j];
      }
      if (coefFinal[i][j] > maxVal) {
        maxVal = coefFinal[i][j];
      }
    }
  }

  // // Normalisation
  // for (int i = 0; i < NB_FRAMES; i++) {
  //   for (int j = 0; j < MFCC_SIZE; j++) {
  //     coefFinal[i][j] = (coefFinal[i][j] - minVal) / (maxVal - minVal);
  //   }
  // }

  //   Serial.println("Tableau après normalisation :");
  // for (int i = 0; i < NB_FRAMES; i++) {
  //   for (int j = 0; j < MFCC_SIZE; j++) {
  //     Serial.print(coefFinal[i][j]);
  //     Serial.print(" ");
  //   }
  //   Serial.println();
  // }

}

void setup() {
  Serial.begin(9600);
  Serial.println("setup");
  setupADC();

  pinMode(buttonPin, INPUT_PULLUP);
  

  mymfcc.create_hamming_window();
  mymfcc.create_mel_filter_bank(); 
  mymfcc.create_dct_matrix();  
}

void loop() {
  // fillBuffer();

  // for (int i = 0; i < OUTPUT_SIZE; i++)
  // {
  //   Serial.println("loop");
  //   Serial.println(i);

  //   Serial.print(">Signal :");
  //   Serial.println(Signal[i]);
  // }
  // CircularBuffer();

  


  
  // delay(1000);
  // mymfcc.compute(frame, mfcc); // calcul des MFCC

  int16_t buttonState = digitalRead(buttonPin);

  if (buttonState == HIGH && lastButtonState == LOW && !hasPrinted) {
    delay(250);
    audioToMFCC();
    // printf("================================================");
    NN_work();
    hasPrinted = true;
  }

  // Met à jour l'état précédent pour la prochaine boucle
  lastButtonState = buttonState;

  if (buttonState == LOW) {
  hasPrinted = false;


}
   
}