#include <Arduino.h>
#include <arduinoFFT.h>
#include "arduinoMFCC.h"
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

// ======= Paramètres =======
#define PinTimer 2

#define BUFFER_SIZE 32000 //256
#define SAMPLING_FREQ 16000

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDR 0x3C

#define MFCC_SIZE 13

// ======= Variables =======
double vReal[BUFFER_SIZE];
double vImag[BUFFER_SIZE];

float mfcc[MFCC_SIZE];

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
arduinoMFCC mymfcc(MFCC_SIZE, MFCC_SIZE, BUFFER_SIZE, SAMPLING_FREQ);

ArduinoFFT<double> FFT(vReal, vImag, BUFFER_SIZE, SAMPLING_FREQ);

int currentXInit = 0;

// ======= Setup =======
void setup() {
  Serial.begin(9600);

  Serial.print("On commence !");

  pinMode(PinTimer, OUTPUT);

  // ADC
  PMC->PMC_PCER1 |= PMC_PCER1_PID37;
  ADC->ADC_MR = ADC_MR_PRESCAL(13) | ADC_MR_STARTUP_SUT64 | ADC_MR_TRACKTIM(15) | ADC_MR_TRANSFER(2);
  ADC->ADC_CHER = ADC_CHER_CH7;

  // DAC
  PMC->PMC_PCER1 |= PMC_PCER1_PID38;
  DACC->DACC_MR = DACC_MR_TRGEN_DIS | DACC_MR_USER_SEL_CHANNEL1 | DACC_MR_WORD_HALF;
  DACC->DACC_CHER = DACC_CHER_CH1;

  // OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (1);
  }
  display.clearDisplay();
  display.print("PEAK :");

  mymfcc.create_hamming_window();
  mymfcc.create_mel_filter_bank();
  mymfcc.create_dct_matrix();
}

// ======= Fonctions =======
void fillBuffer() {
  for (int i = 0; i < BUFFER_SIZE; i++) {
    while (!(ADC->ADC_ISR & ADC_ISR_EOC7)); // Attente fin de conversion
    uint16_t val = ADC->ADC_CDR[7];
    vReal[i] = (double)val;
    vImag[i] = 0.0;
    DACC->DACC_CDR = DACC_CDR_DATA(val); // Replay sur DAC
    ADC->ADC_CR = ADC_CR_START; // Start next
  }
}

int barLength(double d) {
  float fy = 10.0 * (log10(d) + 1.1);
  int y = constrain((int)fy, 0, 86) - 30;
  return max(0, y);
}

void showSpectrum(int peakFreq) {
  display.clearDisplay();

  int displayFFTvalue[BUFFER_SIZE];
  for (int i = 0; i < BUFFER_SIZE; i++) {
    displayFFTvalue[i] = barLength(vReal[i]);
  }

  // Affichage du texte du pic
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.print("PEAK :");
  display.print(peakFreq);
  display.println(" Hz");

  // Affichage du spectre
  for (int i = 0; i < SCREEN_WIDTH; i++) {
    int idx = map(i, 0, SCREEN_WIDTH, 0, BUFFER_SIZE / 2);
    display.drawLine(i, SCREEN_HEIGHT, i, SCREEN_HEIGHT - displayFFTvalue[idx], WHITE);
  }

  display.display();
}

void showMFCC() {
  display.clearDisplay();
  int barWidth = SCREEN_WIDTH / MFCC_SIZE;
  for (int i = 0; i < MFCC_SIZE; i++) {
    int height = map(mfcc[i] * 100, 260, 450, 0, SCREEN_HEIGHT);
    display.fillRect(i * barWidth, SCREEN_HEIGHT - height, barWidth, height, WHITE);
  }
  display.display();
}

// ======= Loop principale =======
void loop() {
  fillBuffer();

  // === FFT ===
  FFT.dcRemoval();
  FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(vReal, vImag, BUFFER_SIZE, FFT_FORWARD);
  FFT.complexToMagnitude(vReal, vImag, BUFFER_SIZE);

  // Trouver le pic
  int peakIndex = FFT.majorPeak(vReal, BUFFER_SIZE, SAMPLING_FREQ);
  showSpectrum(peakIndex);

  // === MFCC ===
  float frame[BUFFER_SIZE];
  for (int i = 0; i < BUFFER_SIZE; i++) {
    frame[i] = (float)vReal[i];
  }
  mymfcc.compute(frame, mfcc);

  // Décommenter pour afficher MFCC au lieu du spectre
  // showMFCC();

  // Afficher aussi MFCC sur la console série
  for (int i = 0; i < MFCC_SIZE; i++) {
    Serial.println(mfcc[i]);
  }
  Serial.println("=======");
}
