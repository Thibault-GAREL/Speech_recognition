#include <Arduino.h>

#define MFCC_SIZE 13 //13
#define DCT_MFCC_SIZE 13 //6

#define FRAME_SIZE 256
#define NB_FRAMES 48
#define FREQ_ECH 8000

// C'est la partie échantillonage avec enchevetrement
#define OUTPUT_SIZE 8000
#define COUPURE 166
int16_t filteredSignal [OUTPUT_SIZE];
float frames [NB_FRAMES][FRAME_SIZE];

void fillBuffer() {
  for (int i = 0; i < OUTPUT_SIZE; i++)
  {
    filteredSignal[i] = i;
  }
  


  for (int frame = 0; frame < NB_FRAMES; frame++) {
    int startIdx = frame * COUPURE;

    for (int i = 0; i < FRAME_SIZE; i++)
    {
      if (startIdx + i < OUTPUT_SIZE) {
        frames[frame][i] = (float) filteredSignal [startIdx + i];
      } else {
        frames[frame][i] = 0.0;
      }

      Serial.print("Frame [");
      Serial.print(frame);
      Serial.print("][");
      Serial.print(i);
      Serial.print("] : ");
      Serial.println(frames[frame][i]);
      Serial.println(frames[frame][i]);
    }
  }
  
}

void setup() {
  Serial.begin(9600);

  fillBuffer();

}

void loop() {
  // put your main code here, to run repeatedly:
}