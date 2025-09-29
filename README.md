# 🎤 MFCC Voice Recognition on Arduino  

![Cpp](https://img.shields.io/badge/Cpp-Embedded-blue.svg)
![Arduino](https://img.shields.io/badge/Arduino-Due-blue.svg)
![MFCC](https://img.shields.io/badge/MFCC-Feature--Extraction-red.svg)
![NeuralNetwork](https://img.shields.io/badge/NN-Classification-red.svg)  

![License](https://img.shields.io/badge/license-MIT-green.svg)  
![Contributions](https://img.shields.io/badge/contributions-welcome-yellow.svg)  

---

## 📝 Project Description  
This project implements a **speech classification pipeline** on Arduino.  
It captures raw audio, computes **MFCC (Mel-Frequency Cepstral Coefficients)**, and feeds them into a lightweight **Neural Network** for binary classification (🔴 *Red* vs 🔵 *Blue*).  

---

## ⚙️ Features  
  🎙️ Real-time audio acquisition at **8 kHz**  

  🪞 Framing & Hamming windowing of audio samples  

  🎛️ FIR digital filtering on input signal  

  📊 MFCC feature extraction (13 coefficients, DCT applied)  

  🧠 Neural Network classification with predefined weights & biases  

  📺 Serial Monitor output for results  

---

## 🎮 Example Output  
Press the button connected to pin `D7` to start recognition:  
```bash
setup
Début fillBuffer !
Fin fillBuffer!
Debut Decoupage
Fin Decoupage
Résultat :
--------- Rouge ! ---------
```

---

## ⚙️ How it works  
  🔹 **Signal Acquisition**  
  - Audio sampled via ADC on pin `A0` at 8 kHz  
  - Filtered using FIR coefficients  

  🔹 **Framing & Windowing**  
  - Signal split into 48 frames of 256 samples each  
  - Hamming window applied before MFCC  

  🔹 **MFCC Extraction**  
  - Mel filter bank  
  - DCT applied  
  - 13 coefficients retained per frame  

  🔹 **Neural Network Inference**  
  - Features flattened with `cnn.h`  
  - Feedforward through a 3-layer NN  
  - Binary output (🔴 Red / 🔵 Blue)  

---

## 🗺️ Schema  
💡 High-level pipeline:  
```bash
[Microphone] --> [ADC @ 8kHz] --> [FIR Filter] --> [Framing + Windowing]  
--> [MFCC Extraction] --> [Flattening] --> [Neural Network] --> [Prediction: Red 🔴 / Blue 🔵]
```

---

## 📂 Repository structure  
```bash
├── Part_1_Signal_sampling/      # Signal sampling
├── Part_2_Signal_cutting/       # Signal cutting
├── Part_3_Signal_Saving/        # Signal saving
├── Part_4_Verification_MFCC/    # Verification of the MFCC with a MFCC plotter
├── Part_5_NN_training/          # Neural network training
├── Part_6_NN_test_MSE/          # Neural network test (MSE)
├── Part_7_NN_training_test_MSE/ # Neural network training + test (MSE)
├── Part_8_Extractor_weight/     # Extractor weight of the NN
├── Part_9_Final_raw_version/    # Raw version
├── Part_10_Final_version/       # Final version
│ ├── NeuralNetwork.h            # NN Lib I used
│ ├── arduinoMFCC.cpp            # MFCC lib I used
│ ├── arduinoMFCC.h
│ ├── cnn.cpp                    # Convolutional NN lib I used
│ ├── cnn.h
│ ├── main.cpp
│
├── LICENSE
└── README.md
```

---

## 💻 Run it on Your Arduino
Clone the repository and open main.ino in Arduino IDE.

Install dependencies:
- arduinoMFCC
- Adafruit_GFX
- Adafruit_SSD1306

Connect hardware:
- Microphone to A0  
- Push button to D7
- (Optional) OLED display with I2C

Upload to your Arduino (DUE nano - ATMega328P) board.

Open Serial Monitor at 9600 baud to see predictions.

## 📖 Inspiration / Sources
It was a school project ECE I made in 2 mounths in Ing 3 !

😆 100% coded and tested directly on Arduino hardware!
