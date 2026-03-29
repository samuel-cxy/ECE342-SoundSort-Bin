# ECE342-SoundSort-Bin
STM32-based smart bin that automatically routes trash to the correct compartment based on its drop sound using machine learning.
1. A trash drop is detected by the ToF and triggers the mic to start recording.
2. Audio is stored in a buffer using DMA, 
3. After feature extraction and prediction, the output will be displayed on the OLED screen.

Hardwares:
- STM32: Nucleo F446ZE
- MIC (I2S): SPH0645LM4H
- ToF (I2C): VL53L1X
- OLED Screen (I2C): SSD1306
- Servo x 2 (PWM): Aideepen 20KG

Project Peripheral ioc: 
- I2S2: Transmission Mode=Master Receive, Communication Standard=I2S Philips, Data and Frame Format=16-bit data on 32-bit frame, Audio Frequency=16 kHz
- DMA for I2S2/SPI2_RX: Direction=Peripheral to Memory, Request Mode=Circular, Priority=High, Peripheral Data Width=Half Word, Memory Data Width=Half Word
- USART3: Mode=Asynchronous, Baud Rate=115200
- I2C1: Speed Mode=Fast Mode, Clock Speed=400000
- TIM4: Clock Source: Internal Clock, Channel1=PWM Generation CH1, Channel2=PWM Generation CH2, Prescaler=83, ARR=19999

Peripheral Pinout:
- I2S: BCLK=PB13, LRCL=PB12, DOUT=PB15
- I2C: SCL=PB8, SDA=PB9
- ToF: GPIO=PC3, XSHUT=PC0
- Servo: Bin Control=PD12, Lid Control=PD13

Machine Learning:
- Model: Multiclass Logistic Regression
- Dataset: Can=80, Glass=80, Paper=80, Plastic=80
- Features: RMS, Peak, Zero-crossing Rate, Decay Ratio, Spectral Centroid, Spectral Rolloff, Active Ratio, Band Energies x 8

Cite:
- VL53L1X API: https://www.st.com/en/embedded-software/stsw-img009.html
- SSD1306 API: UofT ECE342 Lab4
