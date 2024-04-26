# Digital and Analog Synthesizer with adjustable Frequency Scaling
ECE 5780/6780 Mini-Project for SPRING 2024 \
Team Members: Chase Griswold (6780), Fernando Araujo (6780), Alex Baret (5780), Vincent Banh (5780)

Official Documentation with wire diagrams, schematics, and flow charts can be found here (NOTE: Figures mentioned in the README pertain to the figures in the documentation listed): \
./Documentation/6780_CircuitDesign_DDS_REV-A_Final.pdf

This is the path to the finalized project folder for this project: \
./STM32/5780-6780-Final-Project3/  (Other files are previous revisions of the project)

Documentation related to the PCB designs with BoM datasheets can be found here: \
./PCB/  ("Board EXPORTS" contain the PCB design revisions; everything else is datasheets and LBR files)

Additional code provided by Chase can be found here: \
./Chase_Mains/  (Contains code for ADC and I2C control for digital potentiometer)

## Overview
The purpose of this project is to develop a digital and analog synthesizer machine that would take both types of inputs to output a modified synthesized wave, which can be used for voice modelization in further developments of this project. \
Our goals were to utilize various embedded system applications such as:
- Designing two custom PCBs that allow for signal generation and frequency oscillator modulation using physical and digital potentiometers.
- The signal generation PCB generates three analog signals: Square, Triangle, and Sine waves. By adjusting the physical potentiometer, the signal's frequency can be adjusted.
- The frequency oscillator PCB generates a square wave, whose frequency can be adjusted via the I2C digital potentiometer or photoresistor (the photoresistor was extra but cool to have).
- Implementing ADC through the signal generation PCB, sampling each signal through the STM32, and outputting the waves through the Analog Discovery 2 (AD2) device.
- Implementing DAC by configuring the CubeMX project by instantiating timer 7 (TIM7) to clock at a sampling rate of 42 kHz, then using DMA to store the signal to be output again. In this section, we implemented signal addition synthesis to describe how the output signal would look like if two more signals were added to the initial wave.
- I2C and USART communication that changes a signal's frequency from the frequency oscillator PCB where the user interfaces with putty to change the resistance from the I2C digital potentiometer, therefore changing frequency. This section of the project accounts for frequency modulation.
- USART uses interrupts to wait for the user's numerical input (integers from 0 to 255), then delivers the commands to the MCU and out to the I2C-controlled digital potentiometer on the frequency oscillating wave.

Analog waveforms may be generated with two custom PCBs designed specifically for this project. One PCB generates a square wave with its frequency variation controlled via an I2C-controlled digital potentiometer. 

The other PCB is designed to emit analog waveforms and may be configured to generate a square wave, a triangle wave, or a sine wave using LM741 Operational Amplifiers. Waveforms from the second PCB may be sampled using the STM32's onboard ADC. 

Separate from the custom-PCB waveform generation, TIM7 (a timer onboard the STM32) consisting of a 16-bit auto-reload counter driven by a programmable
prescaler, a direct memory access controller (DMA onboard the STM32) used to perform programmable data transfers between memory-mapped peripherals, and a digital-to-analog converter (DAC onboard the STM32) have been utilized to implement digital signal synthesis.

#### Target Hardware Platform(s) ####
- STM32F072 MCU
- Function Generator PCB (Fig. 1)
- I2C Frequency Oscillator PCB (Fig. 3)
- FT232RL chip for USB/USART communication
#### Development Tools ####
- μVision Kiel IDE
- Diligent Waveforms 
#### Programming Languages ####
- C

## Features
- Output analog sine waves using TIM7, DMA, and DAC to PA4
- Square Wave Function Generator PCB that uses an LM324N and a DS18030 Digital Potentiometer
- Control the frequency of the I2C-controlled DS18030 Potentiometer through a PuTTy interface that communicates through USART
- Function Generator PCB utilizing LM741 Operational Amplifiers to generate square, triangle, and sine waves

## Getting Started
1) Acquire an ADC for waveform analysis
2) Diligent Waveforms 
3) USB/USART chip
4) μVision Kiel IDE
5) PuTTY
6) I2C Frequency Oscillator PCB (Fig. 4)
7) Function Generator PCB (Fig. 2)
8) Follow the test setup in Fig. 5 (NOTE: an additional connection from the PC to USART to STM32 is missing from the layout.)

### Installation
The STM32 pinout (Fig. 6):
- PB14 - SDA (I2C)
- PB13 - SCL (I2C)
- PB10 - TX (USART)
- PB11 - RX (USART)
- PC3 - 12-bit ADC input
- PA4 - 12-bit DAC output (forgot to add to the wire diagram)

Connect the TX pin to RX on your USB to TTL UART device and RX to TX. This enables serial console control of the project.\
The SDA and SCL must then be connected to the I2C digital potentiometer PCB's SDA and SCL pins. Pull-up resistors are not needed, as the board has built-in pull-ups.

Alternately, PA4 may be directly connected to an oscilloscope or output device to take advantage of the internal function generation of the project.

## Usage
Usage in all cases save for the UART-controlled digital potentiometer functionality is plug-and-play. There is no need for user input beyond physical adjustments of potentiometers or the operation of the photoresistor.

When operating over UART, the initial connection once the microcontroller has been reset will result in the message "Waiting for USART input. Please enter a number between 0 and 255." being printed to the terminal. This message is intended to be self-explanatory, and an input sanitation/verification system has been implemented to reject any other inputs beyond this range of values.

Due to an issue regarding the input system, it may occasionally be necessary to press the enter key multiple times to receive a response from the microcontroller. Only do this after entering the full value intended.

To use the function generator circuit,  connect an oscilloscope or an ADC to the respective op-amp that generates the desired waveform (square, sawtooth, sine).

To modulate a digital signal wave using additive synthesis, the code allows users to add two Sine waves of different frequencies, takes the average between the values, and then buffers the signals to the intended 12-bit resolution for the output. 

Users may adjust FREQ1 and FREQ2 to whatever musical note frequency they choose. The code will adjust the additive sine signals to the original output signal of 420 Hz and then output the final product through PA4. Images of sample waves simulated in this project can be seen here: \
./Documentation/DAC_waves/

## Architecture
The documentation listed above contains all architecture-related diagrams for this project. Design specifications for the PCBs, BoM, wire diagrams, schematics, and flow charts are also included.
