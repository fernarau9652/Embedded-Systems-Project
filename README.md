# Digital and Analog Synthesizer with adjustable Frequency Scaling
ECE 5780/6780 Mini-Project for SPRING 2024 \
Team Members: Chase Griswold (6780), Fernando Araujo (6780), Alex Baret (5780), Vincent Banh (5780)

## Overview
_Briefly introduce the project, including its purpose, goals, and any relevant context. Mention the target hardware platform, development tools, and programming languages used._ \

The purpose of this project is to develop a digital and analog synthesizer machine that would take both types of inputs to output a modified synthesized wave, which can be used for voice modelization in further developments of this project. \
Our goals were to utilize various embedded system applications such as:
- Designing two custom PCBs that allow for signal generation and frequency oscillator modulation using physical and digital potentiometers.
- DAC and ADC to generate our signals and store them in the DMA while triggering timers to determine sampling rates.
- I2C and USART communication to determine resistance change in our custom PCBs that modulate a signal frequency while implementing interrupts for Putty/Waveforms User interfacing to level the signal modulation to a desired value.

Analog waveforms may be generated with two custom PCBs designed specifically for this project. One PCB generates a square wave with its frequency variation controlled via an I2C-controlled digital potentiometer. 

The other PCB is designed to emit analog waveforms and may be configured to generate a square wave, a triangle wave, or a sine wave using LM741 Operational Amplifiers. Waveforms from the second PCB may be sampled using the STM32's onboard ADC. 

Separate from the custom-PCB waveform generation, TIM7 (a timer onboard the STM32) consisting of a 16-bit auto-reload counter driven by a programmable
prescaler, a direct memory access controller (DMA onboard the STM32) used to perform programmable data transfers between memory-mapped peripherals, and a digital-to-analog converter (DAC onboard the STM32) have been utilized to implement digital signal synthesis.

#### Target Hardware Platform(s) ####
- STM32F072 MCU
- Function Generator PCB
- I2C Frequency Oscillator PCB
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
6) DS18030 Digital Potentiometer PCB
7) Function Generator PCB

### Installation
The pinout is as follows for I2C and UART communication:
- PB14 - SDA
- PB13 - SCL
- PB10 - TX
- PB11 - RX
- PA4 - Analog sine wave output

Connect the TX pin to RX on your USB to TTL UART device, and RX to TX. This enables serial console control of the project. SDA and SCL must then be connected to the I2C digital potentiometer PCB's SDA and SCL pins. Pull-up resistors are not needed as the board has built-in pull-ups.

Alternately, PA4 may be directly connected to an oscilloscope or output device to take advantage of the internal function generation of the project.

## Usage
Usage in all cases save for the UART-controlled digital potentiometer functionality is plug-and-play. There is no need for user input beyond physical adjustments of potentiometers or operation of the photoresistor.

When operating over UART, initial connection once the microcontroller has been reset will result in the message "Waiting for USART input. Please enter a number between 0 and 255." being printed to the terminal. This message is intended to be self-explanatory, and an input sanitation/verification system has been implemented to reject any other inputs beyond this range of values.

Due to an issue regarding the input system, it may occasionally be necessary to press the enter key multiple times to receive a response from the microcontroller. Only do this after entering the full value intended.

## Architecture
Describe the overall architecture of the embedded system, including its software and hardware components. Use diagrams, block diagrams, or UML diagrams to visualize the system's structure and relationships.
