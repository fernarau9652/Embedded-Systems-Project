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
_List the key features and functionalities of the embedded system. This could include hardware components, sensors, actuators, communication protocols, user interfaces, etc._

- Output analog sine waves using TIM7, DMA, and DAC to PA4
- Square Wave Function Generator PCB that uses an LM324N and a DS18030 Digital Potentiometer
- Control the frequency of the I2C-controlled DS18030 Potentiometer through a PuTTy interface that communicates through USART
- Function Generator PCB utilizing LM741 Operational Amplifiers to generate square, triangle, and sine waves

## Getting Started
_Provide instructions for setting up the development environment and getting the project up and running on the target hardware. Include any dependencies, libraries, or tools required._


1) Acquire an ADC for waveform analysis
2) Diligent Waveforms 
3) USB/USART chip
4) μVision Kiel IDE

### Prerequisites
_List any prerequisites or dependencies needed to build and run the project. This could include software tools, compilers, SDKs, etc._



### Installation
Step-by-step instructions for installing and configuring the project. Include commands, configuration files, or scripts necessary to set up the environment.

## Usage
Explain how to use the embedded system once it's up and running. Provide examples, code snippets, or diagrams to illustrate usage scenarios and interactions.

## Architecture
Describe the overall architecture of the embedded system, including its software and hardware components. Use diagrams, block diagrams, or UML diagrams to visualize the system's structure and relationships.

## Contributing
Guidelines for contributing to the project, including how to report bugs, submit feature requests, or contribute code. Specify any coding standards, version control practices, or contribution workflows to follow.

## License
Specify the license under which the project is distributed. Include any relevant copyright notices, disclaimers, or attribution requirements.

## Acknowledgements
Acknowledge any individuals, organizations, or resources that contributed to the project's development. This could include mentors, collaborators, open-source libraries, or third-party tools.

## Contact
Provide contact information for the project maintainers or developers. Include email addresses, social media profiles, or links to issue trackers or discussion forums.
