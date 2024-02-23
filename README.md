# STM32Nucleo-Codec-Shield

A **TLV320AIC3101-Codec** based shield targeting STM32F401RE Nucleo boards and a **multi-threaded** program for real-time audio data processing.

Aim of the project is to dive into Embedded Systems and Real-time OS in order to control Shield's functionalities and to manipulate digital audio signals.

<p align="center">
  <img src="https://github.com/AleDedor/STM32Nucleo-Codec-Shield/assets/36864265/153a3b50-fbee-43ca-8d35-96f8ff4ed200" width="450" />
</p>

## Features:

- 2 input audio jacks (3.5mm)
- 2 output audio jacks (3.5mm)
- buffered input stages and limited noised bandwidth (up to 66Khz)
- a VU meter, made of 6 leds.

## The Repo:

- Datasheet, all the datasheets/docs used during board implementation.
- NAS_loopback_2, a simple code to launch on your F401RE and checks the shield is working fine.
- Schematic, Altium schematic and layout.
- Simulation, an LTSpice simulation for the Analog FE.


Made by Luca Daidone & Alessio Dedor as part of ES&AOS projects.
