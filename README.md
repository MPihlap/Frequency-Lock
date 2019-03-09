# Frequency-Lock
Hardware project, creating a lock you open by singing a tune to it.

## Demonstration video:
https://www.youtube.com/watch?v=SHbAAHV13OE

## Overview
Result of a hardware project done for a course at University of Tartu. It is a custom board design built around the STM32F4 microcontroller. 
The system records audio in 0.2 second buffers, performs RFFT and analyzes the output to see if the correct note is being played.

The code is fully compatible with the STM32F4 Discovery board.
