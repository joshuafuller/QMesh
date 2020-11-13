# QMesh: A Synchronized Flooded Mesh Network for Voice and Data

QMesh is a novel mesh networking protocol designed to provide rapid self-healing and streaming capabilities to mesh networks in order to allow for voice communications. QMesh uses synchronized flooding to provide for self-healing as well as the ability to stream data. QMesh applies two techniques to the LoRa Chirp Spread Spectrum (CSS) waveform in order to make QMesh work: Coded Chirp Spread Spectrum (CCSS) and Chirp Offset Multiple Access (COMA).

## Synchronized Flooding
A flooded mesh network is a mesh network where every node repeats they packets they receive. Synchronized flooding is a form of flooding where every node repeats the data at the same time. Synchronized flooding allows for self-healing an isochronous data transfer, enabling meshed voice communications. A major drawback of synchronized flooding, however, is the massive self-interference that occurs from multiple nodes retransmitting at the same time. QMesh uses two techniques, Coded Chirp Spectrum (CCSS), and Chirp Offset Multiple Access (COMA), that reduces the self-interference and allows for reliable communcation.

## Coded Chirp Spread Spectrum (CCSS)
Forward Error Correction (FEC) is essential to getting reliable communications in the presence of the large amount of self-interference that occurs with synchronized flooding. As a result, QMesh implements a sophisticated form of FEC on top of LoRa, currently Reed-Solomon-Viterbi (RSV). RSV is a combination of convolutional coding concatenated with Reed-Solomon coding. Used in the past in such applications as interplanetary spacecraft, RSV increases communication reliability for both weak signals as well as against interference. 

## Chirp Offset Multiple Access (COMA)
Chirp Offset Multiple Access (COMA) is a technique that adds randomized, controlled offsets to CSS transmissions in order to reduce interference and allow the receiver to receive one of the transmissions. COMA adds offsets in two ways: by "wobbling" the center frequency of the transmission (up to +/-25%), and by adding a timing offset of up to one half of a symbol period.

LoRa has two potential ways to “spread out” transmissions in order to increase the likelihood of successful capture. The first technique involves spreading out transmit energy at the chip level. Given that LoRa chirps are not continuous frequency sweeps, but rather a series of discrete chirps, slightly shifting the frequencies of the chips should reduce their interference between them. Likewise, by shifting the chirps in frequency and/or time, it should be similarly possible to reduce the amount of interference between colliding packets and increase the likelihood of successful capture. Figures 2 and 3 show how frequency and time shift reduce overlap between transmissions and increase the likelihood of successful capture.

## Hardware
QMesh uses fairly high-end STM32 MCUs such as the STM32F4, STM32L4, STM32F7, and STM32H7. These higher-end MCUs contain the large program flash size (1MiB+) and RAM (512KiB+) needed to support the QMesh system. QMesh currently uses the STM32 NUCLEO-144 development boards due to the large amount of I/O available in their "Zio" connectors (Zio connectors are a superset of the Arduino shield form factor). Attached to the NUCLEO-144 boards is a custom board that contains several major components:
1. A 1W SX126X-based LoRa module. 
2. A 128Mbit QSPI NOR flash. This is used to store configuration as well as logging information.
3. 128x32 I2C OLED display. Allows for displaying some basic status information.

## Software
QMesh currently uses the Mbed framework, and is developed with Mbed Studio and the ARMC6 compiler.  The Mbed framework provides key components such as an RTOS, drivers, and a filesystem for the QSPI NOR flash.

## Discord Development
We will be using Discord as our "chat" application for discussing things.

## Licensing.
QMesh is licensed under the GNU GPL License v3; however, a commercial license is also available for entities who do not want to release their source code changes. Please contact Dan Fay at daniel.fay@gmail.com for more information about this.
