# QMesh Mbed Studio Project

QMesh is a synchronized flooded mesh network designed to stream at a low data rate. While the data rate is slow, it should suffice for e.g. Codec2 voice data. This initial Proof-of-Concept leverages the ARM Mbed framework, the STM32 Nucleo series of STM32 development boards, and Arduino and Mbed shields.

## Mitigating the collision/broadcast storm problem

In a synchronized, flooded mesh, all nodes retransmit at roughly the same time, creating collisions. LoRa, a form of Chirp Spread Spectrum (CSS), possesses several unique properties that can allow for successful receipt of one packet in the midst of one or more collisions. First, spread spectrum modulations (CSS as well as DSSS and FHSS) spread signal energy such that concurrent transmissions are less likely to overlap and thus interfere. Second, Semtech's LoRa chipsets allow for a per-data-symbol frequency hopping that can further spread concurrent transmissions from each other. This FHSS can also be combined with a form of slotted ALOHA to greatly reduce the probability that packets will collide at all. Third, LoRa is highly resistant to frequency error. Deliberately adding frequency error can spread out concurrent transmissions so that they do not interfere with each other. 

