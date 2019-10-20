# IDP-2019-DDTR

Software repo for DDTR. Robot to detect and remove mock landmines.

Arduino code in `microcontroller/`
Code for computer vision in `computer-vision/`

Main code for the microcontroller is `NINA_CODE`. This still includes a wait for USB Serial to connect. To run Arduino without USB Serial, **this must be removed**.

Communication via UDP Arduino <-> Python demonstrated in Python program `udp-threaded.py`.
