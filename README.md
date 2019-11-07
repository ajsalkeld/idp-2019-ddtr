# IDP-2019-DDTR

Software repo for DDTR. Robot to detect and remove mock landmines.

Arduino code in `microcontroller/`
Code for computer vision in `computer-vision/`

## Microcontroller code

The main code for the competition is under `microcontroller/NINA_CODE`. `NINA_CODE.ino` is the main executable. A Simple Timer library is included.

Other directories include test-code.

## CV Code

Self-contained in `computer-vision/`. 

Communication via UDP Arduino <-> Python demonstrated in Python script `computer-vision/python-udp-comms/udp-threaded.py`. Copying this code into an IDE gives a way of manually sending commands to the Arduino for debug.
