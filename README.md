# drone-tracking
Control an EACHINE E010 drone from a computer using Python and an ArUco marker.

Using work from [@goebish](https://github.com/goebish/nrf24_multipro) (transmitter protocol) and [@perrytsao] (https://github.com/perrytsao/nrf24_cx10_pc) (input through serial port).




## Hardware Setup
- Arduino Uno
- 2.4GHz nRF24L01+ Wireless Card
- Socket adapter with on-board 3.3V regulator

## Software Setup
- Python 3.6
- OpenCV 3.2.0

## How-To

1. Connect the nRF24L01+ to the Arduino Uno using [perrystao's tutorial](https://github.com/perrytsao/nrf24_cx10_pc/blob/master/README.md)
2. Upload **nrF24_multipro/nRF24_multipro.ino** to your Arduino Uno. You have to modify it if your drone is not an E010 drone.
3. Calibrate your camera using for instance the [calibrate.py](https://github.com/opencv/opencv/blob/master/samples/python/calibrate.py) example in the OpenCV sources. Don't forget to specify the size of your chessboard and the size of the squares in the unit you want to use.
4. Print a board of ArUco markers that fit on your drone. I used [this one](images/board.png) for my E010 drone.
5. Stick the marker to your drone.
6. Put the drone where you want the origin of your world coordinates to be, in the right orientation.
7. In **define_origin.py**: specify the camera channel, the location of your calibration parameters and the size of your marker.
8. Run **define_origin.py**, and press the space bar when the origin marker is detected.
9. In **threadedFly.py**: specify the ID of the ArUco marker that you put on the drone, its size, the camera channel, the port your Arduino Uno is plugged to, the location of your calibration parameters, the location of your origin.npz file.
10. Run **threadedFly.py**.
11. Tune the PID controller gains to make your drone fly as you please.
12. Design your own controller, trajectories, etc. Have fun!

