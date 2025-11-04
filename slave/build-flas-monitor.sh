!/bin/bash

idf.py set-target esp32
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
