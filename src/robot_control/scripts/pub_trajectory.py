#!/usr/bin/env python3
import serial


if __name__ =="__main__":
    ser=serial.Serial("/dev/ttyUSB0",115200,timeout=0.5)
    ser.write(8)
