# Uncomment lines below if you have problems with $PATH
#SHELL := /bin/bash
#PATH := /usr/local/bin:$(PATH)
PORT := /dev/ttyduet

all:
	platformio -f -c vim run

upload:
	platformio -f run --target upload --upload-port $(PORT)

clean:
	platformio -f -c vim run --target clean

program:
	platformio -f -c vim run --target program

uploadfs:
	platformio -f -c vim run --target uploadfs

update:
	platformio -f -c vim update

reset:
	stty -F $(PORT) cs8 1200 hupcl

#to view the serial port with screen.
monitor:
	screen -S duet $(PORT) 115200

