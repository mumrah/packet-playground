# README

Sandbox for playing with AX25 packets over various interfaces. As the code
exists today, the data flow is:

* PC #1 (YAAC) -> KISS Serial -> Arduino #1 -> CC1101 -> RF
* RF -> CC1101 -> Arduino #2 -> KISS Serial -> PC #1 (Serial console)

The eventual desired flow is:

* RPi #1 (G8BPQ) -> KISS i2c -> Arduino #1 -> CC1101 -> RF
* RF -> CC1101 -> Arduino #2 -> KISS i2c -> RPi #2 (G8BPQ)

## KISS Serial

Using KISS framing we can talk to desktop applications like XASTIR, ARPX, YAAC,
or anything capable of supporting AX25 packets encoded with KISS

## CC1101 RF

Using this [CC1101 library](https://github.com/veonik/arduino-cc1101/),
we can send packets over RF to CC1101 modules. The modules interface to the
Arduino over SPI. The packet format is very basic:

```
RF_PACKET = LENGTH DATA
LENGTH    = <int>
DATA      = <int[]>
```

An address field is also supported by the CC1101 modules, but I haven't tried
that yet.

## KISS I2C

We can also talk to devices like a RaspberryPi with KISS framed packets over
the i2c bus. During setup this code declares an address of 0x04 for itself and
then participates in the i2c bus as a slave device.

This was done primarily to interface with the G8BPQ node controller software.
