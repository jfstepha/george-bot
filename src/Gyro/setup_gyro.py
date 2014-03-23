#!/usr/bin/python

import time;
import os;
import timeit;

##############################################################
# constants

I2C_MUX = 29

GPIO_OUTPUT = 1
GPIO_INPUT = 0


##############################################################
##############################################################
class GPIO():
##############################################################
##############################################################

    #######################################################
    def __init__ (self):
    #######################################################
        print "GPIO init"

    #######################################################
    def export_pin(self, pin_no):
    #######################################################
        print "GPIO exporting pin %d " % pin_no
        try:
            f = open( "/sys/class/gpio/export", "wo" )
            f.write( "%d" % pin_no )
            f.close()
        except IOError as detail:
            if "Errno 16" in str(detail):
                print "already exported"
            else:
                print "Not an error 16: " + detail
                raise detail
    #######################################################
    def set_mode(self, pin_no, mode):
    #######################################################
        #print "GPIO setting pin %d mode to %d" % (pin_no, mode)
        if mode == GPIO_INPUT:
            modestr = "in"
        elif mode == GPIO_OUTPUT:
            modestr = "out"
        else:
            raise ValueError("GPIO mode %d is not valid" % mode)
       
        self.export_pin( pin_no );
        f = open( "/sys/class/gpio/gpio%d/direction" % pin_no, "wo")
        f.write ( modestr )
        f.close()

    #######################################################
    def output(self, pin_no, value):
    #######################################################
        # print "GPIO setting pin %d value to %d" % (pin_no, value)
        if value == 0:
            valstr = "0"
        elif value == 1:
            valstr = "1"
        else:
            raise ValueError("GPIO value %s is not valid" % str(value))

        f = open( "/sys/class/gpio/gpio%d/value" % pin_no, "wo")
        f.write( valstr )
        f.close()
    #######################################################
    def input(self, pin_no):
    #######################################################
        # print "GPIO reading pin %d" % pin_no
        # print "file: /sys/class/gpio/gpio%d/value" % pin_no
        f = open( "/sys/class/gpio/gpio%d/value" % pin_no, 'ro')
        #os.system("cat /sys/class/gpio/gpio%d/value" % pin_no)
        valstr = f.read( )
        f.close()
        #print "read %s" % valstr
        return(int(valstr))

##############################################################
##############################################################
class Gyro():
##############################################################
##############################################################

    #######################################################
    def __init__ (self):
    #######################################################
        print "SPI init"
        self.gpio = GPIO();

    #######################################################
    def setupPins(self):
    #######################################################
        print "SPI Setup pins"
        self.gpio.set_mode( I2C_MUX, GPIO_OUTPUT)

        # set all the SPI mux pins to GPIO mode
        self.gpio.output( I2C_MUX, 0)




##############################################################
if __name__ == '__main__':
##############################################################
    print "Main"
    print "initializing the spi class:"
    s = Gyro()
    s.setupPins()
