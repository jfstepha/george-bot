#!/usr/bin/python

import smbus
import time

class Gyro:

  def __init__(self, address, busnum=-1, debug=False):
      self.address = address
      # By default, the correct I2C bus is auto-detected using /proc/cpuinfo
      # Alternatively, you can hard-code the bus version below:
      self.bus = smbus.SMBus(0); # Force I2C0
      self.debug = debug
      self.xoffset = 0
      self.yoffset = 0
      self.zoffset = 0
      self.scale = 8.75 / 1000 * 2 # dps per lsb.  at 250 sensitivity
      self.xt = 0.0
      self.yt = 0.0
      self.zt = 0.0
      self.xh = 0
      self.yh = 0
      self.zh = 0
      self.dt = 1.0 / 95.0 # 95 Hz setting
      self.x = 0
      self.y = 0
      self.z = 0
      self.wm = 0x10 # watermark
      self.spins = 0


  def twoscompl16(self,val):
      if val & 0x8000:
          val = 0 - (0x10000 - val)
      return val


  def whoami(self):
      result = self.readU8(0x0f);
      return result

  def readXYZhex(self):
      xl = self.readU8(0x28);
      xh = self.readU8(0x29);
      yl = self.readU8(0x2a);
      yh = self.readU8(0x2b);
      zl = self.readU8(0x2c);
      zh = self.readU8(0x2d);

      x = xh << 8 | xl
      y = yh << 8 | yl
      z = zh << 8 | zl

      return (x,y,z)

  def hex2floatXYZ( self, xh, yh, zh):
      x = (self.twoscompl16( xh ) - self.twoscompl16( self.xoffset)) * self.scale
      y = (self.twoscompl16( yh ) - self.twoscompl16( self.yoffset)) * self.scale
      z = (self.twoscompl16( zh ) - self.twoscompl16( self.zoffset)) * self.scale
      return (x,y,z)

  def readXYZ(self):
      (xh, yh, zh) = self.readXYZhex()
      (x, y, z) = self.hex2floatXYZ( xh, yh, zh )
      return (x,y,z)

  def readTemp(self):
      temp = self.readU8(0x26);
      return (temp)

  def readStatus(self):
      status = self.readU8(0x26);
      return (status)

  def readFIFOSRC(self):
      status = self.readU8(0x2f);
      return (status)

  def setCR1(self, val):
      self.write8(0x20, val)

  def setCR5(self, val):
      self.write8(0x24, val)

  def setFIFOCtrl(self, val):
      self.write8(0x2e, val)

  def setupCRs(self):
      # bits 7:4 : set DR and BW to 0
      # bits 3:0 : enable all 3 sensors and get out of standby
      self.setCR1(0x0f)

      # FIFO enable
      self.setCR5(0x40)
      # set Stream mode
      # Set watermark to 0x10 (about 1/2 full)
      self.setFIFOCtrl(0x40 | self.wm)

  def write8(self, reg, value):
    "Writes an 8-bit value to the specified register/address"
    try:
      self.bus.write_byte_data(self.address, reg, value)
      if self.debug:
        print "I2C: Wrote 0x%02X to register 0x%02X" % (value, reg)
    except IOError, err:
      return self.errMsg()

          
      
  def readU8(self, reg):
    "Read an unsigned byte from the I2C device"
    try:
      result = self.bus.read_byte_data(self.address, reg)
      if self.debug:
        print ("I2C: Device 0x%02X returned 0x%02X from reg 0x%02X" %
         (self.address, result & 0xFF, reg))
      return result
    except IOError, err:
      return self.errMsg()

  def calibrate(self):
      print "Calibrating gyro... don't move"
      samples = 100
      xt = 0
      yt = 0
      zt = 0

      for i in range(samples):
         while (self.readFIFOSRC() & 0x80 == 0):
             time.sleep(0.0001)

         (x,y,z) = self.readXYZhex()
         xt = xt + x
         yt = yt + y
         zt = zt + z

      print "totals: 0x%x 0x%x 0x%x" % (xt, yt, zt)
      xavg = int( xt / samples)
      yavg = int( yt / samples)
      zavg = int( zt / samples)

      print "averages: 0x%x 0x%x 0x%x" % (xavg, yavg, zavg)
      self.xoffset = xavg
      self.yoffset = yavg
      self.zoffset = zavg

  def spin(self):
      (self.xh, self.yh, self.zh) = self.readXYZhex()
      (self.x,  self.y,  self.z) = self.hex2floatXYZ( self.xh, self.yh, self.zh)
      self.xt = self.xt + self.dt * self.x
      self.yt = self.yt + self.dt * self.y
      self.zt = self.zt + self.dt * self.z

  def multispin(self, maxspins):
      fss = self.readFIFOSRC() & 0x1f
      if fss - self.wm < maxspins:
          i = fss - self.wm 
      else:
          i = maxspins

      self.spins = i

      for j in range(i):
          self.spin()


if __name__ == '__main__':
    print "Gyro main"
    gyro = Gyro(0x6b, debug=False);
    print "Whoami returned 0x%x\n" % gyro.whoami();
    print "Setting up registers:"
    gyro.setupCRs()
    print "Calibrating:"
    gyro.calibrate()
    print "done."

    xt = 0
    yt = 0
    zt = 0

    while(1):
        gyro.multispin(20)

        xh = gyro.xh
        yh = gyro.yh
        zh = gyro.zh

        x = gyro.x
        y = gyro.y
        z = gyro.z

        xt = gyro.xt
        yt = gyro.yt
        zt = gyro.zt

        #print " xyz: 0x%04x 0x%04x 0x%04x  " % ( xh,yh,zh ) + " status: 0x%02x " % gyro.readStatus() + " FIFO: 0x%02x" % gyro.readFIFOSRC() + " xyz: %8.3f %8.3f %8.3f  " % ( x,y,z ) + "tot: %11.6f, %11.6f, %11.6f" % (xt, yt, zt) + " s:%d" % gyro.spins
        print " xyz: %8.3f %8.3f %8.3f  " % ( x,y,z ) + "tot: %11.6f, %11.6f, %11.6f" % (xt, yt, zt) + " s:%d" % gyro.spins
        time.sleep(0.1)
