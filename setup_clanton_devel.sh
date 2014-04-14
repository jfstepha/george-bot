export ROBOT=clanton
export ROSLAUNCH_SSH_UNKNOWN=1
export ROS_MASTER_URI=http://jfstepha-mint:11311
export ENV_LOADER=/home/jfstepha/catkin_ws/devel/env.sh 
export DO_SERVOS=true
source ~/catkin_ws/devel/setup.bash
# SPI setup
#sudo sh -c 'echo 42 > /sys/class/gpio/export'
#sudo sh -c 'echo 43 > /sys/class/gpio/export'
#sudo sh -c 'echo 54 > /sys/class/gpio/export'
#sudo sh -c 'echo 55 > /sys/class/gpio/export'
#sudo sh -c 'echo "out" > /sys/class/gpio/gpio42/direction'
#sudo sh -c 'echo "out" > /sys/class/gpio/gpio43/direction'
#sudo sh -c 'echo "out" > /sys/class/gpio/gpio54/direction'
#sudo sh -c 'echo "out" > /sys/class/gpio/gpio55/direction'

#sudo sh -c 'echo 39 > /sys/class/gpio/export'
#sudo sh -c 'echo 38 > /sys/class/gpio/export'
#sudo sh -c 'echo 16 > /sys/class/gpio/export'
#sudo sh -c 'echo 25 > /sys/class/gpio/export'
#sudo sh -c 'echo "out" > /sys/class/gpio/gpio39/direction'
#sudo sh -c 'echo "out" > /sys/class/gpio/gpio38/direction'
#sudo sh -c 'echo "out" > /sys/class/gpio/gpio16/direction'
#sudo sh -c 'echo "out" > /sys/class/gpio/gpio25/direction'
#sudo sh -c 'echo 1 > /sys/class/gpio/gpio16/value'

sudo /home/jfstepha/catkin_ws/src/george/src/Gyro/setup_gyro.py

# I2C setup
sudo sh -c 'echo 29 > /sys/class/gpio/export'
sudo sh -c 'echo "out" > /sys/class/gpio/gpio29/direction'
sudo sh -c 'echo 0 > /sys/class/gpio/gpio29/value'

# Analog inputs setup
sudo sh -c 'echo 37 > /sys/class/gpio/export'
sudo sh -c 'echo 36 > /sys/class/gpio/export'
sudo sh -c 'echo 23 > /sys/class/gpio/export'
sudo sh -c 'echo "out" > /sys/class/gpio/gpio37/direction'
sudo sh -c 'echo "out" > /sys/class/gpio/gpio36/direction'
sudo sh -c 'echo "out" > /sys/class/gpio/gpio23/direction'
sudo sh -c 'echo 0 > /sys/class/gpio/gpio37/value'
sudo sh -c 'echo 0 > /sys/class/gpio/gpio36/value'
sudo sh -c 'echo 0 > /sys/class/gpio/gpio23/value'




#sudo /home/jfstepha/spi/arduino/read_adc
