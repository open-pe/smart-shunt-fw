
# How to connect to DMM6500 via USB


```

# first check
fab@rpi:~/keithley $ lsusb
Bus 004 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
Bus 003 Device 002: ID 05e6:6500 Keithley Instruments Digital Multimeter
Bus 003 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
Bus 002 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
fab@rpi:~/keithley $ 


"On Unix system, one may have to modify udev rules to allow non-root access to the device you are trying to connect to.
 The following tutorial describes how to do it http://ask.xmodulo.com/change-usb-device-permission-linux.html."
 
 
echo 'SUBSYSTEMS=="usb", ATTRS{idVendor}=="05e6", ATTRS{idProduct}=="6500", GROUP="users", MODE="0666"' \
    > /etc/udev/rules.d/50-myusb.rules
sudo udevadm control --reload



```