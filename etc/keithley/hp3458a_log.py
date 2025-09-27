"""

https://web.archive.org/web/20250114062103mp_/https://xdevs.com/guide/ni_gpib_rpi/
https://web.archive.org/web/20241206034242/https://xdevs.com/datalog_hp/
https://web.archive.org/web/20241127060431mp_/https://xdevs.com/article/hp3458a_gpib/

https://github.com/tin-/teckit/blob/master/devices/hp3458a.py

getting install_lunux_gpib.sh to run under aarch64:
use LINUX_GPIB_VER="4.3.6"
also need to install sudo

sudo apt-get install automake libtool

for some reason i had to manually install the kernel modules in order to modprobe not throw exec errors:

```
fab@rpi:/opt/linux-gpib-4.3.6/linux-gpib-kernel-4.3.6 $ sudo modprobe gpib_common
modprobe: ERROR: could not insert 'gpib_common': Exec format error

cd /opt/linux-gpib-4.3.6/linux-gpib-kernel-4.3.6
sudo make -j6 install
sudo modprobe gpib_common
sudo modprobe ni_usb_gpib

stat /dev/gpib0
  File: /dev/gpib0
  Size: 0         	Blocks: 0          IO Block: 16384  character special file
Device: 0,5	Inode: 662         Links: 1     Device type: 160,0
Access: (0660/crw-rw----)  Uid: (    0/    root)   Gid: (    0/    root)
Access: 2025-02-22 21:17:34.956349256 +0100
Modify: 2025-02-22 21:17:34.956349256 +0100
Change: 2025-02-22 21:17:34.956349256 +0100
 Birth: 2025-02-22 21:17:34.936349318 +0100

```


```

sudo ibtest

```


triggering works, but not sending commands.
[Feb22 21:47] /opt/linux-gpib-4.3.6/linux-gpib-kernel-4.3.6/drivers/gpib/ni_usb/ni_usb_gpib.c: ni_usb_write: Addressing error retval 0 error code=3


trigger() from python calls gpib_trigger/ ibtrg
    .. my_trigger .. create_send_setup .. my_ibcmd









[  +0.309120] usb 3-1: new high-speed USB device number 3 using xhci-hcd
[  +0.151371] usb 3-1: New USB device found, idVendor=3923, idProduct=709b, bcdDevice= 1.01
[  +0.000005] usb 3-1: New USB device strings: Mfr=1, Product=2, SerialNumber=3
[  +0.000003] usb 3-1: Product: GPIB-USB-HS
[  +0.000003] usb 3-1: Manufacturer: National Instruments
[  +0.000002] usb 3-1: SerialNumber: 01AB3A01
[  +0.001908] ni_usb_gpib: probe succeeded for path: usb-xhci-hcd.1-1
[  +0.010132] gpib0: exiting autospoll thread
[  +0.000037] ni_usb_gpib: attach
[  +0.000005] usb 3-1: bus 3 dev num 3 attached to gpib minor 0, NI usb interface 0
[  +0.002651] 	product id=0x709b
[  +0.000380] ni_usb_hs_wait_for_ready: board serial number is 0x1ab3a01
[  +0.000499] /opt/linux-gpib-4.3.6/linux-gpib-kernel-4.3.6/drivers/gpib/ni_usb/ni_usb_gpib.c: ni_usb_hs_wait_for_ready: unexpected data: buffer[6]=0x17, expected 0x2, 0xe or 0xf
[  +0.000004] /opt/linux-gpib-4.3.6/linux-gpib-kernel-4.3.6/drivers/gpib/ni_usb/ni_usb_gpib.c: ni_usb_hs_wait_for_ready: unexpected data: buffer[10]=0x5e, expected 0x96 or 0x07
[  +0.000003] ni_usb_dump_raw_block:

[  +0.000003]  40
[  +0.000001]   1
[  +0.000001]   0
[  +0.000002]   1
[  +0.000001]  30
[  +0.000001]   1
[  +0.000001]  17
[  +0.000001]   5

[  +0.000003]   0
[  +0.000001]   0
[  +0.000001]  5e










[Feb22 22:52] /opt/linux-gpib-4.3.6/linux-gpib-kernel-4.3.6/drivers/gpib/ni_usb/ni_usb_gpib.c: ni_usb_write: Addressing error retval 0 error code=3



[  495.730321] Linux-GPIB 4.3.6 Driver
[  966.828095] usb 1-1.4: new high-speed USB device number 4 using dwc_otg
[  966.958655] usb 1-1.4: New USB device found, idVendor=3923, idProduct=709b, bcdDevice= 1.01
[  966.958687] usb 1-1.4: New USB device strings: Mfr=1, Product=2, SerialNumber=3
[  966.958702] usb 1-1.4: Product: GPIB-USB-HS
[  966.958714] usb 1-1.4: Manufacturer: National Instruments
[  966.958725] usb 1-1.4: SerialNumber: 01AB3A01
[  967.102457] ni_usb_gpib driver loading
[  967.102537] ni_usb_gpib: probe succeeded for path: usb-3f980000.usb-1.4
[  967.102687] usbcore: registered new interface driver ni_usb_gpib
[  967.102694] gpib: registered ni_usb_b interface
[ 1439.471789] ni_usb_gpib: attach
[ 1439.471814] usb 1-1.4: bus 1 dev num 4 attached to gpib minor 0, NI usb interface 0
[ 1439.471895] 	product id=0x709b
[ 1439.471948] ni_usb_hs_wait_for_ready: board serial number is 0x1ab3a01
[ 1439.471997] /opt/linux-gpib-4.3.6/linux-gpib-kernel-4.3.6/drivers/gpib/ni_usb/ni_usb_gpib.c: ni_usb_hs_wait_for_ready: unexpected data: buffer[6]=0x17, expected 0x2, 0xe or 0xf
[ 1439.472006] /opt/linux-gpib-4.3.6/linux-gpib-kernel-4.3.6/drivers/gpib/ni_usb/ni_usb_gpib.c: ni_usb_hs_wait_for_ready: unexpected data: buffer[10]=0x5e, expected 0x96 or 0x07
[ 1439.472013] ni_usb_dump_raw_block:

[ 1439.472019]  40
[ 1439.472022]   1
[ 1439.472026]   0
[ 1439.472029]   1
[ 1439.472032]  30
[ 1439.472035]   1
[ 1439.472038]  17
[ 1439.472041]   5

[ 1439.472048]   0
[ 1439.472050]   0
[ 1439.472053]  5e

[ 1751.568574] /opt/linux-gpib-4.3.6/linux-gpib-kernel-4.3.6/drivers/gpib/ni_usb/ni_usb_gpib.c: ni_usb_write: Addressing error retval 0 error code=3
[ 2274.438806] usb 1-1.4: USB disconnect, device number 4



Traceback (most recent call last):
  File "<stdin>", line 1, in <module>
  File "/usr/local/lib/python3.11/dist-packages/gpib-1.0-py3.11-linux-armv7l.egg/Gpib.py", line 53, in write
    gpib.write(self.id, str)
gpib.GpibError: write() error: No such device or address (errno: 6)

"""

# xDevs.com Python test GPIB app.

# https://xdevs.com/guide/ni_gpib_rpi/
# https://xdevs.com/article/hp3458a_gpib/
# https://xdevs.com/fix/hp3458a/
import sys
import Gpib
import time
import ftplib

port = 21
ip = "192.168.1.102"
password = "datashort"
user = "datashort"

gen = Gpib.Gpib(0,2, timeout=20) # 3245A GPIB Address = 2
inst = Gpib.Gpib(0,3, timeout=20) # 3458A GPIB Address = 3
kei = Gpib.Gpib(0,16, timeout=20) # K2001 GPIB Address = 16
inst.clear()
kei.clear()
gen.clear()
#Setup HP 3245A
#...
#Setup HP 3458A
#...
# Setup Keithley 2001
#...

cnt = 0
tread = 0
temp = 41.0
level = 10.000000                           # Reference level for 3458A data
klevel = 10.0000000                         # Reference level for K2001 data
ppm = 0                                     # deviation from ref level, in ppm

with open('10v_keihp_nplc100.csv', 'a') as o:
    #    o.write("date;hp3458a;level;kei2001;temp;ppm_level;kppm;\r\n")
    o.close()

ftp = ftplib.FTP(ip)                        # Use ftplib
ftp.login(user,password)                    # Login to FTP

while cnt <= 10000000:                      # Main loop, take 10M samples
    cnt+=1
    tread = tread - 1
    with open('10v_keihp_nplc100.csv', 'a') as o:   # Open file to addition
        if (tread == 0):
            tread = 25                      # Send TEMP? to 3458A every 25th measurement
            inst.write("TARM SGL,1")
            inst.write("TEMP?")
            temp = inst.read()
        inst.write("TARM SGL,1")            # Take single reading
        data = inst.read()
        ppm = ((float(data) / level)-1)*1E6  # Calculate ppm dev
        inst.write("DISP OFF, \" %8.4f ppm\"" % float(ppm))  # Display ppm dev on 3458A screen
        time.sleep(1)                       # Do nothing for 1 second
        kei.write("READ?")                  # Read Keithley's data
        keival = kei.read()
        kppm = ((float(keival) / klevel)-1)*1E6
        kei.write(":DISP:WIND2:TEXT:DATA \"  %8.4f ppm\";STAT ON;" % float(kppm))
        khp = (((float(data) / float(keival))-1)*1E6)  # 3458A/K2001 deviation in PPM
        print time.strftime("%d/%m/%Y-%H:%M:%S;") + ("[%8d]: %2.8f , dev %4.4f ppm, K:%2.8f, dev %4.4f ppm, TEMP:%3.1f, K/H %4.2f" % (min, float(data),float(ppm),float(keival),float(kppm),float(temp),float(khp) ) )
        o.write (time.strftime("%d/%m/%Y-%H:%M:%S;") + ("%16.8f;%16.8f;%16.8f;%3.1f;%4.3f;%4.3f\r\n" % (float(data),float(level),float(keival),float(temp),float(ppm),float(kppm) ) ))
        o.close()                           # close file
    file = open('10v_keihp_nplc100.csv','rb')  # link to datafile
    ftp.storbinary('STOR 10v_keihp_nplc100.csv', file) # send file to FTP
    file.close()                               # close file
ftp.quit()