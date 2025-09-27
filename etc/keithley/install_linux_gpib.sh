#!/bin/bash
set -e

# written by Todd Micallef
# install_linux_gpib.sh

# adapted from https://raw.githubusercontent.com/PlesaEEVBlog/RPi_LogNut/master/Install_GPIB_Support.sh
# modified for the newer version of linux-gpib. The new version compiles differently and the commands were changed to reflect it
# samba is optional and installed if needed
# file "thermo_installer.run" is needed for the BME280 support. It should be placed in the boot dir with this file prior to booting the RPI
# Rev 1.0  initial release
# Rev 1.1  moved all tasks into functions so that individual sw packages can be reinstalled
#          added hotplug and gpib_config call on boot
#          added teckit and calkit install options
#          added BME280 support with Fluke 1620 emulation on telnet port 10001
#          additional file is needed for BME280 called thermo_installer.run
# Rev 1.2  added make clean to linux-gpib installer
#          added several packages back that were accidentally removed
# Rev 1.3  added check to make sure gpib_config command is only added once to /etc/rc.local
#          changed $HOME_DIR to reflect the directory where this script is called. It is not tied to /home/pi
#          changed initial check for gpib.conf from /etc to /usr/local/etc in case of upgrading from old version of linux-gpib
#          future change will go to both locations and delete gpib.conf before make is called
#          echo has been added for depmod, ldconfig, and gpib_config calls at the end of the install
# Rev 1.4  added option to do an uninstall. doing a make install twice, even after a make uninstall and make clean, breaks the install
#          fixed issue with adding gpib_config to rc.local file
#          removed some test code
# Rev 1.5  added cd.. to uninstall to bring the cursor out of the linux-gpib dir. If not, there will be a nested dir
#          forgot code to actually check if linux-gpib was previously installed.... it was left out for testing but never added afterward
# Rev 1.6  added ftp, git, mc, and screen to package install.
#          added reboot request after update_system. the kernel may have been updated and a reboot will be needed so linux-gpib can compile without errors
#          added function for UCTRONICS lcd display. basic testing is being performed. It may be removed if it is interfering with installation or function of linux-gpib
# Rev 1.7  added x11vnc package for a remote display
# Rev 1.8  changed LINUX_GPIB_VER from 4.3.3 to 4.3.4
#          changed the Adafruit pitft hat wget link to point to new location on GitHub
#          The HOTPLUG_RULES_FILE could not be created due to a permission error. The "| sudo tee" and "| sudo tee -a" commands were added to allow the creation of the file
#          purge files that check for ssh default password so the popup window on boot no longer appears "libpam-chksshpwd"  https://www.tomshardware.com/uk/how-to/disable-ssh-password-warning-raspberry-pi
#          added option to enable the desktop on the pitft if it is enabled. The user will still have to go into raspi-config and change the default boot mode
#          removed calkit install option.
# Rev 1.9  Made some modifications as per some suggestions from Gerwin J. Thanks for your tips
#          Removed function for uctronics. The code would not work properly so support is stopped for now
#          bash should now be the default shell. The old shell 'sh' is too basic and won't support some features like arrays
#          Removed functions  enable_tft_desktop() and install_tft and replaced with install_pitft. The latest installation routines combine both features
#          goodtft script overwrites /etc/rc.local file. gpib_config command is now called @reboot in root user crontab
# Rev 1.10 Added python-vxi11 package to the install script
# Rev 1.11 Updated linux-gpib install to version 4.3.5

# make sure paths end with a /
HOME_DIR="${HOME}/"
BOOT_DIR="/boot/"
KIT_DIR="/repo/"
GPIB_INSTALL_DIR="/opt"
LINUX_GPIB_VER="4.3.5"
GPIB_FILE="gpib.conf"
GPIB_FILE_PATH="/usr/local/etc/"
TIGHTVNC_FILE="tightvncserver"
TIGHTVNC_FILE_PATH="/etc/init.d/"
#GPIB_FILE_PATH="/etc/"
RC_LOCAL_PATH="/etc/"
SSH_CONFIG_PATH="/etc/ssh/"
HOTPLUG_RULES_FILE="99-linux_gpib_ni_usb.rules"
HOTPLUG_RULES_DIR="/etc/udev/rules.d/"
FBDEV_CONF_FILE="99-fbdev.conf"
FBDEV_CONF_FILE_DIR="/usr/share/X11/xorg.conf.d/"
WELCOME_SCREEN_FILE="/etc/xdg/autostart/piwiz.desktop"
CRONJOB_SERVER_SRCH="/usr/local/sbin/gpib_config"
CRONJOB_SERVER="@reboot /bin/sleep 10 ; /usr/local/sbin/gpib_config"

enable_root()
{
    sudo passwd root
    #sudo nano /etc/ssh/sshd_config
}

reboot_request()
{
    while true; do
        read -p "Do you want to reboot? " yn
        case $yn in
            [Yy]* ) sudo reboot now; break;;
            [Nn]* ) break;;
            * ) echo "Please answer yes or no.";;
        esac
    done
}

update_system()
{
    sudo apt-get update
    sudo apt-get -y upgrade

    sudo apt-get -y dist-upgrade

    sudo apt-get -y install raspberrypi-kernel raspberrypi-kernel-headers raspberrypi-bootloader

    # Remove realvnc before installing tightvnc. Realvnc server doesn't work with some clients
    sudo apt-get remove -y realvnc-vnc-server

    sudo apt-get -y install ftp vsftpd git mc screen i2c-tools bc python3-pip python-dev rpi-update tk-dev build-essential texinfo texi2html libcwidget-dev libncurses5-dev libx11-dev binutils-dev bison flex libusb-1.0-0 libusb-dev libmpfr-dev libexpat1-dev tofrodos subversion autoconf automake libtool libssl-dev build-essential mercurial dos2unix sqlite3 lshw

    # sudo apt-get -y purge wolfram-engine sonic-pi scratch minecraft-pi

    sudo apt-get -y remove libpam-chksshpwd

    sudo apt-get -y autoremove

    sudo apt-get -y autoclean

    sudo apt-get -y install automake libtool

    # SI units prefix support in Python
    # https://github.com/cfobel/si-prefix
    #sudo pip install --upgrade pip
    #sudo pip install si-prefix
    #sudo pip3 install python-vxi11
    echo "System software has been updated. It is recommended to reboot before continuing"
    reboot_request
    sleep 10
}

# Returns true if string $1 is part of any entry in root crontab
crontab_root_exists()
{
  sudo crontab -l 2>/dev/null | sudo grep -q $1  >/dev/null 2>/dev/null
}

uninstall_linux_gpib()
{
    cd linux-gpib-${LINUX_GPIB_VER}
    #cd linux-gpib-kernel-${LINUX_GPIB_VER}
    sudo rm -rf linux-gpib-kernel-${LINUX_GPIB_VER}
    #cd ..
    cd linux-gpib-user-${LINUX_GPIB_VER}
    sudo make_uninstall
    sudo make clean
    cd ..
    sudo rm -rf linux-gpib-user-${LINUX_GPIB_VER}
    sudo rm -f linux-gpib-kernel-${LINUX_GPIB_VER}.tar.gz
    sudo rm -f linux-gpib-user-${LINUX_GPIB_VER}.tar.gz
    cd ..
}

install_linux_gpib()
{
    cd "$GPIB_INSTALL_DIR"
    # test for directory where linux-gpib will be installed
    if [ -d "linux-gpib-${LINUX_GPIB_VER}" ];
    then
        while true; do
            read -p "Install directory for linux-gpib-${LINUX_GPIB_VER} already exists. Do you wish to uninstall it first? " yn
            case $yn in
                [Yy]* ) uninstall_linux_gpib; break;;
                [Nn]* ) break;;
                * ) echo "Please answer yes or no.";;
            esac
        done
    fi

    sudo wget https://sourceforge.net/projects/linux-gpib/files/linux-gpib%20for%203.x.x%20and%202.6.x%20kernels/${LINUX_GPIB_VER}/linux-gpib-${LINUX_GPIB_VER}.tar.gz
    sudo tar xvzf linux-gpib-${LINUX_GPIB_VER}.tar.gz

    cd linux-gpib-${LINUX_GPIB_VER}

    sudo tar xvzf linux-gpib-kernel-${LINUX_GPIB_VER}.tar.gz

    sudo tar xvzf linux-gpib-user-${LINUX_GPIB_VER}.tar.gz

    cd linux-gpib-kernel-${LINUX_GPIB_VER}
    sudo make -j6
    sudo make install

    cd ..
    cd linux-gpib-user-${LINUX_GPIB_VER}
    # compile linux-gpib python bindings
    cd language/python
    sudo python setup.py install
    cd ../..

    sudo ./bootstrap
    sudo ./configure
    sudo make clean
    sudo make -j6
    sudo make install

    # test for location of gpib.conf file. It is either in /etc or /usr/local/etc
    if [ -f "${GPIB_FILE_PATH}${GPIB_FILE}" ]; then
        echo "${GPIB_FILE_PATH}${GPIB_FILE} exist"
    elif [ -f "/etc/${GPIB_FILE}" ]; then
        echo "/etc/${GPIB_FILE} exist"
        GPIB_FILE_PATH="/etc/"
    else
        echo "${GPIB_FILE} does not exist. Copying from template dir to /usr/local/etc/"
        sudo cp /opt/linux-gpib-${LINUX_GPIB_VER}/linux-gpib-user-${LINUX_GPIB_VER}/util/templates/gpib.conf ${GPIB_FILE_PATH}
    fi

    cd ../..

    # Install FxLoad
    sudo mkdir -p /usr/share/usb/agilent_82357a
    sudo mkdir -p /usr/share/usb/ni_usb_gpib
    sudo mkdir -p gpib_firmware
    cd gpib_firmware

    sudo wget http://linux-gpib.sourceforge.net/firmware/gpib_firmware-2008-08-10.tar.gz
    sudo tar xvzf gpib_firmware-2008-08-10.tar.gz

    sudo cp  gpib_firmware-2008-08-10/agilent_82357a/82357a_fw.hex /usr/share/usb/agilent_82357a/
    sudo cp  gpib_firmware-2008-08-10/agilent_82357a/measat_releaseX1.8.hex /usr/share/usb/agilent_82357a/
    sudo cp  gpib_firmware-2008-08-10/ni_gpib_usb_b/niusbb_loader.hex /usr/share/usb/ni_usb_gpib/
    sudo cp  gpib_firmware-2008-08-10/ni_gpib_usb_b/niusbb_firmware.hex /usr/share/usb/ni_usb_gpib/
    sudo apt-get -y install fxload


    if lsusb | grep -q '0957:0518'; then
        sudo sed -i 's/ni_pci/agilent_82357a/g' ${GPIB_FILE_PATH}gpib.conf
        echo "Agilent 82357B found"
        sudo modprobe agilent_82357a
    fi

    if lsusb | grep -q '0957:0718'; then
        sudo sed -i 's/ni_pci/agilent_82357a/g' ${GPIB_FILE_PATH}gpib.conf
        echo "Agilent 82357B found"
        sudo modprobe agilent_82357a
    fi

    if lsusb | grep -q '3923:709b'; then
        sudo sed -i 's/ni_pci/ni_usb_b/g' ${GPIB_FILE_PATH}gpib.conf
        echo "National Instruments NI GPIB-USB-HS found"
        sudo modprobe ni_usb_gpib

        # the following code creates a rules file for hotplugging the NI GPIB_USB_HS adapter
        cd "$HOME_DIR"
        echo 'SUBSYSTEM=="usb", ACTION=="add", ENV{DEVTYPE}=="usb_device", ATTR{idVendor}=="3923", ATTR{idProduct}=="709b", MODE="660", GROUP="username", SYMLINK+="usb_gpib"' | sudo tee ${HOTPLUG_RULES_FILE}
        echo 'SUBSYSTEM=="usb", ACTION=="add", ENV{DEVTYPE}=="usb_device", ATTR{idVendor}=="3923", ATTR{idProduct}=="709b", RUN+="/usr/local/sbin/gpib_config"' | sudo tee -a ${HOTPLUG_RULES_FILE}
        echo 'KERNEL=="gpib[0-9]*", ACTION=="add", MODE="660", GROUP="username"' | sudo tee -a ${HOTPLUG_RULES_FILE}
        sudo cp ${HOTPLUG_RULES_FILE} ${HOTPLUG_RULES_DIR}
        sudo rm ${HOTPLUG_RULES_FILE}
    fi

    echo "running ldconfig"
    sudo ldconfig
    echo "running depmod -a"
    sudo depmod -a
    echo "running gpib_config"
    sudo gpib_config

    # the following lines runs gpib_config on boot. gpib_config command added before the "exit 0" at the end of the file
    # first test if it already exists. if true, skip adding line
    # exit 0 may occur more than once in the file (as part of instructions) so only care about last entry
    if ! crontab_root_exists "$CRONJOB_SERVER_SRCH" ; then
        echo "gpib_config cronjob doesn't exist. Creating new job."
        (sudo crontab -l 2>/dev/null; echo "$CRONJOB_SERVER") | sudo crontab -
    else
        echo "gpib_config cronjob already exists"
    fi
}

# Add support for the Adafruit pitft
install_pitft()
{
    cd "$HOME_DIR"
    sudo apt-get install -y git python3-pip
    sudo pip3 install --upgrade adafruit-python-shell click
    git clone https://github.com/adafruit/Raspberry-Pi-Installer-Scripts.git
    cd Raspberry-Pi-Installer-Scripts
    # specify -u home directory for users other than pi
    sudo python3 adafruit-pitft.py -u "$HOME"
}

# Add support for goodtft
# https://www.dfrobot.com/product-2326.html
# http://www.lcdwiki.com/How_to_install_the_LCD_driver
# MHS35 display 480x320
install_goodtft()
{
    sudo rm -rf LCD-show
    git clone https://github.com/goodtft/LCD-show.git
    chmod -R 755 LCD-show
    cd LCD-show/
    sudo ./MHS35-show
    rotate_options=("0" "90" "180" "270")

    PS3="Select the LCD rotation (1-4): "
    select i in "${rotate_options[@]}"; do
        case $i in
            0) echo "0 selected"
                    sudo ./rotate.sh 0
                    break
                    ;;
            90) echo "90 selected"
                    sudo ./rotate.sh 90
                    break
                    ;;
            180) echo "180 selected"
                    sudo ./rotate.sh 180
                    break
                    ;;
            270) echo "270 selected"
                    sudo ./rotate.sh 270
                    break
                    ;;
        esac
    done
}

# Menu to select model of tft (pitft or goodtft)
select_lcd()
{
    echo "Adding support for Adafruit PiTFT(or compatible) or DFRobot(FIT0820 w/goodtft) LCD"
    echo "Select type of lcd display"
    cd "$HOME_DIR"
    lcd_options=("pitft" "goodtft" "exit")

    PS3="Select the LCD type (1-3): "
    select i in "${lcd_options[@]}"; do
        case $i in
            pitft) echo "pitft selected"
                    install_pitft
                    break
                    ;;
            goodtft) echo "goodtft selected"
                    install_goodtft
                    break
                    ;;
            exit)   break;;
            *)      echo "Please select a valid number (1-3)"
        esac
    done
}

# add BME_280 sensor and Fluke 1620 emulator server on telnet port 10001
# sudo pip install RPi.bme280
install_BME_server()
{
    cd "$BOOT_DIR"
    echo "install_linux_gpib.sh USER is" "$USER"
    # test for existence of thermo_installer.run file
    if [ -f "${BOOT_DIR}/thermo_installer.run" ]; then
        #chmod +x thermo_installer.run
        ./thermo_installer.run

    else
        echo "${BOOT_DIR}/thermo_installer.run does not exist. Cannot install BME280 support"
    fi
}

install_samba()
{
    cd "$HOME_DIR"
    sudo apt-get -y install samba samba-common-bin
}

install_teckit()
{
    ### Check for dir, if not found create it using the mkdir ##
    [ ! -d "$KIT_DIR" ] && sudo mkdir -p "$KIT_DIR"
    cd "$KIT_DIR"
    sudo git clone https://github.com/tin-/teckit
}

# Add support for ds1307 or ds3231 RTC
install_rtc()
{
    echo "Installing support for a hardware RTC"
    cd "$HOME_DIR"
    RTC=""  # No spaces or there will be an error
    rtc_options=("ds1307" "ds3231" "exit")

    PS3="Select the RTC (1-3): "
    select i in "${rtc_options[@]}"; do
        case $i in
            ds1307) echo "ds1307 selected"
                    RTC="ds1307"
                    break
                    ;;
            ds3231) echo "ds3231 selected"
                    RTC="ds3231"
                    break
                    ;;
            exit)   break;;
            *)      echo "Please select a valid number (1-3)"
        esac
    done

    # Download the library if a selection was made
    if [ ! -z "${RTC}" ]; then
        echo "downloading rtc library"
        cd "$HOME_DIR"
        if [[ -d "$HOME_DIR"pi-hats ]]; then  # Check if pi-hats already exists. If so, delete it first or download
            echo "pi-hats already exists. Removing the directory"
            sudo rm -vrf pi-hats
        fi
        sudo git clone https://github.com/Seeed-Studio/pi-hats.git
        cd pi-hats/tools
    fi

    # Install the selected rtc
    if [[ "${RTC}" == "ds1307" ]]; then
        sudo ./install.sh -u rtc_ds1307
    elif [[ "${RTC}" == "ds3231" ]]; then
        sudo ./install.sh -u rtc_ds3231
    fi
}

# https://incoherentmusings.wordpress.com/2016/04/25/setting-up-vnc-server-on-raspberry-pi-to-autostart-on-reboot/
create_tightvnc_init_file()
{
    printf '#!/bin/sh\n' | sudo tee "$TIGHTVNC_FILE_PATH""$TIGHTVNC_FILE" >/dev/null
    printf '### BEGIN INIT INFO\n' | sudo tee -a "$TIGHTVNC_FILE_PATH""$TIGHTVNC_FILE" >/dev/null
    printf '# Provides: tightvncserver\n'  | sudo tee -a "$TIGHTVNC_FILE_PATH""$TIGHTVNC_FILE" >/dev/null
    printf '# Required-Start: $syslog\n'  | sudo tee -a "$TIGHTVNC_FILE_PATH""$TIGHTVNC_FILE" >/dev/null
    printf '# Required-Stop: $syslog\n'  | sudo tee -a "$TIGHTVNC_FILE_PATH""$TIGHTVNC_FILE" >/dev/null
    printf '# Default-Start: 2 3 4 5\n'  | sudo tee -a "$TIGHTVNC_FILE_PATH""$TIGHTVNC_FILE" >/dev/null
    printf '# Default-Stop: 0 1 6\n'  | sudo tee -a "$TIGHTVNC_FILE_PATH""$TIGHTVNC_FILE" >/dev/null
    printf '# Short-Description: vnc server\n'  | sudo tee -a "$TIGHTVNC_FILE_PATH""$TIGHTVNC_FILE" >/dev/null
    printf '# Description:\n'  | sudo tee -a "$TIGHTVNC_FILE_PATH""$TIGHTVNC_FILE" >/dev/null
    printf '#\n'  | sudo tee -a "$TIGHTVNC_FILE_PATH""$TIGHTVNC_FILE" >/dev/null
    printf '### END INIT INFO\n'  | sudo tee -a "$TIGHTVNC_FILE_PATH""$TIGHTVNC_FILE" >/dev/null
    printf '# /etc/init.d/tightvncserver\n'  | sudo tee -a "$TIGHTVNC_FILE_PATH""$TIGHTVNC_FILE" >/dev/null
    printf '# Set the VNCUSER variable to the name of the user to start tightvncserver under\n'  | sudo tee -a "$TIGHTVNC_FILE_PATH""$TIGHTVNC_FILE" >/dev/null
    printf 'VNCUSER="%s"\n' "$USER"  | sudo tee -a "$TIGHTVNC_FILE_PATH""$TIGHTVNC_FILE" >/dev/null
    printf 'case "$1" in\n'  | sudo tee -a "$TIGHTVNC_FILE_PATH""$TIGHTVNC_FILE" >/dev/null
    printf 'start)\n'  | sudo tee -a "$TIGHTVNC_FILE_PATH""$TIGHTVNC_FILE" >/dev/null
    printf '#Change the display number below. The connection port will be 5900 + display\n' | sudo tee -a "$TIGHTVNC_FILE_PATH""$TIGHTVNC_FILE" >/dev/null
    printf 'su $VNCUSER -c "/usr/bin/tightvncserver :1"\n'  | sudo tee -a "$TIGHTVNC_FILE_PATH""$TIGHTVNC_FILE" >/dev/null
    printf 'echo "Starting TightVNC server for $VNCUSER"\n'  | sudo tee -a "$TIGHTVNC_FILE_PATH""$TIGHTVNC_FILE" >/dev/null
    printf ';;\n'  | sudo tee -a "$TIGHTVNC_FILE_PATH""$TIGHTVNC_FILE" >/dev/null
    printf 'stop)\n'  | sudo tee -a "$TIGHTVNC_FILE_PATH""$TIGHTVNC_FILE" >/dev/null
    printf 'pkill Xtightvnc\n'  | sudo tee -a "$TIGHTVNC_FILE_PATH""$TIGHTVNC_FILE" >/dev/null
    printf 'echo "Tightvncserver stopped"\n'  | sudo tee -a "$TIGHTVNC_FILE_PATH""$TIGHTVNC_FILE" >/dev/null
    printf ';;\n'  | sudo tee -a "$TIGHTVNC_FILE_PATH""$TIGHTVNC_FILE" >/dev/null
    printf '*)\n'  | sudo tee -a "$TIGHTVNC_FILE_PATH""$TIGHTVNC_FILE" >/dev/null
    printf 'echo "Usage: /etc/init.d/tightvncserver {start|stop}"\n'  | sudo tee -a "$TIGHTVNC_FILE_PATH""$TIGHTVNC_FILE" >/dev/null
    printf 'exit 1\n'  | sudo tee -a "$TIGHTVNC_FILE_PATH""$TIGHTVNC_FILE" >/dev/null
    printf ';;\n'  | sudo tee -a "$TIGHTVNC_FILE_PATH""$TIGHTVNC_FILE" >/dev/null
    printf 'esac\n'  | sudo tee -a "$TIGHTVNC_FILE_PATH""$TIGHTVNC_FILE" >/dev/null
    printf 'exit 0\n'  | sudo tee -a "$TIGHTVNC_FILE_PATH""$TIGHTVNC_FILE" >/dev/null
}

# Activate vnc to get access to the PIXEL desktop
# https://www.childs.be/blog/post/remove-real-vnc-from-raspberrypi-pixel-and-install-tightvnc
install_vnc()
{
    sudo apt-get install -y tightvncserver
    echo "You will need to enter up to an 8 character password for user ""$USER"
    tightvncserver
    # check for existence of auto start file for tightvncserver
    if [ ! -e "$TIGHTVNC_FILE_PATH""$TIGHTVNC_FILE" ]; then
        echo "file" "$TIGHTVNC_FILE_PATH""$TIGHTVNC_FILE" "does not exist. Creating it now"
        create_tightvnc_init_file
    else
        echo "file" "$TIGHTVNC_FILE_PATH""$TIGHTVNC_FILE" "already exists"
    fi

    echo "chmod 755 file" "$TIGHTVNC_FILE_PATH""$TIGHTVNC_FILE"
    sudo chmod 755 "$TIGHTVNC_FILE_PATH""$TIGHTVNC_FILE"
    echo "performing update-rc.d tightvncserver defaults"
    sudo update-rc.d tightvncserver defaults
}

reboot_recommended()
{
    echo "It is recommended to reboot now before installing more packages."
    reboot_request
}

#####################
# Script starts here
#####################
echo "home dir is ${HOME_DIR}"
echo "Please make sure this script is running with the correct home directory... not root"

# ---------------------------------------------------------------------
# update and install system software
while true; do
    read -p "Do you wish to update and install raspbian system software? " yn
    case $yn in
        [Yy]* ) update_system; break;;
        [Nn]* ) break;;
        * ) echo "Please answer yes or no.";;
    esac
done

# ---------------------------------------------------------------------
# add support for linux-gpib
while true; do
    read -p "Do you wish to install linux-gpib? " yn
    case $yn in
        [Yy]* ) install_linux_gpib; break;;
        [Nn]* ) break;;
        * ) echo "Please answer yes or no.";;
    esac
done

# ---------------------------------------------------------------------
# add support for teckit
while true; do
    read -p "Do you wish to install teckit? " yn
    case $yn in
        [Yy]* ) install_teckit; break;;
        [Nn]* ) break;;
        * ) echo "Please answer yes or no.";;
    esac
done

# ---------------------------------------------------------------------
# add support for samba
while true; do
    read -p "Do you wish to install samba? " yn
    case $yn in
        [Yy]* ) install_samba; break;;
        [Nn]* ) break;;
        * ) echo "Please answer yes or no.";;
    esac
done

# ---------------------------------------------------------------------
#enable tft display if installed
# https://learn.adafruit.com/adafruit-2-2-pitft-hat-320-240-primary-display-for-raspberry-pi/easy-install

while true; do
    read -p "Do you wish to install hardware support for LCD hat? " yn
    case $yn in
        [Yy]* ) select_lcd; break;;
        [Nn]* ) break;;
        * ) echo "Please answer yes or no.";;
    esac
done

# ---------------------------------------------------------------------
#add support for BME280 and Fluke 1620 emulation
while true; do
    read -p "Do you wish to add Fluke 1620 emulation using a BME280? Warning! telnet service will be added to port 10001 " yn
    case $yn in
        [Yy]* ) install_BME_server; echo "A BME280 should be connected to the I2C port."; break;;
        [Nn]* ) break;;
        * ) echo "Please answer yes or no.";;
    esac
done

# ---------------------------------------------------------------------
# add support for RTC hardware
while true; do
    read -p "Do you wish to install hardware support for a ds3231 or ds1307 RTC? " yn
    case $yn in
        [Yy]* ) install_rtc; break;;
        [Nn]* ) break;;
        * ) echo "Please answer yes or no.";;
    esac
done

# ---------------------------------------------------------------------
# add support for VNC
while true; do
    read -p "Do you wish to install support for VNC to view the PIXEL desktop? " yn
    case $yn in
        [Yy]* ) install_vnc; break;;
        [Nn]* ) break;;
        * ) echo "Please answer yes or no.";;
    esac
done

# ---------------------------------------------------------------------
# reboot system
# delay to make sure all writing has been completed before rebooting
echo "Short delay..."
sleep 10
echo "Finished installing options"
reboot_request









