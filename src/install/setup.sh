#!/usr/bin/env bash

# Since 12/10/2023 this file is adapted for Raspberry Pi OS 11 (further than Raspberry Pi OS 10)

# ##################################### initial part (common) #####################################
set -e

print_header () {
  echo
  echo $1
  echo
}


if [ "$EUID" -ne 0 ]; then
  echo "Please run as root (sudo $0)"
  exit
fi


print_header "Deactivating graphical login"
systemctl --quiet set-default multi-user.target


echo -e "\nChecking SWAP SIZE"
SWAPSPACE=$(sed -n "s/^SwapTotal: *\([0-9]*\) kB/\1/p" /proc/meminfo)
echo "Current SWAP SIZE is $SWAPSPACE Kb"
if [ -f /etc/dphys-swapfile ]; then
  if [[ $SWAPSPACE -le 524000 ]]; then
    echo "Increasing SWAP SIZE to 512Mb"
    sudo sed -i 's/^CONF_SWAPSIZE=[0-9]*$/CONF_SWAPSIZE=512/' /etc/dphys-swapfile
    sudo /etc/init.d/dphys-swapfile restart
  else
    echo "SWAP SIZE is sufficient"
  fi
  echo -e "\n"
else
  echo -e "\n\nNOT found /etc/dphys-swapfile file, or not able to parse CONF_SWAPSIZE value"
fi


print_header "configuring config file"
CONFIG=/boot/config.txt
sed -i '/dtparam=spi/d' $CONFIG
sed -i '/startx/d' $CONFIG
sed -i '/gpu_mem/d' $CONFIG
sed -i '/enable_uart/d' $CONFIG
cat <<EOT >> $CONFIG
dtparam=spi=on
start_x=1
gpu_mem=128
enable_uart=1
EOT


print_header "Updating packages"
apt update
apt -y -qq upgrade


os_version=$(lsb_release -r | tr -cd '[[:digit:]]')
echo -e "\n\n#####################################################"
echo "Setup proceeds, based on Raspberry Pi os_version: $os_version"
echo -e "#####################################################\n\n"



# ##################################### RASPBIAN 10 specific ######################################
# (20231012 image) https://downloads.raspberrypi.org/raspios_oldstable_armhf/images/raspios_oldstable_armhf-2023-05-03/2023-05-03-raspios-buster-armhf.img.xz

if [[ "$os_version" == "10" ]]; then

    print_header "Removing other old packages"
    apt remove -y -qq python3-numpy python3-picamera
    apt autoremove -y

    print_header "Installing required packages"
    apt install -y -qq python3-rpi.gpio python3-pigpio python3-gpiozero python3-pil python3-pip python3-venv
    apt install -y -qq libatlas-base-dev python3-h5py libjasper-runtime libqtgui4 libqt4-test

    print_header "Creating python virtual env"
    python3 -m venv .virtualenvs --system-site-packages
    source .virtualenvs/bin/activate

    print_header "Installing required python packages"
    pip3 install --upgrade pip
    pip3 install numpy==1.21.4
    pip3 install "picamera[array]"
    pip3 install st7735==0.0.4.post1
    pip3 install st7789==0.0.4
    pip3 install RubikTwoPhase==1.1.1
    pip3 install getmac==0.8.3

    # hash for opencv for pizero seems to be bad on pywheel, bypass it for the moment. It is ok for pizero 2w
    machine=$(uname -m)
    if [ "$machine" == "armv6l" ]; then
        pip3 install https://www.piwheels.org/simple/opencv-contrib-python/opencv_contrib_python-4.1.0.25-cp37-cp37m-linux_armv6l.whl
    else
        pip3 install opencv-contrib-python==4.4.0.46
    fi

    set +e

    print_header "Configuring vnc server to run at startup and prepare setup for cubotino"
    if ! crontab -l 2>/dev/null | grep -q "Cubotino_T_bash.sh"; then
        (crontab -l 2>/dev/null;\
        echo -e 'MAILTO=""';\
        echo -e '@reboot su - pi -c "/usr/bin/vncserver :0 -geometry 1280x720\"';\
        echo -e '#@reboot su - pi -c "/usr/bin/vncserver :0 -geometry 1920x1080"';\
        echo -e '#@reboot /bin/sleep 5; bash -l /home/pi/cubotino/src/Cubotino_T_bash.sh > /home/pi/cubotino/src/Cubotino_T_terminal.log 2>&1')\
        | crontab -
    fi



# ##################################### RASPBIAN 11 specific #######################################
# (20231012 image) https://downloads.raspberrypi.org/raspios_oldstable_armhf/images/raspios_oldstable_armhf-2023-10-10/2023-05-03-raspios-bullseye-armhf.img.xz

elif [[ "$os_version" == "11" ]]; then

    print_header "Installing required packages"
    apt-get -y install python3-opencv

    print_header "Creating python virtual env"
    python3 -m venv .virtualenvs --system-site-packages
    source .virtualenvs/bin/activate

    print_header "Installing required python packages"
    pip3 install --upgrade pip
    pip3 install st7735==0.0.4.post1
    pip3 install st7789==0.0.4
    pip3 install RubikTwoPhase==1.1.1
    pip3 install getmac==0.8.3

    set +e

    print_header "Configuring vnc server to run at startup and prepare setup for cubotino"
    if ! crontab -l 2>/dev/null | grep -q "Cubotino_T_bash.sh"; then
        (crontab -l 2>/dev/null;\
        echo -e 'MAILTO=""';\
        echo -e '@reboot su - pi -c "/usr/bin/vncserver-virtual :1 -randr=1280x720"';\
        echo -e '#@reboot su - pi -c "/usr/bin/vncserver-virtual :1 -randr=1920x1080"';\
        echo -e '#@reboot /bin/sleep 5; bash -l /home/pi/cubotino/src/Cubotino_T_bash.sh > /home/pi/cubotino/src/Cubotino_T_terminal.log 2>&1')\
        | crontab -
    fi



# ##################################### RASPBIAN <10 | RASPBIAN >11 ###############################
else
    print_header "the setup.sh file does not handle $os_version"
fi



# ##################################### last part (common) ########################################
print_header "Configuring pigpiod to start on boot"
systemctl enable pigpiod

print_header "Reboot now? (y lowercase to confirm)"
read x && [[ "$x" == "y" ]] && /sbin/reboot;


