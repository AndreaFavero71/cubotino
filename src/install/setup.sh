#!/usr/bin/env bash

set -e

print_header () {
  echo
  echo $1
  echo
}

if [ "$EUID" -ne 0 ]
  then echo "Please run as root (sudo $0)"
  exit
fi

print_header "Deactivating graphical login"
systemctl --quiet set-default multi-user.target

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

print_header "Removing old packages"
apt remove -y -qq python3-numpy python3-picamera
apt autoremove -y

print_header "Installing required packages"
apt install -y -qq python3-rpi.gpio python3-pigpio python3-gpiozero python3-pil python3-pip python3-venv
apt install -y -qq libatlas-base-dev python3-h5py libjasper-runtime libqtgui4 libqt4-test

print_header "Creating python virtual env"
python3 -m venv .virtualenvs --system-site-packages
source .virtualenvs/bin/activate

print_header "Installing required python packages"
pip3 install numpy==1.21.4
pip3 install "picamera[array]"
pip3 install st7735==0.0.4.post1
pip3 install RubikTwoPhase==1.0.9
pip3 install getmac==0.8.3

# hash for opencv for pizero seems to be bad on pywheel, bypass it for the moment. It is ok for pizero 2w
machine=$(uname -m)
if [ "$machine" == "armv6l" ]; then
pip3 install https://www.piwheels.org/simple/opencv-contrib-python/opencv_contrib_python-4.1.0.25-cp37-cp37m-linux_armv6l.whl
else
pip3 install opencv-contrib-python==4.1.0.25
fi

print_header "Configuring pigpiod to start on boot"
systemctl enable pigpiod

set +e

print_header "Configuring vnc server to run at startup and prepare setup for cubotino"
if ! crontab -l 2>/dev/null | grep -q "Cubotino_T_bash.sh"; then
    (crontab -l 2>/dev/null; echo 'MAILTO=""'; echo @reboot su - pi -c \"/usr/bin/vncserver :0 -geometry 1280x720\"; echo '#@reboot /bin/sleep 5; bash -l /home/pi/cubotino/src/Cubotino_T_bash.sh > /home/pi/cubotino/src/Cubotino_T_terminal.log 2>&1') | crontab -
fi

print_header "Reboot now? (y/n)"
read x && [[ "$x" == "y" ]] && /sbin/reboot; 
