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
cat <<EOT >> $CONFIG
dtparam=spi=on
start_x=1
gpu_mem=128
EOT

print_header "Updating packages"
apt update
apt -y -qq upgrade

print_header "Installing required packages"
apt install -y -qq python3-numpy python3-picamera python3-opencv python3-rpi.gpio python3-pigpio python3-gpiozero python3-pil python3-spidev python3-pip python3-venv

print_header "Creating python virtual env"
python3 -m venv .virtualenvs --system-site-packages
source .virtualenvs/bin/activate

print_header "Installing required python packages"
pip3 install st7735==0.0.4.post1
pip3 install RubikTwoPhase==1.0.9
pip3 install getmac==0.8.3

print_header "Configuring pigpiod to start on boot"
systemctl enable pigpiod

print_header "Configuring vnc server to run at startup and prepare setup for cubotino"
if ! crontab -l 2>/dev/null | grep -q "Cubotino_T.sh"; then
    (crontab -l 2>/dev/null; echo 'MAILTO=""'; echo '@reboot su - pi -c "/usr/bin/vncserver :0"'; echo '#@reboot /bin/sleep 5; bash -l /home/pi/cubotino/src/Cubotino_T_bash.sh > /home/pi/cubotino/src/Cubotino_T_terminal.log 2>&1') | crontab -
fi

print_header "Reboot now? (y/n)"
read x && [[ "$x" == "y" ]] && /sbin/reboot; 
