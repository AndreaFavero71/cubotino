#!/usr/bin/env bash

# Since 25/10/2026 file adapted for Raspberry Pi OS 13 (further than Raspberry Pi OS 10, 11 and 12)

# ##################################### initial part (common) #####################################

set -e

print_header () {
  echo
  echo $1
  echo
}


venv_is_usable () {
  # returns 0 if the venv exists and its interpreter runs, 1 otherwise
  local v="$1"
  [ -d "$v" ] || return 1
  [ -x "$v/bin/python3" ] || return 1
  "$v/bin/python3" -c "import sys; sys.exit(0)" >/dev/null 2>&1 || return 1
  return 0
}


create_venv () {
  # create (or reuse) a venv at $1, removing a broken one first
  local v="$1"
  if [ -d "$v" ] && ! venv_is_usable "$v"; then
    echo "Removing broken virtualenv at $v"
    rm -rf "$v"
  fi
  python3 -m venv "$v" --system-site-packages
  # shellcheck disable=SC1091
  source "$v/bin/activate"
}


if [ "$EUID" -ne 0 ]; then
  echo "Please run as root (sudo $0)"
  exit
fi


# absolute paths, so relative paths work regardless of where the script is
# invoked from (sudo can change the working directory)
SETUP_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
SRC_DIR="$( dirname "$SETUP_DIR" )"
cd "$SRC_DIR"


os_version=$(lsb_release -r | tr -cd '[[:digit:]]')
echo -e "\n\n#####################################################"
echo "Setup proceeds, based on Raspberry Pi os_version: $os_version"
echo -e "#####################################################\n\n"


print_header "Remote access method"
echo "  1) RealVNC on X11"
echo "     - Full remote desktop, works on Pi Zero 2W"
echo "     - Required for camera tuning and graphical feedback"
echo "  2) Raspberry Pi Connect on Wayland"
echo "     - Screen sharing needs Pi 4/5 and a 64-bit OS"
echo "     - Not suitable for Cubotino GUI setting if Pi Zero or Zero2"
echo "  3) none (local desktop only, no remote service configured)"
read -p "Choose 1, 2 or 3 [1]: " remote_choice
remote_choice="${remote_choice:-1}"
case "$remote_choice" in
  1|2|3) ;;
  *) echo "Invalid choice, defaulting to 2 (Raspberry Pi Connect)"; remote_choice=2 ;;
esac
echo "Selected remote access: $remote_choice"


echo -e "\nChecking SWAP SIZE"
SWAPSPACE=$(sed -n "s/^SwapTotal: *\([0-9]*\) kB/\1/p" /proc/meminfo)
echo "Current SWAP SIZE is $SWAPSPACE Kb"

if [[ "$os_version" =~ ^[0-9]+$ && "$os_version" -ge 12 ]]; then
  # ---------- Trixie+ : rpi-swap ----------
  if [[ "$SWAPSPACE" -le 524000 ]]; then
    echo "Configuring SWAP SIZE to 512Mb via rpi-swap drop-in"
    mkdir -p /etc/rpi/swap.conf.d
    cat > /etc/rpi/swap.conf.d/90-cubotino-swap.conf <<'EOF'
[Main]
Mechanism=swapfile

[File]
FixedSizeMiB=512
EOF
    echo "Swap drop-in written; it will take effect after reboot."
  else
    echo "SWAP SIZE is sufficient"
  fi
else
  # ---------- Buster / Bullseye : dphys-swapfile ----------
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
fi



if [[ "$os_version" =~ ^[0-9]+$ && "$os_version" -ge 12 ]]; then
  CONFIG=/boot/firmware/config.txt
else
  CONFIG=/boot/config.txt
fi
sed -i '/dtparam=spi/d' "$CONFIG"
sed -i '/startx/d' "$CONFIG"
sed -i '/gpu_mem/d' "$CONFIG"
sed -i '/enable_uart/d' "$CONFIG"
cat <<EOT >> "$CONFIG"
dtparam=spi=on
start_x=1
gpu_mem=128
enable_uart=1
EOT


print_header "Updating packages"
apt update
apt -y -qq upgrade






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
    create_venv "$SRC_DIR/.virtualenvs"

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

    print_header "Configuring remote access"
    if [ "$remote_choice" = "1" ]; then
        if ! crontab -l 2>/dev/null | grep -q "Cubotino_T_bash.sh"; then
            (crontab -l 2>/dev/null;\
            echo -e 'MAILTO=""';\
            echo -e '@reboot su - pi -c "/usr/bin/vncserver :0 -geometry 1280x720\"';\
            echo -e '#@reboot su - pi -c "/usr/bin/vncserver :0 -geometry 1920x1080"';\
            echo -e '#@reboot /bin/sleep 5; bash -l /home/pi/cubotino/src/Cubotino_T_bash.sh > /home/pi/cubotino/src/Cubotino_T_terminal.log 2>&1')\
            | crontab -
        fi
    elif [ "$remote_choice" = "2" ]; then
        echo "Raspberry Pi Connect is not supported on OS 10; skipping remote setup"
    else
        echo "No remote access selected"
    fi



# ##################################### RASPBIAN 11 specific #######################################
# (20231012 image) https://downloads.raspberrypi.org/raspios_oldstable_armhf/images/raspios_oldstable_armhf-2023-10-10/2023-05-03-raspios-bullseye-armhf.img.xz

elif [[ "$os_version" == "11" ]]; then

    print_header "Installing required packages"
    apt-get -y install python3-opencv

    print_header "Creating python virtual env"
    create_venv "$SRC_DIR/.virtualenvs"

    print_header "Installing required python packages"
    pip3 install --upgrade pip
    pip3 install st7735==0.0.4.post1
    pip3 install st7789==0.0.4
    pip3 install RubikTwoPhase==1.1.1
    pip3 install getmac==0.8.3

    set +e

    print_header "Configuring remote access"
    if [ "$remote_choice" = "1" ]; then
        if ! crontab -l 2>/dev/null | grep -q "Cubotino_T_bash.sh"; then
            (crontab -l 2>/dev/null;\
            echo -e 'MAILTO=""';\
            echo -e '@reboot su - pi -c "/usr/bin/vncserver-virtual :1 -randr=1280x720"';\
            echo -e '#@reboot su - pi -c "/usr/bin/vncserver-virtual :1 -randr=1920x1080"';\
            echo -e '#@reboot /bin/sleep 5; bash -l /home/pi/cubotino/src/Cubotino_T_bash.sh > /home/pi/cubotino/src/Cubotino_T_terminal.log 2>&1')\
            | crontab -
        fi
    elif [ "$remote_choice" = "2" ]; then
        echo "Raspberry Pi Connect on OS 11 is not validated for Cubotino; skipping"
    else
        echo "No remote access selected"
    fi



# ##################################### RASPBIAN >=12 specific #####################################
# (Bookworm and later) https://www.raspberrypi.com/software/operating-systems/

elif [[ "$os_version" =~ ^[0-9]+$ && "$os_version" -ge 12 ]]; then

    print_header "Installing required packages"
    apt-get -y install python3-opencv
    apt-get -y install build-essential python3-setuptools wget

    print_header "Building pigpiod from source (not packaged on Trixie)"
    # pigpiod is not packaged on Trixie; the daemon binary and its systemd
    # unit come only from this source build. do not remove this block:
    # without it the robot installs but fails at servo init.
    cd /tmp
    wget -q https://github.com/joan2937/pigpio/archive/refs/tags/v79.tar.gz
    tar zxf v79.tar.gz
    cd pigpio-79
    make
    make install
    cd "$SRC_DIR"

    # pigpio v79 'make install' installs the binary but not the systemd unit
    # on trixie. create it explicitly if missing, then enable.
    if [ ! -f /lib/systemd/system/pigpiod.service ]; then
        cat > /lib/systemd/system/pigpiod.service <<'EOF'
[Unit]
Description=Daemon required to control GPIO pins via pigpio
After=network.target

[Service]
ExecStart=/usr/local/bin/pigpiod -l -t1
ExecStop=/bin/systemctl kill pigpiod
Type=forking
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF
        systemctl daemon-reload
    fi
        systemctl enable pigpiod

    print_header "Creating python virtual env"
    create_venv "$SRC_DIR/.virtualenvs"

    print_header "Installing required python packages"
    pip3 install --upgrade pip
    pip3 install st7735==0.0.4.post1
    pip3 install st7789==0.0.4
    pip3 install RubikTwoPhase==1.1.1
    pip3 install getmac==0.8.3
    pip3 install pigpio

    set +e

    print_header "Configuring display server for the chosen remote access"
    if command -v raspi-config >/dev/null 2>&1; then
        if [ "$remote_choice" = "1" ]; then
            # RealVNC requires X11
            raspi-config nonint do_wayland W1 || true
        elif [ "$remote_choice" = "2" ]; then
            # Connect screen sharing requires Wayland
            raspi-config nonint do_wayland W2 || true
        fi
        # choice 3: leave the display server as the image ships it
    fi

    # Pi Zero (armv6l) + Bookworm: hold RealVNC to a working version
    if [ "$remote_choice" = "1" ] && [ "$(uname -m)" = "armv6l" ]; then
        apt-mark hold realvnc-vnc-server 2>/dev/null || true
    fi

    print_header "Configuring remote access"
    if [ "$remote_choice" = "1" ]; then
        if ! crontab -l 2>/dev/null | grep -q "Cubotino_T_bash.sh"; then
            (crontab -l 2>/dev/null;\
            echo -e 'MAILTO=""';\
            echo -e '@reboot su - pi -c "/usr/bin/vncserver-virtual :1 -randr=1280x720"';\
            echo -e '#@reboot su - pi -c "/usr/bin/vncserver-virtual :1 -randr=1920x1080"';\
            echo -e '#@reboot /bin/sleep 5; bash -l /home/pi/cubotino/src/Cubotino_T_bash.sh > /home/pi/cubotino/src/Cubotino_T_terminal.log 2>&1')\
            | crontab -
        fi
    elif [ "$remote_choice" = "2" ]; then
        echo "Raspberry Pi Connect: sign in after reboot with  rpi-connect signin"
        echo "Screen sharing requires the Wayland session that has just been selected"
        # robot auto-start is still wanted, add only that cron line
        if ! crontab -l 2>/dev/null | grep -q "Cubotino_T_bash.sh"; then
            (crontab -l 2>/dev/null;\
            echo -e 'MAILTO=""';\
            echo -e '#@reboot /bin/sleep 5; bash -l /home/pi/cubotino/src/Cubotino_T_bash.sh > /home/pi/cubotino/src/Cubotino_T_terminal.log 2>&1')\
            | crontab -
        fi
    else
        echo "No remote access selected"
    fi


# ##################################### unhandled / unparseable os_version ########################
else
    print_header "the setup.sh file does not handle $os_version"
fi



# ##################################### last part (common) ########################################
print_header "Configuring pigpiod to start on boot"
if systemctl list-unit-files | grep -q '^pigpiod\.service'; then
    systemctl enable pigpiod || echo "WARNING: could not enable pigpiod"
else
    echo "Warning: pigpiod.service not found; skipping enable."
fi

print_header "Remote access summary"
case "$remote_choice" in
  1) echo "RealVNC on X11 was configured."
     echo "Connect to  <pi-ip>:5901  with a VNC viewer after reboot." ;;
  2) if [[ "$os_version" =~ ^[0-9]+$ && "$os_version" -ge 12 ]]; then
         echo "Raspberry Pi Connect on Wayland was configured."
         echo "After reboot, run  rpi-connect signin  and follow the browser instructions."
     else
         echo "Raspberry Pi Connect was requested but is not supported on OS $os_version."
         echo "No remote access was configured; use the local desktop or re-run and pick 1."
     fi ;;
  3) echo "No remote access was configured; use the local desktop." ;;
esac

print_header "Reboot now? (y lowercase to confirm)"
read x && [[ "$x" == "y" ]] && /sbin/reboot;