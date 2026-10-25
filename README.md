# CUBOTino autonomous

This repo contains the relevant files for ....

How to make CUBOTino autonomous: A small, 3D printed, Rubik's cube solver robot
![title image](/images/title.jpg)

Further robot info at: https://www.instructables.com/CUBOTino-Autonomous-Small-3D-Printed-Rubiks-Cube-R/

An impression of the robot: https://youtu.be/dEOLhvVMcUg and https://youtu.be/YV-g_3STsBo

This is forked from the git repo made by heneault (thank you !!!), and updated to latest files versions.

This git repo simplifies the download of the individual files and automates as much a possible the deployment to the Raspberry Pi

This installation has proved to work on different Raspberry Pi models; Please report in case you experience issues.

The installation covers up to OS 13 (Trixie).




# How to use it:
1. Flash your SD card according to the procedure in the [document here](doc/How_to_make_CUBOTino_autonomous_robot_20261025.pdf) , Chapter 7, Step1 to Step3
2. Put the sd card in the pi and power it. You can monitor the boot process if you connect an hdmi monitor to it but it is not essential. 
3. Try to connect to the Raspberry Pi via SSH. On Windows you can use Putty. On linux and mac you can type directly:
```
ssh pi@cubotino.local
```
4. If you can't reach the pi like this, you will have to scan your network to find the IP to use
5. After you are connected via ssh, type the following commands in the shell:
```
git clone https://github.com/AndreaFavero71/cubotino.git
cd cubotino/src
chmod +x install/setup.sh
sudo ./install/setup.sh
```
6. Follow the prompts from the setup.sh script. It will ask you to choose a remote access method.
   -For a Pi Zero 2 W, choose option 1 (RealVNC on X11). This is essential for the graphical tuning steps.
   -Option 2 (Raspberry Pi Connect) is suitable when using Raspberry Pi 4 or 5, as its screen-sharing feature is required for tuning and is not available on Pi Zero/Zero 2 W. 
7. Make sure the script runs without error until the end. It should ask you to reboot. Type 'y' and hit enter.
8. If the question "Reboot (y/n)" doesn't appear, then the installation is facing issues; Try to fix the errors, and run the script again.
9. If the question "Reboot (y/n) does appear, after the reboot you should get the proper environment.


# Build the robot:
Follow the instructions (saved at /doc folder)


# Tuning the servos position via GUI (via VNC):
```
cd ~/cubotino/src
source .virtualenvs/bin/activate
python Cubotino_T_servos_GUI.py
```
![title image](/images/Servo_tuning_GUI_01.PNG)


# Executing manually
From a shell, you can run the main python script like this:
```
cd ~/cubotino/src
source .virtualenvs/bin/activate
python Cubotino_T.py
```
of course, you can replace `Cubotino_T.py` by any other python scripts as mentioned in the documentation.


# Enabling autostart
When everything is tuned and you want to start the software automatically on reboot, edit the crontab for the root user:
```
    sudo crontab -e
```
select the nano editor '1' then go to the last line of the window and remove the leading '#' character of the line. This uncomment the startup script that launches cubotino on startup. You can reboot after to test it.

# VNC connection
You can always connect with ssh. If you prefere VNC you can download the RealVNC client (this is the one I use). You just have that start it like this:
```
vncviewer cubotino.local:1
```
It will ask for the credential. Use 'pi' and the same password you use for ssh. You should have a desktop version in this way

Check out the "How_to_make ...  .pdf" document (at /doc folder) for further info.<br /><br />


# Please leave a feedback if you build it
I hope many of you will decide to build your own CUBOTino, and that you'll enjoy it as much much as I did. <br />
I hope you will also post an "I Made it", on the Instructables site of this project (link above); <br />
I can ensure you, seeing a new born CUBOTino makes me feel very well 🙂

