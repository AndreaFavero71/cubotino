#!/usr/bin/env bash

#######   Andrea Favero,  17 April 2023  #####################################################
#  This bash script activates the venv, and starts the Cubotino_T.py script after the Pi boots
#  When the python script is terminated without errors (long button press), the Pi shuts down
#  (check notes below before uncommenting the "halt -p"command).
##############################################################################################

# activate the venv
source /home/pi/cubotino/src/.virtualenvs/bin/activate

# enter the folder with the main scripts
cd /home/pi/cubotino/src

# runs the robot main script
python Cubotino_T.py

# exit code from the python script
exit_status=$?

# based on the exit code there are three cases to be handled
if [ "${exit_status}" -ne 0 ];
then
    if [ "${exit_status}" -eq 2 ];
    then
        echo ""
        echo "Cubotino_T.py exited on request"
        echo ""
    else
        echo ""
        echo "Cubotino_T.py exited with error"
        echo ""
    fi
	
else
    echo ""
    echo "Successfully executed Cubotino_T.py"
    echo ""

    # ‘halt -p’ command shuts down the Raspberry pi
    # un-comment 'halt -p' command ONLY when the script works without errors
    # un-comment 'halt -p' command ONLY after making an image of the microSD
    #halt -p

fi
