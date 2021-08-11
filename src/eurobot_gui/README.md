# Eurobot GUI

## Install the package

    cd ~/<Your Workspace>/src

    git clone https://github.com/dit-robotics/eurobot_gui.git
    
    cd src
    
    chmod +x eurobot_gui.py

## Run GUI

- If you want to run GUI only:

    ```
    roscore
    rosrun eurobot_gui eurobot_gui.py
    ```

- If you want to start with a launch file, then just add the node 'eurobot_gui'.

## Buttons

- Hit wall: Run hit-wall positioning.

- Start: Ready for unplugging.

## Labels

- Timer: The game time starts to display once the 'Start' button is clicked.

## Notes

- The game time will start when the plug on the robot is pulled away in later versions.

- I need MaHua's code for:
    1. score publisher
    2. plug publisher
    3. command subscriber

- There's only one thing that you need to install: Optimal C font

- Currently, the gui is for 800x480 touch display

## Credit

- Chen, Wei-Chieh. DIT Robotics. March 2020.