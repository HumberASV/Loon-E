#!/bin/bash
# This reinstalls the driver. should have confirmation.
 sudo apt install ./zed.deb -y && sudo systemctl start driver_zed_loader.service && sudo systemctl start zed_x_daemon.service && sudo systemctl start IMU_Daemon.service && sudo systemctl restart nvargus-daemon.service && cd ~/Loon-E && source install/setup.sh && ros2 launch loone launch.py