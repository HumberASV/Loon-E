# Source Setup

First, you need to source the setup files; this depends on your operating system.

### Linux

```bash
source /opt/ros/humble/setup.bash
```

Notes that you can replace `.bash` with any shell and use the corresponding file extention.

If you don’t want to have to source the setup file every time you open a new shell (skipping task 1), then you can add the command to your shell startup script:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

To undo this, locate your system’s shell startup script and remove the appended source command.

### macOs

```zsh
. ~/ros2_install/ros2-osx/setup.bash
```

If you don’t want to have to source the setup file every time you open a new shell (skipping task 1), then you can add the command to your shell startup script:

```zsh
echo "source ~/ros2_install/ros2-osx/setup.bash" >> ~/.bash_profile
```

To undo this, locate your system’s shell startup script and remove the appended source command.

### Windows

```powershell
call C:\dev\ros2\local_setup.bat
```

If you don’t want to have to source the setup file every time you open a new shell (skipping task 1), then create a folder in ‘My Documents’ called ‘WindowsPowerShell’. Within ‘WindowsPowerShell’, create file ‘Microsoft.PowerShell_profile.ps1’. Inside the file, paste

```powershell
C:\dev\ros2_humble\local_setup.ps1
```

PowerShell will request permission to run this script every time a new shell is opened. To avoid that issue you can run:

```powershell
Unblock-File C:\dev\ros2_humble\local_setup.ps1
```

To undo this, remove the new ‘Microsoft.PowerShell_profile.ps1’ file.

### Warning

The exact command depends on where you installed ROS 2. If you’re having problems, ensure the file path leads to your installation.

## Check Enviroment variables

Sourcing ROS 2 setup files will set several environment variables necessary for operating ROS 2. If you ever have problems finding or using your ROS 2 packages, make sure that your environment is properly set up using the following command:

### Linux

```bash
printenv | grep -i ROS
```

### macOS

```zsh
printenv | grep -i ROS
```

### Windows

```powershell
set | findstr -i ROS
```

Check that variables like `ROS_DISTRO` and `ROS_VERSION` are set.

```bash
ROS_VERSION=2
ROS_PYTHON_VERSION=3
ROS_DISTRO=humble
```

If the environment variables are not set correctly, return to the ROS 2 package installation section of the installation guide you followed. If you need more specific help (because environment setup files can come from different places), you can get answers from the community.

## The `ROS_DOMAIN` Variable

See the [domain ID](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Domain-ID.html) article for details on ROS domain IDs.

Once you have determined a unique integer for your group of ROS 2 nodes, you can set the environment variable with the following command:

### Linux

```bash
export ROS_DOMAIN_ID=<your_domain_id>
```

To maintain this setting between shell sessions, you can add the command to your shell startup script:

```bash
echo "export ROS_DOMAIN_ID=<your_domain_id>" >> ~/.bashrc
```

### macOS

```zsh
export ROS_DOMAIN_ID=<your_domain_id>
```

To maintain this setting between shell sessions, you can add the command to your shell startup script:

```zsh
echo "export ROS_DOMAIN_ID=<your_domain_id>" >> ~/.bash_profile
```

### Windows

```powershell
set ROS_DOMAIN_ID=<your_domain_id>
```

If you want to make this permanent between shell sessions, also run:

```powershell
setx ROS_DOMAIN_ID <your_domain_id>
```

## The `ROS_LOCALHOST_ONLY` variable

by default, ROS 2 communication is not limited to localhost. `ROS_LOCALHOST_ONLY` environment variable allows you to limit ROS 2 communication to localhost only. This means your ROS 2 system, and its topics, services, and actions will not be visible to other computers on the local network. Using `ROS_LOCALHOST_ONLY` is helpful in certain settings, such as classrooms, where multiple robots may publish to the same topic causing strange behaviors. You can set the environment variable with the following command:

### Linux

```bash
export ROS_LOCALHOST_ONLY=1
```

To maintain this setting between shell sessions, you can add the command to your shell startup script:

```bash
echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc
```

### macOS

```zsh
export ROS_LOCALHOST_ONLY=1
```

To maintain this setting between shell sessions, you can add the command to your shell startup script:

```zsh
echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bash_profile

```

### Windows

```powershell
set ROS_LOCALHOST_ONLY=1
```

If you want to make this permanent between shell sessions, also run:

```powershell
setx ROS_LOCALHOST_ONLY 1
```
