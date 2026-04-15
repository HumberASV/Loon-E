
# Environment Variables Setup

For all operating systems, you will need to set up some environment variables to ensure ROS 2 operates correctly. This includes sourcing the ROS 2 setup files and setting any necessary ROS 2 environment variables.

First, you need to source the setup files; this depends on your operating system. Then, you need to set your environment variables.

## Environment Variables

### Domain ID

See the [domain ID](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Domain-ID.html) article for details on ROS domain IDs.

### Localhost Only

by default, ROS 2 communication is not limited to localhost. `ROS_LOCALHOST_ONLY` environment variable allows you to limit ROS 2 communication to localhost only. This means your ROS 2 system, and its topics, services, and actions will not be visible to other computers on the local network. Using `ROS_LOCALHOST_ONLY` is helpful in certain settings, such as classrooms, where multiple robots may publish to the same topic causing strange behaviors.

### RMW Implementation

ROS 2 supports multiple RMW implementations, which are the underlying middleware that handles communication. The default RMW implementation is `rmw_fastrtps_cpp`, but you can choose to use a different one if needed. Setting the `RMW_IMPLEMENTATION` environment variable allows you to specify which RMW implementation ROS 2 should use. For example, to set it to `rmw_fastrtps_cpp`

### Example

```bash
ROS_DOMAIN_ID=0 # This is also the default
ROS_LOCALHOST_ONLY=1 # This is also the default
RMW_IMPLEMENTATION=rmw_fastrtps_cpp # This is also the default
```

> [!WARNING]
> The exact commands depend on your operating system and shell. If you’re having problems, ensure the file path leads to your installation and that you’re using the correct syntax for your shell.

> [!IMPORTANT]
> These variables must match if run in a container. If you set `ROS_LOCALHOST_ONLY=1` in your host environment, you must also set it in your container environment. If you set `ROS_DOMAIN_ID=0` in your host environment, you must also set it in your container environment. There is additional information on container use in the [Docker documentation](./DOCKER.md).

---

### Linux

Begin by sourcing the ROS 2 setup file and setting the necessary environment variables:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

> [!NOTE]
>To set environment variables persistently, add them to your shell startup script:
>
>```bash
> echo "export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc
>```
>
> > you can replace `.bash` with any shell and use the corresponding file extention.
> To undo this, locate your system’s shell startup script and remove the appended source command.

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

> [!WARNING]
> The exact command depends on where you installed ROS 2. If you’re having problems, ensure the file path leads to your installation.

## Check Enviroment variables

Sourcing ROS 2 setup files will set several environment variables necessary for operating ROS 2. If you ever have problems finding or using your ROS 2 packages, make sure that your environment is properly set up using the following command:

> MacOS and Linux
>
> ```bash
> printenv | grep -i ROS
> ```
>
> Windows
>
>```powershell
> set | findstr -i ROS
> ```

Check that variables like `ROS_DISTRO` and `ROS_VERSION` are set.

```bash
ROS_VERSION=2
ROS_PYTHON_VERSION=3
ROS_DISTRO=humble
```

If the environment variables are not set correctly, return to the ROS 2 package installation section of the installation guide you followed. If you need more specific help (because environment setup files can come from different places), you can get answers from the community.

### Troubleshooting

Check [DDS](https://docs.ros.org/en/humble/How-To-Guides/DDS-tuning.html) settings if you are having trouble with ROS 2 communication. You may need to set additional environment variables depending on your DDS implementation and network configuration.
