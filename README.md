# Loon-E
Loon-E boat's public repo

# tmp colcon inst
### Prerequisites

- Install ROS 2 following [installation](https://docs.ros.org/en/humble/Installation.html) page.
- Set up your environment following [instructions](./wiki/setup/ENV.md) document.
- Install colcon following [instructions](./wiki/setup/COLCON.md) document.

## Development

### Building Workspace

Ensure you have set up your environment following [instructions](./wiki/setup/ENV.md) document, and have installed `colcon` following [this instructions](./wiki/setup/COLCON.md).

For building the workspace with Windows see the below note: Building with Windows.

Regardless of Linux or macOs distrubution, navigate to the project root `PROJECT-NAME/` and run `colcon build`. It may be needed to use the option `--symlink-install` as some build types do not support `devel` spaces.

For more information on `catkin`'s `devel` space read [this documentation](https://catkin-tools.readthedocs.io/en/latest/advanced/linked_develspace.html).

```bash
colcon build --symlink-install
```

You can test it with the following command:

```bash
colon test --symlink-install
```
### Project Structure

```text
Ros2-Examples/
├── build/                       # Colcon intermediate files
├── install/                     # Colcon package installation
├── log/                         # Colcon Logging information
├── src/                         # Source code
├── wiki/                        # Additional documentation
│   ├── setup/                   # Setup documentation
│   └── CONTRIBUTING.md/         # Instructions on contributing to this repository
├── LICENSE/                     # GPL-3.0 license
└── README.md                    # Primary documentation
```

## Contributing

See the [contribution guide](wiki/CONTRIBUTING) to see how you can contribute!
