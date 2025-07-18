# Test scripts

All scripts have communication with the robots using platform specific APIs without any wrapper libraries.

## Dependencies

### Robot APIs
- [kortex-api 2.6.0](https://artifactory.kinovaapps.com/ui/native/generic-local-public/kortex/API/2.6.0/) for Kinova Gen3

> [!NOTE]
> kortex API should be upgraded to [2.7.0](https://artifactory.kinovaapps.com/ui/repos/tree/General/generic-local-public/kortex/API/2.7.0)

### Kinematics and Dynamics Libraries
- [orocos_kdl](https://github.com/secorolab/orocos_kinematics_dynamics)

### Utilities
- [kdl_parser](https://github.com/secorolab/kdl_parser)

## Setup

- Clone the repository,
- A sample of root directory:
  ```
  ├── bin/
  ├── gen3.urdf
  ├── install/
  ├── kdl_parser/
  ├── Makefile
  ├── orocos_kinematics_dynamics/
  ├── README.md
  └── src/
  ```

- Install required system libraries using apt:
  ```
  sudo apt-get install libtinyxml-dev liburdfdom-dev liburdfdom-headers-dev
  ```

- Setup `kdl_parser`
  ```bash
  make kdl_parser_setup
  ```

- Setup `kortex_api`
  ```bash
  make kortex_setup
  ```

- For each dependency (in general):
  - Navigate to the directory where `CMakeLists.txt` is located
  - Create a build directory at the same level:
    ```
    mkdir build && cd build
    ```
  - Run CMake, specifying the path to the install directory from step
    ```
    cmake .. -DCMAKE_INSTALL_PREFIX=<absolute_path_to_install_directory>
    ```
  - Compile and install the package:
    ```
    make
    make install
    ```

## Build

- The executable is generated in `bin/` folder

### kdl_gc
  
  Computes gravity compensation torques for `Kinova Gen3` using RNEA from KDL
  
  ```bash
  make kdl_gc
  ```
### kinova_read
  
  Reads data from Kinova Gen3 and computes gravity comp. torques using RNEA from KDL

  ```bash
  make kinova_read
  ```

### kinova_kdl_gc
 
  Computes gravity comp. torques using RNEA from KDL and commands them to the robot
  
  ```bash
  make kinova_kdl_gc
  ```

### kinova_kdl_gc_ctrl
  
  - Computes gravity comp. torques using RNEA from KDL, and apply P-control to control the drift.
  - Also, commands them to the robot
  
  ```bash
  make kinova_kdl_gc_ctrl
  ```

