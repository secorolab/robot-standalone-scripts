# Test scripts

All scripts have communication with the robots using platform specific APIs without any wrapper libraries.

## Dependencies

### Robot APIs
- [kortex-api 2.6.0](https://artifactory.kinovaapps.com/ui/native/generic-local-public/kortex/API/2.6.0/) for Kinova Gen3

> [!NOTE]
> kortex API should be upgraded to [2.7.0](https://artifactory.kinovaapps.com/ui/repos/tree/General/generic-local-public/kortex/API/2.7.0)

### Kinematics and Dynamics Libraries
- [orocos_kdl](https://github.com/secorolab/orocos_kinematics_dynamics)
- [pinocchio](https://github.com/stack-of-tasks/pinocchio)

### Utilities
- [kdl_parser](https://github.com/secorolab/kdl_parser)

## Installation instructions
- Clone the repository, then create the following folder structure inside the root directory:
  ```
  ├─bin
  └─install
    ├── include
    └── lib
  ```
- Download and extract the Kortex API, then:
  - Copy all files from the `common` folder into:
`install/include/kortex_api/`
  - Copy all `*.a` files from the release folder into:
`install/lib/`
- Install required system libraries using apt:
  ```
  sudo apt-get install libtinyxml-dev liburdfdom-dev liburdfdom-headers-dev
  ```
- Clone remaining dependencies. For each dependency:
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
    sudo make install
    ```
- Run the relevant command from the `Makefile`. Example to create executable for `kdl_gc` can be found below
  ```
  make kdl_gc
  ```
- The executable is generated in `bin/` folder
