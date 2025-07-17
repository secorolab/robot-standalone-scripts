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
