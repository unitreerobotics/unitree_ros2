<a name="unreleased"></a>
## [Unreleased]

### Features
- **deprecate:** delete deprecated functions that are no longer supported

- **go2:** add some new supported go2 sports APIs



Now the APIs in the following list are supported:

| name                                  | value | example                                                      |
| :------------------------------------ | :---- | :----------------------------------------------------------- |
| ROBOT_SPORT_API_ID_HEART            | 1036  | void SportClient::Heart();   |
| ROBOT_SPORT_API_ID_STATICWALK       | 1061  | void SportClient::StaticWalk(); |
| ROBOT_SPORT_API_ID_TROTRUN          | 1062  | void SportClient::TrotRun(); |
| ROBOT_SPORT_API_ID_ECONOMICGAIT     | 1063  | void SportClient::EconomicGait(); |
| ROBOT_SPORT_API_ID_LEFTFLIP         | 2041  | void SportClient::LeftFlip(); |
| ROBOT_SPORT_API_ID_BACKFLIP         | 2043  | void SportClient::BackFlip(); |
| ROBOT_SPORT_API_ID_HANDSTAND        | 2044  | void SportClient::HandStand(); |
| ROBOT_SPORT_API_ID_FREEWALK         | 2045  | void SportClient::FreeWalk(); |
| ROBOT_SPORT_API_ID_FREEBOUND        | 2046  | void SportClient::FreeBound(); |
| ROBOT_SPORT_API_ID_FREEJUMP         | 2047  | void SportClient::FreeJump(); |
| ROBOT_SPORT_API_ID_FREEAVOID        | 2048  | void SportClient::FreeAvoid(); |
| ROBOT_SPORT_API_ID_CLASSICWALK      | 2049  | void SportClient::ClassicWalk(); |
| ROBOT_SPORT_API_ID_WALKUPRIGHT      | 2050  | void SportClient::WalkUpright(); |
| ROBOT_SPORT_API_ID_CROSSSTEP        | 2051  | void SportClient::CrossStep(); |
| ROBOT_SPORT_API_ID_AUTORECOVERY_SET | 2054  | void SportClient::AutoRecoverySet(); |
| ROBOT_SPORT_API_ID_AUTORECOVERY_GET | 2055  | void SportClient::AutoRecoveryGet(); |
| ROBOT_SPORT_API_ID_SWITCHAVOIDMODE  | 2058  | void SportClient::SwitchAvoidMode(); |

At the same time, you can also refer to the API implementation in `example/src/src/common/ros2.sport_cient.cpp` to communicate with Go2 using ROS2 on your own. An example is in the file `example/src/src/go2/go2_Sport_cient.cpp`.

### BREAKING CHANGE

- The functions or API IDs in the following list are no longer supported. If you need to use a new version, please remove the following references:

| API ID | Macro Definition                      | Function Name        |
| :----- | :------------------------------------ | :------------------- |
| 1011   | ROBOT_SPORT_API_ID_SWITCHGAIT         | SwitchGait()         |
| 1012   | ROBOT_SPORT_API_ID_TRIGGER            | Trigger()            |
| 1013   | ROBOT_SPORT_API_ID_BODYHEIGHT         | BodyHeight()         |
| 1014   | ROBOT_SPORT_API_ID_FOOTRAISEHEIGHT    | FootRaiseHeight()    |
| 1018   | ROBOT_SPORT_API_ID_TRAJECTORYFOLLOW   | TrajectoryFollow()   |
| 1019   | ROBOT_SPORT_API_ID_CONTINUOUSGAIT     | ContinuousGait()     |
| 1021   | ROBOT_SPORT_API_ID_WALLOW             | Wallow()             |
| 1024   | ROBOT_SPORT_API_ID_GETBODYHEIGHT      | (No direct function) |
| 1025   | ROBOT_SPORT_API_ID_GETFOOTRAISEHEIGHT | (No direct function) |
| 1026   | ROBOT_SPORT_API_ID_GETSPEEDLEVEL      | (No direct function) |


<a name="v0.1.0"></a>
## v0.1.0 - 2025-07-23

[Unreleased]: https://github.com/unitreerobotics/unitree_ros2/compare/v0.1.0...HEAD
