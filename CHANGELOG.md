<a name="unreleased"></a>
## [Unreleased]


<a name="v0.3.0"></a>
## [v0.3.0] - 2025-08-15
### Bug Fixes
- **ros2:** fix the bug in `Ros2 Foxy` where `topic_datistics_collector` did not determine the `header` type and directly used `headers.stamp`, the `Humble` feature was marked in `v0.2.0` and can now be used in `Foxy` as well


### Examples
- **example:** add `g1_audio_client_example`

- **example:** add `g1_dual_arm_example`

- **example:** add `g1_ankle_swing_example`

- **example:** add `g1_loco_client_example`

- **example:** add `g1_arm_action_example`

- **example:** add `g1_arm_sdk_dds_example`


### Features
- **g1:** add dex3 example, modify the dds msg to align with sdk2


### BREAKING CHANGE

- `HandCmd.msg` `HandState.msg` `PressSensorState.msg` in `cyclonedds_ws/src/unitree/unitree_hg/msg/` has been modified to align to sdk2,you can view the corresponding file to check the modification items.There is no recommended iteration method, you can update according to your own usage.


<a name="v0.2.0"></a>
## [v0.2.0] - 2025-07-29
### Examples
- **example:** add go2 stand example

- **example:** add go2 sport client example


### Features
- **deprecate:** delete deprecated functions that are no longer supported

- **go2:** add support for RobotStateClient on ROS 2 Humble
Now the APIs in the following list are supported:

| name | value | example | ros2 version|
| :------------------------------------ | :---- | :----------------------------------------------------------- | :------- |
| ROBOT_STATE_API_ID_SERVICE_SWITCH| 1001 | int32_t RobotStateClient::ServiceSwitch(); | humble |
| ROBOT_STATE_API_ID_SET_REPORT_FREQ| 1002 | int32_t RobotStateClient::SetReportFreq(); | humble |
| ROBOT_STATE_API_ID_SERVICE_LIST| 1003 | int32_t RobotStateClient::ServiceList(); | humble |

At the same time, you can also refer to the API implementation in `example/src/src/common/ros2_robot_state_client.cpp` to communicate with Go2 using ROS2 on your own. An example is in the file` example/src/src/go2/g2_robot_state_client.cpp`.
- **go2:** add some new supported go2 sports APIs
Now the APIs in the following list are supported:

| name                                  | value | example                                                      | ros2 version |
| :------------------------------------ | :---- | :----------------------------------------------------------- | :---- |
| ROBOT_SPORT_API_ID_HEART            | 1036  | void SportClient::Heart();   | foxy humble |
| ROBOT_SPORT_API_ID_STATICWALK       | 1061  | void SportClient::StaticWalk(); | foxy humble |
| ROBOT_SPORT_API_ID_TROTRUN          | 1062  | void SportClient::TrotRun(); | foxy humble |
| ROBOT_SPORT_API_ID_ECONOMICGAIT     | 1063  | void SportClient::EconomicGait(); | foxy humble |
| ROBOT_SPORT_API_ID_LEFTFLIP         | 2041  | void SportClient::LeftFlip(); | foxy humble |
| ROBOT_SPORT_API_ID_BACKFLIP         | 2043  | void SportClient::BackFlip(); | foxy humble |
| ROBOT_SPORT_API_ID_HANDSTAND        | 2044  | void SportClient::HandStand(); | foxy humble |
| ROBOT_SPORT_API_ID_FREEWALK         | 2045  | void SportClient::FreeWalk(); | foxy humble |
| ROBOT_SPORT_API_ID_FREEBOUND        | 2046  | void SportClient::FreeBound(); | foxy humble |
| ROBOT_SPORT_API_ID_FREEJUMP         | 2047  | void SportClient::FreeJump(); | foxy humble |
| ROBOT_SPORT_API_ID_FREEAVOID        | 2048  | void SportClient::FreeAvoid(); | foxy humble |
| ROBOT_SPORT_API_ID_CLASSICWALK      | 2049  | void SportClient::ClassicWalk(); | foxy humble |
| ROBOT_SPORT_API_ID_WALKUPRIGHT      | 2050  | void SportClient::WalkUpright(); | foxy humble |
| ROBOT_SPORT_API_ID_CROSSSTEP        | 2051  | void SportClient::CrossStep(); | foxy humble |
| ROBOT_SPORT_API_ID_AUTORECOVERY_SET | 2054  | void SportClient::AutoRecoverySet(); | foxy humble |
| ROBOT_SPORT_API_ID_AUTORECOVERY_GET | 2055  | void SportClient::AutoRecoveryGet(); | humble |
| ROBOT_SPORT_API_ID_SWITCHAVOIDMODE  | 2058  | void SportClient::SwitchAvoidMode(); | foxy humble |

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

[Unreleased]: https://github.com/unitreerobotics/unitree_ros2/compare/v0.3.0...HEAD
[v0.3.0]: https://github.com/unitreerobotics/unitree_ros2/compare/v0.2.0...v0.3.0
[v0.2.0]: https://github.com/unitreerobotics/unitree_ros2/compare/v0.1.0...v0.2.0
