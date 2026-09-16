# rt-control 补丁分支说明

本仓库 fork 自 [ros-controls/ros2_controllers](https://github.com/ros-controls/ros2_controllers)，
供 [SevenovaHangzhou/robot_driver](https://github.com/SevenovaHangzhou/robot_driver)
（拆码垛机器人 RT-Control 实时控制域）使用，由其 `deps.repos` 按 SHA 锁定本仓库 `rt-control` 分支。

`rt-control` 分支 = 上游 `humble` 基线 `cbcf6621` + 以下补丁（按序，每补丁一个 commit）：

| # | 补丁 | 改动说明 |
| --- | --- | --- |
| 0001 | jtc-start-consistency | JointTrajectoryController 启动一致性：接受轨迹前校验起点与当前实际状态一致（新增参数控制阈值/行为） |
| 0002 | use-contract-qos-profiles | diff_drive_controller 与 joint_state_broadcaster 的发布器改用 robot_interfaces_qos 契约 QoS 配置 |
| 0003 | opt-in-pp-gripper-commands | gripper_action_controller 增加可选的 PP（Profile Position）命令模式，用于 ZeroErr PP 夹爪 |

**升级方式**：fetch 上游并快进本仓库 `humble` 分支 → 将 `rt-control` rebase 到新基线 →
按 robot_driver 测试体系重新验证 → 更新 robot_driver 的 `deps.repos` 锁定 SHA。

---

# ros2_controllers

[![Licence](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![codecov](https://codecov.io/gh/ros-controls/ros2_controllers/branch/humble/graph/badge.svg?token=KSdY0tsHm6)](https://codecov.io/gh/ros-controls/ros2_controllers/tree/humble)

Commonly used and generalized controllers for ros2-control framework that are ready to use with many robots, MoveIt2 and Nav2.

## Contributing

As an open-source project, we welcome each contributor, regardless of their background and experience. Pick a [PR](https://github.com/ros-controls/ros2_controllers/pulls) and review it, or [create your own](https://github.com/ros-controls/ros2_controllers/contribute)!
If you are new to the project, please read the [contributing guide](https://control.ros.org/rolling/doc/contributing/contributing.html) for more information on how to get started. We are happy to help you with your first contribution.

## Build status

ROS2 Distro | Branch | Build status | Documentation | Released packages
:---------: | :----: | :----------: | :-----------: | :---------------:
**Rolling** | [`master`](https://github.com/ros-controls/ros2_controllers/tree/master) | [![Rolling Binary Build](https://github.com/ros-controls/ros2_controllers/actions/workflows/rolling-binary-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/ros2_controllers/actions/workflows/rolling-binary-build.yml?branch=master) <br> [![Rolling Semi-Binary Build](https://github.com/ros-controls/ros2_controllers/actions/workflows/rolling-semi-binary-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/ros2_controllers/actions/workflows/rolling-semi-binary-build.yml?branch=master) <br> [![build.ros2.org](https://build.ros2.org/buildStatus/icon?job=Rdev__ros2_controllers__ubuntu_resolute_amd64&subject=build.ros2.org)](https://build.ros2.org/job/Rdev__ros2_controllers__ubuntu_resolute_amd64/) | [control.ros.org](https://control.ros.org/rolling/doc/ros2_controllers/doc/controllers_index.html) | [![Build Status](https://build.ros2.org/buildStatus/icon?job=Rbin_uR64__ros2_controllers__ubuntu_resolute_amd64__binary)](https://build.ros2.org/job/Rbin_uR64__ros2_controllers__ubuntu_resolute_amd64__binary/)
**Lyrical** | [`master`](https://github.com/ros-controls/ros2_controllers/tree/master) | see above <br> [![build.ros2.org](https://build.ros2.org/buildStatus/icon?job=Ldev__ros2_controllers__ubuntu_resolute_amd64&subject=build.ros2.org)](https://build.ros2.org/job/Ldev__ros2_controllers__ubuntu_resolute_amd64/) | [control.ros.org](https://control.ros.org/lyrical/doc/ros2_controllers/doc/controllers_index.html) | [![Build Status](https://build.ros2.org/buildStatus/icon?job=Lbin_uR64__ros2_controllers__ubuntu_resolute_amd64__binary)](https://build.ros2.org/job/Lbin_uR64__ros2_controllers__ubuntu_resolute_amd64__binary/)
**Kilted** | [`kilted`](https://github.com/ros-controls/ros2_controllers/tree/kilted) | [![Kilted Binary Build](https://github.com/ros-controls/ros2_controllers/actions/workflows/kilted-binary-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/ros2_controllers/actions/workflows/kilted-binary-build.yml?branch=master) <br> [![Kilted Semi-Binary Build](https://github.com/ros-controls/ros2_controllers/actions/workflows/kilted-semi-binary-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/ros2_controllers/actions/workflows/kilted-semi-binary-build.yml?branch=master) <br> [![build.ros2.org](https://build.ros2.org/buildStatus/icon?job=Kdev__ros2_controllers__ubuntu_noble_amd64&subject=build.ros2.org)](https://build.ros2.org/job/Kdev__ros2_controllers__ubuntu_noble_amd64/) | [control.ros.org](https://control.ros.org/kilted/doc/ros2_controllers/doc/controllers_index.html) | [![Build Status](https://build.ros2.org/buildStatus/icon?job=Kbin_uN64__ros2_controllers__ubuntu_noble_amd64__binary)](https://build.ros2.org/job/Kbin_uN64__ros2_controllers__ubuntu_noble_amd64__binary/)
**Jazzy** | [`jazzy`](https://github.com/ros-controls/ros2_controllers/tree/jazzy) | [![Jazzy Binary Build](https://github.com/ros-controls/ros2_controllers/actions/workflows/jazzy-binary-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/ros2_controllers/actions/workflows/jazzy-binary-build.yml?branch=master) <br> [![Jazzy Semi-Binary Build](https://github.com/ros-controls/ros2_controllers/actions/workflows/jazzy-semi-binary-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/ros2_controllers/actions/workflows/jazzy-semi-binary-build.yml?branch=master) <br> [![build.ros2.org](https://build.ros2.org/buildStatus/icon?job=Jdev__ros2_controllers__ubuntu_noble_amd64&subject=build.ros2.org)](https://build.ros2.org/job/Jdev__ros2_controllers__ubuntu_noble_amd64/) | [control.ros.org](https://control.ros.org/jazzy/doc/ros2_controllers/doc/controllers_index.html) | [![Build Status](https://build.ros2.org/buildStatus/icon?job=Jbin_uN64__ros2_controllers__ubuntu_noble_amd64__binary)](https://build.ros2.org/job/Jbin_uN64__ros2_controllers__ubuntu_noble_amd64__binary/)
**Humble** | [`humble`](https://github.com/ros-controls/ros2_controllers/tree/humble) | [![Humble Binary Build](https://github.com/ros-controls/ros2_controllers/actions/workflows/humble-binary-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/ros2_controllers/actions/workflows/humble-binary-build.yml?branch=master) <br> [![Humble Semi-Binary Build](https://github.com/ros-controls/ros2_controllers/actions/workflows/humble-semi-binary-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/ros2_controllers/actions/workflows/humble-semi-binary-build.yml?branch=master) <br> [![build.ros2.org](https://build.ros2.org/buildStatus/icon?job=Hdev__ros2_controllers__ubuntu_jammy_amd64&subject=build.ros2.org)](https://build.ros2.org/job/Hdev__ros2_controllers__ubuntu_jammy_amd64/) | [control.ros.org](https://control.ros.org/humble/doc/ros2_controllers/doc/controllers_index.html) | [![Build Status](https://build.ros2.org/buildStatus/icon?job=Hbin_uJ64__ros2_controllers__ubuntu_jammy_amd64__binary)](https://build.ros2.org/job/Hbin_uJ64__ros2_controllers__ubuntu_jammy_amd64__binary/)

## Acknowledgements

The project has received major contributions from companies and institutions [listed on control.ros.org](https://control.ros.org/rolling/doc/acknowledgements/acknowledgements.html)
