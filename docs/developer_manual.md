# 개발자 매뉴얼

이 저장소를 **고치거나 확장하는 사람**이 알아야 할 내용입니다. 기능을 **쓰기만** 하는 사람은
각 튜토리얼(예: [tutorial_cumotion.md](tutorial_cumotion.md))만 보면 됩니다.

측정 기록, 결정 이력, 진행 상황은 노션 프로젝트 페이지(`isaac_ros2`의 process · project_harness · report)에 있습니다.

---

## cuMotion

### 1. 구조

```
호스트                                               컨테이너 isaac_ros_dev-x86_64-container
────────────────────────────                        ──────────────────────────────────────────
rby1_ros2_driver ──robot_ip──▶ 시뮬레이터/실기체      cumotion.launch.py (demo = + RViz)
  /rby1/robot_power·servo·robot_joint ◀─────────────   ① prepare           끝나면 ②를 켬
  /rby1/joint_states ────────────── DDS ───────────▶   ② joint_state_relay → /joint_states
  /rby1/robot_state, get_parameters ───────────────▶   move_group          계획·장면 전용
  /rby1/follow_joint_trajectory ◀───────────────────   cumotion_planner    GPU
  /rby1/stream_control, robot_power·servo ◀─────────   target_executor     움직이는 유일한 노드
rby1_examples (15~20, pub_cartesian_pose) ── /rby1/target_pose ──▶ target_executor
rby1_moveit_scene ── /apply_planning_scene ──▶ move_group ──(질의마다 장면 전체)──▶ cuMotion
```

- **드라이버와 컨테이너는 분리합니다.** 드라이버와 RB-Y1 SDK는 호스트에 두고, 컨테이너에는 드라이버의
  메시지 정의(`rby1_msgs`)만 넣습니다.
- **시뮬레이터와 실기체는 같은 경로입니다.** 드라이버의 `robot_ip`만 다릅니다.
- **MoveIt은 계획만 합니다.** 실행은 `target_executor`가 드라이버의 `follow_joint_trajectory`(FJT)로
  직접 보냅니다(§4).
- **기동은 두 단계입니다**(`bringup.py`). move_group과 cuMotion 플래너는 켜질 때 로봇 모델을 읽으므로, 번들·고정
  자세·팔 움직임을 전부 끝낸 뒤에 켭니다: ① `prepare`(한 번 돌고 종료) → 성공하면 ② 나머지. 실패하면 ②를 켜지 않고
  런치를 끝냅니다. `cumotion.launch.py`와 `demo.launch.py`는 이 모듈의 얇은 래퍼(RViz 기본값만 다름)입니다.
- **설정은 `config/cumotion.yaml` 한 파일**(`robot` / `motion` / `avoid` / `planner` 구역). `settings.py`와 `planner_params.py`는
  이름·타입·범위만 갖고 기본값을 두지 않습니다(§7).

#### 모듈

```
rby1_cumotion/rby1_cumotion/
├── model.py              번들 생성·적재, 축 정준화, SDK 캡슐 → 구, 실행용 번들(activate)
├── planning.py           공유 계획 계층 — 실행 코드 없음
├── execution.py          재시간화·재샘플, 드라이버 FJT 실행·교체(splice), 측정 관절로 도달 확인
├── avoidance.py          움직이는 중 장애물: 앞 구간 충돌 검사(cuRobo RobotWorld), MotionGen 재계획
├── driver.py             드라이버 서비스·액션 접근, 기종·버전·자세 읽기
├── settings.py           config/cumotion.yaml의 robot·motion·avoid 구역: 이름·타입·범위
├── planner_params.py     같은 파일의 planner 구역: 이름·타입·범위
├── bringup.py            런치 본체: prepare → (성공 시) 계획 스택 + 실행기 (+ RViz)
├── prepare.py            ① 드라이버 확인 → 번들 → 비상정지·고장 → 전원·서보 → 준비 자세 → 실행용 번들
├── target_executor.py    ② 목표 토픽 → 계획 → 시간 정하기 → 실행   (궤적을 보내는 유일한 노드)
├── check_plan.py         계획 1회, 움직이지 않음
├── benchmark.py          N회 측정, 움직이지 않음
└── joint_state_relay.py  /rby1/joint_states → /joint_states (런치가 실행)
```

#### 코드 규칙 (테스트가 고정)

| 규칙 | 이유 |
|---|---|
| `planning.py`에 실행 코드 금지 | `check_plan`·`benchmark`가 로봇을 움직일 수 없다는 보장이 여기 걸려 있음 (AST 검사) |
| `DriverExecutor`를 쓰는 노드는 `target_executor.py` 하나 | 실행 경로를 하나로 유지 |
| rclpy `Node` 속성 이름(`executor`, `handle` 등)을 속성·메서드로 쓰지 않음 | 둘 다 실제로 노드 생성을 깨뜨렸음 (`dir(Node)` 검사) |
| 한 호출 위치에서 두 심각도로 로그하지 않음 (`(log.error if x else log.info)(...)`) | rclpy가 `Logger severity cannot be changed between calls`로 예외 — 실행기가 첫 FAILED에서 죽었음 |
| 전원·서보·자세 이동 기능을 여기에 다시 만들지 않음 | `rby1_examples`(01_power_control, 06_zero_pose, 07_joint_command)에 있음 |

### 2. 모델 번들

사용자는 번들을 만들지 않습니다. 이미지에 들어 있고, 기동 시 자동으로 골라집니다.

#### 생성 (호스트, 이미지 빌드 전)

```bash
cd ~/isaac_ros_ws/src/rby1_isaac_ros/docker
./make_bundles.sh
```

| 산출물 | 내용 |
|---|---|
| `docker/bundles/<기종>/` | 7기종(`m_1_0`~`m_1_3`, `a_1_0`~`a_1_2`) × 4그룹 |
| `docker/driver_msgs/rby1_msgs/` | 드라이버 메시지 정의 |

- 입력: 드라이버 `rby1_description` URDF, `rby1_moveit_*` SRDF, SDK `~/sdk/rby1-sdk/models/*/urdf/model_v*.urdf`
- 두 디렉터리는 `.gitignore` 대상입니다. 비어 있으면 이미지 빌드가 `No model bundles: run docker/make_bundles.sh on the host first`로 실패합니다.
- 드라이버의 URDF·SRDF·`rby1_msgs`가 바뀌면 다시 돌리고 이미지를 다시 빌드합니다.
- 낡은 번들 판별: `model.json`의 `source_sha256` = 드라이버 URDF + SRDF + SDK URDF 해시.
- 호스트에서 colcon을 쓰지 않고 `PYTHONPATH`로 `python3 -m rby1_cumotion.model`을 부릅니다. 호스트와 컨테이너가
  워크스페이스를 공유해 양쪽 colcon 빌드가 서로의 `install/`을 깨뜨리기 때문입니다.

#### 구성

```
bundles/m_1_2/                    31 MB
├── robot.urdf  robot.srdf  spheres.yaml  model.json
├── kinematics.yaml  joint_limits.yaml  moveit_controllers.yaml  ros2_controllers.yaml
├── initial_positions.yaml
├── meshes/
└── groups/<그룹>/{robot.xrdf, group.json}     그룹당 74 KB
```

그룹: `right_arm` `left_arm` `right_arm+torso` `left_arm+torso` (디렉터리명은 `+` → `_`).

#### 실행용 번들 (`/tmp/rby1_cumotion/active`)

계획 그룹 밖 관절(반대팔·몸통·머리)은 cuRobo 안에 **고정값으로 컴파일**됩니다(cuMotion 구조). 그래서
런치의 ① `prepare`가 드라이버에서 기종(`model` 파라미터)·버전(`robot_state.robot_version`)·자세(`joint_states`)를
읽어 실행용 번들을 만듭니다. 팔을 준비 자세로 옮기는 것도 **그 전에** 하므로, 고정되는 자세는 이미 최종 자세입니다.

- 무거운 파일은 원본 번들로의 링크, 새로 만드는 것은 그룹 XRDF(자세 의존)뿐 — 1초 이내
- `active.json`에 원본·그룹·자세 출처·하드웨어 모드 기록
- 예제는 `model_directory`가 비어 있으면 이것을 씀. 경로는 `RBY1_ACTIVE_BUNDLE`로 바꿀 수 있음
  (검증용 두 번째 스택이 사용자 번들을 덮어쓰지 않게 할 때)
- 플래너가 뜬 뒤 그룹 밖 관절이 움직이면 모든 계획이 `PostureMismatch`로 즉시 거부됨 → 런치 재기동

### 3. 이미지

- NVIDIA 패키지는 **apt로** 이미지에 들어갑니다(`Dockerfile.cumotion`): cuMotion(`isaac-ros-cumotion`, `-cumotion-moveit`),
  AprilTag(`isaac-ros-apriltag`), 영상 보정(`isaac-ros-image-proc`). 워크스페이스에 소스로 두는 것은 `isaac_ros_common`
  (컨테이너 실행 스크립트)뿐입니다. apt 레이어는 cuRobo 커널 컴파일 뒤에 두어, 추가해도 커널을 다시 컴파일하지 않습니다(재빌드 약 1분)
- 예전에 소스로 빌드한 NVIDIA 패키지는 워크스페이스 `install/`에 있으면 apt 설치본보다 우선하므로 치워야 합니다
  (이 PC: `~/isaac_ros_ws/nvidia_build_backup/`로 옮김, `COLCON_IGNORE`)
- 카메라는 컨테이너에 없습니다: 호스트 `rby1_additional_tools`가 `/camera/image_raw`·`/camera/camera_info`로 넘기고, 컨테이너
  `rby1_apriltag apriltag.launch.py`가 보정 → AprilTag → 대상 필터. 실측(합성 이미지, 태그 7, 200 px, f=640, 0.10 m):
  기대 0.320 m → 0.3216 m, 15 Hz 입력 그대로 15 Hz. **Isaac ROS 노드는 reliable로 구독**하므로 best-effort 발행자의 영상은
  받지 못함(처음 그렇게 만들었다가 QoS 불일치)

`docker/Dockerfile.cumotion`은 isaac_ros_common 레이어 체인(`x86_64 → ros2_humble → realsense → cumotion`)의
마지막 레이어입니다. `docker/isaac_ros_common-config`가 체인을 설정합니다.

| 단계 | 내용 |
|---|---|
| apt | `isaac_ros_cumotion`, `_moveit`, MoveIt, ros2_control |
| `curobo_compat.py` | CUDA 12.6 + torch C++20(`lerp`) + warp 1.15+(`warp.torch`) 호환 |
| 커널 사전 컴파일 | nvcc만 필요(GPU 불필요). 컨테이너마다 3분 → 1.7초. `TORCH_CUDA_ARCH_LIST=9.0+PTX` (툴킷 12.8+ 되면 제거) |
| `cumotion_ik.py` | cuRobo IK 설정 5개를 노드 파라미터로 |
| `world_clear.py` | 지운 장애물이 남는 업스트림 결함 3곳 (§6) |
| `rby1_msgs` | 스테이징 prefix에 빌드 후 파일만 `/opt/ros/humble`로 복사 |
| 번들 | `$RBY1_BUNDLES=/opt/rby1/bundles` |
| bashrc | 셸 진입 시 워크스페이스 `install/setup.bash` 자동 source |

패치는 모두 멱등이며, 대상 문자열이 정확히 1회 일치하지 않으면 **빌드를 실패시킵니다.**

> ⚠️ **`colcon build --install-base /opt/ros/humble` 금지.** colcon이 배포판 `setup.sh`를 자기 것으로 덮어써
> 배포판 환경 훅이 사라지고, URDF를 읽는 모든 노드가 `pluginlib::LibraryLoadException`으로 죽습니다.
> 빌드 단계가 `setup.sh` 원본 여부와 `liburdf_xml_parser.so` 의존성을 검사합니다.

### 4. 실행 경로와 드라이버 동작

```
목표 4×4 → cuMotion 계획 → 이동 시간 결정(아래) → 재시간화(클램프 큐빅 스플라인)·0.05 s 재샘플
        → 최종 자세 유지 0.5 s 추가 → stream_control ON → FJT → stream OFF → 측정 관절로 도달 확인
```

드라이버의 FJT가 궤적의 `time_from_start`를 구간별 `minimum_time`으로 SDK에 스트리밍하므로 **궤적 시간이 곧 속도**입니다.
ros2_control(`rby1_hardware`)을 거치면 드라이버가 `hardware_control` 중이라며 FJT를 거부합니다.

| 드라이버 동작 (`rby1_ros2_driver.cpp`) | 대응 |
|---|---|
| 스트림이 켜진 채 1초간 명령이 없으면 모든 스트림을 끔 | 0.05 s 재샘플(`step` ≤ 0.5). 스트림은 전송 직전에 켬 |
| 스트림이 끊겨도 FJT가 성공으로 보고할 수 있음 | 완료는 측정 관절로 판정 |
| 마지막 웨이포인트를 보내자마자 완료 보고 | 끝에 `hold` 0.5 s. 없으면 빠른 동작 6회 중 2회가 0.03~0.055 rad 모자람 |
| 웨이포인트마다 명령 전송 **후** 구간 시간만큼 잠듦 | 실제 시간이 요청보다 약 15% 김 (미보정) |
| 새 FJT를 받을 때마다 웨이포인트별 **예측 충돌 검사**(SDK) — 궤적 교체마다 팔이 멈칫함 | **드라이버 수정**: 주석 처리(`/* */`와 이유). 자기충돌은 cuMotion 계획과 로봇의 실시간 상태가 맡음 |
| 새 FJT마다 control manager 확인 + 100 ms 대기 | **드라이버 수정**: 이미 enable이면 건너뜀. 주의: 스트림을 연 뒤 control manager를 처음 enable하면 그 첫 FJT에서 스트림이 만료됨(`command stream is expired`) |
| 새 FJT의 시작점을 검사하지 않음 | **드라이버 수정** `start_problem()`(goal 핸들러): 관절마다 \|첫 점 − 현재\| ÷ 첫 점 시간 > 관절 속도 한계 × `fjt_start_velocity_scale`(기본 1.0)이면 **거부**. 느린 궤적이면 차이가 커도 통과. 다른 FJT가 실행 중이면 그것도 **중단**(`Stopped: the trajectory sent to replace it was rejected (...)`) — 문제 있는 교체를 받고 옛 궤적을 계속 가지 않음 |
| `stream_control`을 끄고 약 0.1 s 안에 다시 켜면 새 스트림이 이미 만료(`This command stream is expired`) — 그 궤적은 로봇에 전달되지 않는데 FJT는 성공 보고 (실측: 0.05 s 간격 2/3, 0.3 s 이상 0/3). 목표를 연달아 보내면 걸림 | 실행기: 끄고 `STREAM_REOPEN_GAP`(0.5 s) 전에는 다시 켜지 않음(`DriverExecutor`, MoveIt 실행기 동일). **드라이버 수정**: 전송 중 만료되면 스트림을 한 번 새로 열어 이어 보내고, 그래도 3점 연속 실패하면 goal을 abort(`Could not send the trajectory to the robot …`). `stream_joint`도 만료 시 한 번 새로 열어 보냄 |
| 한 명령 스트림은 **앞선 명령에 없던 부위를 무시** (실측: 머리만 보낸 스트림에 팔+머리를 보내면 팔 0 rad, 오른팔만 보낸 스트림에 왼팔 0 rad; 몸 전체 뒤 머리만은 동작). 머리 추적(머리만 `stream_joint`) 중의 FJT·`robot_joint`가 **성공을 보고하고 팔은 안 움직임** | **드라이버 수정** `fit_stream_to()`: 스트림이 실어 본 부위(`stream_parts_`)에 없는 부위가 오면 옛 스트림을 닫고 0.15 s 뒤 새로 엶(닫자마자 열면 만료, 옛 것을 둔 채 열면 겹친 부위 때문에 새 명령이 안 먹힘). 옛 명령이 아직 움직이면 끊지 않음 — 최소 시간 안이거나, 단발 명령(`robot_joint`·cartesian)이면 관절이 아직 도는 중(\|v\| > 0.02 rad/s): 단발 명령은 0.5 s까지 기다리고, `stream_joint`·`stream_cartesian`은 거부(`[STREAM REJECTED] … still moving other parts`) |
| 머리가 몸과 같은 스트림 — FJT가 머리까지 잡아 팔이 움직이는 동안 머리 명령(머리 추적)이 전부 거부됨 | **드라이버 수정**: 머리는 모바일처럼 **자기 스트림**(`head_stream_handler_`, `send_head()`). FJT는 궤적에 머리 관절이 있을 때만 머리를 잡음(`trajectory_parts()`, `fjt_parts_`), `stream_joint`·`robot_joint`는 FJT가 잡은 부위와 겹칠 때만 거부. 실측: 몸·머리 두 스트림 동시 명령 둘 다 동작(SDK 직접), 팔 FJT 중 머리 20 Hz 명령 180개 거부 0 · 팔 목표 정확히 도달 |
| `set_trajectory_impedance`가 값만 저장하고 FJT는 늘 위치 명령 — 임피던스 궤적 미구현. 스트림이 켜져 있으면 설정 자체를 거부 | **드라이버 수정**: 원래 드라이버(스트림 루프가 이 플래그를 보던 방식)대로 **서비스 하나가 스위치** — FJT와 `stream_joint`의 몸통·팔 명령이 부위별 플래그를 보고 `JointImpedanceControlCommandBuilder`로 보냄(강성·감쇠·토크 한계). 별도 파라미터 없음: 서비스가 값을 안 주면 원래 값(강성 100 N·m/rad, 감쇠 1.0, 토크 10 N·m). 스트림이 켜져 있어도 설정 가능(FJT 중만 거부) — 다음 명령이 새 스트림으로(`stream_kind_changed_`; 스트림 중 전환 실측: 명령 2개(0.19 s) 거부 뒤 이어짐). 실측: FJT 강성 100에서 같은 궤적이 위치 0.400 → 임피던스 0.383 rad, `stream_joint` 50 Hz 팔 굽힘의 끝 오차 위치 0.000 · 임피던스 100/10 0.216 · 500/50 0.000 rad |
| `stream_cartesian`이 지금 자세와 다른 목표를 전부 `IK did not converge`로 거절(1 cm 이동도; 지금 자세 그대로만 통과) | **드라이버 수정** `solve_cartesian_ik()`: SDK `OptimalControl::Solve`의 비용이 J(q̇_old + q̇_new)/2·dt = 자세 오차(사다리꼴)인데, 반복이 직전 q̇을 다음 반복에 넘겨 다음 걸음이 앞 걸음을 되돌림 → 솔버 오차가 w·Δ/dt(1 cm면 100)에서 안 줄어듦. 매 반복 q̇ = 0으로 두고(뉴턴 걸음), 수렴은 솔버 오차 대신 **자세 오차**(0.5 mm, 2 mrad)로 판정 |
| 같은 IK가 손목이 펴진 특이 자세(준비 자세, `right_arm_5` = 0) 근처에서 손목 두 관절을 서로 반대로 크게 돌림 — 1 mm 노이즈를 6 s 따라가자 `right_arm_6`가 −1.94 rad까지 가고, 이후 해가 1 rad씩 달라 가속도 검사에 전부 거절 | **드라이버 수정**: 시작 관절 근처에 머물게 하는 관절 목표(`q_target`, 가중치 1 — 위치 1000·방향 100 대비)를 추가. 호출마다 찍던 디버그 줄(약 35줄 WARN)은 DEBUG로. 실측: 1 mm 노이즈로 30 s 추종에 거절 1/550, `right_arm_6` −0.4~−0.7 rad |
| `stream_joint`·`stream_cartesian` 액션 서버가 끝난 goal을 15분 동안 보관 — 50 Hz로 보내면 상태 배열이 수천 개가 되어 Python 클라이언트가 틱마다 30 ms를 씀(추적 모드 16.5 Hz) | **드라이버 수정**: 두 서버의 `result_timeout` 2 s. 추적 모드 50.0 Hz(§6.2) |
| 스트림 모드의 `robot_joint`·cartesian은 명령 한 번 보내고 `minimum_time`만큼 기다리는데, 그동안 스트림 명령이 없어 1초 타임아웃이 스트림을 닫음 → `minimum_time` > 1 s면 1초 지점에서 멈추고 `kOk` (실측: 3 s 명령이 0.5 rad만 감) | **드라이버 수정**: 대기 루프에서 idle 시계를 갱신 |
| 상태 읽기(`GetState`) 실패 한 번에 `Connection to robot lost`로 노드 종료 — 시뮬레이터가 드물게 `Collision.link2`에 UTF-8이 아닌 문자열을 넣어 protobuf가 응답 전체를 거부(gRPC `UNIMPLEMENTED: No message returned`) | **드라이버 수정**: `get_state_with_retry()`(3회, 5 ms)로 모든 호출 지점 교체(액션 스레드의 예외가 프로세스를 죽이던 것 포함), 읽기 루프는 `state_loss_timeout`(기본 1.0 s) 동안 연속 실패할 때만 종료 |

#### 관절 임피던스와 스트림 소유 (`impedance` 구역)

- 실행기는 움직이기 직전에 설정이 바뀌었으면 드라이버 `set_trajectory_impedance`를 부름(`apply_impedance`). 계획 그룹의 부위만
  켜고 나머지 부위는 위치로 돌림(서비스가 세 부위를 한 번에 설정). 한 번도 켜지 않았으면 부르지 않아 드라이버 상태(기본 위치,
  누가 서비스를 불렀으면 그 설정)가 그대로. 추적 모드도 스트림을 켜기 전에 같은 설정을 넘김 — `stream_joint`가 같은 스위치를 따름
- 기본 강성 500 N·m/rad, 토크 한계 50 N·m: SDK 예제 값(100, 10)으로는 시뮬에서 손목이 목표보다 최대 0.5 rad 모자라
  예제 15가 3/3 실패, 500·50과 1000·100은 통과. 무른 팔은 조금 덜 가므로 도착 확인 실패 메시지가 임피던스일 때 강성·`endpoint_tolerance`를 안내
- 실측(격리 시뮬, cuMotion): 임피던스로 예제 15 ×2, 18(회피), 19(기다림) 모두 DONE, 실행 중 켜고 끄기 동작
- **스트림은 자기가 켠 경우에만 끔**(`owns_stream`): 이동 전에 `robot_state.robot_stream_state`가 이미 켜져 있으면(머리 추적 등
  다른 사용자) 끝나도 그대로 둠. 실측: 머리 20 Hz 명령 800개 동안 MoveIt 예제 3회 — 머리 명령 거부 0

#### 드라이버로 보내는 부분은 두 벌 (Python·C++)

계획 뒤 로봇에 보내는 부분이 cuMotion(`execution.py`, Python — cuRobo 때문에 C++ 불가)과 MoveIt 실행기(드라이버 저장소
`rby1_moveit_executor`, C++)에 따로 있습니다. 한쪽을 고치면 다른 쪽도 맞춥니다:

| 동작 | cuMotion (`execution.py`) | MoveIt (`trajectory.cpp`, `executor.cpp`) |
|---|---|---|
| 0.05 s 균등 재샘플, 끝 정지 | 클램프 큐빅 + 관절 한계 검사 | PCHIP(넘치지 않음) + 한계 안쪽 1e-4 |
| 끝에 0.5 s 유지 | `with_hold` | `with_hold` |
| 스트림 끈 뒤 0.5 s 전 재개방 금지 | `STREAM_REOPEN_GAP` | `kStreamReopenGap` |
| 자기가 켠 스트림만 끔 | `owns_stream` | `owns_stream_` |
| 임피던스 설정 전달 | `apply_impedance`, `impedance_request` | `apply_impedance` |
| 시간값 변환은 총 나노초를 먼저 반올림 | `stamp` | `stamp` |
| 측정 관절로 도착 확인(`endpoint_tolerance`) | `confirm` | `execute` 끝 |

#### 이동 시간 (`motion_duration`, execution.py)

경로는 그대로 두고 시간만 늘리거나 줄이므로 경로 위의 모든 속도는 이동 시간에 반비례합니다. 1 s로 한 번 시간을 입혀
손(tool 프레임)의 최고 선속도·각속도(`ToolChain`: base→tool 체인만 FK)와 관절 속도 비를 구하면, 각 제한이 요구하는
최소 시간이 바로 나옵니다. 이동 시간 = max(`minimum_time`, 선속도, 각속도, 관절 한계×1.02). `duration` > 0이면 고정.

- 기본값(`motion` 구역)은 드라이버 `driver_parameters.yaml`의 `linear_velocity_limit` 1.5 m/s,
  `angular_velocity_limit` 4.712388 rad/s, `minimum_time` 2.0 s와 같음. **드라이버의 FJT는 이 값을 쓰지 않으므로**
  (SDK Cartesian 명령용) 여기서 적용함
- `motion.*`는 실행 중 `ros2 param set`으로 바뀌고, `settings.coerce`로 범위를 검사해 틀리면 거부
- 격리 시뮬 실측: 짧은 이동 2.00 s(`minimum_time`), 0.1 m/s로 30 cm 5.02 s(`linear_velocity_limit`)

#### FJT는 몸 전체를 한 채널로 씁니다

- `joint_names`에 **일부 관절만** 넣어도 됩니다. 드라이버는 궤적 시작 시점의 **측정 자세**를 복사한 뒤 받은 관절만
  덮어써서, 매 웨이포인트마다 torso·오른팔·왼팔·head 명령을 **모두** 보냅니다. 안 보낸 부위는 시작 자세를 유지합니다.
- 대신 실행 중에는 torso·양팔이 점유되고(`is_controlling_`, 머리는 궤적에 머리 관절이 있을 때만),
  새 FJT는 이전 것을 **선점**하며, 그동안 잡힌 부위의 `robot_joint`·`stream_joint`는 거부됩니다. **양팔을 서로 다른 궤적으로
  동시에** 움직일 수는 없습니다.
- cuMotion은 한 번에 한 그룹만 계획하고 나머지를 고정하므로 이 방식이 오히려 맞습니다. 그룹 밖 관절이 움직이면 플래너 모델이 틀려집니다.
- 머리 추적(`rby1_examples`의 `22_marker_tracking`, `follow:=head`)은 `stream_joint`로 머리만 보냅니다. 머리 스트림이 따로라 팔 FJT 중에도 계속 움직입니다
  ([AprilTag §3](#3-마커-예제-드라이버-저장소-rby1_examples-22-23)).
- 부위별 독립 제어가 필요해지면 드라이버에 이미 **`stream_joint`**(부위별, 명령한 부위만 점유)가 있습니다. FJT를 부위별 채널로
  나누는 드라이버 수정은 양팔 동시 계획이 생길 때 검토합니다(cuMotion 플러그인은 `plan_single`만 호출하므로 현재는 불가).

#### 기동 순서 (`prepare` → `target_executor`)

`prepare` (한 번 돌고 종료, 실패 시 `PREPARE_FAILED: <이유>`로 런치 종료):
1. 드라이버에서 기종·버전 → 번들 선택(`robot.model`을 주면 드라이버와 대조)
2. `robot_state`: 비상정지(`emo_state`) 또는 major fault면 거부
3. `robot.enable_robot`이면 `robot_power`·`robot_servo` ON
4. `robot.ready_if_straight`이면 계획 그룹의 팔 중 팔꿈치(`*_arm_3`)가 `straight_elbow`(0.3 rad) 안으로 펴진 팔을
   `robot_joint`로 준비 자세(`READY`, 팔꿈치 −1.57)로 옮기고 측정 관절로 확인
5. 자세를 다시 측정해 실행용 번들 생성 → `PREPARED ...` 출력 후 종료 코드 0

`target_executor` (② 단계에서 계획 스택과 함께 기동):
1. 기종·비상정지·고장 재확인, 전원·서보 ON(이미 켜져 있으면 무해)
2. 워밍업 — 계획 1회(움직이지 않음). 첫 계획의 CUDA 그래프 캡처(13관절 약 9.4 s)가 MoveIt 플러그인의
   하드코딩 5초 대기를 넘기 때문. 장애물이 한 방향을 막으면 다른 축 방향을 시도. 플래너가 아직 뜨는 중이면 그만큼 기다림
3. `READY` 발행 후 목표 대기. 실행 중 들어온 목표는 버림(큐 없음)

### 4.1. 7축 팔과 시작 자세

cuMotion의 목표는 **손 자세(6자유도)뿐**이고 팔은 7축이라, 같은 손 자세에 팔 모양이 1차원으로 무수히 많습니다.
cuRobo MotionGen은 현재 관절을 IK의 regularization 기준(`retract_config`)과 시드 하나로 넘기지만, 가중치가 작고
(`gradient_ik.yml` `null_space_weight` 0.001, `base_cfg.yml` convergence `null_space_cfg` 0.001) 나머지 무작위 시드가 이길 수 있습니다.

측정 (m_1_2 right_arm, 컨테이너에서 cuMotion과 같은 MotionGen 설정, +5 cm x·+5 cm z 후 원래 손 자세로 복귀, 각 10회):

| 시작 자세 | null-space 가중치 | 복귀 후 max\|q − 시작\| (중앙값 / 최대) |
|---|---|---|
| 영점 (팔 곧게 폄) | 0.001 (기본) | 0.20 / 0.76 rad |
| 영점 | 0.1 / 1.0 / 10 | 0.52 / 0.75 · 0.49 / 0.87 · 0.39 / 0.65 rad |
| 준비 자세 `[0,-0.5,0,-1.57,0,0,0]` | 0.001 (기본) | 0.009 / 0.025 rad |
| 준비 자세 | 1.0 | 0.007 / 0.042 rad |

- **원인은 IK 파라미터가 아니라 시작 자세**입니다. 팔을 곧게 편 영점은 특이 자세라 `arm_2`(위팔 롤)와 `arm_6`(손목 롤)이 서로 상쇄하는
  해가 연속으로 존재하고, 복귀 때 그중 하나(비틀린 모양)로 갑니다. null-space 가중치를 올려도 개선되지 않았으므로 파라미터로 노출하지 않았습니다.
- 사용자 흐름은 준비 자세에서 시작합니다 — `prepare`가 펴진 팔을 굽힙니다(§4 기동 순서). 시뮬레이터 실측: 오프셋 왕복 10회 누적 후에도 0.07 rad 이내, `matrix` 절대 복귀는 0.02 rad 이내.
- 관절 목표로 정확히 돌아가는 기능은 cuMotion 경로로는 만들 수 없습니다 — MoveIt 플러그인이 관절 목표도 FK→포즈로 바꿔 IK를 다시 풉니다.
  정확한 관절 자세는 드라이버 `robot_joint`(Rby1JointCommand)로 보냅니다(충돌 회피 없음).
- 실험 중 주의: cuRobo `kinematics.get_state(...).ee_pose`는 **재사용 버퍼**를 돌려줍니다. 나중 계산에 덮어써지므로 보관하려면 `.clone()`.

### 5. 충돌 모델

- **RB-Y1 SDK 캡슐 17개**와 `coltype`/`colaffinity` 마스크를 씁니다. 메시에서 추론하지 않습니다.
- 캡슐을 구로 분해: 중심 간격 `d`에서 `r' = √(r² + (d/2)²)`가 정확한 피복 반지름, 팽창 `--tolerance` 2%. 17캡슐 → 121구.
- `check_frames_agree()`가 빌드마다 무작위 25자세로 드라이버 URDF와 SDK URDF의 링크 위치 일치를 확인(불일치 시 빌드 실패).
- 충돌 쌍: 136쌍 중 SDK 마스크가 정한 43쌍만(`torso_* ↔ arm_{3,4,5}`, `arm_{1..5} ↔ 반대팔`). 무시 집합은 SDK 마스크 → SRDF → 동일 active 조상.
- 캡슐 없는 링크 3개(`link_{right,left}_arm_6`, `link_head_2`)에 반지름 −10 자리표시자 구 — cuRobo가 모든 관절을 잠그려면 필요.
- **머리 덮개 구**(`head_envelope`, 그룹을 만들 때): 머리 두 관절 축이 한 점에서 만나므로, 그 점을 중심으로 머리와 함께 도는 링크
  메시의 가장 먼 꼭짓점 + 2 cm(카메라 몫, `HEAD_MARGIN`)를 반지름으로 한 구를 고정된 목 받침 `link_head_0`에 붙임
  (v1.2: 중심 0.08 m, 반지름 0.0998 m / v1.0: 0.02 m, 0.067 m). 머리가 어디를 보든 덮으므로 `head_0`·`head_1`은 cuRobo에는
  잠긴 채지만 자세 검사에서는 뺌(`unwatched_joints`, `check_locked`). 머리 추적과 cuMotion을 같이 쓰기 위함(C60 (가)). 테스트:
  무작위 머리 자세 20개에서 모든 꼭짓점이 구 안
- 축 정준화: 어깨 20° 기울기를 무게 없는 정렬 링크로 분해(`T∘Rot(a,q) = (T∘R)∘Rot(e,q)∘Rᵀ`). 관절명·값·링크 프레임 보존.
- **cuMotion이 보지 못하는 부분**: 캡슐은 몸통과 팔 0~5번에만 있습니다(머리는 위의 덮개 구). **바퀴는 충돌 검사에서 빠집니다.**
- **계획하는 팔은 손 끝 프레임에서 끝남** (`trim_at_tool`, `robot.body_ends_at_tool`, 10/02): SDK의 `arm_5` 캡슐은 손목뿐 아니라
  그 앞에 달린 것까지 감싼 덮개입니다(반지름 7.5 cm, `ee_right`를 7 cm 지나감 — 손이 장면의 어떤 것에도 14 cm 안으로 못 감).
  그룹을 만들 때(런치마다, 이미지 재빌드 불필요) 그 링크의 구 중 공구 프레임을 `TOOL_MARGIN`(1 cm) 넘게 지나가는 것을 뺌
  (m_1_2: 9개 중 5개, 남은 4개가 손목을 덮음). 공구 앞의 것은 붙인 모듈로 받음(§6). 반대팔은 SDK 덮개 그대로(붙인 모듈은 계획
  팔의 손 끝 것만 구가 되므로). 마지막 손목 관절이 공구를 캡슐 링크 위에서 휘두르는 기종(m_1_3)은 끊을 자리가 없어 그대로 둠
- 장애물을 팔 가까이 두면 **시작 자세부터 충돌**(`INVALID_START_STATE_WORLD_COLLISION`)이 되어 모든 계획이 거부됩니다(팔 구 반지름 3.6~7.6 cm).

### 6. 장애물 처리

- MoveIt cuMotion 플러그인이 **질의마다 계획 장면 전체**를 cuMotion에 보냅니다. cuMotion은 상자·구·원통·메시를 cuRobo
  월드로 바꿉니다(구·원통은 메시로). 슬롯 한도 `collision_cache_cuboid`/`_mesh`(기본 20).
- 장면 도구는 드라이버 저장소 `rby1_moveit/rby1_moveit_scene`(`/apply_planning_scene`), 같은 도메인 어디서든.
  C++ 패키지(명령행 `scene` + 라이브러리 `rby1_moveit_scene::rby1_moveit_scene`). `scene move NAME --velocity VX VY VZ --time T`는
  물체를 `--rate`(기본 10 Hz)로 다시 ADD해 밀고 끝 위치에 둡니다(서비스 클라이언트를 재사용해야 10 Hz가 나옴; 실측 3 s에 30회).
  `/monitored_planning_scene`은 move_group이 4 Hz로 줄여 내보내므로 갱신 주기 측정에는 `/get_planning_scene`을 씀
- 업스트림 결함(이미지에서 `world_clear.py`로 수정) — 패치 전에는 지운 장애물이 재시작 전까지 남았습니다(0/5):

| 위치 | 결함 |
|---|---|
| cuMotion `update_world_objects` | 장면 물체가 0개면 월드를 갱신하지 않음 |
| cuRobo 메시 | 새 월드의 메시가 0개면 이전 메시를 끄지 않음 |
| cuRobo 상자 | 새 월드의 상자가 0개면 이전 상자를 끄지 않음 |

  지면(`add_ground_plane`)은 세 번째 결함 덕에 우연히 유지되던 것이라 패치가 명시적으로 다시 넣습니다.
- nvblox ESDF(`read_esdf_world:=true`)는 미연동.
- **링크에 붙인 물체(attached collision object)** — MoveIt 플러그인이 넘기는 장면에서 cuMotion(`cumotion_planner.py`)은
  `world.collision_objects`만 읽으므로 실행기가 직접 넘김(C62 (a), `attached.py`):
  - 번들 그룹 XRDF에 `attached_object` 프레임(`modifiers.add_frame`, 공구 프레임에 고정)과 자리표시자 구 — cuRobo는 XRDF 구 목록에
    있는 링크만 검사함. 플래너는 이 링크에 구 100칸을 잡아 둠(`extra_collision_spheres`). 자기충돌은 공구와 같은 몸체·손목(한
    관절 위) 링크와는 보지 않음
  - 실행기가 1 s마다(목표를 받으면 바로) `/get_planning_scene`의 `ROBOT_STATE_ATTACHED_OBJECTS`를 읽어, 바뀌었으면 공구와
    고정 관절로만 이어진 링크의 물체를 **덮는 구**로(상자·원통은 모든 점이 어떤 구 안, 메시는 바운딩 박스, 반지름은 올림) →
    플래너 `planner_attach_object` 액션(먼저 detach — `update_link_spheres`가 0번 칸부터 덮어써 이전 구가 남으므로)과
    `Avoider.attach`(RobotWorld·MotionGen의 kinematics)에 넣음. cuRobo `get_bounding_spheres`는 안쪽을 채우는 근사라 상자
    모서리가 빠져서 쓰지 않음
  - 머리에 붙은 것은 머리 덮개 구(§5)가 대신, 그 밖은 로그로 알림. 100구를 넘으면 오류 로그
  - **그리퍼는 모듈로** (10/02, 사용자 결정): 드라이버 저장소 `rby1_moveit_objects/config/gripper.yaml`이 그리퍼 몸통과 손가락
    둘(가장 벌린 자세)을 URDF의 메시·위치로 `ee_right`·`ee_left`에 붙임. 아무것도 안 붙었으면 실행기가 경고
  - **`free_objects`** (실행기 파라미터, 실행 중 변경 가능, 이름에 `*`): 맞는 붙인 모듈은 구에서 빼고(cuMotion은 무엇과 닿아도
    모름), 맞는 붙인 모듈·**로봇 링크**를 MoveIt 허용 충돌 행렬의 기본 허용으로 넣음(`allow_touching`, 처음 읽은 행렬을 바탕으로
    매번 다시 만듦). cuRobo에는 "이 부위와 이 물체만" 같은 짝별 허용이 없어 부위 단위로만 가능
  - **MoveIt이 한 번 더 검사함**: cuMotion 플러그인이 낸 경로를 `move_group`이 자기 장면(URDF 메시, 그리퍼 링크 포함)으로 검증해
    충돌이면 `INVALID_MOTION_PLAN`(−2). 그래서 cuMotion에서 구만 빼서는 손가락이 닿는 경로가 통과하지 못하고, 모듈 없이도
    그리퍼가 닿는 경로는 거절됨(피해 가지는 않음). 이동 중 재계획·추적 모드는 cuRobo 직접이라 이 검증이 없음
  - 실측(격리 시뮬, 손 끝 20 cm 앞 벽): 모듈 없음 +15 cm `INVALID_MOTION_PLAN` / +5 cm DONE(전에는 덮개 때문에 불가),
    모듈 있음 +8 cm `PLANNING_FAILED` / +2 cm DONE, 손가락 free +8 cm DONE, free 해제 후 +8 cm 다시 거절. 모듈을 붙인 채 예제 15 DONE,
    18 3회 중 2회 DONE(1회 `too close`, 메시 장애물을 같이 둔 실행 — 원인 미확인)
  - **덮는 구의 크기**: 상자는 가장 긴 칸을 예산이 남는 동안 계속 쪼갬(`cover_box`). 그리퍼 몸통(12.6×6.5×7.3 cm)은 손가락과
    함께일 때 24구·면 밖 1.4 cm, 손가락이 free면 96구·0.7 cm(전에는 가장 얇은 변이 칸 크기를 정해 4구·3 cm)
  - **집기 흐름 실측** (격리 시뮬, 10/02): 탁자 위 공(반지름 3 cm, 세계 물체)에 손가락을 수평으로 돌려 10 cm 전진 → 공을
    `ee_right`에 attach(touch_links = 그리퍼 링크) → 5 cm 들기 → 옆으로 옮기기 → 다시 내려놓기 → detach → 후퇴.
    공 중심이 공구 프레임 앞 13 cm(손가락 끝 사이)일 때: `free_objects` 없음 → 잡는 자세 `TIMED_OUT`, 손가락 free → DONE.
    잡은 직후 들기는 공의 구가 탁자 1 mm 위라 `too close`로 실패, 공도 free면 DONE(든 뒤에는 공을 free에서 빼고 옮기기·내려놓기 DONE).
    11·12 cm(공이 몸통 쪽)에서는 손가락만 free로는 실패(`IK_FAIL`·`FINETUNE_TRAJOPT_FAIL`), `gripper_*` 전체 free로 전 과정 DONE.
    즉 잡는 구간은 "손가락(+잡은 물체) free", 벗어나면 해제. 물체와 손가락만의 짝별 허용은 없음(위)
  - 실측: 오른손 20 cm 막대 + 막대 끝 앞 벽 + 손 5 cm 앞 → 전: MoveIt 3/3 거부, cuMotion 3/3 실행 / 후: cuMotion도 3/3
    거부(`PLANNING_FAILED`), 막대를 떼면 DONE. 기본 `objects.yaml`(머리 카메라·공구 끝·작업대)로 예제 15·16·18·21 DONE
- cuMotion 런치 기동 직후 첫 `/apply_planning_scene`이 10 s 안에 답하지 않은 적이 있음(물체는 반영됨) — 다시 보내면 됨

- 실행기의 이동 중 검사·재계획·추적은 장면의 상자·구·원통과 **메시**를 읽음(10/02 — 전에는 메시를 건너뜀). 충돌 검사에는 메시
  그대로(cuRobo `Mesh`), 움직이는 장애물의 시간 맞춘 거리 계산에는 그 바운딩 박스를 씀(`objects_from_scene`, `surface_distance`)

### 6.1. 움직이는 중 장애물 회피 (`avoidance.py`, `avoid` 구역)

계획 시점 이후에 생긴 장애물을 처리합니다. **서 있는 장애물은 멈추지 않고 돌아가고, 움직이는 장애물은 경로 위에서 멈춰
지나가길 기다립니다.** 궤적은 계속 FJT로 보내고(스트리밍 아님), 드라이버가 새 goal로 이전 goal을 선점하는 것을 이용해
**실행 중 궤적을 통째로 바꿉니다**.

```
실행 중 check_period(0.05 s)마다:
  now() = 마지막 FJT feedback 시점 + 경과 시간 (feedback을 먼저 처리)
  앞 horizon(0.5 s) 구간을 cuRobo RobotWorld로 충돌 검사 (10점 0.38 ms)
  서 있는 장애물과 충돌 (avoid_once):
         t_join = min(now + lead(0.2), 끝), 충돌 한 스텝 전으로 제한
         t_join < now + replan_time(0.15) → FAILED "too close" (정지)
         t_join의 위치·속도에서 MotionGen 재계획 → splice → FJT 교체 → AVOIDING
         give_up_after(5 s) 넘게 계속 충돌 → FAILED
  움직이는 장애물과 충돌 (brake):
         경로 위에서 제동(slow_to_stop: stop_time 0.6 → 0.3 → 0.1 s) 또는 제동 후 경로를 되짚어 후퇴
         (back_off: 0.25/0.5/1.0/1.5 s, 관절 속도 한계 50%까지) 중 이동·대기 2 s 동안 ±0.3 s 여유로
         안 부딪히는 첫 번째 → FJT 교체(대기 자세 hold = wait_limit + 2 s) → WAITING
         없으면 FAILED "heading for the arm" / 제동 거리도 없으면 FAILED "too close to stop short"

대기 중 (wait_out) check_period마다:
  대기 자세에 움직이는 장애물이 오면 → 더 후퇴 (back_further)
  남은 경로(resume_from: 0에서 가속하며 원래 경로)가 ±0.3 s 여유로 두 번 연속 비었음 → FJT 교체 → RESUMING on the path
  남은 경로를 서 있는 것이 막음(장애물이 멈춤) → 대기 자세에서 MotionGen(속도 0) → 검사 통과 시 RESUMING …
  wait_before_replan(2 s) 넘게 계속 막힘(움직이는 것이라도) → 대기 자세에서 새 길 → RESUMING along a new path (n of at most N)
  대기 자세가 장애물에 너무 붙어 새 길이 INVALID_START_STATE → 한 번은 왔던 경로로 더 물러난 뒤 다시(back_further)
  새 길은 목표마다 max_replans(3)번까지 → 넘으면 FAILED. wait_limit = wait_before_replan × (max_replans + 1) 넘게 대기 → FAILED

새 목표 (retarget, C59) — 실행 중 루프 맨 앞에서 self.pending을 봄:
  REPLACED 발행 → 움직이는 중: t_join = now + lead의 위치·속도에서 MotionGen / 대기 중: 선 자리에서
  계획이 t_join보다 늦게 끝나면 다시(4번까지) → splice → FJT 교체 → EXECUTING toward the new target, replans = 0
  그래도 없으면(HOPELESS 아님) 경로 위에서 제동(slow_to_stop) → 선 자세에서 cuMotion 정식 계획(measure) → retime → 교체
```

| 부분 | 내용 |
|---|---|
| 검사 월드 | `RobotWorldConfig`(MESH 검사기, `n_meshes`/`n_cuboids` = cache). PRIMITIVE 검사기는 구를 무시해서 MESH + `get_collision_check_world()` |
| 재계획 | 별도 MotionGen(MESH, `interpolation_dt` 0.025), 시작 속도를 넘김. 실행기 기동 때 워밍업. 속도는 `replan_seeds`·`replan_steps`·`replan_iters` — 기본 4·24·100으로 계획 0.05~0.13 s(cuRobo 기본값 6·32·0이면 0.13~0.20 s). 그래서 `lead` 0.2, `replan_time` 0.15 |
| 실패 판정 | `IK_FAIL`, `INVALID_START_STATE_*`, `INVALID_QUERY` → 목표까지 길 없음(`no way around the obstacle to the target`). 그 외 실패는 다음 주기에 재시도 |
| `from_start` | 시작 속도를 주면 MotionGen 경로의 앞 몇 점이 시작보다 v·0.07 s쯤 **뒤에** 있음 → 시작에 가장 가까운 점부터 자르고 그 점을 시작 자세로 고정 |
| `splice` | `now + step` ~ `t_join`은 기존 궤적 그대로(드라이버가 이미 받은 구간), 그 뒤는 새 경로를 호 길이로 매개화한 CubicSpline, 경계 조건 (t_join 속도, 끝 0) → 교체 지점에서 속도 연속. 관절 속도 한계를 넘으면 시간을 늘림 |
| 우회 시간 | `detour_time` = max(MotionGen 자체 시간, `motion` 제한이 요구하는 시간). 남은 원래 시간에 맞추지 않음 |
| 경로 위 제동·후퇴·재개 | `slow_to_stop`·`back_off`·`resume_from`(execution.py): 경로는 그대로 두고 시계만 바꿈(제동은 시계 속도 1→0 선형, 후퇴·재개는 smoothstep/선형 가속). 그래서 되짚는 구간은 이미 서 있는 장애물이 없다고 확인된 경로 |
| 움직이는 장애물 | 장면을 `scene_period`마다 읽어 위치 변화로 속도 추정(EMA, 0.01 m/s 미만은 정지물). 새로 나타난 장애물은 두 번째 읽기까지 정지물로 취급됨. 검사는 각 점의 **그 시각** 장애물 위치로(해석적, `moving_collision`), 계획 월드에는 `sweep_time`(1 s) 동안 쓸고 갈 자리의 복사본 4개 — 지금 팔 구에 겹치는 복사본은 뺌(`clear_of`, 없으면 `INVALID_START_STATE_WORLD_COLLISION`) |

- **재계획기 예열**(`warm_replanner`, 기동 시): 한 프로세스의 첫 재계획은 MotionGen 워밍업이 안 거친 경로(시작 속도, 새 목표)를
  타서 0.27 s — `lead`(0.2 s)보다 길어 늦고, 이어서 `FINETUNE_TRAJOPT_FAIL` → `too close` 정지. 기동 때 지금 자세에서 속도 0.05로
  3 cm 앞 목표를 한 번 재계획(0.10 s) → 이후 첫 재계획 0.11~0.14 s, 기동 직후 첫 예제 3/3

함정(모두 실제로 겪음):
- NumPy 열 fancy indexing(`q[:, idx]`)은 Fortran 순서 → cuRobo `joint_vec` 오류. `np.ascontiguousarray` / `.contiguous()`
- 재계획(0.2~0.3 s) 동안 feedback 콜백이 안 돌아 `now`가 옛 값 → `now()`가 먼저 `spin_once`로 비우고 경과 시간을 더함
- **`stamp(t)`가 1.9999999999를 1.0 s로 만들던 결함**(초를 먼저 내림하고 나노초를 반올림 → 1e9가 0으로 감김) → 시간이 1 s 뒤로 가는 점이 생기고, 드라이버는 그 간격(1.05 s)만큼 잠들어 **스트림 타임아웃**. 누적 덧셈으로 시간을 만든 재개 궤적에서 드러남. 총 나노초를 먼저 반올림하도록 수정, 회귀 테스트 있음
- `MpcSolverConfig`는 `tensor_args`를 키워드로만 받음. `RobotWorldConfig`는 `collision_cache`가 아니라 `n_meshes`/`n_cuboids`

격리 시뮬 실측 (m_1_2 right_arm, 수정된 드라이버):

| 경우 | 결과 |
|---|---|
| 30 cm 위로 가는 중 경로 위 6 cm 상자 (스크립트, 3회) | 3/3 DONE, 교체 구간 관절 속도 0.19~0.33 rad/s(정지 없음), 최소 간격 0.113~0.116 m(접촉 0.106), 최종 오차 ≤ 1.8 mm |
| 튜토리얼 §4.4 ④ 명령 그대로 (6 s 이동, 3회) | 3/3 DONE, 재계획 0.18~0.32 s, 0.15~0.30 s 앞에서 교체 |
| 같은 상자, 기본 속도(2 s) 또는 1 s 늦게 놓음 | 충돌까지 0.3 s 미만 → `too close`로 정지 (의도한 동작) |
| 목표가 막힘 | `IK_FAIL` → 정지 |
| 0.1 m/s로 경로를 가로지르는 상자 (5 s 이동, 설정별 3회) | 전부 `too close`로 **정지**, 접촉 없음. 기본: 재계획 1~2회, 간격 0.126~0.144 m / 가벼운 재계획(4·24·100, lead 0.2, replan_time 0.15): 3~6회 연속 교체, 0.111~0.123 m / + horizon 1.0: 2~10회, 0.131~0.191 m. 새 경로마다 0.1 s 안에 다시 충돌 |
| 정지 상자, 가벼운 재계획 | 3/3 DONE, 재계획 0.09~0.13 s, 간격 0.114~0.126 m |
| **대기 방식** — 0.1 m/s로 가로지르는 상자 (5 s 이동, 3회) | 3/3 DONE: 0.1 s 제동 + 0.5~1.0 s 후퇴 → 1.6~2.2 s 대기 → 원래 경로로 재개. 최소 손목–상자 중심 0.114~0.126 m(접촉 0.106) |
| 대기 방식 — 가로지르다 경로 위에서 멈추는 상자 (3회) | 3/3 DONE: 대기 → 멈춘 뒤 대기 자세에서 재계획(0.18~0.32 s) → 돌아감. 0.112~0.121 m |
| 대기 방식 — 기본 속도(2 s), 15 cm 옆에서 가로지름 (2회) | 2/2 DONE(한 번은 대기 2회). 10 cm 옆(나타날 때 이미 팔에 닿음) 2회는 정지 |
| 대기 방식 — 경로 위를 0.02 m/s로 따라 올라가는 상자 (2회) | 5 s 대기 후 FAILED / 상자가 목표를 막아 IK_FAIL — 둘 다 정지 |
| 대기 방식 — 정지 상자 회귀 (3회) | 3/3 DONE (AVOIDING, 0.113~0.116 m) |
| **현재 기본값**(가벼운 재계획)으로 튜토리얼 §4.4 ④⑤⑥ 명령 그대로 (각 3회) | ④ 3/3 DONE(재계획 0.10~0.13 s) / ⑤ 3/3 DONE(대기 1.1~2.0 s 후 원래 경로) / ⑥ 3/3 DONE(대기 후 재계획 0.10~0.14 s로 돌아감), 복귀 전부 DONE |
| 드라이버 교체 시 속도 유지 | 원래 드라이버 0%(멈춤) / 수정 73~96% |
| 드라이버 시작점 검사 | 0.5 rad 떨어진 교체 → 거부 + 실행 중 궤적 중단(팔꿈치 0.235 → 0.007 rad/s) |

새 목표 교체 실측(예제 21, 30 cm 위로 가는 중 1 s에 15 cm 앞 목표): 이동 4 s에서 4/4 멈추지 않고 교체(재계획 0.07~0.19 s,
0.06~0.17 s 앞에서), 이동 2 s에서는 3/3 움직이는 상태 재계획이 `FINETUNE_TRAJOPT_FAIL` → 멈춘 뒤 정식 계획으로 도착. 모두 새 목표
0.2 cm 안. 재계획기(4·24·100)는 짧은 우회용이라 빠른 시작 속도에서 먼 목표를 자주 놓침.
`max_replans: 0`으로 예제 20: `still blocked after 0 new paths`로 정지(의도).

2026-10-01 재확인(사용자 질문 — 예전 "움직이는 물체 방향으로 새 경로가 생겨 계속 막히는" 현상): 그 현상은 움직이는 장애물은 돌아가지
않고 멈춰 기다리는 정책으로 해결된 것이고 P3에서 바꾸지 않음. 멈춘 뒤의 새 길도 장애물의 예측 위치와 시간을 맞춰(±0.3 s) 비어 있을
때만 탐. 팔 경로를 따라 같은 방향으로 0.02 m/s로 올라가며 막는 상자: 3/3 쫓아가지 않고 바로 이유와 함께 정지(`IK_FAIL` 또는
팔 쪽으로 옴). 예제 19 7/8, 20 5/7 → 20의 실패 2건은 대기 자세에서 새 길이 `INVALID_START_STATE`(상자에 너무 붙음)로 바로 포기한
것 → 한 번 물러난 뒤 다시 계획하도록 고침. 이후 20 6/6, 19 2/2(그 분기는 이번엔 나오지 않아 효과는 미확인).

**움직이는 장애물을 돌아가지 않고 기다리는 이유**: MotionGen은 정적 월드에서 계획하므로, 쓸고 갈 자리를 피한 경로도 시간을 맞춰 보면 0.8~1.05 s
뒤에 부딪히고 앞 구간(lead-in)이 스칩니다. 재계획을 2배 빠르게 하고 `horizon`을 1.0 s로 늘려 연속 재계획을 최대 10회까지
이어 가도 9/9 정지 — 반응 속도가 아니라 새 경로가 장애물의 이동을 모르는 것이 원인(위 표 "가로지르는 상자" 행은 그 측정).
그래서 움직이는 동안에는 기다리고, 멈추면 정지물로 돌아갑니다(사용자 결정).

### 6.2. 추적 모드 (`tracking.py`, `tracking` 구역, C60)

점대점(목표마다 경로 전체 계획 + FJT)은 교체에 0.2~0.3 s라 초당 3~5회가 한계라, 계속 바뀌는 목표(마커)는 따로 둡니다.

```
set_tracking(true) (std_srvs/SetBool, 점대점 이동 중이면 거부) → run 루프가 track()으로:
  측정 관절로 추종기(method: mpc → Tracker, ik → Servo) 시작, 임피던스 설정 전달(apply_impedance),
  stream_control ON(이미 켜져 있으면 끄지 않음)
  rate(50 Hz)마다: 장면(정지물 + 움직이는 것의 쓸고 갈 자리)·붙인 모듈 갱신 →
    새 목표면 도착 시각으로 필터 갱신(TargetMotion: 위치·속도·가속도) →
    매 틱 lead(0.12 s) + 추종기 자체 지연(mpc 0.3 s) 앞을 겨눔(최대 lead_max 0.1 m, 정지한 목표는 앞서 겨누지 않음) → 추종기 목표·속도 갱신
    추종기 한 걸음(자기 명령 상태에서) → stream_joint(계획 그룹 부위만, minimum_time = 1/rate)
    추종기가 못 따라가는 이유(follower.note)가 바뀌면 TRACKING: <이유> / following again
    측정 관절을 command_delay(0.1 s) 전 명령과 비교, resync_tolerance(0.15 rad) + 0.1 s × 명령 속도를 5틱 연속 넘으면 측정값에서 다시 시작
  stale_after(0.5 s) 목표 없음 → TRACKING: holding (마지막 목표 유지, 앞서 겨누기 해제)
set_tracking(false) → READY, 켰던 스트림만 끔
```

| 측정 (격리 시뮬, m_1_2 right_arm) | 결과 |
|---|---|
| MPC 한 걸음 (`particle_opt_iters`) | 1회 6.4~8 ms / 4회(기본) 24 ms — 수렴은 비슷(10 cm를 2 s에 0.2 cm) |
| 팔이 `stream_joint`를 따라가는 정도 | 진폭 그대로, **약 0.1 s 늦음** — 관절 하나를 사인으로 흔들어 명령과 측정이 가장 잘 겹치는 시간차: 주기 0.5~4 s에서 95~125 ms, 50·100 Hz와 `minimum_time` 0.005~0.05 s에서 같음(0.1 s면 150 ms). 처음엔 실행기 안에서 0.2 s로 봤음. 지금 명령과 비교하면 빠를 때 거짓 재시작 → `command_delay` |
| MPC만(상태 = 자기 명령) 원 추적(r 5 cm, 4 s) | 평균 2.5 cm 뒤짐. `cspace` 비용·`step_dt`는 영향 없음, 목표를 0.3 s 앞서 주면 0.6 cm |
| **Python 액션 클라이언트 50 Hz** | 루프가 16.5 Hz(틱마다 `spin_once` 30 ms): 드라이버 액션 서버가 끝난 goal을 `result_timeout`(15분) 동안 쥐고 상태 배열로 계속 발행 → **드라이버 수정**: `stream_joint`·`stream_cartesian`은 2 s. 이후 50.0 Hz, `spin` 0.1 ms |
| 로봇 원 추적(r 5 cm, 4 s) | 수정 전 평균 7.7 cm → 수정 후 **1.9 cm**(최대 2.9) |
| 10 cm 한 점 | 1.5 s에 0.1 cm |
| 22 cm 앞 벽 뒤 목표 | 손목 구가 벽 2.2 cm 앞에서 멈춤(손목 구 r 7.6 cm가 공구 프레임 7 cm **앞**에 있음 — 12 cm 앞 벽은 처음부터 겹쳐 MPC가 물러났음) |
| 가짜 마커 TF 원(r 6 cm, 6 s) → 손 목표(`22_marker_tracking`) → 추적 | 처음 평균 1.5~1.8 cm → 정확도 개선 뒤 `ik` 1.4 mm, `mpc` 5 mm(아래 **정확도**) |


**두 추종기 (`tracking.method`)** — 전신 텔레옵에 2~3 s는 너무 느리다는 지적(10/01)으로 응답을 다시 잼(아래 표는 첫 비교 — 그 뒤 고친 결과는 **정확도** 표):

| 측정 | `mpc` (`Tracker`) | `ik` (`Servo`) |
|---|---|---|
| 한 걸음 계산 | 6.5 ms (반복 1) | 9.8 ms (IK 4 seed + 다음 걸음 충돌 검사) |
| 명령만(로봇 지연 제외): 10 cm 스텝이 1 cm 안 | 0.88~1.42 s | 0.26 s |
| 명령만: r 5 cm 원 4 s / 2 s / 1 s 한 바퀴에서 뒤처짐 | 2.3 / 4.6 / 6.6 cm (0.2~0.3 s) | 0.00 / 0.06 / 0.70 cm (0~22 ms) |
| 시뮬 로봇(TF로 잰 손): 10 cm 스텝 | 1.27~1.37 s | 0.67~0.71 s (스텝 목표에 앞서 겨누기가 넘겨 겨눈 뒤 돌아오는 시간 포함) |
| 시뮬 로봇: 원 4 s / 2 s 한 바퀴 | 1.7 / 8.7~9.7 cm | 0.4 / 1.5 cm (`lead` 0.15) |
| 22 cm 앞 벽 뒤로 미끄러지는 목표 | 벽 2.2 cm 앞에서 멈춤 | 손목 구가 닿는 지점(7.2 cm 전진)에서 멈춤, `TRACKING: the target has no collision-free joint solution …`, 목표가 돌아오면 `following again` |

**정확도 (10/01, "최소 1 cm 안으로")** — 기본을 `ik`로 바꾸고 목표 처리와 서보를 고침. 측정: 마커 7을 `base` TF로 30 Hz
발행(진짜 위치를 아는 가짜 마커) → `22_marker_tracking` → 시뮬 로봇, 손(TF `ee_right`)이 "지금 있어야 할 자리"에서 벗어난 거리.

| 손 오차 (mm, 평균 / 최대) | 정지 | 원 r 6 cm, 6 cm/s | 원 r 6 cm, 13 cm/s | 10 cm 건너뛴 뒤 1 cm 안 |
|---|---|---|---|---|
| 고치기 전 (`mpc`, 5 mm 넘게 바뀔 때만 목표 전송) | 0.7 / 1.1 | 18.3 / 25.7 | 53.6 / 73.3 | 2.5 s |
| 고친 뒤 `ik` (기본) | 0.1 / 0.2 | 1.4 / 2.9 | 7.5 / 10.2 | 0.38 s |
| 고친 뒤 `ik`, 마커에 1 mm 노이즈 | 0.8 / 1.5 | 3.2~3.7 / 9~9.6 | 8.4~9.2 / 15 | 0.4~0.5 s |
| 고친 뒤 `ik`, 노이즈 + 마커가 60 ms 늦게 옴 | 0.8 / 1.6 | 5.0 / 11.1 (60 ms 뒤처짐) | 10.3 / 18.8 | 0.5 s |
| 고친 뒤 `mpc` | 1.1 / 1.6 | 5.1 / 9.9 | 28.3 / 47.2 | 1.8 s |

무엇이 오차였고 어떻게 고쳤나:

- **예제가 목표를 5 mm 넘게 바뀔 때만 보냄**(점대점용 문턱) → 마커 표본마다 하나씩(TF stamp가 바뀔 때). `hand.min_distance`·
  `hand.min_angle`·`hand.repeat_after` 삭제. 정지한 마커도 30 Hz로 보내므로 `holding`/`following again`이 번갈아 뜨던 것도 없어짐
- **속도 추정이 틱 시각 기준**: 목표는 33 ms 간격, 틱은 20 ms 간격이라 간격이 20/40 ms로 읽혀 속도가 ±40% 흔들림 → 목표가
  **도착한 시각**(`pending_at`)으로 계산, 앞서 겨누기도 목표가 올 때만이 아니라 **매 틱**(목표의 나이만큼 더, 최대 `CARRY_ON` 0.1 s)
- **앞서 겨누기가 노이즈를 증폭**: 1 mm 노이즈 × (차분 속도 × `lead`)로 정지한 마커에도 손이 7.5 mm(최대 14) 흔들림 →
  `TargetMotion`(g-h-k 필터: 위치·속도·가속도, `tracking.smoothing` 0.75). 정지 판정(`REST_SPEED` 2 cm/s 아래는 앞서 겨누지 않고
  4 cm/s부터 전부) → 정지 0.8 mm. 3 cm 넘게 빗나간 표본(건너뜀)이나 0.5 s 뒤의 표본은 새로 시작. 가속도까지 추정하는 이유:
  원 13 cm/s에서 속도만 쓰면(g-h) 10~14 mm, 가속도까지 쓰면 7.5~9 mm
- **서보가 목표 뒤를 v/2a + 한 틱만큼 따라감**(속도에 따라 30~90 ms) → 목표가 움직이는 속도를 **피드포워드**. 처음엔 IK 해의
  차분을 썼는데 (a) 해가 현재 명령에서 출발해 여유 자유도 쪽으로 자기 움직임을 되먹여 폭주(재동기화 22회), (b) 노이즈를
  1/dt배 증폭. → IK는 **직전 해에서 출발**하고, 피드포워드는 필터된 목표 속도를 **야코비안**(`Servo.jacobian`: FK 8개를 한 번에,
  차분)의 감쇠 최소제곱(`joint_rates`, λ 0.05)으로 관절 속도로 바꿔 씀
- **서보가 작은 변화까지 한 틱에 쫓아감**(오차/dt) → 노이즈 없이도 관절 가속도 평균 4~8 rad/s²로 떨림 → 목표 근처에서는
  거리에 비례(`SERVO_GAIN` 15 /s), 멀리서는 "설 수 있는 속도"(둘이 a/gain²에서 같은 기울기로 만남). 관절 가속도 0.2~0.9 rad/s²
- **IK seed 4개 중 다른 팔 모양의 해가 뽑힐 수 있음** → 성공한 해 중 직전 해와 가장 가까운 것
- **재동기화가 빠른 관절에 잘못 걸려 연쇄**(측정값에서 정지 상태로 다시 시작 → 더 뒤처짐 → 또 걸림, 5회 중 1회 재현, 손 187 ms
  뒤처짐) → 허용 = `resync_tolerance` + 0.1 s × 명령 속도, 5틱 연속일 때만
- `lead` 0.15 → **0.12**(0.15에서는 손이 37 ms 앞섬), `mpc`는 추종기 지연 0.3 s를 코드가 더함(`Tracker.lag`) — 방식을 바꿔도
  `lead`를 다시 맞출 필요 없음

**노이즈가 큰 카메라에서의 떨림 (10/01, "실제로 해 보니 팔이 계속 흔들린다 — MPC로 안 켜서인가")** — 방식 탓이 아니었음. 마커
좌표 노이즈를 키워 재 보면 정지한 마커에서 손이 흔들리는 폭(표준편차)이 노이즈 1 mm에서는 `ik`·`mpc` 모두 0.9 mm지만, **3 mm에서는
`ik` 10~11 mm, `mpc` 13~16 mm**: 속도 추정의 노이즈가 "정지" 문턱(2 cm/s)을 넘으면 앞서 겨누기가 켜져 노이즈를 증폭. 고친 것(`TargetMotion`):

- **정지 문턱이 노이즈를 따라감**: 표본이 예측에서 빗나간 정도(`noise`, 지수 평균)로 카메라 노이즈를 재고, 그 노이즈만으로 생길
  속도의 `NOISE_SPEEDS`(2)배보다 빨라야 "움직임"(1 mm면 2.4 cm/s, 3 mm면 7 cm/s부터, 그 1.5배에서 전부)
- **정지한 목표는 최근 표본의 평균**(`REST_AVERAGE` 0.15 s)으로 — 정지 중에는 필터된 위치의 떨림도 팔에 안 감
- 새로 시작한 목표는 표본 `SETTLE_SAMPLES`(5)개 전에는 앞서 겨누지 않음(처음 두 표본의 차로 속도를 잡던 것도 없앰 — 노이즈 1 mm가
  7 cm/s로 보여 1 cm 튀었음). 1 ms 안에 몰려 온 두 표본은 둘째를 무시(전에는 새로 시작해 앞서 겨누기를 잃고 2~3 cm 튀었음)

| 손 (격리 시뮬, `ik`) | 정지 중 떨림(표준편차) | 원 6 cm/s 평균/최대 | 원 13 cm/s | 10 cm 건너뛴 뒤 1 cm 안 |
|---|---|---|---|---|
| 노이즈 0 | 0.08 mm | 1.8 / 4.3 mm | 8.2 / 11.8 | 0.47 s |
| 노이즈 1 mm | 0.84 (고치기 전 0.94) | 4.1 / 10.2 | 10.4 / 18.5 | 0.44 s |
| 노이즈 3 mm | **1.35 (고치기 전 10.2)** | 17.0 / 35 (전 12.0 / 36) | 16.5 / 35 (전 15.2 / 26) | 0.41 s (전 4.9 s) |

대가: 노이즈가 큰 카메라에서는 느린 목표(문턱 아래)를 "정지"로 보고 0.2~0.3 s 늦게 따라감. `tracking.smoothing`을 올리면 문턱이
내려감(속도 이득 h가 줄어서).

남는 한계: 로봇이 명령을 약 0.1 s 늦게 따라가고(시뮬), 그만큼은 예측으로만 메울 수 있음 → 방향이 급히 바뀌는 목표(원 13 cm/s,
가속도 0.27 m/s²)에서 7~9 mm, 그보다 빠르면 1 cm를 넘음. 카메라·검출 지연(중앙값 창 5프레임도 지연)은 `lead`에 더해야 함.
실제 카메라·로봇에서는 보정(카메라 내부 파라미터, 장착 위치 TF)이 정지 오차를 정함 — 미검증.

- **MPC가 느린 이유는 관절 한계가 아님**: XRDF `acceleration_limits`·`jerk_limits`를 1/10 → 5/100 → 15/500으로 올려도 5 이상은 결과가
  완전히 같음(관절 최고 속도 0.48 rad/s). cuRobo `particle_mpc.yml`의 MPPI(입자 400, 지평 30 스텝, 한 번에 조금씩 평균 이동)가
  한계. `override_particle_file`로 하나씩 바꿔 봄(반복 2회 기준, 원 2 s 뒤처짐 4.4 cm): `init_cov` 0.5 → 3.5, `stop_cfg` 끔 → 3.5,
  자세 가중치 [150, 2000, 30, 40] → 3.3, `smooth_weight` 낮춤 → 4.2, `null_space_weight` 0 → 4.5 cm. 반복 4회(19.7 ms, 50 Hz 한계)도
  3.6 cm. 즉 조정으로는 0.2 s 아래로 못 내려감
- **`Servo`**: 매 걸음 cuRobo `IKSolver.solve_single`(seed·retract = 직전 해 → 가까운 해, 자기·장면 충돌 없는 해만 성공) →
  `servo_step`이 관절마다 속도(`max_speed`)·가속도(`max_acceleration`) 안에서, 목표 속도를 더해(위 **정확도**) 한 걸음 →
  그 걸음을 `check_valid`로 검사, 충돌이면 안 감. 한 걸음 10 ms. 앞을 내다보지 않으므로 **장애물을 돌아가지 않고 앞에서 멈춤**.
  IK 실패(목표가 장애물 안, 닿지 않음)면 그 자리에서 멈추고 이유를 상태로 알림
- cuRobo IK는 처음 받은 목표 텐서를 버퍼로 잡고 이후 목표를 거기에 복사함 → 목표를 매번 새 텐서로 줌(같은 텐서를 다른 데서
  쓰면 값이 바뀜 — 측정 스크립트에서 겪음)
- 전신(몸통 + 양팔 20관절, 양손 목표 `link_poses`) IK도 12 ms, 원 추종 뒤처짐 0 — 계산은 텔레옵에 충분. 실행기 인터페이스(손
  하나의 4×4)가 양손 목표를 받게 하는 것은 하지 않음 — 해볼 만한 것으로만 기록(Notion CPS C64, 10/01 결정)
- 추적 모드도 `impedance` 설정을 따름: 스트림을 켜기 전에 `set_trajectory_impedance`(드라이버가 `stream_joint`에도 적용).
  실측: 임피던스 500/50로 MPC 원 4 s 2.3 cm(위치 1.7 cm)

- MPC는 실행기 프로세스 안(`Tracker`, GPU 약 270 MB, 기동 시 워밍업 0.8 s — `tracking.prepare`). 이름은 `self.follower`
  (`self.tracker`는 움직이는 장애물 추적기)
- MPC 상태는 자기 명령을 이어 씀(측정값을 쓰면 0.1 s 지연이 그대로 되먹임됨). 측정은 이탈 감시에만

### 7. 플래너 설정

`rby1_cumotion/config/cumotion.yaml`이 **모든 기본값의 유일한 원천**입니다: `robot`(7), `motion`(7), `avoid`(12), `planner`(18).
`settings.py`(robot·motion·avoid)와 `planner_params.py`(planner)는 이름·타입·범위만 갖습니다. 노드를 따로 실행해도
같은 파일을 읽습니다(`-p config:=`로 다른 파일).

- 런치 인자로 하나씩 덮어쓰기: `ros2 launch rby1_cumotion demo.launch.py trajopt_finetune_iters:=150` (planner 18종,
  `group` `model` `driver_namespace`)
- 다른 파일: `config:=/경로/파일.yaml`
- 모르는 구역·키, 빠진 키, 범위 밖 값은 **기동 시** 멈춤(`prepare`가 robot, planner 런치가 planner, 실행기가 motion·avoid 검사). `avoid`는 기동 시에만 읽음(실행 중 변경 불가)

| 바꿀 수 있는 것 | 방법 |
|---|---|
| 파이프라인 `isaac_ros_cumotion` ↔ `ompl` | 런치 `pipeline:=ompl` (예제에도 `-p pipeline:=ompl`) |
| 궤적 최적화 | `num_trajopt_time_steps` `num_trajopt_seeds` `num_graph_seeds` `max_attempts` `trajopt_finetune_iters` `interpolation_dt` |
| cuRobo IK | `ik_num_seeds`(32) `ik_position_threshold`(0.005) `ik_rotation_threshold`(0.05) `ik_opt_iters`(0=기본) `ik_particle_opt`(true) — 기본값은 cuRobo 기본값과 같음 |
| 월드 | `collision_cache_mesh/cuboid` `add_ground_plane` `read_esdf_world` `voxel_size` |

| 바꿀 수 없는 것 | 이유 |
|---|---|
| IK 알고리즘(KDL·TRAC-IK) | cuMotion은 cuRobo GPU IK만. `kinematics.yaml`은 OMPL에만 적용 |
| 외부 IK 해 고정 | 플러그인이 관절 목표도 FK→포즈→cuRobo IK로 다시 풂 |
| 플러그인 결과 대기 5 s | 하드코딩 |

`time_dilation_factor`: 비우면 MoveIt 스케일링, 값을 주면 `override_moveit_scaling_factors`가 함께 켜짐. 실행 속도는 어차피 `motion` 구역이 정합니다.

### 8. 로봇 없이 개발 — `hardware:=mock`

```bash
ros2 launch rby1_cumotion cumotion.launch.py hardware:=mock     # 드라이버 불필요
ros2 run rby1_cumotion check_plan
```

ros2_control mock 하드웨어로 계획만 합니다. `prepare`는 로봇을 건드리지 않고 번들만 고르며(기종은 `robot.model`,
비우면 `$RBY1_MODEL`/`m_1_2`), 자세는 번들에 저장된 자세. 실행기는 켜지 않습니다.
사용자 흐름에서는 쓰지 않습니다(드라이버가 있으면 시뮬레이터로 충분).

### 9. 성능 측정

```bash
ros2 run rby1_cumotion benchmark --ros-args -p runs:=10 -p output:=/tmp/bench.json
```

다음을 만족하지 않으면 측정을 거부합니다(PID까지 알려줌). 모드는 떠 있는 런치를 따릅니다.

| 불변식 | mock | driver |
|---|---|---|
| `move_group` | 1 | 1 |
| `cumotion_planner_node` | cuMotion 1 / OMPL 0~1 | 같음 |
| `ros2_control_node` | 1 | 0 |
| 로봇 상태 | 그룹 컨트롤러 active | 드라이버 관절 상태가 살아 있음 |

- 호스트에서 ROS 프로세스를 강제 종료하는 테스트를 반복하면 FastDDS 공유 메모리 잔재가 쌓여 새 구독자가 데이터를 못 받습니다
  (`RTPS_TRANSPORT_SHM Error ... open_and_lock_file failed`). `fastdds shm clean`이 쓰이지 않는 것만 지웁니다.
- 노드 개수는 **컨테이너 전체**를 셉니다. 다른 세션이 떠 있는 컨테이너에서는 돌리지 마십시오.
- 정리는 프로세스 그룹 단위로(`pkill -f "ros2 launch"`는 자식을 남김):

```bash
for p in $(ps -eo pid,args | grep -E "[r]os2 launch rby1_cumotion|[m]ove_group|[c]umotion_planner_node|[r]os2_control_node" | awk '{print $1}'); do
    kill -TERM -$(ps -o pgid= -p $p | tr -d ' ')
done
```

- 빈 공간 짧은 이동은 OMPL이 약 3배 빠릅니다. 이 과제로는 cuMotion 채택을 판단할 수 없습니다(장애물 환경 벤치 필요). 측정값은 노션 process에 있습니다.

### 10. 회귀 테스트

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_driver_ws/install/setup.bash       # 모델 픽스처·rby1_msgs
cd ~/isaac_ros_ws/src/rby1_isaac_ros/rby1_cumotion
python3 -m pytest test/ -q -p no:cacheprovider -p no:anyio
```

`-p no:anyio` 필수(없으면 수집 단계가 깨져 0 passed). 호스트에서 1개 skip(설치된 cuMotion 노드와의 대조는 컨테이너 전용).

| 파일 | 대상 |
|---|---|
| `test_model.py` | 번들 생성·적재, 축 정준화, SDK 캡슐, 그룹, 실행용 번들 |
| `test_examples.py` | 역할 분리, 환경 불변식, rclpy 속성·로그 심각도 규칙 |
| `test_execution.py` | 재시간화, 4×4 검증, 드라이버 명령, 실행기 기동 검사, 런치 모드 |
| `test_planner_params.py` | YAML 원천, 덮어쓰기, 범위, IK 패치 일치 |
| `test_image.py` | 이미지 레이어 규칙, 패치 |
| `test_avoidance.py` | 장면 변환, 속도 추적, 쓸기, 표면 거리, 시간 맞춘 충돌, `from_start`, `splice`, 제동·후퇴·재개, `stamp` |
| `test_motion_validation.py` | 궤적 검증, 고정 관절, 계획 실패 메시지, 워밍업 방향 |
| `test_attached.py` | 붙인 모듈의 덮는 구(상자·원통 모서리까지), 손 끝·머리·그 밖 분류, `free_objects`로 빼기와 MoveIt 허용 행렬, 구 100개 한도 |
| `test_tracking.py` | 목표 필터(앞서 겨누기와 최대 거리, 오래된 표본, 끊긴 뒤·건너뛴 뒤 재시작, 정지 판정, 노이즈 억제, 등속·회전 추종), 모드 전환(이동 중 거부), IK 서보 한 걸음(목표를 넘지 않음, 속도·가속도 한계, 작은 변화를 쫓지 않음, 목표 속도 피드포워드), 야코비안 관절 속도(특이점에서 작게), `tracking.method` 검사 |

드라이버 저장소: `rby1_moveit_scene` gtest 13개(C++, `colcon test --packages-select rby1_moveit_scene`), `rby1_examples` 37개(`pub_cartesian_pose` 8, 목표 예제 10, 마커 예제 19), `rby1_moveit_executor` gtest 12 + pytest 3(C++, `colcon test --packages-select rby1_moveit_executor`), `rby1_moveit_objects` gtest 6, `rby1_additional_tools` gtest(카메라 모델). AprilTag는 [AprilTag §5](#5-검증과-테스트).
`rby1_examples` 전체를 돌리면 기존 예제 파일들의 flake8/pep257 검사 2건이 실패합니다(이번 변경과 무관).

#### 목표 예제 (드라이버 저장소 `rby1_examples` 15~21)

튜토리얼 §4의 예제는 `ros2 run rby1_examples NN_moveit_<이름>`입니다(15~21, 기존 예제 번호에 이어서). 22·23은 마커 예제([AprilTag §3](#3-마커-예제-드라이버-저장소-rby1_examples-22-23)).

- **목표 인터페이스는 플래너와 무관**: 두 실행기(cuMotion `target_executor`, 드라이버 저장소 `rby1_moveit_executor`)가 같은
  노드 이름 `/rby1_target_executor`, 토픽 `/rby1/target_pose`·`/rby1/target_status`, 파라미터 `duration`·`minimum_time`을 씀.
  한 도메인에 계획 스택은 하나뿐(각 런치가 다른 `move_group`을 거부)이므로 예제는 어느 쪽인지 몰라도 됨
- 실행기는 읽기 전용 파라미터 `watches_while_moving`(cuMotion: `avoid.enabled`, MoveIt: `false`)을 둠 — 움직이는 중을
  보는 예제 18~21은 이것이 거짓이면 안내하고 끝냄. 분기 파라미터 없음
- 공통 단계는 `moveit_target.py`(`TargetExample`: 목표 보내기·결과 확인, 상자 넣기·밀기·빼기, 준비 자세). 장면은
  `/apply_planning_scene`을 직접 부름(`rby1_moveit_scene`에 의존하지 않음)
- 이동 시간은 예제가 바꾸지 않음 — 18~21 전에 `ros2 param set /rby1_target_executor duration …`(튜토리얼)
- 시작 전 오른팔을 **준비 자세 관절**로(드라이버 `robot_joint`, 0.02 rad 안이면 건너뜀). 7축 팔은 왕복 후 같은 손 자세에서 다른
  팔 모양으로 돌아오고(MoveIt+OMPL은 1~3 rad까지), 그 차이만으로 장애물 예제 결과가 갈렸음
- 예제 기하: 가로지르는 상자는 준비 자세 팔뚝 위 7 cm(처음 2 cm 배치는 타이밍에 따라 성공·실패가 갈림), 막힌 목표는 **손목이
  갈 자리**의 상자(손 자세가 손목 위치를 정하므로 어떤 팔 모양·플래너로도 못 피함; 선반 배치는 MoveIt 메시 충돌 모델에선 피해 감)

격리 시뮬 실측(각 3회, 번호 예제): cuMotion 15~20 18/18 `EXAMPLE_DONE`, MoveIt 실행기 15~17 9/9, 18~20은 안내 후 종료.
예제 21(새 목표 교체): 이동 4 s에서 4/4 멈추지 않고 교체, 이동 2 s(`duration 0`)에서 3/3 "멈춘 뒤 서서 계획" — 모두 새 목표 0.2 cm 안.

### 11. 레퍼런스

#### `cumotion.launch.py` / `demo.launch.py` 인자

비어 있는 인자는 설정 파일 값을 씁니다.

| 인자 | 기본값 | 뜻 |
|---|---|---|
| `config` | `''` (설치된 `cumotion.yaml`) | 설정 파일 |
| `model` | `''` | 기종(`m_1_2`). 비우면 드라이버에서 읽음. 주면 드라이버와 대조 |
| `model_directory` | `''` | 번들 경로 직접 지정 (`model`보다 우선) |
| `group` | `''` | `robot.group` 덮어쓰기 |
| `driver_namespace` | `''` | `robot.driver_namespace` 덮어쓰기 |
| `hardware` | `driver` | `mock`은 개발용(§8) |
| `pipeline` | `isaac_ros_cumotion` | `ompl` |
| `start_executor` | `true` | 실행기 (driver일 때만) |
| `rviz` | cumotion: `false` / demo: `true` | |
| `start_moveit`, `start_state_publisher`, `start_planner` | `true` | |
| planner 설정 18종 | 설정 파일 | §7 |

#### 노드 파라미터

| 노드 | 파라미터 | 기본값 |
|---|---|---|
| 공통 (`check_plan`, `benchmark`, `target_executor`) | `model_directory`, `group` | `''` (실행용 번들) |
| 〃 | `pipeline` | `isaac_ros_cumotion` |
| 〃 | `offset_xyz` | `[0.03, 0.0, 0.0]` — check_plan·워밍업 이동량, 크기 ≤ 0.1 |
| 〃 | `planning_time`, `server_timeout`, `state_max_age` | `30.0`, `120.0`, `2.0` |
| 〃 | `locked_joint_tolerance` | `0.01` rad |
| 〃 | `warmup`, `warmup_attempts` | `true`, `6` (끄지 말 것) |
| `target_executor` | `config` | `''` (설치된 `cumotion.yaml`) |
| 〃 | `driver_namespace`, `enable_robot` | 설정 `robot` 구역 |
| 〃 | `duration`, `minimum_time`, `linear_velocity_limit`, `angular_velocity_limit`, `step`, `hold`, `endpoint_tolerance` | 설정 `motion` 구역 — 실행 중 변경 가능 |
| 〃 | `target_topic`, `status_topic` | `/rby1/target_pose`, `/rby1/target_status` (노드 이름 `rby1_target_executor`, MoveIt 실행기와 같음) |
| 〃 | `watches_while_moving` (읽기 전용) | `avoid.enabled` |
| `prepare` (명령행) | `--config --hardware --model --model-directory --group --driver-namespace` | 설정 `robot` 구역 |
| `benchmark` | `hardware`, `runs`, `settle`, `output` | `''`(런치 따름), `10`, `0.5`, `''` |
| `pub_cartesian_pose` | `ref_link`, `target_link`, `offset_xyz`, `matrix`, `wait_for_result`, `timeout` | `base`, `ee_right`, `[0,0,0]`, 미설정, `true`, `120.0` |

#### 알려진 한계

| 항목 | 상태 |
|---|---|
| 실기체 | 미검증 (코드 경로는 시뮬레이터와 같음) |
| 양팔 동시 Cartesian | 불가 — 플러그인이 `plan_single`만 호출 |
| v1.0 `ee_*` | SDK와 드라이버 URDF가 46.1 mm 다름 → `pub_cartesian_pose`의 현재 자세 기준 offset이 어긋남 |
| 움직이는 장애물 | 돌아가지 않고 기다림. 경로 위 어디에 서도 부딪히면 정지 (§6.1) |
| `planning failed: INVALID_MOTION_PLAN … MoveIt, checking it against its own robot model, finds it in collision` | cuMotion 모델에 없는 링크(공구 프레임 앞의 그리퍼)가 장애물에 닿는 경로 | ✔ | 그리퍼 모듈을 붙이거나(`config:=gripper.yaml`), 닿아도 되는 부위를 `free_objects`에 |
| `nothing on ee_right is in cuMotion's model …` | 손 끝에 붙인 모듈이 없음(팔 모델은 `ee`에서 끝남) | ✔ | 위와 같음, 또는 `robot.body_ends_at_tool: false` |
| 링크에 붙인 물체 | cuMotion은 손 끝에 단단히 붙은 것만(덮는 구, §6), 머리는 덮개 구, 그 밖은 무시. MoveIt(OMPL)은 전부 반영 |
| 이동 시간 | 실제 약 15% 김 (드라이버가 웨이포인트마다 전송 후 잠듦) |
| Jetson Orin, CUDA 12.8+ | 미검증 / PTX 우회 중 |

### 12. 트러블슈팅

사용자 튜토리얼에는 트러블슈팅을 두지 않습니다. **노드와 런치가 실패할 때 원인과 할 일을 로그에 직접 말합니다**(아래
"로그 안내" 열이 ✔인 것). 새 실패를 만들면 메시지 끝에 할 일을 붙이고, 로그로 안내할 수 없는 것(환경·빌드·ROS 자체
오류)만 이 표에 둡니다.

#### 설치·환경 (로그로 안내할 수 없음)

| 증상 | 원인 | 해결 |
|---|---|---|
| 이미지 빌드가 `No model bundles: run docker/make_bundles.sh on the host first` | `make_bundles.sh`를 건너뜀 | 실행 후 다시 빌드 (메시지가 안내) |
| `nvidia-smi`가 `Driver/library version mismatch`, `Failed to initialize NVML` | GPU 드라이버 문제 | [env_setup 10장](env_setup_ubuntu_22_04.md) |
| 패키지 빌드가 `[Errno 17] File exists: ... resource/rby1_cumotion` | 이전 빌드 잔재 | `rm -rf build/rby1_cumotion install/rby1_cumotion` 후 다시 빌드 |
| `ros2 run rby1_cumotion ...`이 `Package not found` | 워크스페이스 빌드 전 | 튜토리얼 2.2 |
| 컨테이너 프롬프트가 `root@…` | `-u admin` 없이 들어옴 | 나가서 `isaac-ros`로 다시. `isaac-ros`가 root로 들어가면 `~/.bashrc`의 함수를 [env_setup 8.2](env_setup_ubuntu_22_04.md)대로(같은 함수가 두 번 있으면 뒤의 것이 쓰임). 기동 중에 root면 `read_robot`이 원인을 로그에 붙임 |
| `RTPS_TRANSPORT_SHM Error ... open_and_lock_file failed` 뒤 토픽이 안 옴 | 강제로 끈 ROS 프로세스들의 공유 메모리 잔재 | 호스트에서 `fastdds shm clean` 후 그 노드를 다시 실행 |
| `Couldn't parse parameter override rule ... Sequence should be of same type` | `pub_cartesian_pose`의 `matrix`에 정수가 섞임. rclpy가 노드 코드 전에 실패 | 모든 값을 `1.0`처럼 소수로 |
| 손은 되돌아왔는데 팔 모양이 비틀림 | 팔을 곧게 편 자세(영점)에서 움직임 (§4.1) | `robot.ready_if_straight` 확인(기본 켜짐). 지금 모양은 `robot_joint` 관절 명령으로 |

#### 드라이버 (호스트 터미널)

| 메시지 | 원인 | 로그 안내 | 해결 |
|---|---|---|---|
| `[CONNECTION ERROR] Failed to connect to robot at address ...` | 시뮬레이터가 안 떴거나 `robot_ip` 틀림 | ✔ (드라이버 자체) | 확인 목록을 따름 |
| `String field 'rb.api.Collision.link2' contains invalid UTF-8` 뒤 종료 | 시뮬레이터가 드물게 깨진 상태 한 프레임 — 수정 전 드라이버는 한 번에 종료 | — | 드라이버 재빌드(`get_state_with_retry`, `state_loss_timeout`) |
| `State read failed (N in a row …)` | 상태 읽기가 잠깐 실패 | ✔ | 한두 번이면 무시, 1초 넘게 이어지면 연결 끊김 |
| `[STREAM TIMEOUT] No stream commands received for 1.05 seconds` | 궤적 점 사이 간격이 1 s 넘음(과거 `stamp()` 결함, T76) 또는 실행기가 멈춤 | — | 궤적 시간 간격의 최대값 확인 |
| `start too far from the robot: joint ... needs X rad/s` (FJT 거부) | 새 궤적의 첫 점이 지금 자세에서 속도 한계로 갈 수 없음 | ✔ | 실행기 쪽 궤적 생성 확인(splice·resume 시작점) |
| `AVOIDING` 때 팔이 멈칫 | 수정 전 드라이버(점별 사전 충돌 검사, 매번 제어 모드 재설정) | — | 드라이버 재빌드 |

#### cuMotion 런치 — 준비 단계 (`PREPARE_FAILED: …`, 컨테이너 터미널)

| 메시지 | 원인 | 로그 안내 |
|---|---|---|
| `Another cuMotion launch is running in this container (pid …)` | 이전 런치가 살아 있음 — 두 개는 GPU 메모리에 안 들어가고 서로의 계획 요청에 답함 | ✔ Ctrl+C |
| `No RB-Y1 driver answers under /rby1 -- start it on the host …` | 드라이버가 없거나 `ROS_DOMAIN_ID`가 다름 | ✔ |
| `Driver under /rby1 did not publish joint_states and robot_state in time. You are root …` | root 셸 (T49) | ✔ |
| `Driver reports robot_version 0.0 …` | 수정 전 드라이버 | ✔ 재빌드 명령 / `model:=` |
| `Driver runs an RB-Y1 'a' but … is m_1_2; leave model out …` | `model:=`/`robot.model`이 로봇과 다름 | ✔ |
| `Emergency stop is pressed; release it and start again` | 비상정지 | ✔ |
| `Control manager is in a major fault. Reset it with: …` | 로봇 고장 상태 | ✔ 리셋 명령 |
| `Arm did not reach the ready pose: … Check that nothing blocks it, or …` | 준비 자세로 가다 막힘 | ✔ |
| `Group … starts in self-collision … Move the robot to another posture …` | 지금 자세가 충돌 모델상 자기충돌 | ✔ |
| `… is out of range` / `unknown … settings` / `missing … settings` | 설정 파일 값·이름 | ✔ 항목·파일 경로 |

#### 계획·실행 (`FAILED: …`, 실행기와 `pub_cartesian_pose`)

| 메시지 | 원인 | 로그 안내 |
|---|---|---|
| `Driver service unavailable … is the driver running on the host, same ROS domain?` | 드라이버 없음 | ✔ |
| `No current robot joint states on /joint_states … Start it (ros2 launch rby1_driver …)` | 기동 뒤 드라이버가 꺼짐. cuMotion은 다시 켜지 않아도 됨 | ✔ |
| `Driver runs an RB-Y1 … restart the cuMotion launch against this driver` | 다른 로봇 기준으로 떠 있음 | ✔ |
| `Joint … is at … but the planner has it locked at … Restart the launch` | 기동 후 그룹 밖 관절이 움직임 (T45, T61) | ✔ |
| `Planner never became ready. Last answer: … Usually an obstacle touches the arm …` | 장애물이 팔에 붙음, 또는 cuMotion 기동 중 | ✔ |
| `planning failed: PLANNING_FAILED … no collision-free motion …` | 닿지 않거나 장애물 — 이유는 cumotion_planner 로그(`IK_FAIL` 등) | ✔ |
| `planning failed: TIMED_OUT … Send the target again; if it keeps timing out, restart …` | 플러그인 5 s 대기 (T2) | ✔ |
| `… is too fast … Use at least X s` | 고정 `duration`이 너무 짧음 | ✔ |
| `Driver rejected the trajectory -- ros2_control may be holding the robot …, or its start is too far …` | 다른 프로그램이 하드웨어 제어 점유, 또는 드라이버 시작점 검사 | ✔ 드라이버 로그 |
| `Driver aborted the trajectory (-1): Stopped: the trajectory sent to replace it was rejected (start too far …)` | 교체 궤적이 거부돼 드라이버가 팔을 멈춤 | ✔ 드라이버 문구 |
| `joints stopped short of the endpoint … Send the target again, slower …` | 스트림 끊김(드라이버 STREAM TIMEOUT) | ✔ |
| `obstacle too close to plan around in time; stopped. …` | 정지물이 팔 바로 앞(`replan_time` 안)에 나타남 | ✔ |
| `no way around the obstacle to the target (IK_FAIL); stopped. …` | 목표까지 길이 막힘 | ✔ |
| `obstacle already touches the arm (INVALID_START_STATE_…); stopped. …` | 장애물이 팔에 붙은 채 나타남 | ✔ |
| `obstacle still in the way after 5.0 s of avoiding; stopped. …` / `moving obstacle still in the way after … of waiting; …` | `avoid.give_up_after` 초과 | ✔ |
| `moving obstacle too close to stop short of it; …` / `… heading for the arm …` | 움직이는 장애물이 너무 가까움 / 경로 위 어디에 서도 부딪힘 | ✔ |
| `new target: no path to it from the moving arm (…); stopped` | 새 목표로 바꾸는데 움직이는 중에도, 멈춘 뒤에도 길이 없음(`HOPELESS`면 이유가 붙음) | ✔ |
| `still blocked after N new paths; stopped. … (avoid.max_replans)` | 멈춘 채 새 길을 `max_replans`번 만들어도 막힘 | ✔ |
| `set_tracking` 응답 `a move is in progress; switch once it is DONE or FAILED` | 점대점 이동 중 | ✔ |
| `FAILED: tracking: the driver refuses stream_joint …` | 1초 동안 `stream_joint`가 전부 거부(궤적·다른 명령이 팔을 잡음, 스트림 닫힘) | ✔ |
| `FAILED: tracking: no current joint states for 1 s` | 드라이버 관절 상태가 끊김 | ✔ |
| `the arm is X rad from the MPC command: carrying on from where it is` (경고) | 팔이 명령을 못 따라감(막힘, 한계) — MPC가 측정값에서 다시 시작 | ✔ |
| `Timed out waiting for a subscriber on /rby1/target_pose -- start a target executor …` (`pub_cartesian_pose`, 예제) | 실행기가 안 떠 있음 | ✔ |
| `Driver aborted the trajectory (-1): Could not send the trajectory to the robot: … command stream failed …` | 드라이버 스트림이 만료된 채 다시 열리지 않음(§4) | ✔ |
| `Driver aborted the trajectory (-2): Limit exceeded: joint … at waypoint 0` | 관절이 한계 값 바로 위에 있음(OMPL이 거기 둠) — MoveIt 실행기는 한계 안쪽 1e-4 rad로 보내도록 수정 | ✔ 드라이버 문구 |
| `EXAMPLE_FAILED: step N (…): expected DONE, got …` (예제) | 그 단계의 실행기 결과가 기대와 다름 — 뒤에 실행기 메시지가 붙음 | ✔ |
| `EXAMPLE_FAILED: this example needs a planner that watches the way while the arm moves` | 18~21을 MoveIt 실행기로 | ✔ |

#### AprilTag·마커 예제 ([tutorial_apriltag.md](tutorial_apriltag.md), [tutorial_cumotion.md 4.7](tutorial_cumotion.md))

| 메시지·증상 | 원인 | 로그 안내 | 해결 |
|---|---|---|---|
| `CAMERA_FAILED: realsense_publisher is not built …` | `ros-humble-librealsense2` 없이 빌드됨 | ✔ | `sudo apt install ros-humble-librealsense2` 뒤 `colcon build --packages-select rby1_additional_tools` |
| `ros2 run … realsense_publisher`가 시작하자마자 `std::bad_array_new_length`, `double free` | Intel 저장소 librealsense2로 빌드됨 — 내장 Fast DDS와 ROS Fast DDS 충돌(§AprilTag 4) | — | `sudo apt install ros-humble-librealsense2` 뒤 다시 빌드(CMake가 ROS 것을 먼저 고름). 그 전에는 `camera.launch.py`로 띄우면 CycloneDDS로 띄움 |
| `N of 30 fps: the camera exposes each frame for … ms (auto exposure, too little light …)` | 어두워서 자동 노출이 길어짐(D405는 fps를 지키는 옵션이 없음) → 영상이 끊겨 보임 | ✔ | 조명, 또는 `realsense.yaml`의 `auto_exposure: false` + `exposure`(33000 µs 아래) |
| `N of 30 fps: the camera delivers frames late (longest gap … ms …)` | USB 케이블·포트, 또는 다른 프로그램이 같은 카메라를 건드림 | ✔ | 케이블·포트, 다른 프로그램 종료 |
| `N of 30 fps: publishing takes … ms a frame` | 구독자가 큰 영상을 느리게 받아 감 | ✔ | 구독자 줄이기, 해상도 낮추기 |
| `no frames from the RealSense: … opened by another program?` | 카메라가 빠졌거나 **다른 프로그램이 같은 카메라를 엶**(한 번에 한 프로그램만) — 실측: 다른 프로세스가 연 순간부터 0 Hz, `rs-enumerate-devices`도 `No device detected` | ✔ | 다른 프로그램을 끄고 `camera.launch.py` 재기동 |
| `the Intel(R) RealSense(TM) … is on USB (2-4) but the kernel's video driver is not attached to it …` | libusb로 카메라를 여는 다른 프로그램이 커널 드라이버(uvcvideo)를 떼어 내고 돌려놓지 않음 — `lsusb`에는 보이지만 `/dev/video*`가 없고 `rs-enumerate-devices`도 `No device detected`. 실측(10/01): `ros2_camera_pub`라는 프로세스가 연 뒤 이 상태 | ✔ | 그 프로그램을 끄고 카메라를 뽑았다 꽂음(또는 `sudo modprobe -r uvcvideo && sudo modprobe uvcvideo`) |
| `no RealSense found -- is it plugged into a USB 3 port` / `could not open WxH @ fps …` | 연결 안 됨, USB 2, 지원하지 않는 모드 | ✔ | `rs-enumerate-devices`로 모드 확인 |
| `the intrinsics are for WxH, the image is … (another aspect ratio)` | 보정 파일과 영상의 가로세로 비율이 다름 | ✔ | 같은 해상도로 보정 |
| 보정 파일을 바꿨는데 마커 거리가 그대로 | 컨테이너의 보정 노드가 처음 받은 카메라 모델을 계속 씀 | — | `apriltag.launch.py` 재기동 |
| `target_tags.yaml`에서 `target_ids`·`size`를 고쳤는데 그대로(예: `[7, 8]`로 바꿨는데 7번만 나옴, 거리가 `size` 비율만큼 틀림) | 런치를 다시 띄우지 않았거나, 패키지를 `--symlink-install` 없이 빌드해 설치 폴더의 **복사본**을 읽음(10/01 사용자 실습: 소스는 `[7,8]`·0.08인데 실행 중인 값은 `[7]`·0.10) | ✔ 시작 로그 `markers: … target ids … (settings: 경로)` | `rm -rf build/rby1_apriltag install/rby1_apriltag` 후 `colcon build --symlink-install …`, 런치 재기동. 당장은 `ids:=7,8 size:=0.08` |
| `/target_marker/pose`가 안 옴 | 카메라 토픽이 안 넘어옴(`ROS_DOMAIN_ID`), `width`·`height`가 영상 크기와 다름, 마커 패밀리·번호(`ids:=`·`target_ids`) | — | 컨테이너에서 `ros2 topic hz /camera/image_raw`·`/tag_detections` 확인 |
| 다른 카메라 드라이버를 붙였더니 검출이 안 됨 | Isaac ROS 노드는 **reliable**로 구독 — best-effort 발행자의 영상은 못 받음 | — | 발행자를 reliable로 |
| 거리가 일정 비율로 틀림 | `size`(검은 사각형 한 변)가 실제와 다름, 또는 보정 없이 `horizontal_fov` 모델 | — | 자로 재서 `target_tags.yaml`의 `size`(또는 `size:=`), 보정 파일 `intrinsics:=` |
| `… has no apriltag: ros__parameters: size / tag_family …` | 예전 형식의 설정 파일(`target_tag_filter` 구역만 있음) | ✔ | `apriltag:` 구역을 추가하거나 `size:=`·`tag_family:=` |
| 원거리에서 자세가 떨림 | 창이 짧음 | — | `target_tags.yaml`의 `window_size` 8~10 |
| `no transform link_head_2 <- camera_optical_frame … run camera.launch.py on this host` | 장착 위치 TF 없음 | ✔ | |
| `head: no marker on /target_marker_7/pose: is it in view …` | 마커 좌표가 안 옴: 마커가 안 보임, 번호·패밀리·크기 불일치, 카메라·검출이 안 떠 있음 (전에는 아무 말 없이 머리만 안 움직였음) | ✔ | RViz Marker view에서 마커 위에 축이 얹히는지 |
| `head: the robot is not enabled (control manager idle) …` | 전원·서보를 안 켬 | ✔ | `ros2 run rby1_examples 06_zero_pose` |
| `head: no head angles yet: is the driver publishing /rby1/joint_states?` | 드라이버 없음 | ✔ | |
| `hand: /rby1_target_executor/set_tracking is not available …` | cuMotion 기동이 없음(MoveIt 실행기는 추적 모드가 없음) | ✔ | cuMotion 기동 후 `READY` |
| `hand: tracking mode refused: a move is in progress …` | 점대점 이동 중 | ✔ | 끝나면 예제가 다시 요청 |
| `hand: no marker target_marker_7 on TF in base yet …` / `EXAMPLE_FAILED: no target_marker_12 on TF in base within 20 s …` | 마커가 안 보임, `ids`에 그 번호가 없음, 로봇 TF(cuMotion 기동)나 카메라 장착 TF(`camera.launch.py`)가 없음 | ✔ | `ros2 run tf2_ros tf2_echo base target_marker_7` |
| `23_marker_shuttle`이 `FAILED: planning failed …`로 끝남 | 마커 아래 지점이 닿지 않거나 장애물·로봇 몸과 겹침 | ✔ | `offset`, 마커 위치 |
| `/rby1/stream_control is not available` / `/rby1/stream_joint is not available` | 드라이버 없음 | ✔ | |
| `stream_control refused: … power on and servo the robot, and turn gravity compensation off` | 전원·서보 꺼짐, 중력 보상 중 | ✔ | `06_zero_pose` 등으로 켬 |
| `head command rejected: …; retrying in 0.5 s` | 팔 궤적 실행 중, 다른 명령이 아직 움직이는 중, 스트림 닫힘 | ✔ | 드라이버 로그의 `[STREAM REJECTED]`가 이유 |
| 드라이버 `[STREAM REJECTED] stream_joint ignored: another command on the stream is still moving other parts.` | 머리 추적 중 `robot_joint` 등 다른 부위 명령이 움직이는 중 — 끝나면 머리 추적이 이어 감 | ✔ | |
| 정지 이미지(`camera:=file`)로 마커가 가운데가 아니면 머리가 한계까지 감 | 영상이 머리를 따라 바뀌지 않아 오차가 줄지 않음 — 정상 | — | 가운데 마커 이미지로 확인 |

#### 알아 둘 것 (튜토리얼에서 옮김)

- v1.0 기종은 드라이버가 계산하는 손 위치가 cuMotion 모델과 46 mm 다름 → `offset_xyz` 대신 `matrix` (T48)
- 실제 이동 시간은 계산보다 약 15% 김 (T42)
- 양팔을 동시에 서로 다른 목표로 움직일 수 없음 (한 번에 한 그룹)
- 기동 로그의 `No 3D sensor plugin(s) defined for octomap updates` ERROR는 정상 (T7)

---

## AprilTag

사용법은 [tutorial_apriltag.md](tutorial_apriltag.md)와 [tutorial_cumotion.md 4.7](tutorial_cumotion.md). 트러블슈팅은 §12의 **AprilTag·마커 예제**.

### 1. 구조

| 위치 | 패키지 (C++) | 노드·런치 | 하는 일 |
|---|---|---|---|
| 호스트 | `rby1_additional_tools` (드라이버 저장소) | `camera.launch.py` → `camera_publisher` | 웹캠·파일(OpenCV) 또는 RealSense 드라이버 → `/camera/image_raw`, `/camera/camera_info`. 장착 위치 TF `link_head_2 → camera_link → camera_optical_frame` |
| 컨테이너 | `rby1_apriltag` (이 저장소) | `apriltag.launch.py` | `component_container_mt` 안에 `RectifyNode`(`output_width`·`output_height` = `width`·`height`) → `AprilTagNode`(`size`, `tag_family`) → `/tag_detections`, 그리고 `target_tag_filter` |
| 호스트 | `rby1_examples` (드라이버 저장소, Python) | `22_marker_tracking` | 손: TF `base → target_marker_<id>` → 손 목표 `/rby1/target_pose`(추적 모드, §6.2). 머리: `/target_marker_<id>/pose` → 드라이버 `stream_joint`(머리만) |
| 호스트 | `rby1_examples` | `23_marker_shuttle` | TF `base → target_marker_<id>` 두 개 → 점대점 목표 `/rby1/target_pose` 왕복 |

- **카메라는 호스트에 둡니다.** 컨테이너에 장치 마운트·카메라 드라이버가 없고, 카메라를 바꿔도 이미지를 다시 만들지 않습니다.
  NVIDIA 패키지는 apt(§3 이미지). NITROS 무복사는 컨테이너 안(보정 → 검출)에서만 쓰이고, 호스트 → 컨테이너는 일반 ROS 메시지입니다
- 영상 발행자는 **reliable** — Isaac ROS 노드가 reliable로 구독합니다
- `target_tag_filter.launch.py`는 필터만 따로 띄울 때(검출을 다른 곳에서 할 때)
- **토픽 자료형**: 카메라 노드 → `/camera/image_raw`(`sensor_msgs/Image`, `bgr8`)·`/camera/camera_info`. 보정 노드 → `/image_rect`·
  `/camera_info_rect`(같은 타입 — 컨테이너 안에서는 NITROS로 복사 없이 넘기고, 밖에서는 일반 `sensor_msgs/Image`로 구독됨. 호스트에서
  30 Hz 확인). `/…/nitros` 토픽은 그 협상용. 검출 → `/tag_detections`, TF `tag36h11:<id>`
- **영상이 끊길 때 어디서 끊기는지**: `realsense_publisher`가 5초마다 전달 fps를 보고, 모자라면 원인을 구분해 경고 — 발행에 걸린
  시간(구독자), 프레임의 실제 노출 시간(자동 노출), 장치 타임스탬프 간격(USB·다른 프로그램). 전송 자체는 문제가 아니었음: 같은
  파이프라인(호스트 발행 → 컨테이너 보정 + 구독자 3개)에 1280×720 파일 영상을 넣으면 30.1 Hz, 최대 간격 37 ms, UDP 버퍼 오류 0
- **RViz**: `camera.launch.py rviz:=true`가 `config/camera.rviz`로 RViz를 띄움 — Image(영상), Camera(영상 위에 TF 축을 겹침:
  `camera_info`로 투영하므로 마커 위에 `target_marker_<id>` 축이 얹히면 검출·보정이 맞는 것), TF, Pose(`/target_marker/pose`).
  합성 영상(태그 7·12)으로 확인: 두 마커 위에 축, 3D에서 0.47 m 앞. 검출 결과를 그려 넣은 영상 토픽은 따로 만들지 않음
- 마커 설정은 `config/target_tags.yaml` 한 파일: `apriltag` 구역(`size`, `tag_family` — Isaac `AprilTagNode`로)과
  `target_tag_filter` 구역. 런치가 파일을 읽어(`marker_settings`) 노드 파라미터로 넘기고 `markers: tag36h11, black square 0.1 m`를
  출력. 런치 인자 `size:=`·`tag_family:=`·`ids:=7,12`는 주어졌을 때만 파일 값을 덮어씀. 처음엔 `size`가 런치 인자에만 있어
  설정 파일만 보면 크기를 어디서 정하는지 알 수 없었음
- 마커의 3차원 자세는 깊이가 아니라 **네 꼭짓점의 화소 위치 + 실제 크기(`size`) + 카메라 모델**로 계산(PnP). 크기는 노드에
  하나뿐이라 함께 쓰는 마커는 같은 크기여야 함
- **`rby1_apriltag`는 마커를 찾아 좌표를 내는 데까지**. 그 좌표로 로봇을 움직이는 것은 예제(`rby1_examples`)

### 2. 대상 마커 필터 (`target_tag_filter`)

Python에서 C++로 옮겼고(`src/target_tag_filter.cpp`, 계산은 `src/pose_average.cpp`) 파라미터와 토픽은 그대로입니다.

- **안정화**: 마커 번호마다 최근 `window_size` 프레임의 **카메라 기준** 자세를 모아, 위치는 성분별 중앙값(짝수 개면 가운데 두 값의
  평균 = `numpy.median`), 회전은 회전 행렬 합의 SVD로 가장 가까운 회전(`U Vᵀ`, 행렬식이 음수면 `U`의 마지막 열 부호를 뒤집음).
  역변환(예: 마커 기준 카메라)한 뒤에 평균하면 작은 각도 노이즈가 거리만큼 곱해진 위치 오차가 섞이므로, 뒤집기 전 카메라 기준에서 평균합니다
- **좌표계 이름**: Isaac ROS AprilTag는 검출마다 붙은 `pose.header.frame_id`를 **비워 둡니다**(배열 헤더에만 카메라 프레임).
  Python판은 빈 이름을 그대로 발행해 TF 조회가 안 됐습니다 → 비었으면 배열 헤더 값을 씀(`detection_frame()`)
- 출력: `/target_marker/pose`(모든 대상을 차례로), `/target_marker_<id>/pose`, TF `<카메라 프레임> → target_marker_<id>`

### 3. 마커 예제 (드라이버 저장소 `rby1_examples` 22, 23)

마커 좌표로 로봇을 움직이는 것은 Python 예제입니다. 처음에는 머리 추적이 `rby1_additional_tools`의 C++ 노드, 손 추종이
`rby1_apriltag`의 C++ 노드 `marker_target`이었는데, 예제 성격이라 `rby1_examples`로 옮겼습니다(로직은 같음). 공통 계산은
`marker.py`(마커 TF 읽기 `seen`, `hand_target`, `inside`, `moved`), 파라미터는 `--ros-args -p`, 기본값은 각 파일 맨 위.

| 예제 | 하는 일 |
|---|---|
| `22_marker_tracking` | `follow:=hand`(기본) 손이 마커를 따라감 / `head` 머리가 따라감 / `both`. 한 노드에 `Hand`·`Head` 두 부품 |
| `23_marker_shuttle` | 두 마커의 `offset`(기본 `base` z −10 cm) 지점을 `cycles`번 왕복(점대점). `moveit_target.py`의 `run`·`go`를 그대로 씀 |

#### 머리 (`22_marker_tracking`의 `Head`)

- **조준 오차**: 마커 위치를 TF의 **회전만** 써서 `link_head_2` 축으로 옮긴 방향 `d`에서 pan = atan2(d_y, d_x) → `head_0`(z축),
  tilt = −atan2(d_z, √(d_x²+d_y²)) → `head_1`(y축, +가 아래). 회전만 쓰므로 "카메라에서 본 방향"이고, 오차가 0이면 마커가
  영상 가운데입니다. 이동까지 넣어 `link_head_2` 원점에서 조준하면 카메라가 4 cm 위에 있어 0.32 m 앞 가운데 마커에도 tilt
  오차 −0.116 rad가 남아 머리가 한계까지 올라갔습니다(처음 구현)
- **제어**: 마커가 올 때마다 목표 = 측정 머리 각 + `gain` × 오차(`deadband` 이하는 그 축 유지), 범위로 자름. `rate` Hz 타이머가
  명령을 목표 쪽으로 `max_speed`/`rate`씩 옮겨 `stream_joint`(머리만, `minimum_time` = 1/`rate`)로 보냄. 영상 지연은 보정하지
  않으므로 `gain` < 1
- **정확도**: 남는 오차는 `head.deadband` 이내 — 0.02 → **0.005 rad**(0.3°, 0.8 m 앞에서 화면 중심 4 mm)로 줄임. 실측: 가짜
  마커 (0.5, 0.3) → (0.495, 0.298), (−0.4, −0.2) → (−0.396, −0.198) rad
- **상태**: 대기 → 추적 → (`lost_timeout`) 멈춤 → (`return_after`) 원위치 → 대기. 대기에서는 명령을 보내지 않으므로 드라이버가
  1초 뒤 스트림을 닫습니다
- **다른 명령과**: 드라이버가 머리에 스트림을 따로 두고, cuMotion은 머리를 덮개 구로 보며 자세 검사에서 빼므로(§5) 팔이
  움직이는 동안에도 계속 추적. 드라이버가 거부하면 0.5 s 쉼. 스트림은 꺼진 것을 본 뒤 0.5 s가 지나야 켬 — 실행기가 FJT 뒤 끈 지 14 ms 만에
  켜서 만료된 스트림을 받은 적이 있음. 실측: 머리 20 Hz 명령을 계속 보내며 cuMotion 예제 15·18·19·21 각 2회 8/8 DONE, 머리 명령
  거부 0 / 실제 추적기로 예제 15·16 DONE. Python판: 세상에 고정된 가짜 마커(머리 각 (0.5, 0.3) → (−0.4, −0.2) rad)에
  (0.491, 0.292) → (−0.390, −0.191)로 수렴(`deadband` 0.02 안), 놓친 뒤 멈춤·원위치, 팔 FJT 4 s와 동시에 돌려 머리 명령 거부 0
- 드라이버 쪽 조건(한 스트림은 앞선 명령에 없던 부위를 무시 등)은 [§4 드라이버 동작](#4-실행-경로와-드라이버-동작)

#### 손 (`22_marker_tracking`의 `Hand`)

- 시작할 때 실행기의 `set_tracking(true)`를 부르고(이동 중이라 거부되면 1 s마다 다시), 끝낼 때(Ctrl+C) `false`. 그래서 rclpy의
  SIGINT 처리를 끄고(`SignalHandlerOptions.NO`) `KeyboardInterrupt`에서 서비스를 부른 뒤 종료
- 목표 = 마커 위치 + `hand.offset`(`base` 방향, 기본 로봇 쪽 20 cm), 방향은 시작 때 손(`hand.tool_frame`) 그대로 또는
  `hand.follow_orientation`이면 마커 방향 × `hand.orientation_offset_rpy`. `hand.workspace_min`·`max` 상자로 자름
- TF를 `hand.rate`(60 Hz)로 보고 **마커 표본이 새로 올 때마다 목표 하나**(stamp가 바뀔 때; 움직였든 아니든 — 실행기가 그것으로
  마커 속도를 추정). 마커 TF가 `hand.lost_after`(0.5 s)보다 오래되면 안 보냄 → 실행기가 마지막 목표에서 멈춤
- 실측은 cuMotion §6.2 **정확도** 표(정지 0.1 mm, 6 cm/s 1.4 mm, 13 cm/s 7.5 mm). 마커가 사라지면 `marker … lost` 뒤 멈춤,
  Ctrl+C·SIGTERM에 실행기 `READY`(스크립트가 SIGINT를 무시한 채 띄워도 되도록 핸들러를 직접 검). `follow:=both`로 머리와 동시에도 같음

#### 드라이버만으로 따라가기 (`24_marker_cartesian_stream`)

"실제로 해 보니 팔이 계속 흔들린다 — MPC의 한계인가"(10/01)를 가려 보기 위한 비교 예제. 마커 좌표(`/target_marker_<id>/pose`)를
드라이버 `get_cartesian_pose`(머리 자세)와 TF(카메라 장착)로 `base`에 놓고, 목표 = 마커 + `offset`(작업 영역 상자 안, 지수
평활 `smoothing`), 명령은 `max_speed`/`rate`씩 다가가며 `stream_cartesian`으로(기준 `link_torso_5` — 드라이버 IK가 팔 관절만
쓰도록; `base` 기준이면 IK가 몸통도 써서 팔만으로는 못 감). 앞서 겨누기·장애물 검사 없음. 5초마다 명령·거절 수와 마커 좌표의
표준편차를 출력.

| 실측 (격리 시뮬, 평균 mm) | 정지 | 원 6 cm/s | 원 13 cm/s | 10 cm 건너뛴 뒤 1 cm 안 | 정지 중 손 떨림(표준편차) |
|---|---|---|---|---|---|
| `24` (드라이버 스트림, `smoothing` 0.5) | 0.5 | 20.7 | 38.7 | 0.86 s | 0.1 |
| `24`, 마커에 1 mm 노이즈 | 0.4 | 20.9 | 38.5 | 0.85 s | 0.44 |
| `22` (cuMotion `ik`) | 0.1 | 1.4 | 7.5 | 0.38 s | — |
| `22`, 마커에 1 mm 노이즈 | 0.8 | 3.2~3.7 | 8.4~9.2 | 0.4~0.5 s | 오차 최대 1.5 |

즉 드라이버 스트림은 약 0.33 s 늦게 따라가지만 노이즈에는 둔하고, cuMotion `ik`는 지연이 거의 없는 대신(앞서 겨누기) 마커
노이즈에 더 민감 — 실제 카메라에서 손이 떨리면 `tracking.smoothing`↑·`tracking.lead`↓로 맞춤(튜토리얼 §4.7 ⑤).

#### 왕복 (`23_marker_shuttle`)

- 두 마커가 `fresh`(1 s) 안에 보일 때까지 `marker_wait`(20 s) 기다림 → 구간마다 그 마커를 다시 읽고(안 보이면 마지막 위치)
  `hand_target(마커, offset, 준비 자세 손 방향)`을 점대점 목표로 → `DONE`을 기다림. 실행기가 구간마다 계획하므로 장면의 장애물은
  cuMotion·MoveIt(OMPL) 어느 쪽이든 피함(cuMotion은 이동 중에 생긴 것도)
- `box:=true`: 첫 왕복 뒤 두 지점 한가운데에 6 cm 상자(두 지점이 30 cm보다 가까우면 손목 구가 상자와 겹치므로 넣지 않음), 끝나면 치움
- 구간이 끝나면 손(드라이버 `get_cartesian_pose`)과 지점의 거리를 출력하고 `tolerance`(5 mm)를 넘으면 실패로 끝냄
- 실측(격리 시뮬, 마커 7·12를 `base` (0.45, −0.45, 1.25)·(0.45, −0.10, 1.25)에 가짜 TF): 3왕복 6구간 모두 `DONE`, **끝점 오차
  0.0~0.6 mm**. 손이 두 지점의
  가운데를 지나는 최소 거리 — 상자 없이 5.3~6.0 cm, 상자를 넣은 뒤 14.1~14.6 cm(돌아감)

### 4. 카메라 노드 (C35)

- 웹캠(`camera_publisher`, OpenCV V4L2)과 RealSense(`realsense_publisher`, librealsense C++ API — `ros-humble-librealsense2` 2.58) 두 가지로 한정. realsense-ros는
  안 쓰는 토픽까지 전부 내서 뺌. 구조는 camera_ws `core/camera_processing.py`를 따름: 시리얼로 장치 선택, 켠 스트림만 `rs2::config`,
  시작 후 `skip_frames`(10) 버림, `auto_exposure_priority` 끔(자동 노출이 fps를 낮추지 못하게 — camera_ws가 19.6 fps로 떨어진 기록),
  USB 2 연결이면 경고, 해상도가 안 열리면 USB 3 안내
- 설정은 노드별 YAML(`config/webcam.yaml`, `config/realsense.yaml`)이 원천, 런치 인자는 `camera`, `config`, `source`, `intrinsics`, `mount`만
- 컬러 센서 찾기: 컬러 스트림 프로필이 있는 센서(D405는 스테레오 모듈 하나가 컬러도 내므로 적외선 노출 설정을 건너뜀). 깊이는
  `depth_scale`로 mm 16UC1(D405는 0.1 mm 단위 → 나눔). 깊이·적외선 좌표계는 `get_extrinsics_to(컬러)`로 컬러 좌표계 아래 정적 TF
  (rs2 회전은 열 우선). `align_depth_to_color`면 `rs2::align`으로 컬러 화소·좌표계
- 카메라 모델: `use_custom_intrinsics`면 `read_intrinsics`(camera_ws 형식 `camera_matrix` 3×3 목록 + `dist_coeffs`, 또는 ROS
  `camera_calibration` 형식) → `camera_info`가 영상 크기에 맞춰 fx·fy·cx·cy를 비율로 조정(비율이 다르면 거부, 왜곡은 그대로).
  아니면 RealSense 공장값(Brown-Conrady → `plumb_bob`), 웹캠은 `horizontal_fov` 핀홀
- **Isaac `RectifyNode`는 처음 받은 `camera_info`로 보정 맵을 만들고 바꾸지 않음**: 실측 — 같은 컨테이너 파이프라인에 fx 640 →
  1280 파일을 차례로 주면 둘 다 0.322 m, 파이프라인을 새로 띄우면 1280에서 0.643 m(두 배, 기대대로). 카메라 모델을 바꾸면
  `apriltag.launch.py` 재기동(튜토리얼에 적음)
- **`ros-humble-librealsense2`(2.58.4, `/opt/ros`)로 빌드**: CMake가 `/opt/ros/$ROS_DISTRO`의 librealsense2를 먼저 찾고, 없을 때만
  시스템(Intel 저장소) 것을 씀. 어느 쪽으로 빌드했는지는 `share/rby1_additional_tools/realsense_from`(`ros`|`system`)에 남고
  `camera.launch.py`가 읽음. `package.xml`에 `librealsense2`(rosdep 키 → `ros-humble-librealsense2`)
- **Intel 저장소 librealsense2 2.56은 Fast DDS를 내장하고 그 심볼을 내보냄**(6946개) → ROS(Fast DDS)와 같은 프로세스에서 노드를 만들 때
  ROS의 `rmw_fastrtps`가 librealsense 쪽 `DomainParticipantFactory`를 잡아 시작하자마자 죽음(`std::bad_array_new_length`,
  `double free`; gdb로 확인). ROS 패키지는 그런 심볼이 0개. `realsense_from`이 `system`이면 런치가 이 노드만
  `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`로 띄움(Fast DDS 노드들과 네트워크로 통신, 컨테이너 AprilTag가 22 Hz로 받음 확인) — 대비책
- 실측(D405, USB 3.2): 공장 보정 fx 654.0 fy 653.2. 1280×720 컬러 **30.0 Hz**(ROS 패키지 2.58.4, Fast DDS, 충돌 없음). Intel 2.56으로는
  같은 모드가 약 21 fps였음(장치 타임스탬프 기준 — 장치 한계가 아니라 그 라이브러리 판의 문제였음), 848×480은 25~28 fps(컬러·깊이·
  적외선 4개 동시). D405 센서는 `auto_exposure_priority`가 없어 노드가 경고(어두우면 자동 노출이 fps를 더 낮춤 → 수동 노출)
- librealsense2가 아예 없으면 CMake가 `realsense_publisher`만 빼고 빌드, 런치가 `CAMERA_FAILED: realsense_publisher is not built`로 안내

| 카메라 | 메모 |
|---|---|
| RealSense D435·D455 | 컬러는 RGB 센서. D455는 1280×800 |
| RealSense D405 | RGB 센서가 없어 컬러도 스테레오 모듈이 냄 — 노출은 컬러 설정 하나 |
| 빠른 추적 | 848×480 @ 60~90 fps: 설정 파일과 `apriltag.launch.py`(`width`·`height`)를 같이. NVIDIA 기준 검출은 RTX에서 약 0.8 ms |
| ZED | 드라이버가 이미 보정된 영상을 냄 — 보정 노드를 빼고 AprilTag에 바로 물리면 되지만 `apriltag.launch.py`는 아직 보정을 항상 거침 |
| USB 웹캠 | 보정 권장: camera_ws 결과나 `camera_calibration` 파일을 `intrinsics:=`로 |

### 5. 검증과 테스트

격리 시뮬(도메인 77)에서 합성 영상(`camera:=file`, 태그 7, 200 px, f = 640 (그때는 `horizontal_fov:=90`, 지금은 같은 값의 보정 파일 `intrinsics:=`))으로:

- 가운데에서 왼쪽 300 px·위 120 px 마커 → `/target_marker/pose` `camera_optical_frame` (−0.151, −0.060, 0.322), 기대 (−0.15, −0.06, 0.32)
- 머리가 pan +(왼쪽)·tilt −(위)로 약 0.7 rad/s로 돌다 범위 끝(±1.5)에서 멈춤(정지 영상이라 오차가 안 줄어듦). 가운데 마커면 가만히 있음
- 마커를 치우면 0.5 s 뒤 멈춤, 3 s 뒤 (0, 0)으로 돌아가 명령을 멈춤
- 머리 추적을 켠 채 MoveIt 예제 15: 드라이버 수정 전 3/3 `FAILED … stopped short`(팔 안 움직임), 수정 후 6/6 `EXAMPLE_DONE`.
  추적 중 `robot_joint`로 2.5 rad 이동: 수정 전 1초 지점에서 멈춤, 수정 후 끝까지(머리 명령은 그동안 거부됐다가 이어 감)
- 예제 13(몸 전체 30 Hz 스트림)은 스트림을 다시 열지 않고 그대로 동작
- MoveIt 실행기 런치의 `robot_state_publisher`가 있으면 `tf2_echo base target_marker_7`이 나옴

```bash
# 컨테이너
colcon build --symlink-install --base-paths src/rby1_isaac_ros/rby1_apriltag --packages-select rby1_apriltag
colcon test --base-paths src/rby1_isaac_ros/rby1_apriltag --packages-select rby1_apriltag   # gtest 5: 중앙값, 짝수 개, SVD 회전, 빈 입력, 좌표계 이름
# 호스트
colcon test --packages-select rby1_additional_tools                                          # gtest: 카메라 모델(보정 파일 두 형식, 크기 맞춤)
python3 -m pytest ~/ros2_driver_ws/src/rby1_ros2/rby1_examples/test/test_marker_examples.py  # 19: 손 목표(오프셋·방향·상자), 조준 오차, 한 걸음, 범위, 상자 위치, 쿼터니언·평활·속도 제한 걸음
```
