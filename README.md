# RBY1 Isaac ROS (`rby1_isaac_ros`)

Rainbow Robotics의 로봇 플랫폼(RBY1 등)에서 NVIDIA Isaac ROS의 하드웨어 가속 비전 노드(`isaac_ros_apriltag`, `isaac_ros_visual_slam`, `isaac_ros_nvblox` 등)를 컨테이너 환경에서 통합 구동하기 위한 예제 및 애플리케이션 개발 레포지토리입니다.

본 레포지토리는 RBY1 특화 예제 노드, 통합 런치 파일, 설정을 관리하며, 대용량 외부 의존성인 NVIDIA 공식 패키지(`isaac_ros_common`, `isaac_ros_apriltag` 등)는 `.gitignore`로 제외되어 호스트 환경 버전에 맞추어 `src/` 경로에 클론하여 사용합니다.

---

## 1. ⚙️ 사전 호스트 환경 구성 (필수 선행)

본 워크스페이스를 구동하기 전, 사용 중인 호스트 OS 버전에 맞는 사전 환경 설정(NVIDIA 드라이버, Docker, Container Toolkit, Buildx, 디바이스 마운트, `isaac_ros_common` 클론 및 검증)을 먼저 완료해야 합니다.

* 📌 **Ubuntu 22.04 LTS (ROS 2 Humble / release-3.2 권장)**:  
  👉 **[docs/env_setup_ubuntu_22_04.md](docs/env_setup_ubuntu_22_04.md)**
* 📌 **Ubuntu 24.04 LTS (ROS 2 Jazzy / release-4.0+ 권장)**:  
  👉 **[docs/env_setup_ubuntu_24_04.md](docs/env_setup_ubuntu_24_04.md)**

---

## 2. 🧩 기본 개발 및 빌드 워크플로우

호스트 설정이 완료되었다면 아래 단계에 따라 필요한 패키지를 구성하고 빌드합니다.

### Step 1. 필요한 Isaac ROS 패키지 클론 (`src/` 경로)

#### 🔹 Ubuntu 22.04 LTS 호스트 (ROS 2 Humble / release-3.2)
소스로 받는 것은 **도커 개발 래퍼 `isaac_ros_common` 하나**입니다. cuMotion, AprilTag(`isaac_ros_apriltag`), 영상 보정
(`isaac_ros_image_proc`)은 NVIDIA apt 패키지로 이 저장소의 이미지 레이어(`docker/Dockerfile.cumotion`)에 들어갑니다.

```bash
cd ~/isaac_ros_ws/src
git clone -b release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git   # 환경 구성 단계에서 클론하지 않은 경우
```

> 예전 방식대로 `isaac_ros_apriltag`, `isaac_ros_image_pipeline`을 `src/`에 클론해 빌드했다면, 그 빌드 결과(`install/`)가
> apt 설치본보다 우선합니다. 지우거나 `install/`에서 빼 두십시오(소스 폴더는 남아 있어도 `rby1_isaac_ros`만 빌드하면 상관없음).

#### 🔹 Ubuntu 24.04 LTS 호스트 (ROS 2 Jazzy / release-4.0 이상)
> ⚠️ 이 저장소의 이미지 레이어는 Humble(release-3.2) 기준입니다. 아래 Jazzy 절차는 확인하지 않았습니다.
```bash
cd ~/isaac_ros_ws/src

# 1. 도커 개발 래퍼 (환경 구성 단계에서 클론하지 않은 경우)
git clone -b release-4.0 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git

# 2. 비전 및 AprilTag 패키지
git clone -b release-4.0 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag.git
git clone -b release-4.0 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline.git

# [Visual SLAM 필요 시]
# git clone -b release-4.0 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git
```

---

### Step 2. 컨테이너 구동 및 세션 진입

`isaac_ros_common`은 실행 중인 하드웨어(x86 PC 또는 Jetson ARM64)를 자동으로 감지하여 최적의 GPU 가속 컨테이너를 구동합니다.

#### ⚡ 스마트 단축 커맨드 사용 (`isaac-ros` 권장)
환경 구성 단계에서 `~/.bashrc`에 단축 함수를 등록해 두었으므로, 어느 디렉터리에 있든 터미널에서 아래 한 단어로 컨테이너에 즉시 접속할 수 있습니다:
```bash
isaac-ros
```
> (수동 실행 시: `cd ~/isaac_ros_ws/src/isaac_ros_common && ISAAC_ROS_WS=$HOME/isaac_ros_ws ./scripts/run_dev.sh`)  
> 실행 완료 시 컨테이너 프롬프트(`admin@<hostname>:/workspaces/isaac_ros-dev$`)로 자동 진입합니다.

---

### Step 3. 컨테이너 내부 의존성 설치 (`rosdep`)
컨테이너 내부에서 패키지가 요구하는 모든 NITROS 및 GXF 가속 라이브러리를 공식 패키지 저장소로부터 자동 해결합니다:

```bash
# 컨테이너 내부에서 실행
cd /workspaces/isaac_ros-dev

# 1. rosdep을 통한 Isaac ROS 핵심 의존성(NITROS 등) 일괄 설치
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y

```

> 카메라는 컨테이너가 아니라 **호스트에서** 켭니다. 영상은 토픽(`/camera/image_raw`, `/camera/camera_info`)으로 컨테이너에
> 넘어옵니다 — 드라이버 저장소의 `rby1_additional_tools`(`camera.launch.py`: 웹캠·파일·RealSense). RealSense를 쓰면
> 호스트에 `sudo apt install ros-humble-librealsense2`(realsense-ros는 필요 없음).

---

### Step 4. 패키지 빌드 (`colcon build`)
빌드 시간을 단축하고 테스트 패키지 컴파일 에러를 방지하기 위해 테스트 플래그를 비활성화(`BUILD_TESTING=OFF`)하고 필요한 패키지 단위로 빌드합니다:

```bash
# 컨테이너 내부에서 실행
# 1. 이 저장소의 패키지만 빌드 (NVIDIA 패키지는 이미지에 apt로 들어 있음)
colcon build --base-paths src/rby1_isaac_ros --symlink-install --cmake-args -DBUILD_TESTING=OFF

# 2. 빌드 환경 반영 (오버레이 적용)
source /opt/ros/humble/setup.bash             # Jazzy의 경우 /opt/ros/jazzy/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
```

---

## 3. 📚 패키지별 구성 및 실행 매뉴얼 (Tutorials & Modules)

본 워크스페이스에서 활용 가능한 NVIDIA Isaac ROS 가속 노드별 상세 가이드 및 튜토리얼 목록입니다. 각 모듈의 상세 런치 설정, 파라미터 튜닝, 카메라 기종별 대응 매뉴얼은 링크된 문서를 참조하십시오.

### 📌 패키지 매뉴얼 목록

* 🦾 **[cuMotion 기반 RBY1 경로 계획 및 실행](docs/tutorial_cumotion.md)**:  
  👉 **[docs/tutorial_cumotion.md](docs/tutorial_cumotion.md)** — 설치부터 예제(이동·속도·장애물)까지 순서대로
  * 4×4 목표 토픽을 보내면 GPU(cuMotion)가 장애물을 피하는 경로를 계획하고 RB-Y1 드라이버로 실행
  * 움직이는 도중 경로에 나타난 장애물은 멈추지 않고 다시 계획해 돌아감. 움직이는 장애물은 경로 위에서 멈춰 지나가길 기다렸다가 이어 감
  * 시뮬레이터·실기체 동일 절차, 로봇 기종·버전·자세는 기동 시 드라이버에서 자동으로 읽음
  * 개발자용 내용(구조·모델 번들·이미지·충돌 모델·설정·테스트·트러블슈팅)은 👉 **[docs/developer_manual.md](docs/developer_manual.md)**

* 🎯 **[AprilTag 마커 추적과 머리 추적](docs/tutorial_apriltag.md)**:  
  👉 **[docs/tutorial_apriltag.md](docs/tutorial_apriltag.md)**
  * 카메라는 호스트에서(웹캠·RealSense·이미지 파일), 영상만 토픽으로 컨테이너에 → GPU 보정·검출(cuAprilTag)
  * **`rby1_apriltag`**(C++): 대상 마커 선별, 중앙값+SVD 안정화, 표준 `PoseStamped`·TF 발행
  * 마커 좌표로 로봇 움직이기(드라이버 저장소 `rby1_examples`): `22_marker_tracking`(손·머리가 마커를 따라감), `23_marker_shuttle`(두 마커 사이 왕복, 장애물 회피)

* 🧭 **[Visual SLAM (cuVSLAM 3D 오도메트리)](docs/tutorial_visual_slam.md)** *(작성 예정)*:
  * 스테레오 카메라 및 IMU 융합 GPU 가속 실시간 6-DoF 로봇 위치 추정

* 🧱 **[nvblox (GPU 실시간 3D 복원 & 매핑)](docs/tutorial_nvblox.md)** *(작성 예정)*:
  * TSDF / ESDF 복셀 기반 실시간 3D 매핑 및 로봇 장애물 회피 연동

---

### ⚡ cuMotion 빠른 시작 요약 (Quickstart)

```bash
# 설치 (처음 한 번)
cd ~/isaac_ros_ws/src/rby1_isaac_ros/docker && ./make_bundles.sh              # 호스트: 모델 파일 준비
cp ~/isaac_ros_ws/src/rby1_isaac_ros/docker/isaac_ros_common-config ~/.isaac_ros_common-config
cd ~/isaac_ros_ws/src/isaac_ros_common/scripts && ./run_dev.sh -d ~/isaac_ros_ws   # 이미지 → 컨테이너
colcon build --base-paths src/rby1_isaac_ros/rby1_cumotion \
    --packages-select rby1_cumotion --symlink-install                         # 컨테이너

# 실행 (매번)
ros2 launch rby1_driver rby1_ros2_driver.launch.py                            # 호스트: 드라이버
ros2 run rby1_examples 06_zero_pose                                           # 호스트: 전원·서보 + 영점
ros2 launch rby1_cumotion demo.launch.py                                      # 컨테이너: 준비 → cuMotion → 실행기 (RViz)
ros2 run rby1_examples 15_moveit_move_hand                                    # 호스트: 예제 (튜토리얼 §4 표)
```
> 컨테이너 터미널은 `isaac-ros`(= `docker exec -u admin`)로 엽니다. `root@…` 셸에서는 드라이버 토픽이 오지 않습니다.
> cuMotion 런치가 떠 있으면 목표를 받을 때 **로봇이 움직입니다.** 속도·그룹 등 설정은 `rby1_cumotion/config/cumotion.yaml` 한 곳. 문제가 생기면 그 터미널의 메시지가 할 일을 알려 줍니다(자세히: [developer_manual 트러블슈팅](docs/developer_manual.md#12-트러블슈팅)).
> 드라이버(`~/ros2_driver_ws`)는 빌드된 상태여야 합니다 — 드라이버 저장소 README.

---

### ⚡ AprilTag 빠른 시작 요약 (Quickstart)

```bash
# 설치 (처음 한 번, 컨테이너) — 이미지는 cuMotion과 같음
colcon build --symlink-install --base-paths src/rby1_isaac_ros/rby1_apriltag --packages-select rby1_apriltag

# 실행 (매번)
ros2 launch rby1_additional_tools camera.launch.py                            # 호스트: 카메라 → /camera/image_raw
ros2 launch rby1_apriltag apriltag.launch.py                                  # 컨테이너: 보정 → 검출 → /target_marker/pose
ros2 topic echo /target_marker/pose --once                                    # 호스트: 확인
ros2 run rby1_examples 22_marker_tracking --ros-args -p follow:=head          # 호스트: 머리가 마커를 따라감 (드라이버 필요)
ros2 run rby1_examples 22_marker_tracking                                     # 호스트: 손이 마커를 따라감 (cuMotion 기동 필요, 추적 모드를 스스로 켬)
ros2 run rby1_examples 23_marker_shuttle                                      # 호스트: 두 마커(ids:=7,12) 아래 지점 사이 왕복
```
> 마커 크기(`size`, 검은 사각형 한 변 m)와 찾을 번호(`target_ids`)는 `rby1_apriltag/config/target_tags.yaml` — 이번 실행만 바꾸려면 `size:=0.08 ids:=7,12`. 마커 예제는 [docs/tutorial_cumotion.md 4.7](docs/tutorial_cumotion.md). 자세히: [docs/tutorial_apriltag.md](docs/tutorial_apriltag.md).


---

## 4. 🛠️ 컨테이너 세션 관리 치트시트

* **멀티 터미널 추가 접속 (호스트 터미널에서 실행)**:
  컨테이너가 실행 중인 상태에서 새 터미널 창을 열고 `isaac-ros`를 입력하면 자동으로 실행 중인 컨테이너에 attach됩니다:
  ```bash
  isaac-ros
  # 또는 직접 명령어: docker exec -it -u admin -w /workspaces/isaac_ros-dev isaac_ros_dev-x86_64-container bash
  ```
  > ⚠️ `-u admin`을 빼면 root로 들어가고, root 셸에서는 호스트 ROS 노드(예: RB-Y1 드라이버)의 토픽이 오지 않습니다.
  > 예전에 등록한 `isaac-ros` 함수에 `-u admin`이 없다면 [env_setup 8.2](docs/env_setup_ubuntu_22_04.md)대로 고치십시오.
* **세션 종료**: 컨테이너 셸에서 `exit`
* **컨테이너 강제 중지**: `docker stop isaac_ros_dev-x86_64-container`
* **💡 의존성 보존 및 도커 이미지 커밋 (매번 apt 재설치 방지)**:
  컨테이너 내부에서 `apt`나 `rosdep`으로 설치한 패키지는 컨테이너 재생성 시 초기화됩니다. 의존성 설치 후 호스트 터미널에서 현재 컨테이너를 도커 이미지로 커밋해 두면 영구적으로 보존됩니다:
  ```bash
  # 호스트 PC 터미널에서 실행:
  docker commit isaac_ros_dev-x86_64-container isaac_ros_dev-x86_64:latest
  ```
