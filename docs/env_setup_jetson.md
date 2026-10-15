# [환경 구성] Jetson Orin (JetPack 6.x, ROS 2 Humble / release-3.2)

Jetson Orin에서 Isaac ROS `release-3.2` 컨테이너를 쓰기까지 **해야 하는 것만** 순서대로 적었습니다. JetPack에 그래픽 드라이버·CUDA·Docker가
들어 있어서 PC 문서([env_setup_ubuntu_22_04.md](env_setup_ubuntu_22_04.md))의 드라이버·Docker 설치는 하지 않습니다.

- 설치 중 문제가 생기면 👉 [developer_manual.md — 환경 구성](developer_manual.md#환경-구성)

> [!IMPORTANT]
> **Jetson은 JetPack 버전이 Ubuntu·ROS·Isaac ROS 버전을 정합니다.** 이 저장소가 지원하는 것은 **JetPack 6.1·6.2**
> (Ubuntu 22.04, ROS 2 Humble, Isaac ROS `release-3.2`)뿐입니다. JetPack 7(Ubuntu 24.04, ROS 2 Jazzy, Isaac ROS 4.x 이상)은
> ROS 배포판과 설치 방식이 달라 이 문서로는 되지 않습니다.

> [!CAUTION]
> **저장 공간이 모자라면 이미지를 받거나 만드는 중간에 실패합니다.** 이미지는 x86 PC 실측 약 37 GB(2026-10-15)이고, 받는 동안과
> 빌드 중에는 그보다 더 필요합니다. Jetson에서의 크기는 재지 않았습니다. 시작하기 전에 남은 공간을 확인하십시오:
> `df -h /var/lib/docker /var/lib/containerd`의 `Avail`. 2026-10-13에는 Jetson AGX Orin의 내장 저장소에서 이미지를 받다가
> `no space left on device`로 멈췄습니다.

> [!WARNING]
> 2026-10-13에 Jetson AGX Orin에서 2장의 이미지 빌드 직전까지 따라 했습니다. **이미지 빌드부터 그 뒤는 Jetson에서 확인되지 않았습니다.**

> [!IMPORTANT]
> 시작하기 전에 호스트에 ROS 2 Humble, RB-Y1 SDK, 드라이버 워크스페이스를 빌드해 둡니다(드라이버 저장소 README).

---

## 1. 호스트 준비

```bash
# 1. JetPack 확인 — R36, REVISION: 4.x (JetPack 6.1·6.2) 여야 합니다
cat /etc/nv_tegra_release

# 2. Docker 확인 — JetPack에 들어 있습니다. docker.io를 새로 설치하지 않습니다
docker --version
docker buildx version
docker info 2>/dev/null | grep -i runtimes                 # nvidia 가 보여야 합니다
df -h /var/lib/docker /var/lib/containerd                  # 남은 공간(Avail) — 위 CAUTION의 필요 용량 이상

# 3. git-lfs, docker 그룹
sudo apt update && sudo apt install -y git git-lfs
sudo usermod -aG docker $USER
newgrp docker

# 4. CDI 설정 — run_dev.sh가 Jetson에서는 GPU를 nvidia.com/gpu=all 이라는 이름으로 요청합니다
sudo nvidia-ctk cdi generate --mode=csv --output=/etc/cdi/nvidia.yaml
sudo systemctl restart docker

# 5. PVA 허용 목록 패키지
sudo apt-get install -y software-properties-common
sudo apt-key adv --fetch-key https://repo.download.nvidia.com/jetson/jetson-ota-public.asc
sudo add-apt-repository 'deb https://repo.download.nvidia.com/jetson/common r36.4 main'
sudo apt-get update
sudo apt-get install -y pva-allow-2
```

- 2번에서 `docker buildx version`이 안 나오면 PC 문서 [4장](env_setup_ubuntu_22_04.md#4-docker-buildx)을 합니다(명령이 arm64 파일을 받습니다).
- 4·5번의 출처는 NVIDIA의 [Jetson Setup for VPI](https://nvidia-isaac-ros.github.io/v/release-3.2/getting_started/hardware_setup/compute/jetson_vpi.html)입니다(2026-10-13 대조).

---

## 2. 저장소 받기, 이미지 만들기

```bash
# 1. 워크스페이스 위치를 한 번 정해 둡니다 (어디든 됩니다)
echo 'export ISAAC_ROS_WS=$HOME/isaac_ros_ws' >> ~/.bashrc && source ~/.bashrc

# 2. 저장소 두 개를 같은 src/ 에 받습니다
mkdir -p $ISAAC_ROS_WS/src && cd $ISAAC_ROS_WS/src
git clone -b release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
git clone https://github.com/JeonSangMin-sw2/rby1_isaac_ros.git

# 3. 이미지에 넣을 로봇 모델과 드라이버 메시지 준비
cd $ISAAC_ROS_WS/src/rby1_isaac_ros/docker
./make_bundles.sh                      # 첫 두 줄에 찾은 드라이버 워크스페이스와 SDK 경로가 나옵니다
ls bundles                             # a_1_0 … m_1_3 폴더 7개

# 4. 이 저장소의 이미지 레이어를 쓰도록 설정
cp isaac_ros_common-config ~/.isaac_ros_common-config

# 5. 이미지 만들고 컨테이너에 들어가기 (처음에는 오래 걸립니다)
cd $ISAAC_ROS_WS/src/isaac_ros_common && ./scripts/run_dev.sh
```

- 1~5를 **모두** 합니다. PC 문서 6장의 Dockerfile 패치는 **하지 않습니다**(x86 전용).
- 5번이 끝나면 프롬프트가 `admin@…:/workspaces/isaac_ros-dev$`가 됩니다. `exit`로 나옵니다.

---

## 3. `isaac-ros` 명령 등록과 컨테이너 쓰기

PC 문서 [7.2](env_setup_ubuntu_22_04.md#72-isaac-ros-명령-등록)의 함수를 그대로 `~/.bashrc`에 넣습니다. 컨테이너 이름(`aarch64`)은 함수가
알아서 고릅니다. 이후 컨테이너는 `isaac-ros`로 켜고 들어갑니다([7.3](env_setup_ubuntu_22_04.md#73-컨테이너-쓰기)).

```bash
isaac-ros
```

---

## 4. 컨테이너에서 빌드하고 확인하기

```bash
# 컨테이너 안에서
colcon build --base-paths src/rby1_isaac_ros --symlink-install --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash

echo $TORCH_CUDA_ARCH_LIST                                   # 8.7
python3 -c "import torch; print(torch.cuda.is_available(), torch.cuda.get_device_capability())"   # True (8, 7)
python3 -c "import curobo.curobolib.kinematics, curobo.curobolib.geom"   # 다시 컴파일한다는 줄 없이 끝나야 합니다
```

> [!NOTE]
> 로봇 시뮬레이터 이미지는 x86 전용이라 Jetson에서는 뜨지 않습니다. 실제 로봇에 붙거나, 같은 네트워크의 PC에서 시뮬레이터를 켜고
> 드라이버의 `robot_ip`를 그 PC 주소로 둡니다.

다음은 [README 2장](../README.md#2--컨테이너에서-패키지-빌드)과 [cuMotion 가이드](tutorial_cumotion.md)입니다.
