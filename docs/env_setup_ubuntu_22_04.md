# [환경 구성] Ubuntu 22.04 LTS (ROS 2 Humble / release-3.2)

본 문서는 **Ubuntu 22.04 LTS 호스트 PC(x86_64)** 및 **Jetson Orin (JetPack 6.x)** 환경에서 Isaac ROS `release-3.2` 컨테이너를 구동하기 위한 호스트 사전 요구사항 설정 및 검증 가이드입니다.

호스트 설정부터 `isaac_ros_common` 컨테이너에 정상 진입하고 단축 커맨드를 등록하는 단계까지 완료하면 기본 환경 구성이 끝납니다.

> [!NOTE]
> **Jetson은 [9장](#9-jetson-orin-jetpack-6x-설치-순서)만 따라 합니다.** 2~8장은 x86 PC용이고, Jetson에서 필요한 명령은 9장에 순서대로 있습니다.

**전체 순서**

| 순서 | 할 일 | 어디에 |
|---|---|---|
| 1 | 호스트에 ROS 2 Humble, RB-Y1 SDK, 드라이버 워크스페이스 빌드 | 드라이버 저장소(`rby1-ros2`) README |
| 2 | 호스트 준비: 그래픽 드라이버, Docker, Buildx, 디바이스 마운트 | 이 문서 2~5장 |
| 3 | 저장소 두 개 클론: `isaac_ros_common`, `rby1_isaac_ros` | 6장 |
| 4 | (x86 PC만) NVIDIA Dockerfile 패치 | 7장 |
| 5 | 이미지 만들기, 컨테이너 진입, `isaac-ros` 등록 | 8장 |
| 6 | 컨테이너 안에서 이 저장소의 패키지 빌드(`colcon build`) | [README 2장](../README.md#2--컨테이너에서-패키지-빌드) |

1번이 먼저인 이유: 5번에서 이미지에 넣을 로봇 모델과 메시지 정의를 드라이버 워크스페이스에서 가져옵니다.
6번의 `colcon build`는 5번에서 만든 컨테이너 **안에서만** 됩니다(NVIDIA 패키지가 그 이미지에 들어 있습니다).

---

## 1. 지원 버전 매트릭스

* **호스트 OS**: Ubuntu 22.04 LTS
* **타깃 Isaac ROS 브랜치**: **`release-3.2`** (Humble 기준 공식 권장 표준)
* **컨테이너 내부 환경**: Ubuntu 22.04 LTS + ROS 2 Humble
* **NVIDIA 드라이버**: 호스트 GPU 아키텍처에 맞는 오픈소스 커널 모듈(`-open`) 권장

---

## 2. NVIDIA 그래픽 드라이버 설정

> [!NOTE]
> **Jetson은 이 장을 건너뜁니다.** 드라이버는 JetPack에 들어 있습니다([9장](#9-jetson-orin-jetpack-6x-설치-순서)).

### 2.1. 현재 설치된 드라이버 확인
이미 NVIDIA 드라이버가 설치되어 있다면 먼저 정상 동작 여부를 확인합니다:
```bash
nvidia-smi
```
* 상단에 GPU 모델명과 드라이버 버전이 정상 출력된다면 드라이버 재설치는 건너뛰고 **[3. Docker 및 NVIDIA Container Toolkit]**으로 넘어가도 됩니다.

### 2.2. 드라이버 설치 또는 변경 (필요 시)
특정 드라이버 버전을 수동으로 강제 설치(예: 580)할 경우 디스플레이 서버(Xorg)나 모니터 출력 포트 설정에 따라 **화면 미출력(No Signal / 블랙스크린)** 현상이 일어날 수 있습니다. 

따라서 우분투가 현재 장착된 GPU 하드웨어를 자동 감지하여 **가장 안정적인 공식 권장 드라이버를 자동 설치하도록 구성하는 것(`ubuntu-drivers`)을 강력 권장**합니다.

```bash
# 1. 커널 헤더 및 dkms 빌드 도구 설치
sudo apt update
sudo apt install -y linux-headers-$(uname -r) build-essential dkms

# 2. 내 GPU에 적합한 추천 드라이버 확인
ubuntu-drivers devices

# 3. [권장] 시스템 권장(recommended) 드라이버 자동 일괄 설치
sudo ubuntu-drivers install

# 4. 재부팅하여 새 드라이버 적용
sudo reboot
```

> [!WARNING]
> **설치 후 모니터 화면이 나오지 않거나 검은 화면(Black Screen)일 때 대처법**:
> 1. **외장 GPU 화면 출력 고정 (노트북/하이브리드 그래픽)**:  
>    메인보드 내장 그래픽과 충돌하는 경우, `sudo prime-select nvidia` 후 재부팅합니다.
> 2. **Secure Boot(보안 부팅) 확인**:  
>    메인보드 BIOS 설정에서 `Secure Boot`가 켜져 있으면 커널이 신규 드라이버 로드를 차단해 화면이 꺼질 수 있습니다. BIOS에서 Secure Boot를 비활성화(`Disabled`)해야 합니다.
> 3. **원상 복구 (긴급 롤백)**:  
>    화면이 검게 멈춘 경우 `Ctrl + Alt + F3`을 눌러 텍스트 터미널(TTY)로 진입한 후, 아래 명령어로 드라이버를 삭제하고 재부팅하면 즉시 기본 화면으로 복구됩니다:
>    ```bash
>    sudo apt purge "*nvidia*" -y
>    sudo reboot
>    ```

---

## 3. Docker 및 NVIDIA Container Toolkit 설정

컨테이너 내부에서 GPU 가속을 활용하기 위해 툴킷과 런타임을 구성합니다.

> [!IMPORTANT]
> **Jetson은 이 장 대신 [9.1절](#91-호스트-준비)을 합니다.** JetPack에 Docker와 툴킷이 이미 있어서 아래 설치 명령은 충돌합니다.

```bash
# 1. Docker 설치 — `docker --version`이 이미 나오면 이 두 줄은 건너뜁니다.
#    (Docker 공식 저장소로 설치돼 있으면 docker.io가 `containerd.io : Conflicts: containerd`로 거부됩니다)
sudo apt update
sudo apt install -y docker.io

# 1-1. git-lfs와 docker 그룹 (항상)
sudo apt install -y git git-lfs
sudo usermod -aG docker $USER
newgrp docker

# 2. NVIDIA Container Toolkit 저장소 등록 및 설치
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo apt update
sudo apt install -y nvidia-container-toolkit

# 3. Docker 데몬에 nvidia 런타임 등록 및 데몬 재시작
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

---

## 4. Docker Buildx 수동 설치 (Ubuntu 22.04 전용)

Ubuntu 22.04는 기본 apt 저장소에 최신 `docker-buildx`가 포함되어 있지 않으므로, GitHub 바이너리를 다운로드하여 CLI 플러그인으로 등록합니다.

```bash
mkdir -p ~/.docker/cli-plugins
# PC(x86_64)는 amd64, Jetson(aarch64)은 arm64 파일을 받습니다
ARCH=$([ "$(uname -m)" = "aarch64" ] && echo arm64 || echo amd64)
curl -sSL https://github.com/docker/buildx/releases/download/v0.14.1/buildx-v0.14.1.linux-${ARCH} -o ~/.docker/cli-plugins/docker-buildx
chmod +x ~/.docker/cli-plugins/docker-buildx

# 설치 확인 (v0.14+ 버전 출력 확인)
docker buildx version
```

---

## 5. 카메라 및 센서 디바이스 마운트 설정 (공통 필수)

RealSense 및 USB 카메라 등의 장치를 컨테이너 안으로 자동 포워딩하기 위해 호스트 홈 디렉터리에 실행 인자 설정 파일을 생성합니다.

```bash
echo "-v /dev:/dev" > ~/.isaac_ros_dev-dockerargs
```

---

## 6. 저장소 클론 (`isaac_ros_common` `release-3.2`, `rby1_isaac_ros`)

워크스페이스 `src/`에 두 개를 받습니다. NVIDIA의 `isaac_ros_common`은 컨테이너를 만들고 켜는 스크립트이고, `rby1_isaac_ros`는
이 저장소입니다(이미지 레이어와 RB-Y1 패키지).

```bash
mkdir -p ~/isaac_ros_ws/src && cd ~/isaac_ros_ws/src
git clone -b release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
git clone https://github.com/JeonSangMin-sw2/rby1_isaac_ros.git      # 이미 받았으면 건너뜁니다
```

두 저장소는 **같은 `src/` 안에** 나란히 있어야 합니다. 워크스페이스 자체는 어디에 둬도 됩니다(8.1절).

---

## 7. [중요 필수] x86_64 PC Dockerfile 사전 패치

> [!IMPORTANT]
> **이 패치는 일반 PC (Ubuntu 22.04 x86_64) 환경에서만 필요합니다.**  
> * **x86_64 PC**: x86 apt 저장소에 없는 Jetson 전용 코덱 패키지(`nvv4l2`)를 설치하려다 `Exit code 100` 에러가 발생하므로 아래 예외 처리가 필수입니다.
> * **Jetson (ARM64 / aarch64)**: JetPack 기본 런타임에 이미 포함되어 있어 **이 패치가 전혀 필요 없으며, 아무 수정 없이 빌드**됩니다.  
> *(참고: `Dockerfile.x86_64`와 `Dockerfile.aarch64`는 모두 `Dockerfile.base`를 가리키는 심볼릭 링크입니다).*

* **대상 파일**: `~/isaac_ros_ws/src/isaac_ros_common/docker/Dockerfile.base` (또는 심볼릭 링크인 `Dockerfile.x86_64`)

#### `nvv4l2` 누락 방지 예외 처리 (423번 라인 `extended-amd64` 스테이지 부근)
`release-3.2`에서는 구버전(3.1)에 있던 보안 패키지 버전 충돌(`nghttp2` 등)이 이미 엔비디아 측에서 해결되어 제거되었으므로, **x86 PC 빌드 시 아래 423번 라인의 `nvv4l2` 예외 처리 하나만 적용**해주시면 됩니다:

```dockerfile
# 수정 전:
# apt-get update && apt-get install -y \
#     nvv4l2 \
#     && ln -s /usr/lib/x86_64-linux-gnu/libnvcuvid.so.1 /usr/lib/x86_64-linux-gnu/libnvcuvid.so \
#     && ln -s /usr/lib/x86_64-linux-gnu/libnvidia-encode.so.1 /usr/lib/x86_64-linux-gnu/libnvidia-encode.so

# 수정 후:
RUN --mount=type=cache,target=/var/cache/apt \
    (apt-get update && apt-get install -y nvv4l2 || true) \
    && (ln -s /usr/lib/x86_64-linux-gnu/libnvcuvid.so.1 /usr/lib/x86_64-linux-gnu/libnvcuvid.so || true) \
    && (ln -s /usr/lib/x86_64-linux-gnu/libnvidia-encode.so.1 /usr/lib/x86_64-linux-gnu/libnvidia-encode.so || true)
```

---

## 8. 환경 구성 완료 검증 및 스마트 단축 커맨드(`isaac-ros`) 등록

호스트 설정이 올바르게 되었는지 `isaac_ros_common` 컨테이너를 빌드하고 진입하여 검증합니다.

### 8.1. 최초 컨테이너 빌드 & 진입 테스트

이 저장소(`rby1_isaac_ros`)는 cuMotion·AprilTag·영상 보정을 **이미지 레이어 하나**(`docker/Dockerfile.cumotion`)로 넣습니다.
아래 1~3을 **순서대로 모두** 합니다. 1·2를 빼면 이 저장소의 레이어가 없는 이미지가 만들어지거나 빌드가 `No model bundles`로 멈춥니다.

```bash
# 1. 이미지에 넣을 로봇 모델과 드라이버 메시지 준비 (드라이버 워크스페이스가 빌드돼 있어야 함)
cd ~/isaac_ros_ws/src/rby1_isaac_ros/docker
./make_bundles.sh                      # 첫 두 줄에 찾은 드라이버 워크스페이스와 SDK 경로가 나옵니다
ls bundles                             # a_1_0 … m_1_3 폴더 7개

# 2. 이 저장소의 이미지 레이어를 쓰도록 설정
cp isaac_ros_common-config ~/.isaac_ros_common-config

# 3. 이미지 만들고 컨테이너에 들어가기
cd ~/isaac_ros_ws/src/isaac_ros_common
ISAAC_ROS_WS=$HOME/isaac_ros_ws ./scripts/run_dev.sh
```

* 멈추면 [9.4절 막힐 때](#94-막힐-때)의 표를 봅니다(PC에도 해당하는 항목이 있습니다).
* 첫 빌드는 수십 분 걸립니다. 여기서 만드는 것은 **이미지**이고, 이 저장소의 패키지 빌드(`colcon build`)는 그 다음에
  컨테이너 안에서 합니다([README 2장](../README.md#2--컨테이너에서-패키지-빌드)).
* 최초 실행 시 도커 베이스 이미지 빌드가 진행되며, 완료 후 아래와 같은 컨테이너 셸이 뜨면 **환경 구성이 100% 성공**한 것입니다:
  ```bash
  admin@<hostname>:/workspaces/isaac_ros-dev$
  ```
* 컨테이너 확인 후 `exit` 명령어로 빠져나옵니다.

**워크스페이스를 다른 경로에 뒀을 때** — 이 문서의 명령은 `~/isaac_ros_ws`로 적혀 있습니다. 다른 곳이면 경로를 한 번 정해 두고,
명령의 `~/isaac_ros_ws` 자리에 `$ISAAC_ROS_WS`를 씁니다. 나머지 경로는 스스로 찾습니다.

```bash
echo 'export ISAAC_ROS_WS=$HOME/<내 경로>' >> ~/.bashrc && source ~/.bashrc
```

| 무엇 | 어떻게 찾나 |
|---|---|
| 드라이버 워크스페이스 (`make_bundles.sh`) | 지금 터미널에 source된 것 → 없으면 홈 아래에서 빌드된 것 하나. 여러 개면 멈추고 `DRIVER_WS=<경로>`를 요구합니다 |
| SDK의 `models` (`make_bundles.sh`) | 빌드된 드라이버가 링크한 SDK → 없으면 `~/sdk/rby1-sdk`. 직접 주려면 `RBY1_SDK_MODELS=<경로>` |
| 이 저장소의 이미지 레이어 (`~/.isaac_ros_common-config`) | `isaac_ros_common` 옆의 `rby1_isaac_ros`. 두 저장소가 같은 `src/`에 있기만 하면 됩니다 |

### 8.2. 어디서나 한 번에 접속하는 스마트 단축 커맨드 등록 (`isaac-ros`)
매번 긴 경로로 이동해서 스크립트를 칠 필요 없이 터미널 어디서든 컨테이너가 꺼져 있으면 자동 구동하고, 이미 켜져 있으면 즉시 추가 터미널로 연결하는 스마트 단축 함수를 `~/.bashrc`에 등록합니다:

```bash
# ~/.bashrc에 스마트 단축 함수 추가 (호스트 터미널에서 1회 실행)
cat << 'EOF' >> ~/.bashrc

# Isaac ROS CLI Helper
isaac-ros() {
    # Elsewhere than ~/isaac_ros_ws: export ISAAC_ROS_WS in ~/.bashrc, above this function.
    local target_ws="${ISAAC_ROS_WS:-${HOME}/isaac_ros_ws}"
    # x86_64 on a PC, aarch64 on a Jetson: run_dev.sh names the container after it.
    local container_name="isaac_ros_dev-$(uname -m)-container"

    if docker ps --format '{{.Names}}' | grep -q "^${container_name}$"; then
        echo "Attaching to running container: ${container_name}..."
        # -u admin: run_dev.sh creates admin with your host UID. As root, ROS 2
        # shared-memory transport cannot exchange data with host processes
        # (e.g. the RB-Y1 driver): discovery works, topics never arrive.
        docker exec -it -u admin -w /workspaces/isaac_ros-dev "${container_name}" bash
    else
        echo "No running container found. Starting via run_dev.sh..."
        cd "${target_ws}/src/isaac_ros_common" && ISAAC_ROS_WS="${target_ws}" ./scripts/run_dev.sh
    fi
}
EOF

source ~/.bashrc

# 이제 터미널 어디서든 아래 한 단어로 컨테이너 자동 시작 및 진입!
isaac-ros
```

> [!IMPORTANT]
> **컨테이너에는 `admin`으로 들어가십시오 (root 아님).** 프롬프트가 `root@…`이면 잘못 들어온 것입니다.
> ROS 2(FastDDS)는 같은 PC의 프로세스끼리 **공유 메모리**로 데이터를 주고받는데, 받는 쪽이 만든 영역에
> 보내는 쪽이 써야 합니다. 컨테이너의 root가 만든 영역에는 호스트의 일반 사용자 프로세스(예: RB-Y1
> 드라이버)가 쓸 수 없어서, **서비스 탐색은 되는데 토픽이 안 오는** 증상이 납니다. `run_dev.sh`가 만드는
> `admin`은 호스트 사용자와 UID가 같아 문제가 없습니다.
>
> 이전에 `-u admin` 없이 등록했다면 `~/.bashrc`의 `docker exec -it "${container_name}" bash` 줄을 위처럼 고치고
> `source ~/.bashrc` 하십시오.

### 8.3. 컨테이너 쓰기

이제부터 컨테이너는 `isaac-ros` 한 단어로 켜고 들어갑니다. 아래 명령의 컨테이너 이름은 PC(`x86_64`)와 Jetson(`aarch64`)에서
`$(uname -m)`이 알아서 바꿉니다.

```bash
isaac-ros                                                    # 켜기·들어가기. 터미널을 더 열 때도 같은 명령
exit                                                         # 컨테이너 셸에서 나오기
docker stop isaac_ros_dev-$(uname -m)-container              # 컨테이너 끄기 (호스트)
```

> [!WARNING]
> 직접 들어갈 때(`docker exec -it -u admin -w /workspaces/isaac_ros-dev isaac_ros_dev-$(uname -m)-container bash`) `-u admin`을
> 빼면 root로 들어가고, root 셸에서는 호스트 ROS 노드(예: RB-Y1 드라이버)의 토픽이 오지 않습니다.

> [!TIP]
> 컨테이너 안에서 `apt`나 `rosdep`으로 설치한 패키지는 컨테이너를 새로 만들면 사라집니다. 남기려면 호스트에서 지금 컨테이너를
> 이미지로 저장합니다: `docker commit isaac_ros_dev-$(uname -m)-container isaac_ros_dev-$(uname -m):latest`

---

## 9. Jetson Orin (JetPack 6.x) 설치 순서

JetPack이 그래픽 드라이버·CUDA·Docker를 이미 갖고 있어서, Jetson은 2~8장 대신 **이 장을 위에서 아래로** 따라 합니다.

> [!CAUTION]
> **저장 공간이 모자라면 이미지를 받거나 만드는 중간에 실패합니다.** 이미지는 x86 PC 실측 약 37 GB(2026-10-15)이고, 받는 동안과
> 빌드 중에는 그보다 더 필요합니다. Jetson에서의 크기는 재지 않았습니다. 시작하기 전에 남은 공간을 확인하십시오:
> `df -h /var/lib/docker /var/lib/containerd`의 `Avail`. 2026-10-13에는 Jetson AGX Orin의 내장 저장소에서 이미지를 받다가
> `no space left on device`로 멈췄습니다.

> [!WARNING]
> 2026-10-13에 Jetson AGX Orin에서 9.2절의 이미지 빌드 직전까지 따라 했고, 그때 나온 문제를 9.4절에 넣었습니다.
> **이미지 빌드부터 그 뒤는 Jetson에서 확인되지 않았습니다.**

**먼저**: 호스트에 ROS 2 Humble, RB-Y1 SDK, 드라이버 워크스페이스를 빌드해 둡니다(드라이버 저장소 README).

### 9.1. 호스트 준비

```bash
# 1. JetPack 확인 — R36, REVISION: 4.x (JetPack 6.1·6.2) 여야 합니다
cat /etc/nv_tegra_release

# 2. Docker 확인 — JetPack에 들어 있습니다. docker.io를 새로 설치하지 않습니다
docker --version
docker buildx version
docker info 2>/dev/null | grep -i runtimes                 # nvidia 가 보여야 합니다
df -h /var/lib/docker /var/lib/containerd                  # 남은 공간(Avail) — 9장 맨 위의 필요 용량 이상

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

# 6. 카메라 같은 장치를 컨테이너에 넘기기
echo "-v /dev:/dev" > ~/.isaac_ros_dev-dockerargs
```

- 2번에서 `docker buildx version`이 안 나오면 [4장](#4-docker-buildx-수동-설치-ubuntu-2204-전용)을 합니다(명령이 arm64 파일을 받습니다).
- 4·5번의 출처는 NVIDIA의 [Jetson Setup for VPI](https://nvidia-isaac-ros.github.io/v/release-3.2/getting_started/hardware_setup/compute/jetson_vpi.html)입니다(2026-10-13 대조).
  NVIDIA가 `release-3.2`에 적은 지원 버전은 JetPack 6.1·6.2뿐입니다.

### 9.2. 저장소 받기, 이미지 만들기

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

- 3번 대신 다른 PC에서 만든 `docker/bundles/`와 `docker/driver_msgs/`를 같은 자리에 복사해도 됩니다(아키텍처와 무관한 파일).
- [7장](#7-중요-필수-x86_64-pc-dockerfile-사전-패치)의 Dockerfile 패치는 **하지 않습니다**(x86 전용).
- 5번이 끝나면 프롬프트가 `admin@…:/workspaces/isaac_ros-dev$`가 됩니다. 컨테이너 이름은 `isaac_ros_dev-aarch64-container`입니다.
- **이어서 [8.2절](#82-어디서나-한-번에-접속하는-스마트-단축-커맨드-등록-isaac-ros)의 `isaac-ros` 함수를 등록합니다.** 이후 컨테이너는
  `isaac-ros`로 켜고 들어갑니다([8.3절](#83-컨테이너-쓰기)).

### 9.3. 컨테이너에서 빌드하고 확인하기

```bash
# 컨테이너 안에서
colcon build --base-paths src/rby1_isaac_ros --symlink-install --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash

echo $TORCH_CUDA_ARCH_LIST                                   # 8.7
python3 -c "import torch; print(torch.cuda.is_available(), torch.cuda.get_device_capability())"   # True (8, 7)
python3 -c "import curobo.curobolib.kinematics, curobo.curobolib.geom"   # 다시 컴파일한다는 줄 없이 끝나야 합니다
ros2 run rby1_cumotion replan_benchmark --runs 10            # 재계획 시간 — 로봇·ROS 없이
ros2 run rby1_cumotion check_plan                            # 계획 1회, 로봇은 움직이지 않음 (cuMotion을 켠 뒤)
```

- 로봇 시뮬레이터 이미지는 x86 전용이라 Jetson에서는 뜨지 않습니다. 실제 로봇에 붙거나, 같은 네트워크의 PC에서 시뮬레이터를 켜고
  드라이버의 `robot_ip`를 그 PC 주소로 둡니다.
- 문서의 계획·추적 시간은 RTX 5070 값입니다. Jetson에서는 `replan_benchmark`로 다시 잽니다. 이동 중 회피는 재계획이 약 0.2초
  안에 끝나는 것을 전제로 하므로(`avoid.lead`), 그보다 길면 `avoid.lead`를 늘리거나 `avoid.replan_seeds`·`replan_steps`를 줄입니다.

### 9.4. 막힐 때

| 증상 | 원인 | 조치 |
|---|---|---|
| `containerd.io : Conflicts: containerd` (`apt install docker.io`) | Docker가 이미 있습니다 | `docker.io`를 빼고 `git-lfs`만 설치합니다. 한 줄에 같이 넣었다면 `git-lfs`도 설치되지 않은 것입니다 |
| `RBY1 SDK models not found …` | SDK를 못 찾음 | `RBY1_SDK_MODELS=<rby1-sdk>/models ./make_bundles.sh` |
| `Driver workspace: N found under …` | 빌드된 드라이버 워크스페이스가 없거나 여러 개 | `DRIVER_WS=<경로> ./make_bundles.sh` |
| `Specified isaac does not exist: …` | `run_dev.sh -d`에 준 경로가 틀림 | `echo $ISAAC_ROS_WS`를 확인하고 `-d` 없이 실행 |
| `Could not resolve Dockerfiles for target image ids: …cumotion` | `~/.isaac_ros_common-config`가 다른 경로를 가리킴(예전에 복사한 파일). 바로 윗줄 `Using configured docker search paths`에 그 경로가 나옵니다 | 9.2절 4번의 `cp`를 다시 합니다 |
| `failed to extract layer … no space left on device` (이미지를 받는 중) | 오류에 나온 경로(`/var/lib/containerd` 또는 `/var/lib/docker`)의 디스크가 가득 참 | `df -h <그 경로>`로 확인. 9장 맨 위의 필요 용량만큼 비운 뒤(`docker system df`로 지울 수 있는 양 확인) 다시 실행 |
| `No model bundles: run docker/make_bundles.sh …` (이미지 빌드 중) | 9.2절 3번을 건너뜀 | 3번을 하고 5번을 다시 실행 |
| 이미지는 만들어졌는데 컨테이너가 안 뜸(`nvidia.com/gpu` 관련 오류) | CDI 설정이 없음 | 9.1절 4번 |
| `… expected one occurrence to patch` (이미지 빌드 중) | NVIDIA가 apt 패키지를 새 버전으로 바꿈(x86에서도 똑같이 멈춥니다) | 메시지가 가리키는 파일을 보고 `docker/patches/`를 맞춥니다 |
| `no kernel image is available for execution on the device` | 커널이 다른 GPU 세대로 컴파일됨 | `echo $TORCH_CUDA_ARCH_LIST`가 `8.7`인지 확인 |
| 첫 계획이 실패 | 5초 안에 안 끝나면 MoveIt 플러그인이 포기합니다(x86에서도 13관절 그룹의 첫 계획은 9.4초) | 런치의 워밍업이 끝난 뒤 다시 보냅니다 |

### 9.5. x86 PC와 달라지는 것

| 항목 | x86_64 PC | Jetson Orin |
|---|---|---|
| 컨테이너 이름 | `isaac_ros_dev-x86_64-container` | `isaac_ros_dev-aarch64-container` — 문서의 명령은 `$(uname -m)`으로 적혀 있어 바꿀 필요가 없습니다(`isaac-ros` 함수도 알아서 고릅니다) |
| 7장의 Dockerfile 패치 | 필요 | **하지 않습니다** |
| 이미지 빌드 파일 | `docker/Dockerfile.cumotion` | **같은 파일.** `run_dev.sh`가 넘기는 `PLATFORM`(amd64·arm64)을 보고 아래 줄만 달라집니다 |
| 바탕 이미지 | `tritonserver:24.08-py3` | `12.6.11-devel-aarch64-ubuntu22.04` (NVIDIA의 `Dockerfile.base`가 고릅니다) |
| torch | 빌드 시점의 최신(버전 고정 없음) | `2.5.0a0 … nv24.08` (JetPack 6.1용으로 고정) |
| NVIDIA apt 패키지 | cuRobo 3.2.5, cuMotion 3.2.7, AprilTag 3.2.5, image_proc 3.2.10 | **같은 버전**(2026-10-13에 저장소 목록 대조) |
| `docker/patches/`의 패치 3개 | 적용 | **그대로 적용됩니다.** 고치는 파일 4개가 x86 패키지와 바이트 단위로 같습니다 |
| cuRobo 커널을 컴파일하는 GPU 세대 | `9.0+PTX` (RTX 50 우회) | `8.7` — `docker/Dockerfile.cumotion`이 플랫폼을 보고 고릅니다 |
| MoveIt (호스트) | `sudo apt install ros-humble-moveit` | 같음 |
| 계획·추적 시간 | 문서의 수치(RTX 5070) | **다시 재야 합니다** |

---

## 10. GPU 문제 해결 — cuMotion을 돌리기 전에

cuMotion(cuRobo)은 CUDA 커널을 직접 컴파일하고 실행합니다. 아래 다섯 가지는 이 프로젝트에서
실제로 겪은 것이고, **`nvidia-smi`가 정상으로 보여도 발생합니다.**

### 10.1. `torch.cuda.is_available()`가 True여도 GPU 준비가 끝난 게 아닙니다

```javascript
nvcc fatal : Unsupported gpu architecture 'compute_120'
RuntimeError: Error building extension 'kinematics_fused_cu'
```

torch는 sm_120 커널을 **미리 빌드해서** 배포하므로 True를 반환합니다. 하지만 cuRobo는
런타임에 `nvcc`로 JIT 컴파일하고, `release-3.2` 컨테이너의 CUDA 12.6은 최대 `compute_90`까지만
압니다. Blackwell(RTX 50xx)은 `compute_120`이라 CUDA 12.8+ 툴킷이 필요합니다.

**확인**

```bash
nvcc --version                                   # 컨테이너 안
python3 -c "import torch; print(torch.cuda.get_device_capability())"
```

**우회** — PTX로 빌드하고 드라이버가 실행 시점에 변환하게 합니다.

```javascript
TORCH_CUDA_ARCH_LIST=9.0+PTX
```

이 값은 `Dockerfile.cumotion`에 ENV로 들어 있습니다. 네이티브 대비 최적은 아니므로
**툴킷을 12.8+로 올리는 것이 정공법**입니다.

### 10.2. 커널 컴파일에는 GPU가 필요 없습니다

`nvcc`만 있으면 되므로 `docker build` 단계에서 미리 컴파일할 수 있습니다(5개 확장, 66 MB, 3분 2초).
그래서 이 프로젝트의 이미지는 커널을 내장하고, 컨테이너마다 3분씩 재컴파일하던 것이
**import 1.7초**로 끝납니다.

> [!NOTE]
> cuRobo는 `load()` 호출 **전에** 무조건 `kinematics_fused_cu not found, JIT compiling...`을
> 출력합니다. **캐시 미적중이 아닙니다.** 소요 시간으로 판단하십시오.

### 10.3. 캐시 디렉터리 권한

```javascript
PermissionError: [Errno 13] Permission denied: '/home/triton-server'
```

컨테이너를 임의 UID로 실행하면 이미지 기본 사용자의 홈에 쓸 수 없습니다.
고정된 쓰기 가능 경로를 이미지 ENV로 굽습니다.

```javascript
TORCH_EXTENSIONS_DIR=/opt/rby1/torch_extensions
TRITON_CACHE_DIR=/opt/rby1/triton
XDG_CACHE_HOME=/opt/rby1/xdg
```

빌드 마지막에 `chmod -R a+rwX /opt/rby1`을 겁니다.

### 10.4. 컨테이너가 실행 중 GPU 접근을 잃습니다

```javascript
Failed to initialize NVML: Unknown Error
```

`/dev/nvidia*`는 멀쩡한데 NVML만 실패합니다. systemd cgroup 재로드 시 발생하는 알려진 문제로,
**컨테이너 재시작으로 복구**됩니다. 이미지를 다시 빌드할 필요는 없습니다.

```bash
docker restart isaac_ros_dev-$(uname -m)-container
```

### 10.5. `Driver/library version mismatch`

부팅 후 `unattended-upgrade`가 NVIDIA 패키지를 교체하면, **메모리에 로드된 커널 모듈**과
**설치된 NVML/사용자 라이브러리** 버전이 어긋납니다. 2026-09-15 실제 사례:

| 항목 | 값 |
|---|---|
| 부팅 | 08:53:37 |
| `unattended-upgrade` 실행 | 09:26:31–09:27:15 |
| 메모리에 로드된 커널 모듈 | 580.173.02 |
| 설치된 NVML / 사용자 라이브러리 | 580.178.04 |
| 디스크의 커널 모듈 / DKMS | 580.178.04 (현재 커널에 설치 완료) |

**호스트에서 발생한 문제이므로 Docker 이미지를 다시 빌드해도 해결되지 않습니다.**
새 모듈은 이미 설치돼 있으니 재부팅이 해법입니다.

```bash
# 현재 값 확인
cat /sys/module/nvidia/version      # 메모리에 로드된 것
modinfo -F version nvidia           # 디스크에 있는 것
nvidia-smi

# 작업 저장 후
sudo reboot

# 재부팅 이후 — 세 값이 일치해야 합니다
cat /sys/module/nvidia/version
nvidia-smi
docker run --rm --gpus all --entrypoint nvidia-smi isaac_ros_dev-$(uname -m):latest
```

[Ubuntu 공식 문서](https://ubuntu.com/server/docs/how-to/graphics/install-nvidia-drivers/)도
업데이트 후 모듈/라이브러리 불일치에 재부팅을 안내합니다.

### 10.6. GPU 메모리

**cuMotion 플래너 1개가 GPU 7\~10.5 GB를 점유합니다.** 12 GB 카드에는 하나만 뜹니다.

```bash
nvidia-smi --query-gpu=memory.used,memory.total --format=csv,noheader
```

이전 인스턴스를 확실히 종료하지 않고 다시 띄우면 `torch.OutOfMemoryError`로 죽습니다.
`pkill -f "ros2 launch"`는 자식 노드를 남기므로 **프로세스 그룹 단위로** 정리해야 합니다
(방법은 [tutorial_cumotion.md](tutorial_cumotion.md) §8).

---

## 11. 다음 단계

호스트 환경 구성 및 검증이 완료되었습니다.

- 👉 **[Isaac ROS 메인 가이드](../README.md)**
- 👉 **[RBY1 cuMotion 가이드](tutorial_cumotion.md)** — 켜기, 예제, 설정 (이미지와 패키지 빌드는 README 2장)
