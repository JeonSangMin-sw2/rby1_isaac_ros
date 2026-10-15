# [환경 구성] Ubuntu 22.04 LTS PC (ROS 2 Humble / release-3.2)

x86_64 PC에서 Isaac ROS `release-3.2` 컨테이너를 쓰기까지 **해야 하는 것만** 순서대로 적었습니다. 이 문서를 끝내면 이미지와 컨테이너,
컨테이너 단축 명령 `isaac-ros`가 준비됩니다.

- Jetson은 👉 [env_setup_jetson.md](env_setup_jetson.md)
- 설치 중 문제가 생기면 👉 [developer_manual.md — 환경 구성](developer_manual.md#환경-구성)

> [!IMPORTANT]
> 시작하기 전에 호스트에 ROS 2 Humble, RB-Y1 SDK, 드라이버 워크스페이스를 빌드해 둡니다(드라이버 저장소 README).
> 7장에서 이미지에 넣을 로봇 모델을 드라이버 워크스페이스에서 가져옵니다.

---

## 1. 지원 버전

* **호스트 OS**: Ubuntu 22.04 LTS
* **Isaac ROS**: **`release-3.2`** (ROS 2 Humble)
* **컨테이너 내부 환경**: Ubuntu 22.04 LTS + ROS 2 Humble
* **NVIDIA 드라이버**: 호스트 GPU 아키텍처에 맞는 오픈소스 커널 모듈(`-open`) 권장

---

## 2. NVIDIA 그래픽 드라이버

```bash
nvidia-smi                               # GPU 모델명과 드라이버 버전이 나오면 3장으로 넘어갑니다
```

나오지 않으면 우분투가 권장하는 드라이버를 설치합니다(버전을 직접 고르면 화면이 안 나오는 경우가 있습니다).

```bash
sudo apt update
sudo apt install -y linux-headers-$(uname -r) build-essential dkms
ubuntu-drivers devices                   # 권장(recommended) 드라이버 확인
sudo ubuntu-drivers install
sudo reboot
```

> [!NOTE]
> 설치 뒤 화면이 나오지 않으면 [developer_manual — 그래픽 드라이버 설치 뒤 화면이 안 나올 때](developer_manual.md#1-그래픽-드라이버-설치-뒤-화면이-안-나올-때).

---

## 3. Docker, NVIDIA Container Toolkit

```bash
# 1. Docker — `docker --version`이 이미 나오면 이 두 줄은 건너뜁니다
sudo apt update
sudo apt install -y docker.io

# 2. git-lfs와 docker 그룹 (항상)
sudo apt install -y git git-lfs
sudo usermod -aG docker $USER
newgrp docker

# 3. NVIDIA Container Toolkit
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt update
sudo apt install -y nvidia-container-toolkit

# 4. Docker에 nvidia 런타임 등록
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

---

## 4. Docker Buildx

Ubuntu 22.04의 apt 저장소에는 최신 `docker-buildx`가 없어서 GitHub에서 받습니다.

```bash
mkdir -p ~/.docker/cli-plugins
# PC(x86_64)는 amd64, Jetson(aarch64)은 arm64 파일을 받습니다
ARCH=$([ "$(uname -m)" = "aarch64" ] && echo arm64 || echo amd64)
curl -sSL https://github.com/docker/buildx/releases/download/v0.14.1/buildx-v0.14.1.linux-${ARCH} -o ~/.docker/cli-plugins/docker-buildx
chmod +x ~/.docker/cli-plugins/docker-buildx
docker buildx version                    # v0.14 이상
```

---

## 5. 저장소 클론

워크스페이스 `src/`에 두 개를 받습니다. NVIDIA의 `isaac_ros_common`은 컨테이너를 만들고 켜는 스크립트이고, `rby1_isaac_ros`는
이 저장소입니다(이미지 레이어와 RB-Y1 패키지).

```bash
mkdir -p ~/isaac_ros_ws/src && cd ~/isaac_ros_ws/src
git clone -b release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
git clone https://github.com/JeonSangMin-sw2/rby1_isaac_ros.git      # 이미 받았으면 건너뜁니다
```

> [!NOTE]
> 두 저장소는 **같은 `src/` 안에** 나란히 있어야 합니다. 워크스페이스를 `~/isaac_ros_ws`가 아닌 곳에 두면
> `echo 'export ISAAC_ROS_WS=$HOME/<내 경로>' >> ~/.bashrc && source ~/.bashrc`를 해 두고, 이 문서 명령의 `~/isaac_ros_ws` 자리에
> `$ISAAC_ROS_WS`를 씁니다.

---

## 6. NVIDIA Dockerfile 패치 (x86 PC 필수)

NVIDIA의 바탕 Dockerfile이 x86 저장소에 없는 Jetson 전용 패키지(`nvv4l2`)를 설치하려다 `Exit code 100`으로 멈춥니다.
`~/isaac_ros_ws/src/isaac_ros_common/docker/Dockerfile.base`의 `extended-amd64` 단계(423번 줄 부근)를 아래처럼 고칩니다.

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

## 7. 이미지 만들기와 컨테이너

### 7.1. 이미지 만들기

아래 1~3을 **순서대로 모두** 합니다. 1·2를 빼면 이 저장소의 레이어가 없는 이미지가 만들어지거나 빌드가 `No model bundles`로 멈춥니다.

```bash
# 1. 이미지에 넣을 로봇 모델과 드라이버 메시지 준비
cd ~/isaac_ros_ws/src/rby1_isaac_ros/docker
./make_bundles.sh                      # 첫 두 줄에 찾은 드라이버 워크스페이스와 SDK 경로가 나옵니다
ls bundles                             # a_1_0 … m_1_3 폴더 7개

# 2. 이 저장소의 이미지 레이어를 쓰도록 설정
cp isaac_ros_common-config ~/.isaac_ros_common-config

# 3. 이미지 만들고 컨테이너에 들어가기 (처음에는 수십 분)
cd ~/isaac_ros_ws/src/isaac_ros_common
ISAAC_ROS_WS=$HOME/isaac_ros_ws ./scripts/run_dev.sh
```

프롬프트가 `admin@<hostname>:/workspaces/isaac_ros-dev$`로 바뀌면 됩니다. `exit`로 나옵니다.

> [!NOTE]
> 이미지는 약 37 GB입니다(2026-10-15 실측). 받는 동안과 빌드 중에는 그보다 더 필요합니다.

### 7.2. `isaac-ros` 명령 등록

터미널 어디서든 한 단어로 컨테이너를 켜고(꺼져 있으면) 들어가는(켜져 있으면) 함수를 `~/.bashrc`에 넣습니다. 한 번만 합니다.

```bash
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
```

### 7.3. 컨테이너 쓰기

```bash
isaac-ros                                                    # 켜기·들어가기. 터미널을 더 열 때도 같은 명령
exit                                                         # 컨테이너 셸에서 나오기
docker stop isaac_ros_dev-$(uname -m)-container              # 컨테이너 끄기 (호스트)
```

> [!IMPORTANT]
> 컨테이너에는 **`admin`으로** 들어갑니다(프롬프트 `admin@…`). `root@…`이면 호스트 드라이버의 토픽이 오지 않습니다 —
> 나가서 `isaac-ros`로 다시 들어갑니다. 이유는 [developer_manual — 컨테이너에 root로 들어가면](developer_manual.md#4-컨테이너에-root로-들어가면).

> [!TIP]
> 컨테이너 안에서 `apt`나 `rosdep`으로 설치한 패키지는 컨테이너를 새로 만들면 사라집니다. 남기려면 호스트에서 지금 컨테이너를
> 이미지로 저장합니다: `docker commit isaac_ros_dev-$(uname -m)-container isaac_ros_dev-$(uname -m):latest`

---

## 8. 다음 단계

- 👉 **[README 2장](../README.md#2--컨테이너에서-패키지-빌드)** — 컨테이너 안에서 이 저장소의 패키지 빌드
- 👉 **[RBY1 cuMotion 가이드](tutorial_cumotion.md)** — 켜기, 예제, 설정
