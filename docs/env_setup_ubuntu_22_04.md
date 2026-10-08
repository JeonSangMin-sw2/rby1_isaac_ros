# [환경 구성] Ubuntu 22.04 LTS (ROS 2 Humble / release-3.2)

본 문서는 **Ubuntu 22.04 LTS 호스트 PC(x86_64)** 및 **Jetson Orin (JetPack 6.x)** 환경에서 Isaac ROS `release-3.2` 컨테이너를 구동하기 위한 호스트 사전 요구사항 설정 및 검증 가이드입니다.

호스트 설정부터 `isaac_ros_common` 컨테이너에 정상 진입하고 단축 커맨드를 등록하는 단계까지 완료하면 기본 환경 구성이 끝납니다.

---

## 1. 지원 버전 매트릭스

* **호스트 OS**: Ubuntu 22.04 LTS
* **타깃 Isaac ROS 브랜치**: **`release-3.2`** (Humble 기준 공식 권장 표준)
* **컨테이너 내부 환경**: Ubuntu 22.04 LTS + ROS 2 Humble
* **NVIDIA 드라이버**: 호스트 GPU 아키텍처에 맞는 오픈소스 커널 모듈(`-open`) 권장

---

## 2. NVIDIA 그래픽 드라이버 설정

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

```bash
# 1. Docker 기본 설치 (이미 설치된 경우 생략)
sudo apt update
sudo apt install -y docker.io git git-lfs
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
curl -sSL https://github.com/docker/buildx/releases/download/v0.14.1/buildx-v0.14.1.linux-amd64 -o ~/.docker/cli-plugins/docker-buildx
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

## 6. `isaac_ros_common` 클론 (`release-3.2`)

도커 환경 구동을 위해 NVIDIA 공식 `isaac_ros_common` 패키지를 워크스페이스 `src/` 디렉터리에 클론합니다:

```bash
cd ~/isaac_ros_ws/src
git clone -b release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
```

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
```bash
cd ~/isaac_ros_ws/src/isaac_ros_common
ISAAC_ROS_WS=$HOME/isaac_ros_ws ./scripts/run_dev.sh
```
* 최초 실행 시 도커 베이스 이미지 빌드가 진행되며, 완료 후 아래와 같은 컨테이너 셸이 뜨면 **환경 구성이 100% 성공**한 것입니다:
  ```bash
  admin@<hostname>:/workspaces/isaac_ros-dev$
  ```
* 컨테이너 확인 후 `exit` 명령어로 빠져나옵니다.

### 8.2. 어디서나 한 번에 접속하는 스마트 단축 커맨드 등록 (`isaac-ros`)
매번 긴 경로로 이동해서 스크립트를 칠 필요 없이 터미널 어디서든 컨테이너가 꺼져 있으면 자동 구동하고, 이미 켜져 있으면 즉시 추가 터미널로 연결하는 스마트 단축 함수를 `~/.bashrc`에 등록합니다:

```bash
# ~/.bashrc에 스마트 단축 함수 추가 (호스트 터미널에서 1회 실행)
cat << 'EOF' >> ~/.bashrc

# Isaac ROS CLI Helper
isaac-ros() {
    local target_ws="${HOME}/isaac_ros_ws"
    local container_name="isaac_ros_dev-x86_64-container"

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

> ⚠️ **컨테이너에는 `admin`으로 들어가십시오 (root 아님).** 프롬프트가 `root@…`이면 잘못 들어온 것입니다.
> ROS 2(FastDDS)는 같은 PC의 프로세스끼리 **공유 메모리**로 데이터를 주고받는데, 받는 쪽이 만든 영역에
> 보내는 쪽이 써야 합니다. 컨테이너의 root가 만든 영역에는 호스트의 일반 사용자 프로세스(예: RB-Y1
> 드라이버)가 쓸 수 없어서, **서비스 탐색은 되는데 토픽이 안 오는** 증상이 납니다. `run_dev.sh`가 만드는
> `admin`은 호스트 사용자와 UID가 같아 문제가 없습니다.
>
> 이전에 `-u admin` 없이 등록했다면 `~/.bashrc`의 `docker exec -it "${container_name}" bash` 줄을 위처럼 고치고
> `source ~/.bashrc` 하십시오.

---

## 9. Jetson Orin (JetPack 6.x) 특이사항

Jetson Orin 환경의 경우 JetPack이 드라이버와 CUDA를 기본 제공하므로:
1. Docker 데몬 설정 확인:
   ```bash
   cat /etc/docker/daemon.json
   # "default-runtime": "nvidia" 항목 존재 확인
   ```
2. 디바이스 마운트 인자 생성:
   ```bash
   echo "-v /dev:/dev" > ~/.isaac_ros_dev-dockerargs
   ```
3. 동일하게 `isaac-ros` 단축 커맨드로 실행하면 `aarch64` 전용 컨테이너로 자동 진입합니다.

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

> ⚠️ cuRobo는 `load()` 호출 **전에** 무조건 `kinematics_fused_cu not found, JIT compiling...`을
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
docker restart isaac_ros_dev-x86_64-container
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
docker run --rm --gpus all --entrypoint nvidia-smi isaac_ros_dev-x86_64:latest
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
- 👉 **[RBY1 cuMotion 순차 가이드](tutorial_cumotion.md)** — 이미지 빌드부터 실기체까지
