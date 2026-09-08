# [환경 구성] Ubuntu 24.04 LTS (ROS 2 Jazzy / release-4.0+)

본 문서는 **Ubuntu 24.04 LTS 호스트 PC(x86_64)** 및 **Jetson Orin (JetPack 7.x)** 환경에서 Isaac ROS `release-4.0` 이상 컨테이너를 구동하기 위한 호스트 사전 요구사항 설정 및 검증 가이드입니다.

호스트 설정부터 `isaac_ros_common` 컨테이너에 정상 진입하고 단축 커맨드를 등록하는 단계까지 완료하면 기본 환경 구성이 끝납니다.

---

## 1. 지원 버전 매트릭스

* **호스트 OS**: Ubuntu 24.04 LTS
* **타깃 Isaac ROS 브랜치**: **`release-4.0` 이상** 또는 `main` (Jazzy 지원 브랜치)
* **컨테이너 내부 환경**: Ubuntu 24.04 LTS + ROS 2 Jazzy
* **NVIDIA 드라이버**: 560+ 오픈소스 커널 모듈(`-open`) 권장

---

## 2. NVIDIA 그래픽 드라이버 설정

### 2.1. 현재 설치된 드라이버 확인
이미 NVIDIA 드라이버가 설치되어 있다면 먼저 정상 동작 여부를 확인합니다:
```bash
nvidia-smi
```
* 상단에 GPU 모델명과 드라이버 버전이 정상 출력된다면 드라이버 재설치는 건너뛰고 **[3. Docker 및 Docker Buildx 설치]**로 넘어가도 됩니다.

### 2.2. 드라이버 설치 또는 변경 (필요 시)
특정 드라이버 버전을 수동으로 강제 설치할 경우 디스플레이 서버나 모니터 출력 설정에 따라 **화면 미출력(No Signal / 블랙스크린)** 현상이 일어날 수 있습니다.

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

## 3. Docker 및 Docker Buildx 설치 (Ubuntu 24.04는 apt 지원)

Ubuntu 24.04는 `docker-buildx`를 공식 apt 저장소에서 바로 설치할 수 있습니다:

```bash
sudo apt update
sudo apt install -y docker.io docker-buildx git git-lfs
sudo usermod -aG docker $USER
newgrp docker

# buildx 설치 확인
docker buildx version
```

---

## 4. NVIDIA Container Toolkit 설정

```bash
# 1. 저장소 GPG 키 및 소스 리스트 등록
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# 2. 툴킷 설치
sudo apt update
sudo apt install -y nvidia-container-toolkit

# 3. Docker 런타임 등록 및 데몬 재시작
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

---

## 5. 카메라 및 센서 디바이스 마운트 설정 (공통 필수)

RealSense 및 USB 카메라 등의 장치를 컨테이너 안으로 자동 포워딩하기 위해 호스트 홈 디렉터리에 실행 인자 설정 파일을 생성합니다.

```bash
echo "-v /dev:/dev" > ~/.isaac_ros_dev-dockerargs
```

---

## 6. 환경 구성 완료 검증 및 단축 커맨드 등록

호스트 설정이 올바르게 되었는지 `isaac_ros_common` 컨테이너를 빌드하고 진입하여 검증합니다.

### 6.1. 레포지토리 및 common 패키지 확인
`rby1_isaac_ros` 레포지토리에 `isaac_ros_common`이 포함되어 있으므로 별도 클론 없이 바로 사용합니다:
```bash
cd ~/isaac_ros_ws/src/rby1_isaac_ros/isaac_ros_common
```

### 6.2. 최초 컨테이너 빌드 & 진입 테스트
```bash
cd ~/isaac_ros_ws/src/rby1_isaac_ros/isaac_ros_common
ISAAC_ROS_WS=$HOME/isaac_ros_ws ./scripts/run_dev.sh
```
* 최초 실행 시 도커 베이스 이미지 빌드가 진행되며, 완료 후 아래와 같은 컨테이너 셸이 뜨면 **환경 구성이 100% 성공**한 것입니다:
  ```bash
  admin@<hostname>:/workspaces/isaac_ros-dev$
  ```
* 컨테이너 확인 후 `exit` 명령어로 빠져나옵니다.

### 6.3. 어디서나 한 번에 접속하는 스마트 단축 커맨드 등록 (`isaac-ros`)
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
        docker exec -it "${container_name}" bash
    else
        echo "No running container found. Starting via run_dev.sh..."
        cd "${target_ws}/src/rby1_isaac_ros/isaac_ros_common" && ISAAC_ROS_WS="${target_ws}" ./scripts/run_dev.sh
    fi
}
EOF

source ~/.bashrc

# 이제 터미널 어디서든 아래 한 단어로 컨테이너 자동 시작 및 진입!
isaac-ros
```

---

## 7. 다음 단계

호스트 환경 설정 및 검증이 완료되었습니다. 이제 메인 가이드로 돌아가 원하는 패키지를 추가하고 빌드 및 구동을 진행합니다:  
👉 **[Isaac ROS 메인 가이드로 이동](../README.md)**
