# RM520N-ROS2

RM520N(Plus)'s Examples for ROS2.

/ Version 1.0


## Package Information

- Package:	rm520n_multi_driver
- Node:		rm520n_multi_driver_node
 
Published Topics:
```

```

## Quick Start

Launch the node (defualt serial port is `/dev/ttyUSB0` for UART on the UP2 pin header):
```
ros2 launch rm520n_multi_driver rm520n_multi_driver.launch.py
```
```
ros2 launch rm520n_multi_driver rm520n_multi_driver.launch.py \
  params_file:=/path/to/your_params.yaml
```


## Set up serial Devices rules

### RM520N devices(ttyUSB6 →  symlink name: rm520n) 
```
KERNEL=="ttyUSB*", KERNELS=="2-3.1:1.3", SYMLINK+="rm520n", GROUP="dialout", MODE="0660"
```
### RM520N-gps devices(ttyUSB4 →  symlink name: rm520n-gps) 
```
KERNEL=="ttyUSB*", KERNELS=="2-3.1:1.1", SYMLINK+="gps", GROUP="dialout", MODE="0660"
```

# RM520N Multi Driver - ROS2 패키지 기술문서

## 1. 개요

### 1.1 패키지 정보
- **패키지명**: rm520n_multi_driver
- **버전**: 0.0.1
- **설명**: Quectel RM520N-GL 모듈의 GPS 데이터 및 네트워크 상태를 통합 관리하는 ROS2 드라이버
- **라이선스**: Apache-2.0
- **관리자**: sylee@avgenius.kr

### 1.2 주요 기능
- GPS 위치 정보 수신 및 발행 (NMEA 문장 파싱)
- 네트워크 상태 모니터링 (LTE/5G SA/5G NSA)
- 통신사 정보, 신호 강도(RSRP, RSRQ, SINR) 실시간 수집
- 이중 시리얼 포트 관리 (GPS/AT 명령 분리)

## 2. 시스템 요구사항

### 2.1 의존성 패키지
- **빌드 도구**: ament_cmake
- **ROS2 패키지**:
  - rclcpp (ROS2 C++ 클라이언트 라이브러리)
  - rclcpp_components (컴포넌트 기반 노드 지원)
  - std_msgs (표준 메시지 타입)
  - sensor_msgs (센서 데이터 메시지)
  - tod_msgs (네트워크 상태 커스텀 메시지)

### 2.2 하드웨어 요구사항
- Quectel RM520N-GL 5G 모듈
- USB 시리얼 인터페이스 (기본: /dev/ttyUSB0)
- GPS 안테나 (모듈 연결)

## 3. 아키텍처

### 3.1 노드 구조
```
RM520NMultiDriverNode
├── GPS 시리얼 인터페이스 (SerialInterface)
├── AT 명령 인터페이스 (ATInterface)
├── NMEA 파서 (NMEAParser)
└── 타이머 기반 퍼블리셔
    ├── GPS 데이터 발행 (gps_fix)
    └── 네트워크 상태 발행 (network_status)
```

### 3.2 컴포넌트 설명

#### 3.2.1 SerialInterface
시리얼 포트 통신을 담당하는 저수준 인터페이스입니다.

**주요 기능**:
- 시리얼 포트 초기화 및 구성 (보드레이트, 패리티 등)
- DTR/RTS 모뎀 제어 신호 설정
- 라인 단위 읽기/쓰기 (AT 명령용)
- Raw 데이터 읽기 (GPS NMEA 스트림용)
- 버퍼 플러시 기능

**지원 보드레이트**: 9600, 19200, 38400, 57600, 115200, 230400, 921600 bps

#### 3.2.2 ATInterface
AT 명령을 통한 모듈 제어 및 상태 조회 인터페이스입니다.

**주요 AT 명령**:
- `AT+QSPN`: 통신사명 조회
- `AT+QENG="servingcell"`: 현재 연결된 셀 정보 (신호 강도, 접속 기술)
- `AT+CEREG?`: 네트워크 등록 상태
- `AT+CGATT?`: PS(Packet Switched) Attach 상태
- `AT+QGPS=1`: GPS 모듈 활성화

**파싱 데이터**:
- 접속 기술 (LTE, NR5G-SA, NR5G-NSA)
- RSRP (Reference Signal Received Power)
- RSRQ (Reference Signal Received Quality)
- SINR (Signal to Interference plus Noise Ratio)
- 등록/연결 상태

#### 3.2.3 NMEAParser
GPS NMEA 0183 포맷 문장을 파싱하여 위경도 좌표로 변환합니다.

**지원 문장 타입**:
- `$GPGGA` / `$GNGGA`: GPS Fix 데이터 (고도 포함)
- `$GPRMC` / `$GNRMC`: 권장 최소 항법 정보

**파싱 출력**:
- 위도 (Latitude, 십진수 도)
- 경도 (Longitude, 십진수 도)
- 고도 (Altitude, 미터) - GGA 문장만 해당

**검증 기능**:
- NMEA Checksum 검증
- Fix Quality 확인 (GGA)
- Status 확인 (RMC: A=유효, V=무효)

#### 3.2.4 RM520NMultiDriverNode
메인 ROS2 노드로, 모든 컴포넌트를 통합 관리합니다.

**타이머 콜백**:
- `timerCallback()`: 네트워크 상태 발행 (기본 500ms 주기)
- `gpsCallback()`: GPS 데이터 발행 (기본 1000ms 주기)

**멀티스레딩**:
- `atPollingWorker()`: 백그라운드 스레드에서 AT 명령 폴링하여 최신 네트워크 상태 유지
- Mutex를 통한 스레드 안전 보장

## 4. 발행 토픽

### 4.1 gps_fix
- **타입**: `sensor_msgs/msg/NavSatFix`
- **주기**: 1000ms (설정 가능)
- **내용**:
  - `header.stamp`: 타임스탬프
  - `header.frame_id`: "gps"
  - `latitude`: 위도 (도)
  - `longitude`: 경도 (도)
  - `altitude`: 고도 (미터)
  - `status.status`: Fix 상태 (FIX/NO_FIX)
  - `status.service`: GPS 서비스 타입

### 4.2 network_status
- **타입**: `tod_msgs/msg/NetworkStatus`
- **주기**: 500ms (설정 가능)
- **내용**:
  - `stamp`: 타임스탬프
  - `access_technology`: 접속 기술 (LTE/NR5G-SA/NR5G-NSA/UNKNOWN)
  - `carrier`: 통신사명 (예: SKTelecom)
  - `rsrp`: RSRP 값 (dBm)
  - `rsrq`: RSRQ 값 (dB)
  - `sinr`: SINR 값 (dB)
  - `signal_quality`: 신호 품질 (0-100%)
  - `registered`: 네트워크 등록 여부
  - `attached`: PS Attach 여부
  - `connected`: 전체 연결 상태
  - `gps_status`: GPS Fix 여부

**신호 품질 계산**:
RSRP 값을 기준으로 0-100% 선형 변환합니다.
- RSRP ≥ -44 dBm: 100%
- RSRP ≤ -140 dBm: 0%
- 중간값: 선형 보간

## 5. 설정 파라미터

### 5.1 파라미터 파일 (rm520n_params.yaml)

```yaml
/**:
  ros__parameters:
    # GPS 시리얼 포트 설정
    gps_port: "/dev/gps"
    gps_baudrate: 115200
    
    # AT 명령 시리얼 포트 설정
    at_port: "/dev/rm520n"
    at_baudrate: 115200
    
    # 폴링 주기 설정
    polling_interval_ms: 500    # 네트워크 상태 발행 주기
    gps_interval_ms: 1000       # GPS 데이터 발행 주기
    
    # 토픽명 설정
    publish_topics:
      gps_topic: "gps_fix"
      status_topic: "network_status"
```

### 5.2 파라미터 설명

| 파라미터 | 타입 | 기본값 | 설명 |
|---------|------|--------|------|
| gps_port | string | /dev/ttyUSB4 | GPS NMEA 데이터 시리얼 포트 |
| gps_baudrate | int | 115200 | GPS 포트 보드레이트 |
| at_port | string | /dev/ttyUSB5 | AT 명령 시리얼 포트 |
| at_baudrate | int | 115200 | AT 포트 보드레이트 |
| polling_interval_ms | int | 1000 | 네트워크 상태 발행 주기 (밀리초) |
| gps_interval_ms | int | 1000 | GPS 데이터 발행 주기 (밀리초) |
| publish_topics.gps_topic | string | gps_fix | GPS 토픽명 |
| publish_topics.status_topic | string | network_status | 네트워크 상태 토픽명 |

## 6. 빌드 및 설치

### 6.1 빌드
```bash
cd ~/ros2_ws
colcon build --packages-select rm520n_multi_driver
```

### 6.2 설치 경로
- **라이브러리**: `${CMAKE_INSTALL_LIBDIR}`
- **실행 파일**: `${CMAKE_INSTALL_LIBDIR}/rm520n_multi_driver`
- **헤더 파일**: `include/`
- **Launch 파일**: `share/rm520n_multi_driver/launch/`
- **설정 파일**: `share/rm520n_multi_driver/config/`

## 7. 실행 방법

### 7.1 기본 실행
```bash
ros2 launch rm520n_multi_driver rm520n_multi_driver.launch.py
```

### 7.2 커스텀 파라미터 파일 사용
```bash
ros2 launch rm520n_multi_driver rm520n_multi_driver.launch.py \
  params_file:=/path/to/your_params.yaml
```

### 7.3 Launch 파라미터

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| namespace | rm520n | ROS 네임스페이스 |
| node_name | rm520n_multi_driver | 노드명 |
| use_component | false | 컴포넌트 컨테이너 사용 여부 |
| use_intra_process | true | 프로세스 내 통신 활성화 (컴포넌트 모드) |
| container_name | rm520n_multi_driver_container | 컨테이너명 |
| container_executable | component_container_mt | 컨테이너 실행 파일 (MT=멀티스레드) |
| params_file | config/rm520n_params.yaml | 파라미터 파일 경로 |
| log_level | info | 로그 레벨 (debug/info/warn/error/fatal) |

### 7.4 컴포넌트 모드 실행
```bash
ros2 launch rm520n_multi_driver rm520n_multi_driver.launch.py \
  use_component:=true
```

## 8. 동작 시퀀스

### 8.1 초기화 단계
1. 파라미터 로드
2. AT 시리얼 포트 연결 및 초기화
3. GPS 활성화 명령 전송 (`AT+QGPS=1`)
4. GPS 시리얼 포트 연결
5. 퍼블리셔 생성
6. AT 폴링 워커 스레드 시작
7. 타이머 시작 (GPS, 네트워크 상태)

### 8.2 런타임 동작

**AT 폴링 워커** (백그라운드 스레드):
```
while (!stop_worker_) {
    1. AT 명령으로 네트워크 상태 조회
    2. 파싱 결과를 latest_status_에 저장 (mutex 보호)
    3. 500ms 대기
}
```

**네트워크 상태 발행** (timerCallback):
```
1. 최신 네트워크 상태를 mutex로 안전하게 복사
2. NetworkStatus 메시지 구성
3. network_status 토픽 발행
```

**GPS 데이터 발행** (gpsCallback):
```
1. GPS 시리얼 포트에서 NMEA 문장 읽기
2. NMEA 파서로 위경도 추출
3. Fix 여부 확인
4. NavSatFix 메시지 구성
5. gps_fix 토픽 발행
```

## 9. 에러 처리

### 9.1 시리얼 포트 에러
- 포트 열기 실패 시 `std::runtime_error` 예외 발생
- 에러 메시지에 errno 포함
- 재시도 없이 노드 종료

### 9.2 GPS 파싱 에러
- Checksum 불일치: 데이터 무시
- Fix Quality 0: STATUS_NO_FIX로 발행
- 파싱 예외: WARN 로그 출력, gps_status false 설정

### 9.3 AT 명령 에러
- 타임아웃: 빈 벡터 반환
- ERROR 응답: at_ok() 함수에서 false 반환
- GPS 활성화 실패: ERROR 로그 출력, 계속 실행

## 10. 성능 고려사항

### 10.1 스레드 안전성
- `std::mutex`를 사용한 네트워크 상태 동기화
- 각 시리얼 포트별 독립적인 mutex

### 10.2 블로킹 방지
- AT 명령에 타임아웃 설정 (기본 300ms)
- 시리얼 읽기에 타임아웃 설정 (기본 1000ms)
- 백그라운드 스레드로 AT 폴링 분리

### 10.3 리소스 관리
- RAII 패턴으로 시리얼 포트 자동 종료
- 소멸자에서 워커 스레드 안전 종료

## 11. 확장 가능성

### 11.1 추가 가능한 기능
- 다양한 NMEA 문장 타입 지원 (GSA, GSV 등)
- AT 명령 캐시 최적화
- 동적 파라미터 재설정
- 진단(Diagnostics) 메시지 발행

### 11.2 커스터마이징
- `NMEAParser::parse()` 확장으로 추가 문장 타입 지원
- `ATInterface`에 새로운 AT 명령 추가
- 파라미터 파일로 토픽명, 주기 변경

## 12. 문제 해결

### 12.1 GPS Fix 없음
- 안테나 연결 확인
- 실외 환경에서 테스트
- `AT+QGPS?` 명령으로 GPS 활성화 상태 확인

### 12.2 네트워크 연결 없음
- SIM 카드 삽입 및 인식 확인
- `AT+CEREG?` 명령으로 등록 상태 확인
- 통신사 신호 범위 확인

### 12.3 시리얼 포트 권한 에러
```bash
sudo chmod 666 /dev/ttyUSB*
# 또는
sudo usermod -aG dialout $USER
```

## 13. 참고 자료

- Quectel RM520N-GL AT 명령 가이드
- NMEA 0183 프로토콜 사양서
- ROS2 rclcpp API 문서
- sensor_msgs/NavSatFix 메시지 정의
