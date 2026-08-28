> Archived historical guide. It contains legacy `AUTO_ROUTE`/`AUTO_HYBRID` instructions and should not be used as the current mode contract. See `docs/architecture.md` and `docs/validation/FIELD_TEST.md`.

# 자율주행 개발 및 시험

## 소프트웨어 구조

- `autonomous_car/state.py`: 주행 상태와 센서 상태 표준 모델
- `autonomous_car/safety/`: 장애물 검사와 최종 안전 제한
- `autonomous_car/recording/`: 센서·카메라 기록과 로그 재생
- `autonomous_car/localization/`: ENU 좌표와 IMU/GNSS 방향 융합
- `autonomous_car/control/`: 조향, 스로틀, Pure Pursuit, 차선 보정
- `autonomous_car/routes/`: RTK 경로 필터링과 재샘플링
- `autonomous_car/modes/`: AUTO_ROUTE 계획기
- `autonomous_car/perception/`: 카메라 객체와 라이다 거리 융합
- `autonomous_car/simulation/`: 비주행 경로 추종 검증

## 소프트웨어 긴급정지

현재 차량은 별도 물리 긴급정지 스위치 없이 컨트롤러와 대시보드의 소프트웨어 긴급정지를 사용합니다.

- 컨트롤러 A/× 버튼: 누르는 동안만 주행을 허용하는 데드맨
- 컨트롤러 B/○ 버튼: 서버와 Arduino에 동시에 래치되는 긴급정지
- 대시보드 `긴급정지`: 컨트롤러와 같은 소프트웨어 래치 작동
- 대시보드 `정지 상태 해제`: 작업자가 안전을 확인한 후 래치 해제
- 통신 중단: 마지막 명령 후 300ms가 지나면 Arduino가 구동과 조향을 정지
- Safety Supervisor 명령 만료: 생성 후 300ms가 지난 제어 요청은 `COMMAND_TIMEOUT`으로 거부
- 자동 조향 추종 실패: 목표각과 실제각 오차가 7°를 1초 넘게 유지되면 `STEERING_TRACKING_ERROR`로 정지

`arduino/motor_serial/motor_serial.ino`은 `ESTOP`과 `RESET_ESTOP` 직렬 명령을 처리하며, 더 이상 Arduino 디지털 핀 2의 물리 스위치를 요구하지 않습니다. 소프트웨어 긴급정지는 라즈베리파이, 통신선 또는 Arduino 전원 자체가 고장 난 경우 독립적인 전원 차단 기능을 제공하지 않습니다.

Pi에서 Arduino Uno용 컴파일만 검증하려면 다음 명령을 사용합니다. 이 명령은 보드에 업로드하지 않습니다.

```bash
sh /home/gnss/camera-stream/arduino/compile_uno.sh
```

현재 빌드 결과는 플래시 14,246바이트(약 44%), SRAM 840바이트(약 41%)이며 Arduino Uno에 업로드와 검증을 완료했습니다.

## 기록 세션

각 세션은 `/home/gnss/camera-stream/recordings/run_*/`에 저장됩니다.

- `metadata.json`
- `vehicle_state.csv`, `gnss.csv`, `imu.csv`, `steering.csv`, `control.csv`
- `arduino.csv`(직렬 응답 나이, 구동·조향 PWM, watchdog, E-STOP)
- `lidar_summary.csv`, `lidar_raw.bin`(zlib 압축 원시 스캔)
- `camera.mp4`, `camera_timestamps.csv`(개별 JPEG는 기본 저장하지 않음)
- `perception.csv`, `events.csv`, `route.csv`
- 처리 후 `processed_route.json`

GNSS, IMU, 조향, 라이다, Arduino, 카메라 상태에는 `is_valid`, `data_age`, `error_code`가 기록됩니다. Arduino 포트가 열려 있어도 직렬 응답이 0.5초 이상 없으면 `ARDUINO_TIMEOUT`으로 정지합니다.

## 실차 시험 순서

소프트웨어 회귀 검증은 프로젝트 루트에서 다음 명령으로 실행합니다.

```bash
python3 -m autonomous_car.simulation.validate_autonomy
```

기록 세션 무결성과 스트림 개수는 다음 명령으로 확인합니다.

```bash
python3 -m autonomous_car.recording.inspect_log /home/gnss/camera-stream/recordings/run_YYYY-MM-DD_HH-MM-SS
```

실외 자동주행 기록의 횡오차, RTK 유지율, 조향 오차, 곡선 감속, 정지 원인과 완주 여부는 다음 명령으로 판정합니다.

```bash
python3 scripts/evaluate_field_run.py \
  /home/gnss/camera-stream/recordings/run_YYYY-MM-DD_HH-MM-SS \
  --required-mode AUTO_ROUTE \
  --maximum-cross-track-error 0.30 \
  --maximum-steering-error 7.0 \
  --minimum-rtk-fixed-ratio 0.99 \
  --minimum-sensor-valid-ratio 0.99 \
  --require-completion
```

## 카메라 내부 보정과 오프라인 검증

9×6 내부 코너 체커보드를 여러 위치·거리·기울기로 보여주면서 이미지를 수집합니다.

```bash
cd /home/gnss/camera-stream
.venv/bin/python3 scripts/capture_camera_calibration.py \
  --count 20 \
  --interval 1.5 \
  --output-dir camera-calibration-images
```

체커보드 실제 한 칸 크기가 25mm라면 다음과 같이 내부 파라미터와 왜곡 계수를 생성합니다. 최소 8장의 유효 이미지가 필요하며 생성된 값은 실행 중인 서버에 바로 재로드됩니다.

```bash
.venv/bin/python3 scripts/calibrate_camera.py \
  camera-calibration-images \
  --columns 9 \
  --rows 6 \
  --square-size-m 0.025
```

보정 상태는 `GET /api/camera/calibration`에서 확인합니다. 보정이 있으면 차선 인식과 사람-라이다 융합 모두 동일한 왜곡 보정 영상과 계산된 수평 FOV를 사용합니다.

저장 영상의 차선 검출률, 평균 신뢰도와 프레임 간 조향 보정 점프는 다음 명령으로 오프라인 판정합니다.

```bash
.venv/bin/python3 scripts/evaluate_lane_video.py \
  recordings/run_YYYY-MM-DD_HH-MM-SS/camera.mp4 \
  --calibration camera-calibration.json \
  --minimum-detection-ratio 0.70 \
  --minimum-mean-confidence 0.55 \
  --maximum-correction-jump 3.0
```

실시간 AUTO_HYBRID도 기본 신뢰도 0.55 미만을 유효 차선으로 사용하지 않습니다. 저신뢰 결과가 5프레임 연속되면 `LANE_CONFIDENCE_LOW`를 기록하고 AUTO_ROUTE로 전환하며, 임계값은 `AUTO_LANE_MIN_CONFIDENCE`로 조정할 수 있습니다.

1. 어린이 탑승 없이 바퀴를 지면에서 띄웁니다.
2. 컨트롤러 B/○와 대시보드 긴급정지로 구동·조향 모터가 모두 정지하는지 확인합니다.
3. 게임패드 A 버튼 또는 키보드 Shift를 놓으면 300ms 이내 정지하는지 확인합니다.
4. 조향 목표각과 실제각 오차를 좌·중앙·우에서 측정합니다.
5. 폐쇄된 평지에서 20%, 30%, 40% 스로틀의 GNSS 평균속도와 정지거리를 기록합니다.
6. 5~10m 직선 경로를 0.2~0.3m/s로 추종합니다.
7. 완만한 원호와 S자 경로를 시험합니다.
8. 동일 경로를 10회 반복하며 횡오차와 정지 원인을 확인합니다.
9. 장애물과 RTK 해제 시험에서 즉시 정지하는지 확인합니다.
10. AUTO_ROUTE 검증 후에만 AUTO_HYBRID 카메라 보정을 활성화합니다.

## 현재 제한

- 사용자의 결정에 따라 별도 물리 긴급정지 스위치는 사용하지 않으며, 컨트롤러·대시보드 소프트웨어 긴급정지와 Pi/Arduino 이중 300ms 타임아웃을 사용합니다.
- 소프트웨어 긴급정지는 Pi, Arduino 또는 모터 드라이버 자체 고장 시 독립적인 전원 차단을 제공하지 않는다는 잔여 위험이 있습니다.
- 스로틀 보정표 기본값은 임시값이므로 실측 후 `/api/throttle/calibration`으로 교체해야 합니다.
- AUTO_ROUTE와 AUTO_HYBRID는 폐쇄구역 실차 검증 전까지 사람을 태우고 사용하면 안 됩니다.
