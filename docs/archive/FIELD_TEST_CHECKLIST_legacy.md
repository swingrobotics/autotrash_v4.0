> Archived historical checklist. It is centered on legacy `AUTO_ROUTE`/`AUTO_HYBRID` modes. Use `docs/validation/FIELD_TEST.md` for current validation.

# 폐쇄 공간 실차 시험 체크리스트

어린이와 탑승자는 절대 태우지 않는다. 사람·차량이 없는 평탄한 폐쇄 공간에서 작업자 두 명 이상이 차량 양쪽을 확인한다. 소프트웨어 E-STOP만 사용하는 현재 구성의 잔여 위험을 이해한 상태에서 진행한다.

## 시험 전 공통 확인

- [ ] 배터리 고정, 배선과 안테나 고정
- [ ] 조향·구동부 주변에 손, 옷, 케이블 없음
- [ ] 컨트롤러 B/○와 대시보드 긴급정지 작동 확인
- [ ] 대시보드 모드 `DISARMED`, 구동 PWM 0, 조향 PWM 0
- [ ] Arduino 응답 나이 0.5초 미만
- [ ] 라이다 연결, 약 10Hz
- [ ] GNSS `RTK FIXED`, 데이터 나이 0.3초 미만
- [ ] IMU 데이터 나이 0.1초 미만
- [ ] 조향 센서 유효, 자석 감지
- [ ] 기록 디스크 여유 공간 512MB 이상

## A. 조향 실제 오차

바퀴를 지면에서 띄우고 수행한다.

1. 중앙 0°, 좌측 중간, 좌측 최대, 우측 중간, 우측 최대를 각각 5회 명령한다.
2. 목표각, 실제각, 도달 시간, 최대 오버슈트를 기록한다.
3. 방향을 바꿔 백래시와 좌우 응답 차이를 확인한다.

통과 기준:

- [ ] 모든 명령에서 기계 한계 초과 없음
- [ ] 정상 도달 후 절대 오차 3° 이하
- [ ] 7° 초과 오차가 1초 이상 지속되지 않음
- [ ] 중앙 복귀 후 바퀴가 시각적으로 중앙

## B. 스로틀-GNSS 속도 보정

평지 직선에서 20%, 30%, 35%, 40%를 각각 최소 5초 유지하고 양방향으로 반복한다. 시작 전 작업자가 기록 도구를 실행한다.

```bash
cd /home/gnss/camera-stream
.venv/bin/python3 scripts/record_throttle_calibration.py \
  --duration 180 \
  --output throttle-field.csv
```

측정 항목:

- [ ] 출발 가능한 최소 스로틀
- [ ] 각 스로틀의 안정 구간 GNSS 평균속도
- [ ] 명령 0 이후 실제 정지거리
- [ ] 배터리 전압 또는 잔량

보정표를 대시보드의 스로틀 보정에 입력하고 `/api/throttle/calibration`에서 저장값을 확인한다.

## C. 5~10m 직선 AUTO_ROUTE

1. `MANUAL_ASSIST`에서 0.2~0.3m/s로 직선 경로를 RECORD한다.
2. RTK FIX가 아닌 구간이 없는지 확인한다.
3. 세션에서 경로를 처리하고 `processed_route.json`을 생성한다.
4. 차량을 시작점 1m 이내, 방향 오차 30° 이내로 놓는다.
5. AUTO_ROUTE preflight가 모두 통과하는지 확인한다.
6. 첫 시험은 0.2m/s로 한 번만 실행한다.

시험 후:

```bash
.venv/bin/python3 scripts/evaluate_field_run.py \
  recordings/run_YYYY-MM-DD_HH-MM-SS \
  --required-mode AUTO_ROUTE \
  --maximum-cross-track-error 0.30 \
  --maximum-steering-error 7.0 \
  --minimum-rtk-fixed-ratio 0.99 \
  --minimum-sensor-valid-ratio 0.99 \
  --require-completion
```

통과 기준:

- [ ] 종료점에서 구동 PWM 0
- [ ] 최대 횡오차 0.30m 이하
- [ ] 좌우 진동이 지속되지 않음
- [ ] `AUTO_ROUTE_COMPLETED` 이벤트 존재
- [ ] 비정상 stop reason 없음

## D. 원호와 S자

직선 시험을 통과한 뒤 진행한다.

- [ ] 완만한 원호 1회
- [ ] S자 1회
- [ ] 곡선 평균 목표속도가 직선보다 낮음
- [ ] 최대 횡오차 0.30m 이하
- [ ] 종료점 정지

각 경로가 한 번 통과하면 동일 경로를 10회 반복한다.

- [ ] 10회 모두 경로 이탈 없음
- [ ] 횡오차 최대·평균·95백분위 기록
- [ ] 반복 중 RTK FIX 유지율 99% 이상

## E. 장애물과 센서 장애

사람을 장애물로 사용하지 않는다. 가벼운 상자 또는 폼 장애물을 사용한다.

1. 1.5m 밖에서 진입해 감속 시작을 확인한다.
2. 0.8m 안에서 저속 제한을 확인한다.
3. 앞 범퍼 기준 0.6m 안에서 완전 정지를 확인한다.
4. 장애물 제거 후 1.5초 동안 정지한 뒤 저속 재출발하는지 확인한다.

통과 기준:

- [ ] `OBSTACLE_STOP` 기록
- [ ] 정지 중 스로틀을 올려도 구동 PWM 0
- [ ] 조향과 무관하게 장애물 안전이 우선

센서 장애 시험:

- [ ] AUTO_ROUTE 중 NTRIP 또는 RTK FIX 해제 시 즉시 FAULT 정지
- [ ] 라이다 데이터 중단 시 `LIDAR_TIMEOUT`
- [ ] Arduino 응답 중단 시 `ARDUINO_TIMEOUT`
- [ ] 게임패드 A/× 입력 시 즉시 MANUAL_ASSIST 전환

## F. 카메라 보정과 AUTO_HYBRID

실내에서 먼저 9×6 내부 코너 체커보드 20장을 촬영한다.

```bash
.venv/bin/python3 scripts/capture_camera_calibration.py \
  --count 20 \
  --interval 1.5 \
  --output-dir camera-calibration-images

.venv/bin/python3 scripts/calibrate_camera.py \
  camera-calibration-images \
  --columns 9 \
  --rows 6 \
  --square-size-m 0.025
```

- [ ] `/api/camera/calibration`이 `calibrated: true`
- [ ] RMS 오차 기록
- [ ] 직선·곡선 저장 영상 오프라인 분석 통과
- [ ] 카메라를 가리면 0.3초 안에 AUTO_ROUTE 폴백
- [ ] 저신뢰 차선 5프레임 후 `LANE_CONFIDENCE_LOW`
- [ ] 프레임 급변 시 `LANE_TEMPORAL_JUMP`

AUTO_ROUTE와 같은 경로에서 AUTO_HYBRID를 실행한다.

- [ ] 카메라 보정으로 조향이 갑자기 튀지 않음
- [ ] AUTO_ROUTE 대비 횡오차가 같거나 감소
- [ ] 장애물 정지는 카메라 보정보다 우선
- [ ] 카메라를 끄거나 가려도 RTK 단독으로 계속 안전 주행

## 시험 종료

- [ ] `DISARMED`
- [ ] 구동 PWM 0
- [ ] 조향 PWM 0
- [ ] RECORD 정상 종료
- [ ] 세션 이름에 시험 종류와 결과 라벨 지정
- [ ] `WORK_PROGRESS.md`에 결과와 세션 경로 기록
