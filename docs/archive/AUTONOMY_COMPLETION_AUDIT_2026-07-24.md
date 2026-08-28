> Archived historical document dated 2026-07-24. It contains legacy mode names/status and is not the current `main` contract.

# 자율주행 지침서 완료 감사

최종 갱신: 2026-07-24

상태 정의:

- `PASS`: 현재 코드, Pi 실행 결과 또는 실제 시험 결과로 확인됨
- `READY`: 구현과 비주행 검증은 끝났지만 실제 환경 시험이 남음
- `PENDING`: 사용자 작업이나 외부 환경이 필요함
- `EXCLUDED`: 사용자가 범위에서 제외함

## 0단계 공통 안전 기반

| 요구사항 | 상태 | 현재 증거 |
|---|---|---|
| 물리 긴급정지 | EXCLUDED | 사용자 결정에 따라 별도 물리 스위치를 사용하지 않고 컨트롤러 B/○와 대시보드 소프트웨어 E-STOP을 사용 |
| 소프트웨어 E-STOP 래치 | PASS | Pi와 Arduino가 동시에 래치되며 B/○ 시험에서 구동·조향 PWM 0 확인 |
| Arduino 명령 타임아웃 | PASS | Arduino와 Pi가 각각 300ms 타임아웃을 적용하고 실제 데드맨 해제 252ms 정지 확인 |
| Arduino 응답 정지 감시 | PASS | 마지막 직렬 응답이 0.5초를 넘으면 `ARDUINO_TIMEOUT`; 실제 응답 나이 약 0.03초 |
| 바퀴를 띄운 시험 구조 | PASS | 바퀴를 띄운 상태에서 20%, 30%, 35% 공회전 시험 완료 |
| 최대 스로틀 제한 | PASS | 개발 단계 최대 명령 0.35 |
| 상태 머신 | PASS | `DISARMED`, `MANUAL_ASSIST`, `RECORD`, `AUTO_ROUTE`, `AUTO_HYBRID`, `EMERGENCY_STOP`, `FAULT` 전환 검증 |
| 센서 시간·유효성 표준화 | PASS | GNSS, IMU, 라이다, 조향, Arduino, 카메라에 timestamp, `is_valid`, `data_age`, `error_code` 제공 |
| 모든 명령의 Safety 통과 | PASS | 수동·자동 명령 모두 `SafetySupervisor.evaluate()` 이후 MotorController로 전달 |
| 정지 원인 화면·로그 표시 | PASS | `stop_reason`, `fault_code`, watchdog 원인을 API, 대시보드, CSV에 기록 |

## 라이다 표시와 안전

| 요구사항 | 상태 | 현재 증거 |
|---|---|---|
| 전체 빨간색 오표시 수정 | PASS | 거리별 빨강·노랑·청록 팔레트 적용; 실제 스캔에서 세 색상 분포 확인 |
| 차량 폭 기반 통로 검사 | PASS | 차량 좌표 +x 전방, 검사 반폭 0.45m |
| 라이다-앞 범퍼 오프셋 | PASS | 0.254m 적용 |
| 감속·저속·정지 구간 | PASS | 1.5m 감속, 0.8m 저속, 0.6m 정지 로직 검증 |
| 라이다 끊김 정지 | PASS | 0.3초 초과 `LIDAR_TIMEOUT` 검증 |
| 장시간 안정성 | PASS | GPIO18 Hi-Z 상태 30분, 100% 연결, 약 10Hz, 연결 끊김 0회 |
| 실제 이동 중 정지거리 | PENDING | 폐쇄된 평지에서 장애물 접근 시험 필요 |

## MANUAL_ASSIST

| 완료 조건 | 상태 | 현재 증거 또는 남은 작업 |
|---|---|---|
| 게임패드 입력 정규화와 데드존 | PASS | 축 -1~1 정규화, 스로틀·조향 데드존 적용 |
| 데드맨을 누를 때만 주행 | PASS | A/× 또는 Shift, 해제 시 252ms 정지 |
| 게임패드 단절 정지 | PASS | 브라우저 이벤트 정지 + Pi/Arduino 300ms 이중 watchdog |
| 브라우저 단절 정지 | PASS | 명령 heartbeat 중단 시 Pi/Arduino watchdog |
| 목표 조향각 폐루프 | PASS | AS5600 실제각, P 제어, 최소 PWM, 1° 허용오차, 45°/s 목표 변화율 |
| 좌우 기계 한계 보호 | PASS | Arduino와 Pi 양쪽 기준값·안전 여유 적용 |
| 조향 추종 실패 정지 | PASS | 7° 오차가 1초 지속되면 `STEERING_TRACKING_ERROR` |
| 중앙 입력에서 차량 정지 | PASS | 0 스로틀 실제 RECORD 시험에서 구동 PWM 0 |
| 좌·중앙·우 실제 목표각 오차 | PENDING | 바퀴를 띄운 상태에서 세 목표점 반복 측정 필요 |
| 실제 주행 중 장애물 점진 감속 | PENDING | 폐쇄된 평지 시험 필요 |
| 정지 구간에서 스로틀 차단 | READY | 소프트웨어 검증 완료, 실제 이동 차량 시험 필요 |

## RECORD

| 완료 조건 | 상태 | 현재 증거 또는 남은 작업 |
|---|---|---|
| 센서별 비동기 CSV | PASS | 권장 주기별 GNSS, IMU, 조향, Arduino, 제어, 라이다, 인식, 경로 저장 |
| 원시 라이다 저장 | PASS | zlib JSON 프레임 바이너리 저장·재생 검증 |
| 카메라 영상과 프레임 timestamp | PASS | 640px H264 MP4와 개별 프레임 monotonic timestamp |
| 요청·제한·최종 스로틀 분리 | PASS | `requested_throttle`, `limited_throttle`, `final_throttle` |
| 10분 무중단 기록 | PASS | 605.1초, 약 12MB, 5,761프레임, 114,465 센서 샘플, 오류 없음 |
| 카메라·조향·스로틀 동기화 | PASS | 프레임별 실제·목표 조향각과 요청·최종 스로틀 기록·재생 검증 |
| 세션 이름·삭제·재생 | PASS | 대시보드와 API 구현 |
| RTK FIX 구간 분리 | PASS | `rtk_status`, RTK FIX 전용 `route.csv` |
| ENU 변환·필터·0.2m 재샘플링 | PASS | `RouteProcessor` 구현과 검증 |
| 지도 경로 표시 | READY | 대시보드 구현 완료, 실제 RTK FIX 이동 경로 확인 필요 |
| 동일 로그 계기판 복원 | PASS | 605.1초 세션 재생과 상태 복원 확인 |

## AUTO_ROUTE

| 완료 조건 | 상태 | 현재 증거 또는 남은 작업 |
|---|---|---|
| 시작 전 RTK·GNSS·IMU·라이다·Arduino·조향 검사 | PASS | preflight 전체 차단 조건 검증 |
| 시작점 1m·방향 30° 제한 | PASS | preflight 검증 |
| ENU 좌표와 Pure Pursuit | PASS | 직선·원호·S자 각 10회 시뮬레이션 |
| 0.8m 전방 주시거리 | PASS | 기본 설정 |
| 곡선 감속 | PASS | 조향 비율에 따라 기본 속도의 최대 60% 감속 |
| 장애물 독립 안전정지 | PASS | Safety Supervisor 우선순위와 1.5초 재출발 지연 검증 |
| RTK 해제·GNSS/IMU timeout 정지 | PASS | fault 시뮬레이션 검증 |
| 경로 이탈·방향 반대·GNSS 점프 정지 | PASS | 1.0m, 60°, 1.5m 기준 검증 |
| 조향 추종 실패 정지 | PASS | 7°/1초 기준 |
| 수동 즉시 전환 | PASS | A/× 수동 개입 시 AUTO 종료·이벤트 기록 |
| 종료점 정지 | PASS | 시뮬레이션과 완료 이벤트 구현 |
| 5~10m 직선 실제 추종 | PENDING | RTK FIX 가능한 폐쇄 공간 필요 |
| 원호·S자 실제 추종 | PENDING | 직선 통과 후 실행 |
| 동일 경로 10회 반복 | PENDING | 실외 반복 시험 필요 |
| 실제 장애물·RTK 해제 정지 | PENDING | 폐쇄 공간 안전 시험 필요 |

## AUTO_HYBRID

| 완료 조건 | 상태 | 현재 증거 또는 남은 작업 |
|---|---|---|
| 비학습 차선 ROI 인식 | PASS | 합성 직선 차선 신뢰도 약 0.94 |
| 횡오차·방향오차 출력 | PASS | meter와 degree 출력 |
| 신뢰도 기반 RTK 보정 | PASS | `RTK 조향 + confidence × correction` |
| 카메라 보정각 제한 | PASS | 최대 5° |
| 프레임 간 급변 거부 | PASS | 0.35m 또는 12° 점프 시 `LANE_TEMPORAL_JUMP` |
| 저신뢰 결과 거부 | PASS | 신뢰도 0.55 미만 5프레임 시 `LANE_CONFIDENCE_LOW` |
| 카메라 끊김 RTK 폴백 | PASS | 0.3초 timeout 시 AUTO_ROUTE 전환 |
| 사람-라이다 융합 정지 | PASS | 카메라 객체 방향과 라이다 실거리 융합 검증 |
| 카메라 내부 보정 파이프라인 | READY | 수집·계산·왜곡 보정·재로드 구현 및 합성 체커보드 12장 계산 성공 |
| 실제 카메라 보정 파일 | PENDING | 9×6 체커보드 실물 촬영 필요 |
| 저장 영상 오프라인 검증 | PASS | 기존 영상 100프레임 처리; 실내 오검출 평균 신뢰도 0.422 확인 |
| 실제 직선·곡선 차선 안정성 | PENDING | 차선이 있는 폐쇄 공간 영상 필요 |
| AUTO_ROUTE 대비 오차 감소 | PENDING | 동일 실외 경로 A/B 시험 필요 |
| 저속 통합 실차 주행 | PENDING | AUTO_ROUTE 실차 완료 후 실행 |

## 현재 결론

소프트웨어 구현, Pi 배포, 실내 비주행·공회전·장기 라이다·기록 검증은 완료됐다. 최종 완료를 증명하려면 다음 외부 증거가 필요하다.

1. 실제 9×6 체커보드 카메라 보정
2. 좌·중앙·우 조향 목표각 실제 오차 측정
3. 폐쇄된 평지의 스로틀-GNSS 속도와 정지거리 측정
4. RTK FIX 상태 5~10m 직선, 원호, S자 경로 추종
5. 동일 경로 10회 반복, 실제 장애물, RTK 해제 시험
6. AUTO_ROUTE와 AUTO_HYBRID 동일 경로 비교
