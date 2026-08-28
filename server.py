#!/usr/bin/env python3

import glob
import base64
import csv
try:
    import grp
except ImportError:
    grp = None
import json
import math
import os
import re
import shutil
import socket
import ssl
import subprocess
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import quote

from camera_stream.config import (
    ARDUINO_DEVICE,
    AUTO_LANE_MIN_CONFIDENCE,
    AUTO_OBSTACLE_RESTART_DELAY_SECONDS,
    AUTO_STEERING_ERROR_TIMEOUT_SECONDS,
    AUTO_STEERING_MAX_ERROR_DEGREES,
    CAMERA_CALIBRATION_PATH,
    CAMERA_DEVICE as DEVICE,
    CAMERA_FRAMERATE as FRAMERATE,
    CAMERA_SIZE as SIZE,
    DRIVE_DIRECTION_SIGN,
    GPS_DEVICE,
    GPSD_CONTROL_SOCKET,
    HOST,
    IMU_ATTITUDE_DEADBAND_DEGREES,
    IMU_CALIBRATION_PATH,
    IMU_HEADING_DEADBAND_DEGREES,
    IMU_MOUNTING_YAW_OFFSET_DEGREES,
    IMU_TURN_RATE_THRESHOLD_DPS,
    IMU_YAW_DIRECTION_SIGN,
    LIDAR_CAMERA_FOV_DEGREES,
    LIDAR_CAMERA_YAW_DEGREES,
    LIDAR_CRAWL_DISTANCE_M,
    LIDAR_DEVICE,
    LIDAR_MAX_OVERLAY_DISTANCE_MM,
    LIDAR_SAFETY_HALF_WIDTH_M,
    LIDAR_SLOW_DISTANCE_M,
    LIDAR_STOP_DISTANCE_M,
    LIDAR_TO_FRONT_BUMPER_M,
    MANUAL_MAX_THROTTLE,
    MOTOR_BAUD,
    MOTOR_MIN_PWM,
    MOTOR_START_BOOST_SECONDS,
    MOTOR_TIMEOUT_SECONDS,
    NTRIP_CONFIG_PATH,
    PORT,
    RECORD_CAMERA_FPS,
    RECORDINGS_PATH,
    STEER_CENTER_TIMEOUT_SECONDS,
    STEER_CENTER_RAW,
    STEER_CONTROL_KP,
    STEER_LEFT_REFERENCE_RAW,
    STEER_LIMIT_ALLOWANCE_RAW,
    STEER_MANUAL_PWM,
    STEER_MIN_PWM,
    STEER_RIGHT_REFERENCE_RAW,
    STEER_TARGET_RATE_DEGREES_PER_SECOND,
    STEER_TARGET_TOLERANCE_DEGREES,
    THROTTLE_CALIBRATION_PATH,
)
from camera_stream.steering import (
    calibration_valid as steering_calibration_valid,
    safety_limit as steering_safety_limit,
    signed_raw_delta as signed_steering_raw_delta,
)
from camera_stream.lidar import LidarMonitor
from camera_stream.motor import drive_pwm_magnitude
from camera_stream.camera import Camera
from autonomous_car import ControlRequest, DriveMode, SafetyContext, SensorStatus, VehicleStateMachine
from autonomous_car.safety import (
    ObstacleChecker,
    RestartDelayGuard,
    SafetySupervisor,
    SteeringTrackingGuard,
)
from autonomous_car.recording import LogReplay, RecordManager
from autonomous_car.modes import (
    AutoRoutePlanner,
    HybridFallbackGuard,
    LaneContinuityFilter,
)
from autonomous_car.routes import RouteProcessor
from autonomous_car.control import LaneController, ThrottleCalibration
from autonomous_car.localization import HeadingEstimator
from autonomous_car.perception import CameraCalibration, ObjectDetector

try:
    import termios
except ImportError:
    termios = None

try:
    import pty
    import tty
except ImportError:
    pty = None
    tty = None

try:
    import qwiic_icm20948
except ImportError:
    qwiic_icm20948 = None


LOGO_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "camera_stream",
    "assets",
    "swing-logo-white.png",
)


INDEX_HTML = """<!doctype html>
<html lang="ko">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>GNSS Autonomy Console</title>
  <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">
  <style>
    :root {
      color-scheme: dark;
      --bg: #101210;
      --panel: #181b18;
      --panel-2: #1d211d;
      --line: #303530;
      --muted: #939a93;
      --text: #f0f2ed;
      --cyan: #abc98a;
      --green: #9dcc82;
      --amber: #d5b878;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      min-height: 100vh;
      background:
        radial-gradient(circle at 15% -10%, #17353a 0, transparent 32%),
        var(--bg);
      color: var(--text);
      font-family: Inter, ui-sans-serif, system-ui, sans-serif;
      letter-spacing: .01em;
    }
    header {
      height: 56px;
      padding: 0 18px;
      display: flex;
      align-items: center;
      justify-content: space-between;
      border-bottom: 1px solid var(--line);
      background: #070a0ddd;
      backdrop-filter: blur(18px);
      position: sticky;
      top: 0;
      z-index: 10;
    }
    .brand { display: flex; align-items: center; gap: 12px; }
    .brand-mark {
      width: 30px;
      height: 30px;
      border: 1px solid #4ce2d0aa;
      border-radius: 50%;
      display: grid;
      place-items: center;
      color: var(--cyan);
      box-shadow: 0 0 24px #41e4d233;
    }
    .brand h1 { margin: 0; font-size: 12px; letter-spacing: .18em; }
    .brand p { margin: 2px 0 0; color: var(--muted); font-size: 9px; }
    .header-status { display: flex; align-items: center; gap: 9px; }
    .header-action-button {
      min-height: 34px;
      padding: 6px 10px;
      border: 1px solid #2a3d47;
      border-radius: 8px;
      background: #0a1116;
      color: #a8b8bf;
      font: 750 9px ui-monospace, monospace;
      letter-spacing: .08em;
      cursor: pointer;
    }
    .header-action-button:hover,
    .header-action-button.active {
      border-color: #41e4d288;
      color: var(--cyan);
      background: #10252a;
    }
    .network-summary { display: flex; align-items: center; gap: 8px; }
    .network-pill {
      min-width: 116px;
      padding: 5px 8px;
      border: 1px solid #24343d;
      border-radius: 8px;
      background: #0a1015;
    }
    .network-pill span {
      display: block;
      color: var(--muted);
      font: 700 8px ui-monospace, monospace;
      letter-spacing: .14em;
    }
    .network-pill strong {
      display: block;
      margin-top: 3px;
      color: var(--text);
      font: 700 11px ui-monospace, monospace;
      white-space: nowrap;
    }
    .network-pill strong.good { color: var(--green); }
    .network-pill strong.warn { color: var(--amber); }
    .power-button {
      padding: 8px 11px;
      border: 1px solid #ff5b6e88;
      border-radius: 8px;
      background: #351017;
      color: #ff8794;
      font: 800 9px ui-monospace, monospace;
      letter-spacing: .12em;
      cursor: pointer;
      transition: border-color .15s ease, background .15s ease, color .15s ease;
    }
    .power-button:hover {
      border-color: #ff8794;
      background: #52141f;
      color: #fff0f2;
    }
    .power-button:disabled { opacity: .6; cursor: wait; }
    .restart-button {
      padding: 8px 11px;
      border: 1px solid #7a6740;
      border-radius: 8px;
      background: #292317;
      color: #d8bd82;
      font: 800 9px ui-monospace, monospace;
      cursor: pointer;
    }
    .restart-button:hover { border-color: #a58b56; background: #352d1d; color: #f0d9a8; }
    .restart-button:disabled { opacity: .6; cursor: wait; }
    main {
      width: min(1700px, 100%);
      min-height: calc(100vh - 56px);
      margin: auto;
      padding: 10px;
      display: grid;
      grid-template-columns: minmax(0, 1.55fr) minmax(440px, 1fr);
      grid-template-rows: auto auto 360px 250px 48px 18px;
      grid-template-areas:
        "camera lidar"
        "gnss imu"
        "map map"
        "steering steering"
        "links links"
        "footer footer";
      gap: 8px;
      align-items: stretch;
    }
    .panel {
      min-width: 0;
      min-height: 0;
      background: linear-gradient(145deg, #10171d, #0b1015);
      border: 1px solid var(--line);
      border-radius: 11px;
      overflow: hidden;
      box-shadow: 0 16px 50px #0005;
    }
    .panel-head {
      height: 38px;
      padding: 0 12px;
      display: flex;
      align-items: center;
      justify-content: space-between;
      border-bottom: 1px solid var(--line);
    }
    .panel-title { font-size: 10px; font-weight: 750; letter-spacing: .12em; }
    .panel-head-actions { display: flex; align-items: center; gap: 7px; }
    .panel-action-button,
    .device-toggle {
      border: 1px solid #31515a;
      border-radius: 7px;
      background: #10262a;
      color: var(--cyan);
      font: 750 8px ui-monospace, monospace;
      letter-spacing: .08em;
      cursor: pointer;
    }
    .panel-action-button { padding: 5px 9px; }
    .panel-action-button:disabled { opacity: .4; cursor: not-allowed; }
    .header-drive-button {
      padding: 5px 10px;
      border: 1px solid #5a3f31;
      border-radius: 7px;
      background: #2a1710;
      color: var(--amber);
      font: 800 8px ui-monospace, monospace;
      letter-spacing: .08em;
      cursor: pointer;
    }
    .header-drive-button.armed {
      border-color: #31553a;
      background: #17331d;
      color: var(--green);
    }
    .tag {
      border: 1px solid #2b3c46;
      border-radius: 999px;
      padding: 4px 8px;
      color: var(--muted);
      font: 700 9px ui-monospace, monospace;
    }
    .tag.live { color: var(--green); border-color: #31553a; background: #17331d55; }
    .tag.turn-left { color: #f6c760; border-color: #6a5424; background: #392b1055; }
    .tag.turn-right { color: #67a7ff; border-color: #31527a; background: #102a4a55; }
    .camera-wrap {
      position: relative;
      height: auto;
      aspect-ratio: 16 / 9;
      background: #020304;
    }
    .camera-wrap img {
      display: block;
      width: 100%;
      height: 100%;
      object-fit: cover;
      object-position: center;
      background: #000;
    }
    .lane-canvas,
    .detection-canvas {
      position: absolute;
      inset: 0;
      width: 100%;
      height: 100%;
      pointer-events: none;
    }
    .camera-overlay {
      position: absolute;
      inset: 0;
      pointer-events: none;
      background:
        linear-gradient(90deg, #41e4d244 1px, transparent 1px) center / 25% 100%,
        linear-gradient(#41e4d244 1px, transparent 1px) center / 100% 25%;
      opacity: .11;
    }
    .camera-meta {
      position: absolute;
      left: 14px;
      bottom: 12px;
      padding: 7px 10px;
      border: 1px solid #ffffff1f;
      border-radius: 6px;
      background: #05080bbb;
      font: 10px ui-monospace, monospace;
      color: #d6e5e8;
    }
    .detection-meta {
      position: absolute;
      right: 14px;
      bottom: 12px;
      padding: 7px 10px;
      border: 1px solid #41e4d255;
      border-radius: 6px;
      background: #05080bcc;
      color: var(--cyan);
      font: 700 10px ui-monospace, monospace;
    }
    .lane-meta {
      position: absolute;
      right: 14px;
      top: 12px;
      padding: 7px 10px;
      border: 1px solid #f6c76066;
      border-radius: 6px;
      background: #05080bcc;
      color: var(--amber);
      font: 700 10px ui-monospace, monospace;
    }
    .camera-panel { grid-area: camera; }
    .telemetry-column {
      display: contents;
    }
    .lidar-panel {
      grid-area: lidar;
      display: grid;
      grid-template-rows: 38px auto auto;
      align-self: start;
    }
    .lidar-view {
      position: relative;
      width: 100%;
      aspect-ratio: 2 / 1;
      background:
        radial-gradient(circle at center, #12313955 0, transparent 62%),
        #05090d;
    }
    .lidar-view canvas {
      display: block;
      width: 100%;
      height: 100%;
    }
    .lidar-scale {
      position: absolute;
      right: 10px;
      bottom: 8px;
      color: var(--muted);
      font: 700 8px ui-monospace, monospace;
      pointer-events: none;
    }
    .lidar-summary {
      padding: 8px;
      display: grid;
      grid-template-columns: minmax(0, 2fr) minmax(135px, 1fr);
      gap: 8px;
      border-top: 1px solid var(--line);
      background: #080d11;
    }
    .lidar-summary-values {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 6px;
    }
    .lidar-summary-value {
      min-width: 0;
      padding: 7px 8px;
      border: 1px solid #203039;
      border-radius: 7px;
      background: #090e12;
    }
    .lidar-summary-value span {
      display: block;
      color: var(--muted);
      font: 700 8px ui-monospace, monospace;
      letter-spacing: .08em;
    }
    .lidar-summary-value strong {
      display: block;
      margin-top: 4px;
      overflow: hidden;
      color: var(--cyan);
      font: 750 14px ui-monospace, monospace;
      text-overflow: ellipsis;
      white-space: nowrap;
    }
    .lidar-summary-value small {
      display: block;
      margin-top: 3px;
      color: var(--muted);
      font: 700 9px ui-monospace, monospace;
    }
    .lidar-summary-value strong b { font: inherit; color: var(--cyan); }
    .lidar-summary-value.coordinate strong { color: var(--amber); font-size: 11px; }
    .lidar-drive {
      padding: 8px;
      display: grid;
      grid-template-columns: minmax(0, 1fr);
      grid-template-rows: auto minmax(64px, 1fr) auto;
      align-content: stretch;
      gap: 8px;
      border: 1px solid #25343d;
      border-radius: 8px;
      background: #090e12;
    }
    .lidar-drive-readout {
      grid-column: 1;
      grid-row: 1;
      display: flex;
      align-items: baseline;
      justify-content: space-between;
      color: var(--muted);
      font: 700 8px ui-monospace, monospace;
    }
    .lidar-drive-readout strong {
      color: var(--cyan);
      font: 800 18px ui-monospace, monospace;
    }
    .lidar-drive-track {
      grid-column: 1;
      grid-row: 2;
      align-self: stretch;
      justify-self: center;
      position: relative;
      width: 14px;
      min-height: 78px;
      overflow: hidden;
      border-radius: 999px;
      background: #1b262d;
    }
    .lidar-drive-track::after {
      content: "";
      position: absolute;
      left: 0;
      top: 50%;
      width: 100%;
      height: 1px;
      background: #d7e7ea77;
    }
    .lidar-drive-fill {
      position: absolute;
      right: 0;
      bottom: 50%;
      left: 0;
      width: 100%;
      height: 0;
      background: var(--cyan);
      box-shadow: 0 0 10px #41e4d2aa;
    }
    .lidar-drive-fill.reverse {
      top: 50%;
      bottom: auto;
      background: var(--amber);
      box-shadow: 0 0 10px #f6c76088;
    }
    .lidar-drive-button {
      grid-column: 1;
      grid-row: 3;
      width: 100%;
      min-height: 34px;
    }
    .steering-panel { grid-area: steering; }
    .links-panel { grid-area: links; }
    .gnss-panel { grid-area: gnss; }
    .imu-panel { grid-area: imu; }
    .gnss-panel .metric { min-height: 38px; padding: 6px 8px; }
    .gnss-panel .metric-value { margin-top: 3px; font-size: 14px; }
    .gnss-panel .metric-value.small { font-size: 11px; }
    .metric-grid {
      display: grid;
      grid-template-columns: repeat(2, 1fr);
      gap: 1px;
      background: var(--line);
    }
    .metric { padding: 9px 10px; background: var(--panel); min-height: 58px; }
    .metric-label { color: var(--muted); font-size: 8px; letter-spacing: .12em; }
    .metric-value {
      margin-top: 5px;
      font: 650 17px ui-monospace, monospace;
      color: var(--text);
    }
    .metric-value.small { font-size: 13px; color: var(--amber); }
    .metric-unit { color: var(--muted); font-size: 10px; }
    .imu-layout {
      height: calc(100% - 38px);
      padding: 9px 10px;
      display: grid;
      gap: 7px;
    }
    .compass {
      width: 116px;
      height: 116px;
      position: relative;
      border: 1px solid #34505a;
      border-radius: 50%;
      background:
        radial-gradient(circle, transparent 48%, #17242a 49%, #17242a 50%, transparent 51%),
        linear-gradient(90deg, transparent 49.5%, #33454e 50%, transparent 50.5%),
        linear-gradient(transparent 49.5%, #33454e 50%, transparent 50.5%);
      box-shadow: inset 0 0 24px #0008, 0 0 22px #41e4d211;
    }
    .compass-label {
      position: absolute;
      color: var(--muted);
      font: 700 8px ui-monospace, monospace;
    }
    .compass-label.n { top: 7px; left: 50%; transform: translateX(-50%); color: var(--cyan); }
    .compass-label.s { bottom: 7px; left: 50%; transform: translateX(-50%); }
    .compass-label.e { right: 8px; top: 50%; transform: translateY(-50%); }
    .compass-label.w { left: 8px; top: 50%; transform: translateY(-50%); }
    .needle {
      position: absolute;
      left: 50%;
      top: 14px;
      width: 2px;
      height: 44px;
      transform-origin: 50% 44px;
      transform: translateX(-50%) rotate(0deg);
      background: linear-gradient(var(--cyan), #eafcff);
      box-shadow: 0 0 10px #41e4d2;
      transition: transform .11s linear;
    }
    .needle::after {
      content: "";
      position: absolute;
      width: 8px;
      height: 8px;
      left: -3px;
      bottom: -4px;
      border-radius: 50%;
      background: #dffefd;
      box-shadow: 0 0 12px #41e4d2;
    }
    .heading-label { color: var(--muted); font-size: 9px; letter-spacing: .14em; }
    .heading-value {
      margin: 6px 0 10px;
      font: 650 18px ui-monospace, monospace;
      color: var(--cyan);
    }
    .angle-strip { display: grid; grid-template-columns: repeat(3, 1fr); gap: 8px; }
    .angle-box {
      padding: 7px 8px;
      border: 1px solid #203039;
      border-radius: 7px;
      background: #090e12;
    }
    .angle-name { color: var(--muted); font-size: 8px; letter-spacing: .12em; }
    .angle-value { margin-top: 4px; font: 650 14px ui-monospace, monospace; }
    .angle-value.primary { color: var(--cyan); }
    .angle-rate { margin-top: 3px; color: var(--muted); font: 700 9px ui-monospace, monospace; }
    .calibration { margin-top: 2px; }
    .imu-layout .compass { display: none; }
    .imu-layout > div:not(.compass) {
      display: grid;
      grid-template-columns: repeat(2, 1fr);
      gap: 6px;
    }
    .imu-layout .heading-label { display: none; }
    .imu-layout .heading-value {
      margin: 0;
      padding: 7px 8px;
      border: 1px solid #203039;
      border-radius: 7px;
      background: #090e12;
      font: 650 14px ui-monospace, monospace;
    }
    .imu-layout .heading-value::before {
      content: "GLOBAL YAW";
      display: block;
      margin-bottom: 5px;
      color: var(--muted);
      font: 700 8px ui-monospace, monospace;
      letter-spacing: .12em;
    }
    .imu-layout .angle-strip { display: contents; }
    .imu-layout .calibration {
      grid-column: 1 / -1;
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 4px 6px;
      margin: 0;
    }
    .imu-layout .calibration-track,
    .imu-layout .calibration-message { grid-column: 1 / -1; }
    .calibrate-button {
      width: 100%;
      padding: 6px 8px;
      border: 1px solid #31515a;
      border-radius: 7px;
      background: #10262a;
      color: var(--cyan);
      font: 700 8px ui-monospace, monospace;
      letter-spacing: .06em;
      cursor: pointer;
    }
    .calibrate-button:disabled { opacity: .45; cursor: not-allowed; }
    .drive-control {
      height: calc(100% - 38px);
      padding: 8px 10px;
      display: grid;
      grid-template-columns: 180px minmax(0, 1fr);
      grid-template-rows: 1fr auto;
      gap: 6px 10px;
    }
    .drive-row {
      display: grid;
      grid-template-columns: 1fr auto;
      align-items: center;
      gap: 10px;
    }
    .drive-button {
      width: 100%;
      padding: 8px 10px;
      border: 1px solid #5a3f31;
      border-radius: 8px;
      background: #2a1710;
      color: var(--amber);
      font: 800 10px ui-monospace, monospace;
      letter-spacing: .12em;
      cursor: pointer;
    }
    .drive-button.armed {
      border-color: #31553a;
      background: #17331d;
      color: var(--green);
    }
    .drive-readout {
      color: var(--text);
      font: 750 16px ui-monospace, monospace;
      text-align: right;
    }
    .drive-track {
      position: relative;
      height: 9px;
      overflow: hidden;
      border-radius: 999px;
      background: #1b262d;
    }
    .drive-track::after {
      content: "";
      position: absolute;
      left: 50%;
      top: 0;
      width: 1px;
      height: 100%;
      background: #d7e7ea55;
    }
    .drive-fill {
      position: absolute;
      top: 0;
      left: 50%;
      width: 0;
      height: 100%;
      background: var(--cyan);
      box-shadow: 0 0 10px #41e4d2aa;
      transform-origin: left center;
    }
    .drive-fill.reverse {
      transform-origin: right center;
      background: var(--amber);
      box-shadow: 0 0 10px #f6c76088;
    }
    .drive-help {
      color: var(--muted);
      font: 8px ui-monospace, monospace;
      line-height: 1.35;
    }
    .steering-control {
      height: calc(100% - 38px);
      padding: 9px 10px 8px;
      display: grid;
      grid-template-columns: minmax(0, 1fr) 150px;
      grid-template-rows: minmax(0, 1fr);
      grid-template-areas: "metrics throttle";
      gap: 9px;
    }
    .steering-metrics {
      grid-area: metrics;
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 6px;
    }
    .steering-metric {
      padding: 7px 8px;
      border: 1px solid #203039;
      border-radius: 7px;
      background: #090e12;
    }
    .steering-value {
      margin-top: 4px;
      color: var(--cyan);
      font: 750 13px ui-monospace, monospace;
    }
    .steering-buttons {
      display: grid;
      grid-template-columns: 1fr .75fr 1fr;
      gap: 5px;
    }
    .steering-button {
      min-height: 36px;
      border: 1px solid #31515a;
      border-radius: 7px;
      background: #10262a;
      color: var(--cyan);
      font: 800 9px ui-monospace, monospace;
      cursor: pointer;
      touch-action: none;
      user-select: none;
    }
    .steering-button.stop {
      border-color: #5a3f31;
      background: #2a1710;
      color: var(--amber);
    }
    .steering-button.zero {
      width: 100%;
      min-height: 32px;
      color: var(--green);
    }
    .steering-button.active {
      border-color: var(--cyan);
      background: #174046;
      box-shadow: 0 0 12px #41e4d255;
    }
    .steering-button:disabled { opacity: .4; cursor: not-allowed; }
    .steering-config {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      align-content: start;
      gap: 10px;
    }
    .steering-config-field {
      display: grid;
      gap: 5px;
      color: var(--muted);
      font: 8px ui-monospace, monospace;
      letter-spacing: .08em;
    }
    .steering-config-field input {
      width: 100%;
      min-width: 0;
      padding: 9px 10px;
      border: 1px solid #203039;
      border-radius: 7px;
      outline: none;
      background: #090e12;
      color: var(--text);
      font: 700 12px ui-monospace, monospace;
    }
    .steering-config-field input:focus { border-color: var(--cyan); }
    .steering-config-save { grid-column: 1 / -1; }
    .throttle-gauge {
      grid-area: throttle;
      min-width: 0;
      padding: 8px;
      display: flex;
      align-items: center;
      justify-content: center;
      gap: 12px;
      border: 1px solid #25343d;
      border-radius: 8px;
      background: #090e12;
    }
    .throttle-bar {
      position: relative;
      width: 20px;
      height: 122px;
      overflow: hidden;
      border: 1px solid #2b3c46;
      border-radius: 999px;
      background: #0c1419;
    }
    .throttle-bar::after {
      content: "";
      position: absolute;
      left: 0;
      top: 50%;
      width: 100%;
      height: 1px;
      background: #d7e7ea88;
    }
    .throttle-bar-fill {
      position: absolute;
      left: 0;
      bottom: 50%;
      width: 100%;
      height: 0;
      background: var(--cyan);
      box-shadow: 0 0 10px #41e4d2aa;
      transition: height .08s linear;
    }
    .throttle-bar-fill.reverse {
      background: var(--amber);
      box-shadow: 0 0 10px #f6c76088;
    }
    .throttle-readout {
      display: grid;
      place-content: center;
      text-align: center;
    }
    .throttle-readout span {
      color: var(--muted);
      font: 700 8px ui-monospace, monospace;
      letter-spacing: .12em;
    }
    .throttle-readout strong {
      margin-top: 4px;
      color: var(--cyan);
      font: 800 22px ui-monospace, monospace;
    }
    .calibration-track {
      height: 3px;
      margin-top: 0;
      border-radius: 3px;
      overflow: hidden;
      background: #1b262d;
    }
    .calibration-progress {
      width: 0;
      height: 100%;
      background: var(--cyan);
      box-shadow: 0 0 8px var(--cyan);
      transition: width .12s linear;
    }
    .calibration-message {
      min-height: 12px;
      margin-top: 0;
      color: var(--muted);
      font: 8px ui-monospace, monospace;
    }
    .links-panel {
      position: relative;
      overflow: visible;
      z-index: 20;
    }
    .links-panel .panel-head {
      height: 100%;
      justify-content: flex-start;
      border-bottom: 0;
    }
    .device-toggle {
      padding: 6px 9px;
      display: flex;
      align-items: center;
      gap: 8px;
    }
    .device-toggle-arrow { transition: transform .18s ease; }
    .links-panel.expanded .device-toggle-arrow { transform: rotate(90deg); }
    .links-panel .device-list {
      position: absolute;
      right: 0;
      bottom: calc(100% + 8px);
      width: min(820px, calc(100vw - 20px));
      padding: 10px;
      display: none;
      grid-template-columns: repeat(5, minmax(0, 1fr));
      gap: 7px;
      border: 1px solid var(--line);
      border-radius: 11px;
      background: #0b1116f5;
      box-shadow: 0 20px 50px #000b;
      backdrop-filter: blur(16px);
    }
    .links-panel.expanded .device-list { display: grid; }
    .device {
      display: grid;
      grid-template-columns: 9px 1fr;
      grid-template-rows: auto auto;
      align-items: center;
      gap: 3px 6px;
      min-width: 0;
      padding: 6px 7px;
      border: 1px solid #1a252c;
      border-radius: 7px;
      background: #090e12;
      font-size: 10px;
    }
    .device:last-child { border: 1px solid #1a252c; }
    .dot { width: 7px; height: 7px; border-radius: 50%; background: #3b454b; }
    .dot.on { background: var(--green); box-shadow: 0 0 10px #7df78b99; }
    .device-state {
      grid-column: 2;
      overflow: hidden;
      color: var(--muted);
      font: 8px ui-monospace, monospace;
      text-overflow: ellipsis;
      white-space: nowrap;
    }
    .network-grid {
      display: grid;
      grid-template-columns: repeat(2, 1fr);
      gap: 1px;
      background: var(--line);
    }
    .network-cell {
      min-height: 78px;
      padding: 14px;
      background: var(--panel);
    }
    .network-label {
      color: var(--muted);
      font-size: 8px;
      letter-spacing: .14em;
    }
    .network-value {
      margin-top: 9px;
      font: 650 16px ui-monospace, monospace;
      color: var(--text);
      word-break: break-all;
    }
    .network-value.good { color: var(--green); }
    .network-value.warn { color: var(--amber); }
    .map-panel { grid-area: map; }
    .map-panel { min-height: 360px; }
    .map {
      height: calc(100% - 38px);
      position: relative;
      overflow: hidden;
      background: #071014;
    }
    #leaflet-map {
      width: 100%;
      height: 100%;
      background: #071014;
    }
    .leaflet-container {
      font-family: Inter, ui-sans-serif, system-ui, sans-serif;
      background: #071014;
    }
    .map-empty {
      position: absolute;
      left: 14px;
      bottom: 14px;
      z-index: 450;
      display: grid;
      gap: 8px;
      width: min(460px, calc(100% - 28px));
      padding: 10px 12px;
      border: 1px solid #ffffff1c;
      border-radius: 9px;
      background: #071014dd;
      color: var(--muted);
      font-size: 11px;
    }
    .map-empty strong { color: #b7c5ca; letter-spacing: .12em; }
    .footer-line {
      grid-area: footer;
      display: flex;
      justify-content: space-between;
      color: #53616a;
      font: 9px ui-monospace, monospace;
      letter-spacing: .08em;
      align-items: center;
    }
    .modal-backdrop[hidden] { display: none; }
    .modal-backdrop {
      position: fixed;
      inset: 0;
      z-index: 100;
      display: grid;
      place-items: center;
      padding: 18px;
      background: #020507cc;
      backdrop-filter: blur(8px);
    }
    .modal-card {
      width: min(520px, 100%);
      padding: 16px;
      border: 1px solid #2b404a;
      border-radius: 13px;
      background: linear-gradient(145deg, #111b21, #0a1015);
      box-shadow: 0 30px 90px #000d;
    }
    .modal-head {
      margin-bottom: 14px;
      display: flex;
      align-items: flex-start;
      justify-content: space-between;
      gap: 16px;
    }
    .modal-head p {
      margin: 7px 0 0;
      color: var(--muted);
      font-size: 10px;
      line-height: 1.5;
    }
    .modal-close {
      width: 30px;
      height: 30px;
      border: 1px solid #31434c;
      border-radius: 7px;
      background: #0b1318;
      color: var(--text);
      font-size: 20px;
      cursor: pointer;
    }
    .settings-grid {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 10px;
    }
    .settings-field {
      display: grid;
      gap: 5px;
      color: var(--muted);
      font: 8px ui-monospace, monospace;
      letter-spacing: .08em;
    }
    .settings-field[hidden] { display: none; }
    .settings-field.wide { grid-column: 1 / -1; }
    .settings-field input,
    .settings-field textarea {
      width: 100%;
      min-width: 0;
      padding: 9px 10px;
      border: 1px solid #203039;
      border-radius: 7px;
      outline: none;
      background: #090e12;
      color: var(--text);
      font: 700 12px ui-monospace, monospace;
    }
    .settings-field textarea { min-height: 88px; resize: vertical; line-height: 1.5; }
    .settings-field input:focus,
    .settings-field textarea:focus { border-color: var(--cyan); }
    .settings-checks {
      grid-column: 1 / -1;
      display: flex;
      flex-wrap: wrap;
      gap: 16px;
      color: #a8b8bf;
      font: 700 9px ui-monospace, monospace;
    }
    .settings-checks label { display: flex; align-items: center; gap: 7px; }
    .settings-actions {
      grid-column: 1 / -1;
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 8px;
    }
    .settings-button {
      min-height: 34px;
      border: 1px solid #2a4049;
      border-radius: 8px;
      background: #0b151a;
      color: var(--cyan);
      font: 750 10px ui-monospace, monospace;
      cursor: pointer;
    }
    .settings-button.secondary { color: var(--amber); }
    .settings-button.wide { grid-column: 1 / -1; }
    .settings-button:disabled { opacity: .45; cursor: wait; }
    .settings-status {
      grid-column: 1 / -1;
      min-height: 32px;
      padding: 9px 10px;
      border: 1px solid #1d2d35;
      border-radius: 7px;
      background: #081015;
      color: var(--muted);
      font: 700 9px ui-monospace, monospace;
      line-height: 1.45;
      word-break: break-word;
    }
    .settings-status.good { color: var(--green); }
    .settings-status.warn { color: var(--amber); }
    .wifi-list {
      grid-column: 1 / -1;
      max-height: 300px;
      overflow: auto;
      display: grid;
      border: 1px solid #253740;
      border-radius: 9px;
      background: #070d11;
    }
    .wifi-menu-card { width: min(440px, 100%); }
    .wifi-toolbar {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 12px;
    }
    .wifi-radio-state {
      display: flex;
      align-items: center;
      gap: 9px;
      color: var(--text);
      font: 750 11px ui-monospace, monospace;
    }
    .wifi-radio-dot {
      width: 8px;
      height: 8px;
      border-radius: 50%;
      background: var(--green);
      box-shadow: 0 0 10px #70f59d99;
    }
    .wifi-radio-state small {
      color: var(--muted);
      font-size: 8px;
      font-weight: 700;
    }
    .wifi-scan-button {
      padding: 7px 10px;
      border: 1px solid #2a4049;
      border-radius: 7px;
      background: #0b151a;
      color: var(--cyan);
      font: 750 9px ui-monospace, monospace;
      cursor: pointer;
    }
    .wifi-network {
      width: 100%;
      min-height: 46px;
      padding: 8px 11px;
      display: flex;
      align-items: center;
      gap: 10px;
      border: 0;
      border-bottom: 1px solid #18272e;
      background: transparent;
      color: var(--text);
      text-align: left;
      font: 700 11px ui-monospace, monospace;
      cursor: pointer;
    }
    .wifi-network:last-child { border-bottom: 0; }
    .wifi-network:hover { background: #102028; }
    .wifi-network.selected { background: #123039; color: var(--cyan); }
    .wifi-network.active { color: var(--green); }
    .wifi-network-name {
      min-width: 0;
      flex: 1;
      overflow: hidden;
      text-overflow: ellipsis;
      white-space: nowrap;
    }
    .wifi-network small { color: var(--muted); font-size: 8px; }
    .wifi-signal-bars {
      width: 23px;
      height: 18px;
      display: flex;
      align-items: flex-end;
      gap: 2px;
    }
    .wifi-signal-bars i {
      width: 4px;
      border-radius: 1px;
      background: #33434a;
    }
    .wifi-signal-bars i:nth-child(1) { height: 4px; }
    .wifi-signal-bars i:nth-child(2) { height: 8px; }
    .wifi-signal-bars i:nth-child(3) { height: 12px; }
    .wifi-signal-bars i:nth-child(4) { height: 16px; }
    .wifi-signal-bars.level-1 i:nth-child(-n+1),
    .wifi-signal-bars.level-2 i:nth-child(-n+2),
    .wifi-signal-bars.level-3 i:nth-child(-n+3),
    .wifi-signal-bars.level-4 i:nth-child(-n+4) { background: currentColor; }
    .wifi-lock {
      position: relative;
      width: 10px;
      height: 8px;
      border-radius: 2px;
      background: #718087;
    }
    .wifi-lock::before {
      content: "";
      position: absolute;
      left: 2px;
      top: -6px;
      width: 6px;
      height: 7px;
      border: 2px solid #718087;
      border-bottom: 0;
      border-radius: 5px 5px 0 0;
      box-sizing: border-box;
    }
    .wifi-connected-mark { width: 12px; color: var(--green); font-weight: 900; }
    .wifi-empty {
      padding: 28px 16px;
      color: var(--muted);
      text-align: center;
      font: 700 9px ui-monospace, monospace;
    }
    .wifi-connect-panel[hidden] { display: none; }
    .wifi-connect-panel {
      display: grid;
      gap: 10px;
      padding: 12px;
      border: 1px solid #2a4049;
      border-radius: 9px;
      background: #0a1217;
    }
    .wifi-selected-network {
      display: flex;
      align-items: center;
      gap: 9px;
      color: var(--cyan);
      font: 800 11px ui-monospace, monospace;
    }
    .wifi-selected-network small { margin-left: auto; color: var(--muted); font-size: 8px; }
    .wifi-footer-actions { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }
    @media (max-width: 1100px) {
      main {
        grid-template-columns: minmax(0, 1.4fr) minmax(300px, .7fr);
        grid-template-rows: auto;
        grid-template-areas:
          "camera lidar"
          "gnss imu"
          "map map"
          "steering steering"
          "links links"
          "footer footer";
        padding: 10px;
      }
      .map { height: 280px; }
      .steering-panel { min-height: 250px; }
      .links-panel { min-height: 52px; }
    }
    @media (max-width: 760px) {
      main {
        grid-template-columns: 1fr;
        grid-template-areas:
          "camera"
          "lidar"
          "gnss"
          "imu"
          "map"
          "steering"
          "links"
          "footer";
      }
      header { padding: 0 14px; }
      .network-summary { display: none; }
      .network-summary { gap: 6px; }
      .network-pill { min-width: 98px; padding: 5px 7px; }
      .steering-panel { min-height: 360px; }
      .steering-control {
        grid-template-columns: 1fr;
        grid-template-rows: auto;
        grid-template-areas:
          "metrics"
          "throttle";
      }
      .links-panel { min-height: 52px; }
      .links-panel .device-list { grid-template-columns: repeat(2, minmax(0, 1fr)); }
      .modal-card .steering-config { grid-template-columns: 1fr; }
      .modal-card .steering-config-save { grid-column: 1; }
      .settings-grid { grid-template-columns: 1fr; }
      .settings-field.wide, .settings-checks, .settings-actions,
      .settings-status, .wifi-list { grid-column: 1; }
      .lidar-summary { grid-template-columns: 1fr; }
    }
    main {
      grid-template-columns: minmax(0, 1.75fr) minmax(400px, .95fr);
      grid-template-rows: auto 460px auto 18px;
      grid-template-areas:
        "camera lidar"
        "map map"
        "details details"
        "footer footer";
    }
    .lidar-panel { align-self: stretch; }
    .status-strip { grid-area: status; }
    .status-grid {
      display: grid;
      grid-template-columns: repeat(4, minmax(0, 1fr));
      gap: 1px;
      background: var(--line);
    }
    .status-item {
      position: relative;
      min-width: 0;
      min-height: 72px;
      padding: 10px 11px;
      display: flex;
      flex-direction: column;
      justify-content: center;
      background: #0a1015;
    }
    .status-item span {
      color: var(--muted);
      font: 700 8px ui-monospace, monospace;
      letter-spacing: .1em;
    }
    .status-item strong {
      margin-top: 6px;
      overflow: hidden;
      color: var(--text);
      font: 750 15px ui-monospace, monospace;
      text-overflow: ellipsis;
      white-space: nowrap;
    }
    .status-item strong b { color: var(--cyan); font: inherit; }
    .status-item small {
      margin-top: 2px;
      color: var(--muted);
      font: 700 8px ui-monospace, monospace;
    }
    .status-item.accent strong { color: var(--green); }
    .status-item.coordinate strong { color: var(--amber); font-size: 12px; }
    .status-item.throttle strong { color: var(--cyan); }
    .core-throttle-track {
      position: relative;
      height: 4px;
      margin-top: 7px;
      overflow: hidden;
      border-radius: 999px;
      background: #1b262d;
    }
    .core-throttle-track .drive-fill {
      left: 50%;
      bottom: 0;
      height: 100%;
    }
    .details-panel { grid-area: details; z-index: 2; overflow: hidden; }
    .details-panel .panel-head {
      height: 46px;
      justify-content: space-between;
      border-bottom: 0;
    }
    .details-panel.expanded .panel-head { border-bottom: 1px solid var(--line); }
    .details-content {
      display: none;
      grid-template-columns: minmax(0, 1.35fr) minmax(420px, 1fr);
      gap: 10px;
      padding: 10px;
      background: #080d11;
    }
    .details-panel.expanded .details-content { display: grid; }
    .details-telemetry {
      display: grid;
      grid-template-columns: 116px minmax(0, 1fr) minmax(190px, .8fr);
      align-items: center;
      gap: 10px;
    }
    .details-values {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 6px;
    }
    .detail-value {
      min-width: 0;
      padding: 8px 9px;
      border: 1px solid #203039;
      border-radius: 7px;
      background: #090e12;
    }
    .detail-value.wide { grid-column: 1 / -1; }
    .detail-value span {
      display: block;
      color: var(--muted);
      font: 700 8px ui-monospace, monospace;
      letter-spacing: .08em;
    }
    .detail-value strong {
      display: block;
      margin-top: 5px;
      color: var(--cyan);
      font: 750 13px ui-monospace, monospace;
    }
    .details-calibration {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 6px;
    }
    .details-calibration .calibration-track,
    .details-calibration .calibration-message { grid-column: 1 / -1; }
    .details-panel .device-list {
      position: static;
      width: auto;
      padding: 0;
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      align-content: center;
      gap: 7px;
      border: 0;
      border-radius: 0;
      background: transparent;
      box-shadow: none;
      backdrop-filter: none;
    }
    .map-panel { min-height: 460px; }
    .map { height: calc(100% - 38px); }

    @media (max-width: 1100px) {
      main {
        grid-template-rows: auto 380px auto 18px;
        grid-template-areas:
          "camera lidar"
          "map map"
          "details details"
          "footer footer";
      }
      .status-grid { grid-template-columns: repeat(4, minmax(0, 1fr)); }
      .details-content { grid-template-columns: 1fr; }
      .map-panel { min-height: 380px; }
      .map { height: calc(100% - 38px); }
    }
    @media (max-width: 760px) {
      main {
        grid-template-areas:
          "camera"
          "lidar"
          "map"
          "details"
          "footer";
      }
      .status-grid { grid-template-columns: repeat(2, minmax(0, 1fr)); }
      .details-panel .panel-head { height: auto; min-height: 52px; gap: 8px; }
      .details-panel .panel-head-actions { flex-wrap: wrap; justify-content: flex-end; }
      .details-telemetry { grid-template-columns: 1fr; }
      .details-telemetry .compass { margin: auto; }
      .details-panel .device-list { grid-template-columns: repeat(2, minmax(0, 1fr)); }
      .map-panel { min-height: 360px; }
    }
    /* Calm industrial visual system */
    body {
      background: #101210;
      letter-spacing: 0;
    }
    header {
      height: 64px;
      padding: 0 22px;
      background: #141714f2;
      border-color: #2b302b;
      backdrop-filter: blur(12px);
    }
    .brand { gap: 10px; }
    .brand-mark {
      width: 34px;
      height: 34px;
      border: 0;
      border-radius: 9px;
      background: #abc98a;
      color: #182014;
      box-shadow: none;
      font-size: 17px;
      font-weight: 800;
    }
    .brand h1 {
      font-size: 15px;
      font-weight: 680;
      letter-spacing: -.01em;
    }
    .brand p { margin-top: 3px; font-size: 10px; }
    .header-status { gap: 7px; }
    .header-action-button,
    .restart-button,
    .power-button,
    .panel-action-button,
    .header-drive-button,
    .calibrate-button,
    .device-toggle {
      border-radius: 7px;
      font-family: Inter, ui-sans-serif, system-ui, sans-serif;
      font-size: 11px;
      font-weight: 620;
      letter-spacing: 0;
    }
    .header-action-button {
      min-height: 36px;
      padding: 7px 12px;
      border-color: #353b35;
      background: #1b1f1b;
      color: #c5cac4;
    }
    .header-action-button:hover,
    .header-action-button.active {
      border-color: #718761;
      background: #242b21;
      color: #d7e8c8;
    }
    .network-pill {
      min-width: 108px;
      padding: 6px 9px;
      border-color: #303530;
      border-radius: 7px;
      background: #171a17;
    }
    .network-pill span { font-family: Inter, ui-sans-serif, system-ui, sans-serif; letter-spacing: 0; }
    .network-pill strong { font-size: 11px; }
    .power-button {
      min-height: 36px;
      padding: 7px 12px;
      border-color: #654247;
      background: #26191b;
      color: #db9ea3;
      letter-spacing: 0;
    }
    .power-button:hover { background: #322023; border-color: #8b5b61; color: #f0c1c5; }
    .restart-button {
      min-height: 36px;
      padding: 7px 12px;
      border-color: #62563a;
      background: #252118;
      color: #d5c08d;
      letter-spacing: 0;
    }
    main {
      width: min(1780px, 100%);
      min-height: calc(100vh - 64px);
      padding: 14px;
      gap: 12px;
    }
    .panel {
      border-color: #2c312c;
      border-radius: 9px;
      background: #181b18;
      box-shadow: 0 8px 24px #00000026;
    }
    .panel-head {
      height: 42px;
      padding: 0 14px;
      border-color: #2c312c;
    }
    .panel-title {
      font-size: 12px;
      font-weight: 650;
      letter-spacing: 0;
    }
    .tag {
      padding: 4px 8px;
      border-color: #343a34;
      border-radius: 6px;
      font-family: Inter, ui-sans-serif, system-ui, sans-serif;
      font-size: 10px;
      font-weight: 600;
    }
    .tag.live { border-color: #496044; background: #253023; color: #b4d69c; }
    .camera-wrap,
    .lidar-view { background-color: #0d0f0d; }
    .camera-overlay { opacity: .045; }
    .camera-meta,
    .detection-meta,
    .lane-meta {
      border-color: #ffffff20;
      border-radius: 6px;
      background: #111411d9;
      box-shadow: none;
      font-size: 10px;
    }
    .lidar-view {
      background:
        radial-gradient(circle at center, #48583e2e 0, transparent 62%),
        #0d0f0d;
    }
    .lidar-panel {
      width: 100%;
      min-width: 0;
      max-width: 100%;
      grid-template-rows: 42px auto minmax(170px, auto);
    }
    .lidar-view {
      min-width: 0;
      max-width: 100%;
      overflow: hidden;
    }
    .lidar-view canvas { max-width: 100%; }
    .lidar-summary {
      width: 100%;
      min-width: 0;
      max-width: 100%;
      min-height: 170px;
      grid-template-columns: minmax(0, 1fr) 148px;
      overflow: hidden;
      border-color: #2c312c;
      background: #141714;
    }
    .lidar-summary-values {
      min-width: 0;
      height: 100%;
      grid-template-columns: repeat(6, minmax(0, 1fr));
      grid-template-rows: repeat(3, minmax(0, 1fr));
      align-content: stretch;
    }
    .lidar-summary-values > .lidar-summary-value { grid-column: span 2; }
    .lidar-summary-values > .lidar-summary-value:nth-last-child(-n + 2) { grid-column: span 3; }
    .lidar-drive { width: 148px; min-width: 0; min-height: 152px; overflow: hidden; }
    .lidar-summary-value,
    .lidar-drive,
    .detail-value,
    .device,
    .angle-box,
    .steering-metric {
      border-color: #303530;
      background: #1a1e1a;
      box-shadow: none;
    }
    .lidar-summary-value span,
    .lidar-drive-readout,
    .status-item span,
    .detail-value span,
    .metric-label,
    .angle-name {
      font-family: Inter, ui-sans-serif, system-ui, sans-serif;
      letter-spacing: 0;
    }
    .lidar-summary-value strong { color: #dfe7d8; font-size: 15px; }
    .lidar-drive-fill,
    .drive-fill,
    .throttle-bar-fill,
    .calibration-progress { box-shadow: none; }
    .status-strip { background: transparent; border: 0; box-shadow: none; }
    .status-strip .panel-head {
      height: 36px;
      padding: 0 2px 8px;
      border: 0;
    }
    .status-grid {
      gap: 8px;
      padding: 0;
      background: transparent;
    }
    .status-item {
      min-height: 76px;
      padding: 11px 12px;
      border: 1px solid #2c312c;
      border-radius: 8px;
      background: #181b18;
    }
    .status-item strong { margin-top: 7px; font-size: 16px; font-weight: 680; }
    .status-item.accent strong { color: #b8d99e; }
    .status-item.coordinate strong { color: #d8c59d; }
    .core-throttle-track,
    .lidar-drive-track,
    .calibration-track { background: #2b302b; }
    .details-panel .panel-head { min-height: 48px; }
    .details-content { background: #141714; }
    .device-toggle { border-color: #353b35; background: #1b1f1b; color: #d8ddd6; }
    .map-panel { background: #171a17; }
    .map,
    #leaflet-map,
    .leaflet-container { background: #141714; }
    .map-empty {
      border-color: #ffffff1f;
      border-radius: 7px;
      background: #171a17e8;
      box-shadow: none;
    }
    .footer-line { color: #747b74; }
    .dot.on { background: #9dcc82; box-shadow: none; }

    @media (max-width: 760px) {
      header { height: auto; min-height: 64px; padding: 10px 12px; gap: 10px; }
      .brand p { display: none; }
      .header-status { margin-left: auto; }
      .header-action-button, .restart-button, .power-button { padding: 7px 9px; }
      main { padding: 9px; gap: 9px; }
      .lidar-summary { grid-template-columns: 1fr; }
      .lidar-drive { width: 100%; min-height: 152px; }
    }
    /* Left settings drawer */
    main {
      grid-template-rows: auto 460px;
      grid-template-areas:
        "camera lidar"
        "map map";
    }
    .details-panel {
      position: fixed;
      left: 0;
      top: 64px;
      bottom: 0;
      width: min(460px, calc(100vw - 58px));
      z-index: 120;
      overflow: visible;
      border-left: 0;
      border-radius: 0 10px 10px 0;
      background: #181b18;
      box-shadow: 16px 0 40px #00000066;
      transform: translateX(-100%);
      transition: transform .2s ease;
    }
    .details-panel.expanded { transform: translateX(0); }
    .details-panel .panel-head {
      position: relative;
      height: 58px;
      min-height: 58px;
      padding: 0 12px 0 16px;
      border-bottom: 1px solid var(--line);
    }
    .drawer-title {
      font-size: 14px;
      font-weight: 680;
    }
    .details-panel .device-toggle {
      position: absolute;
      left: 100%;
      top: 62px;
      width: 44px;
      height: 44px;
      padding: 0;
      display: grid;
      place-items: center;
      border-left: 0;
      border-radius: 0 8px 8px 0;
      background: #242924;
      box-shadow: 8px 4px 18px #00000044;
    }
    .details-panel .device-toggle-arrow {
      font-size: 14px;
      transition: transform .2s ease;
    }
    .links-panel.expanded .device-toggle-arrow { transform: rotate(180deg); }
    .details-content {
      height: calc(100% - 58px);
      overflow-y: auto;
      grid-template-columns: 1fr;
      align-content: start;
      padding: 12px;
    }
    .details-telemetry {
      grid-template-columns: 100px minmax(0, 1fr);
      align-items: start;
    }
    .details-telemetry .compass { width: 100px; height: 100px; }
    .details-calibration { grid-column: 1 / -1; }
    .details-panel .device-list { grid-template-columns: repeat(2, minmax(0, 1fr)); }
    @media (max-width: 1100px) {
      main {
        grid-template-rows: auto 380px;
        grid-template-areas:
          "camera lidar"
          "map map";
      }
    }
    @media (max-width: 760px) {
      main {
        grid-template-areas:
          "camera"
          "lidar"
          "map";
      }
      .details-panel {
        top: 64px;
        width: min(420px, calc(100vw - 50px));
      }
      .details-panel .device-toggle { width: 42px; height: 42px; }
      .details-telemetry { grid-template-columns: 1fr; }
    }
    .drawer-section {
      min-width: 0;
      padding: 12px;
      border: 1px solid #303630;
      border-radius: 9px;
      background: #151815;
    }
    .drawer-section-head {
      min-height: 24px;
      margin-bottom: 10px;
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 8px;
      color: #d8ddd6;
      font-size: 12px;
      font-weight: 680;
    }
    .section-state {
      color: #9ba39a;
      font: 600 9px ui-monospace, monospace;
    }
    .section-state.good { color: #9dcc82; }
    .section-state.warn { color: #d6b56f; }
    .system-status-grid {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 7px;
    }
    .system-status-item {
      min-width: 0;
      min-height: 58px;
      padding: 9px 10px;
      display: flex;
      flex-direction: column;
      justify-content: space-between;
      gap: 7px;
      border: 1px solid #2b312b;
      border-radius: 7px;
      background: #1b1f1b;
    }
    .system-status-item.wide { grid-column: 1 / -1; }
    .system-status-item span {
      color: #8f978e;
      font-size: 9px;
    }
    .system-status-item strong {
      overflow: hidden;
      color: #e1e5df;
      font: 650 12px ui-monospace, monospace;
      text-overflow: ellipsis;
      white-space: nowrap;
    }
    .drawer-refresh-button {
      padding: 5px 8px;
      border: 1px solid #3a423a;
      border-radius: 6px;
      background: #202520;
      color: #c9cec7;
      font-size: 9px;
      cursor: pointer;
    }
    .drawer-refresh-button:disabled { opacity: .55; cursor: wait; }
    .drawer-section .details-telemetry { padding: 0; }
    .drawer-section .device-list { grid-template-columns: 1fr; }
    .drawer-section .device {
      min-height: 48px;
      padding: 8px 9px;
      background: #1b1f1b;
      border-color: #2b312b;
    }
    /* SWING black and firebrick theme */
    :root {
      --bg: #000000;
      --panel: #0b0b0b;
      --panel-2: #111111;
      --line: #3a1717;
      --muted: #8f8f8f;
      --text: #f2f2f2;
      --cyan: #e8e8e8;
      --green: #39c56b;
      --amber: #d6aa5f;
    }
    body { background: #000; color: var(--text); }
    header {
      background: #070707f2;
      border-color: #351717;
    }
    .brand { gap: 12px; }
    .brand-logo {
      width: 76px;
      height: 44px;
      object-fit: contain;
      object-position: center;
      flex: 0 0 auto;
    }
    .brand h1 { color: #fff; }
    .brand p { color: #8d8d8d; }
    .network-pill,
    .panel,
    .lidar-summary-value,
    .drawer-section,
    .system-status-item,
    .drawer-section .device {
      background: #0d0d0d;
      border-color: #361717;
    }
    .panel {
      box-shadow: 0 8px 28px #0008, inset 0 1px 0 #b2222226;
    }
    .panel-head {
      background: #101010;
      border-color: #351717;
    }
    .panel-title,
    .drawer-title,
    .drawer-section-head { color: #f5f3f3; }
    .tag,
    .header-action-button,
    .panel-action-button,
    .header-drive-button,
    .calibrate-button,
    .device-toggle,
    .drawer-refresh-button,
    .settings-button,
    .steering-button {
      border-color: #5a2222;
      background: #111111;
      color: #dedede;
    }
    .tag.live,
    .section-state.good,
    .dot.on {
      border-color: #247943;
      background: #08170d;
      color: #55cf7f;
      box-shadow: none;
    }
    .header-drive-button.armed {
      border-color: #b22222;
      background: #b22222;
      color: #fff;
    }
    .restart-button {
      border-color: #702a2a;
      background: #111111;
      color: #d8d8d8;
    }
    .restart-button:hover,
    .header-action-button:hover,
    .panel-action-button:hover,
    .drawer-refresh-button:hover { border-color: #b22222; background: #181010; color: #fff; }
    .power-button {
      border-color: #b22222;
      background: #b22222;
      color: #fff;
    }
    .power-button:hover { border-color: #d04a4a; background: #8f1b1b; color: #fff; }
    .camera-body,
    .lidar-view,
    .map,
    #leaflet-map,
    .leaflet-container { background: #050505; }
    .lidar-summary,
    .details-content,
    .map-panel,
    .details-panel { background: #090909; }
    .details-panel { box-shadow: 16px 0 42px #000c; }
    .details-panel .device-toggle { background: #b22222; border-color: #b22222; color: #fff; }
    .map-panel {
      position: relative;
      z-index: 1;
      isolation: isolate;
    }
    .details-panel { z-index: 1200; }
    .modal-backdrop { z-index: 2000; }
    .drawer-section-head {
      position: relative;
      padding-left: 10px;
    }
    .drawer-section-head::before {
      content: "";
      position: absolute;
      left: 0;
      top: 4px;
      bottom: 4px;
      width: 3px;
      border-radius: 2px;
      background: #b22222;
    }
    .core-throttle-track,
    .lidar-drive-track,
    .calibration-track { background: #231010; }
    .core-throttle-fill,
    .lidar-drive-fill,
    .calibration-progress { background: #b22222; box-shadow: none; }
    .compass {
      border-color: #5b2525;
      background:
        radial-gradient(circle, transparent 48%, #261010 49%, #261010 50%, transparent 51%),
        linear-gradient(90deg, transparent 49.5%, #5b2525 50%, transparent 50.5%),
        linear-gradient(transparent 49.5%, #5b2525 50%, transparent 50.5%);
    }
    .needle { background: linear-gradient(#b22222, #fff); box-shadow: none; }
    .needle::after { background: #fff; box-shadow: none; }
    .section-state.warn { color: #dc7777; }
    .status-item.accent strong,
    .status-item.coordinate strong,
    .lidar-summary-value strong { color: #f1f1f1; }
    .modal-backdrop { background: #000d; }
    .modal-card,
    .settings-field input,
    .settings-field textarea,
    .steering-config-field input,
    .wifi-network { background: #0b0b0b; border-color: #3d1a1a; }
    .wifi-network:hover { background: #161111; color: #f1f1f1; }
    .wifi-network.selected { border-color: #287c48; background: #08170d; color: #76d996; }
    .device-state,
    .system-status-item span,
    .lidar-summary-value span,
    .detail-value span { color: #8f8f8f; }
    .system-status-item strong,
    .detail-value strong { color: #ededed; }
    .network-summary .network-pill {
      border-color: #1f6439;
      background: #061109;
    }
    .network-summary .network-pill span { color: #69c98b; }
    #internet-state.good,
    #pc-ping.good { color: #39c56b; }
    #internet-state.warn,
    #pc-ping.warn { color: #d87070; }
    #ntrip-settings-open,
    #wifi-settings-open {
      border-color: #27804a;
      background: #07170d;
      color: #62d188;
    }
    #ntrip-settings-open:hover,
    #wifi-settings-open:hover {
      border-color: #39c56b;
      background: #0b2514;
      color: #a4efbd;
    }
    #ntrip-settings-open.active,
    #wifi-settings-open.active {
      border-color: #39c56b;
      background: #176b36;
      color: #fff;
    }
    .autonomy-status-grid {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 10px;
    }
    .autonomy-status-item {
      display: grid;
      gap: 5px;
      padding: 12px;
      border: 1px solid #3d1a1a;
      border-radius: 8px;
      background: #0b0b0b;
    }
    .autonomy-status-item span { color: #8f8f8f; font-size: 11px; }
    .autonomy-status-item strong { color: #f1f1f1; }
    .route-error-panel {
      padding: 10px;
      border: 1px solid #3d1a1a;
      border-radius: 8px;
      background: #080808;
    }
    #route-error-chart { width: 100%; height: 120px; display: block; }
    #route-error-summary { margin-top: 6px; color: #bdbdbd; font: 700 10px ui-monospace, monospace; }
    .record-replay-panel {
      padding: 12px;
      border: 1px solid #3d1a1a;
      border-radius: 8px;
      background: #080808;
    }
    .record-replay-head {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 12px;
      color: #f1f1f1;
      font: 700 11px ui-monospace, monospace;
    }
    .record-replay-controls {
      display: grid;
      grid-template-columns: minmax(0, 1fr) auto;
      gap: 8px;
      margin-top: 10px;
    }
    .record-replay-controls input[type="range"] { padding: 0; accent-color: #b22222; }
    .record-replay-grid {
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 7px;
      margin-top: 10px;
    }
    .record-replay-grid div {
      display: grid;
      gap: 4px;
      min-width: 0;
      padding: 8px;
      border: 1px solid #281414;
      border-radius: 6px;
      background: #0d0d0d;
    }
    .record-replay-grid span { color: #777; font-size: 9px; }
    .record-replay-grid strong {
      overflow: hidden;
      color: #e8e8e8;
      font: 700 10px ui-monospace, monospace;
      text-overflow: ellipsis;
      white-space: nowrap;
    }
    .autonomy-actions { display: grid; grid-template-columns: repeat(2, 1fr); gap: 8px; }
    .autonomy-actions .danger { border-color: #b22222; background: #4c1010; color: #fff; }
    #autonomy-modal .modal-card {
      width: min(1100px, calc(100vw - 24px));
      max-height: calc(100vh - 24px);
      display: grid;
      grid-template-rows: auto auto minmax(0, 1fr);
      overflow: hidden;
    }
    #autonomy-modal .modal-head { margin-bottom: 10px; }
    .autonomy-safety-actions {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 8px;
      margin-bottom: 10px;
    }
    .autonomy-safety-actions .settings-button {
      min-height: 42px;
      font-size: 12px;
      font-weight: 750;
    }
    .autonomy-safety-actions .danger {
      border-color: #b22222;
      background: #4c1010;
      color: #fff;
    }
    #autonomy-modal .settings-grid {
      min-height: 0;
      padding-right: 6px;
      overflow-y: auto;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      grid-template-areas:
        "summary summary"
        "chart session"
        "chart label"
        "replay throttle"
        "replay message"
        "actions actions";
      align-items: start;
      scrollbar-color: #7f2424 #141010;
      scrollbar-width: thin;
    }
    #autonomy-modal .autonomy-status-grid { grid-area: summary; }
    #autonomy-modal .route-error-panel { grid-area: chart; }
    #autonomy-modal .autonomy-session-field { grid-area: session; }
    #autonomy-modal .autonomy-label-field { grid-area: label; }
    #autonomy-modal .record-replay-panel { grid-area: replay; }
    #autonomy-modal .autonomy-throttle-field { grid-area: throttle; }
    #autonomy-modal .autonomy-message { grid-area: message; }
    #autonomy-modal .autonomy-actions {
      grid-area: actions;
      grid-template-columns: repeat(4, minmax(0, 1fr));
    }
    #autonomy-modal .autonomy-status-grid {
      grid-template-columns: repeat(3, minmax(0, 1fr));
    }
    @media (max-width: 700px) {
      #autonomy-modal .modal-card {
        width: calc(100vw - 12px);
        max-height: calc(100vh - 12px);
        padding: 12px;
      }
      #autonomy-modal .settings-grid {
        grid-template-columns: repeat(2, minmax(0, 1fr));
      }
      #autonomy-modal .autonomy-actions {
        grid-template-columns: repeat(2, minmax(0, 1fr));
      }
    }
    #autonomy-open { border-color: #793030; }
    @media (max-width: 760px) {
      .brand-logo { width: 62px; height: 36px; }
    }
  </style>
</head>
<body>
  <header>
    <div class="brand">
      <img class="brand-logo" src="/assets/swing-logo-white.png" alt="SWING">
      <div>
        <h1>차량 대시보드</h1>
      </div>
    </div>
    <div class="header-status">
      <div class="network-summary">
        <div class="network-pill"><span>인터넷</span><strong id="internet-state">확인 중</strong></div>
        <div class="network-pill"><span>PC PING</span><strong id="pc-ping">--</strong></div>
      </div>
      <button class="header-action-button" id="ntrip-settings-open" type="button">RTK 설정</button>
      <button class="header-action-button" id="wifi-settings-open" type="button">네트워크</button>
      <button class="header-action-button" id="autonomy-open" type="button">주행 모드</button>
      <button class="restart-button" id="restart-button" type="button">재시작</button>
      <button class="power-button" id="power-button" type="button">종료</button>
    </div>
  </header>
  <span id="wifi-state" hidden></span>
  <span id="gps-tag" hidden></span>
  <span id="imu-tag" hidden></span>
  <main>
    <section class="panel camera-panel">
      <div class="panel-head">
          <span class="panel-title">전방 카메라</span>
      </div>
      <div class="camera-wrap">
        <img id="camera-stream" src="/stream.mjpg" alt="Live camera stream">
        <canvas id="lane-canvas" class="lane-canvas"></canvas>
        <canvas id="detection-canvas" class="detection-canvas"></canvas>
        <div class="camera-overlay"></div>
        <div class="camera-meta">C930e · 1280×720 · 30 FPS · MJPEG</div>
        <div class="lane-meta" id="lane-meta">차선 인식 준비 중</div>
        <div class="detection-meta" id="detection-meta">AI 모델 준비 중</div>
      </div>
    </section>

    <aside class="telemetry-column">
      <section class="panel lidar-panel">
        <div class="panel-head">
          <span class="panel-title">전방 라이다</span>
          <span class="tag" id="lidar-tag">거리 확인 중</span>
        </div>
        <div class="lidar-view">
          <canvas id="lidar-map" aria-label="360도 라이다 거리 지도"></canvas>
        </div>
        <div class="lidar-summary">
          <div class="lidar-summary-values">
            <div class="lidar-summary-value">
              <span>출발 기준 회전각</span>
              <strong id="lidar-relative-yaw">--.-°</strong>
            </div>
            <div class="lidar-summary-value">
              <span>북쪽 기준 방향</span>
              <strong id="lidar-global-heading">---.-°</strong>
            </div>
            <div class="lidar-summary-value">
              <span>앞바퀴 조향각</span>
              <strong id="lidar-steering-angle">--.--°</strong>
              <small id="lidar-steering-input">입력 0%</small>
            </div>
            <div class="lidar-summary-value">
              <span>조향 센서 원시값</span>
              <strong id="lidar-steering-raw">----</strong>
            </div>
            <div class="lidar-summary-value"><span>GNSS 수신 상태</span><strong id="fix">수신 대기</strong></div>
            <div class="lidar-summary-value"><span>사용 중인 위성</span><strong><b id="satellites">--</b>개</strong></div>
            <div class="lidar-summary-value coordinate"><span>위도</span><strong id="latitude">--.-------</strong></div>
            <div class="lidar-summary-value coordinate"><span>경도</span><strong id="longitude">---.-------</strong></div>
          </div>
          <div class="lidar-drive">
            <div class="lidar-drive-readout">
              <span>구동 출력</span>
              <strong id="lidar-drive-throttle">0%</strong>
            </div>
            <div class="lidar-drive-track" aria-hidden="true">
              <div class="lidar-drive-fill" id="lidar-drive-fill"></div>
            </div>
            <button class="header-drive-button lidar-drive-button" id="lidar-drive-button" type="button">주행 활성화</button>
          </div>
        </div>
      </section>

    </aside>

    <section class="panel links-panel details-panel">
      <div class="panel-head">
        <button class="device-toggle" id="device-toggle" type="button" aria-expanded="false" aria-label="시스템 및 설정 열기">
          <span class="device-toggle-arrow">▶</span>
        </button>
        <span class="drawer-title">시스템 및 설정</span>
      </div>
      <div class="details-content">
        <section class="drawer-section">
          <div class="drawer-section-head">
            <span>라즈베리파이 시스템 상태</span>
            <span class="section-state" id="pi-system-state">확인 중</span>
          </div>
          <div class="system-status-grid">
            <div class="system-status-item"><span>호스트</span><strong id="pi-hostname">--</strong></div>
            <div class="system-status-item"><span>CPU 부하</span><strong id="pi-cpu">--</strong></div>
            <div class="system-status-item"><span>CPU 온도</span><strong id="pi-temperature">--</strong></div>
            <div class="system-status-item"><span>메모리</span><strong id="pi-memory">--</strong></div>
            <div class="system-status-item"><span>저장공간</span><strong id="pi-storage">--</strong></div>
            <div class="system-status-item"><span>가동 시간</span><strong id="pi-uptime">--</strong></div>
          </div>
        </section>

        <section class="drawer-section">
          <div class="drawer-section-head">
            <span>센서 및 보정</span>
            <button class="panel-action-button" id="steering-config-open" type="button" disabled>조향 설정</button>
          </div>
        <div class="details-telemetry">
          <div class="compass">
            <span class="compass-label n">N</span><span class="compass-label e">E</span>
            <span class="compass-label s">S</span><span class="compass-label w">W</span>
            <div class="needle" id="heading-needle"></div>
          </div>
          <div class="details-values">
            <div class="detail-value"><span>좌우 기울기</span><strong id="imu-roll">--.-°</strong></div>
            <div class="detail-value"><span>앞뒤 기울기</span><strong id="imu-pitch">--.-°</strong></div>
            <div class="detail-value"><span>조향 센서 원시값</span><strong id="steering-raw">----</strong></div>
            <div class="detail-value"><span>조향 중앙 기준값</span><strong id="steering-zero">미설정</strong></div>
            <div class="detail-value wide"><span>조향 허용 범위</span><strong id="steering-range">---- ~ ----</strong></div>
          </div>
          <div class="details-calibration">
            <button class="calibrate-button" id="calibrate-button" type="button">20초 IMU 보정</button>
            <button class="calibrate-button" id="heading-zero-button" type="button">Yaw 0° 재설정</button>
            <div class="calibration-track"><div class="calibration-progress" id="calibration-progress"></div></div>
            <div class="calibration-message" id="calibration-message">Rotate slowly around all 3 axes</div>
          </div>
        </div>
        </section>

        <section class="drawer-section">
          <div class="drawer-section-head">
            <span>장치 관리</span>
            <button class="drawer-refresh-button" id="device-refresh" type="button">상태 새로고침</button>
          </div>
        <div class="device-list">
          <div class="device"><span class="dot" id="camera-dot"></span><span>USB 카메라</span><span class="device-state" id="camera-state">확인 중</span></div>
          <div class="device"><span class="dot" id="gps-dot"></span><span>ZED-F9P GNSS</span><span class="device-state" id="gps-state">확인 중</span></div>
          <div class="device"><span class="dot" id="imu-dot"></span><span>9DoF IMU (I²C)</span><span class="device-state" id="imu-state">확인 중</span></div>
          <div class="device"><span class="dot" id="lidar-dot"></span><span>LD06 LiDAR</span><span class="device-state" id="lidar-state">확인 중</span></div>
          <div class="device"><span class="dot" id="arduino-dot"></span><span>Arduino 모터 I/O</span><span class="device-state" id="arduino-state">확인 중</span></div>
        </div>
        </section>
        <div id="drive-help" hidden></div>
      </div>
    </section>

    <section class="panel map-panel">
      <div class="panel-head">
        <span class="panel-title">위치 및 경로</span>
      </div>
      <div class="map">
        <div id="leaflet-map"></div>
        <div class="map-empty">
          <strong id="map-fix">GNSS 위치 대기 중</strong>
          <span id="map-coordinates">gpsd 연결 대기 중</span>
        </div>
      </div>
    </section>

  </main>
  <div class="modal-backdrop" id="steering-config-modal" role="dialog" aria-modal="true" aria-labelledby="steering-config-title" hidden>
    <section class="modal-card">
      <div class="modal-head">
        <div>
          <div class="panel-title" id="steering-config-title">조향 센서 설정</div>
          <p>설치 방향에 맞춰 오른쪽·중앙·왼쪽 원시값과 안전 여유를 입력합니다.</p>
        </div>
        <button class="modal-close" id="steering-config-close" type="button" aria-label="닫기">×</button>
      </div>
      <div class="steering-config">
        <label class="steering-config-field">
          오른쪽 기준 원시값
          <input id="steering-config-right" type="number" min="0" max="4095" step="1" inputmode="numeric">
        </label>
        <label class="steering-config-field">
          중앙 0° 원시값
          <input id="steering-config-center" type="number" min="0" max="4095" step="1" inputmode="numeric">
        </label>
        <label class="steering-config-field">
          왼쪽 기준 원시값
          <input id="steering-config-left" type="number" min="0" max="4095" step="1" inputmode="numeric">
        </label>
        <label class="steering-config-field">
          바깥쪽 안전 여유
          <input id="steering-config-allowance" type="number" min="0" max="300" step="1" inputmode="numeric">
        </label>
        <button class="steering-button zero steering-config-save" id="steering-config-save" type="button" disabled>조향 보정값 저장</button>
      </div>
    </section>
  </div>
  <div class="modal-backdrop" id="ntrip-settings-modal" role="dialog" aria-modal="true" aria-labelledby="ntrip-settings-title" hidden>
    <section class="modal-card">
      <div class="modal-head">
        <div>
          <div class="panel-title" id="ntrip-settings-title">RTK 보정 연결</div>
          <p>보정 서비스에서 받은 접속 정보를 저장하면 RTCM 데이터를 ZED-F9P로 전달합니다.</p>
        </div>
        <button class="modal-close" id="ntrip-settings-close" type="button" aria-label="닫기">×</button>
      </div>
      <div class="settings-grid">
        <label class="settings-field wide">서버 주소
          <input id="ntrip-host" type="text" autocomplete="off" placeholder="caster.example.com">
        </label>
        <label class="settings-field">포트
          <input id="ntrip-port" type="number" min="1" max="65535" value="2101">
        </label>
        <label class="settings-field">마운트포인트
          <input id="ntrip-mountpoint" type="text" autocomplete="off" placeholder="VRS-RTCM32">
        </label>
        <label class="settings-field">아이디
          <input id="ntrip-username" type="text" autocomplete="username">
        </label>
        <label class="settings-field">비밀번호
          <input id="ntrip-password" type="password" autocomplete="current-password" placeholder="저장된 값은 비워두면 유지">
        </label>
        <div class="settings-checks">
          <label><input id="ntrip-enabled" type="checkbox" checked> 저장 후 자동 연결</label>
        </div>
        <div class="settings-status" id="ntrip-settings-status">설정 정보를 불러오는 중입니다.</div>
        <div class="settings-actions">
          <button class="settings-button secondary" id="ntrip-stop" type="button">연결 중지</button>
          <button class="settings-button" id="ntrip-save" type="button">저장 및 연결</button>
        </div>
      </div>
    </section>
  </div>
  <div class="modal-backdrop" id="wifi-settings-modal" role="dialog" aria-modal="true" aria-labelledby="wifi-settings-title" hidden>
    <section class="modal-card wifi-menu-card">
      <div class="modal-head">
        <div>
          <div class="panel-title" id="wifi-settings-title">네트워크 연결</div>
          <p>라즈베리파이가 직접 연결할 무선 네트워크를 선택하세요.</p>
        </div>
        <button class="modal-close" id="wifi-settings-close" type="button" aria-label="닫기">×</button>
      </div>
      <div class="settings-grid">
        <div class="wifi-toolbar settings-field wide">
          <div class="wifi-radio-state">
            <span class="wifi-radio-dot"></span>
            <span>Wi-Fi</span>
            <small id="wifi-interface-state">켜짐</small>
          </div>
          <button class="wifi-scan-button" id="wifi-rescan" type="button">↻ 다시 검색</button>
        </div>
        <div class="settings-status" id="wifi-settings-status">주변 Wi-Fi를 검색할 수 있습니다.</div>
        <div class="wifi-list" id="wifi-network-list">
          <div class="wifi-empty">검색 중...</div>
        </div>
        <div class="wifi-connect-panel settings-field wide" id="wifi-connect-panel" hidden>
          <div class="wifi-selected-network">
            <span class="wifi-signal-bars level-4"><i></i><i></i><i></i><i></i></span>
            <span id="wifi-selected-name">네트워크 선택</span>
            <small id="wifi-selected-security"></small>
          </div>
          <input id="wifi-ssid" type="hidden">
          <label class="settings-field" id="wifi-password-field">비밀번호
            <input id="wifi-password" type="password" autocomplete="current-password" placeholder="Wi-Fi 비밀번호">
          </label>
          <div class="settings-actions">
            <button class="settings-button secondary" id="wifi-selection-cancel" type="button">취소</button>
            <button class="settings-button" id="wifi-connect" type="button">연결</button>
          </div>
        </div>
        <div class="wifi-footer-actions settings-field wide">
          <button class="settings-button secondary" id="wifi-disconnect" type="button">현재 Wi-Fi 연결 해제</button>
          <button class="settings-button" id="wifi-hidden-network" type="button">숨겨진 네트워크</button>
        </div>
      </div>
    </section>
  </div>
  <div class="modal-backdrop" id="autonomy-modal" role="dialog" aria-modal="true" aria-labelledby="autonomy-title" hidden>
    <section class="modal-card">
      <div class="modal-head">
        <div>
          <div class="panel-title" id="autonomy-title">주행 모드 및 기록</div>
          <p>모든 주행 명령은 안전 관리자를 통과하며, 자동주행 시작 전 조건을 검사합니다.</p>
        </div>
        <button class="modal-close" id="autonomy-close" type="button" aria-label="닫기">×</button>
      </div>
      <div class="autonomy-safety-actions">
        <button class="settings-button danger" id="emergency-stop" type="button">긴급정지</button>
        <button class="settings-button secondary" id="safety-reset" type="button">정지 상태 해제</button>
      </div>
      <div class="settings-grid">
        <div class="autonomy-status-grid settings-field wide">
          <div class="autonomy-status-item"><span>현재 모드</span><strong id="autonomy-mode">DISARMED</strong></div>
          <div class="autonomy-status-item"><span>정지 원인</span><strong id="autonomy-stop-reason">DISARMED</strong></div>
          <div class="autonomy-status-item"><span>기록 상태</span><strong id="autonomy-recording">대기</strong></div>
          <div class="autonomy-status-item"><span>자동 경로</span><strong id="autonomy-route-state">미로드</strong></div>
          <div class="autonomy-status-item"><span>긴급정지</span><strong id="autonomy-software-estop">B/○ 버튼 대기</strong></div>
          <div class="autonomy-status-item"><span>카메라 보정</span><strong id="autonomy-hybrid-fallback">대기</strong></div>
        </div>
        <div class="route-error-panel settings-field wide">
          <canvas id="route-error-chart" aria-label="자율주행 횡오차 그래프"></canvas>
          <div id="route-error-summary">횡오차 기록 없음</div>
        </div>
        <label class="settings-field autonomy-session-field">기록 세션 이름
          <input id="autonomy-session" type="text" list="autonomy-session-list" autocomplete="off" placeholder="run_2026-07-22_001">
          <datalist id="autonomy-session-list"></datalist>
        </label>
        <label class="settings-field autonomy-label-field">세션 설명
          <input id="autonomy-session-label" type="text" maxlength="80" autocomplete="off" placeholder="예: 운동장 직선 경로 1차">
        </label>
        <div class="record-replay-panel settings-field">
          <div class="record-replay-head">
            <span>기록 상태 재생</span>
            <strong id="record-replay-time">0.0 / 0.0초</strong>
          </div>
          <div class="record-replay-controls">
            <input id="record-replay-position" type="range" min="0" max="0" value="0" step="0.1" aria-label="기록 재생 위치">
            <button class="settings-button secondary" id="record-replay-load" type="button">상태 불러오기</button>
          </div>
          <div class="record-replay-grid">
            <div><span>모드</span><strong id="record-replay-mode">--</strong></div>
            <div><span>GNSS</span><strong id="record-replay-gnss">--</strong></div>
            <div><span>IMU 방향</span><strong id="record-replay-imu">--</strong></div>
            <div><span>조향</span><strong id="record-replay-steering">--</strong></div>
            <div><span>스로틀</span><strong id="record-replay-throttle">--</strong></div>
            <div><span>전방 거리</span><strong id="record-replay-lidar">--</strong></div>
          </div>
        </div>
        <label class="settings-field autonomy-throttle-field">속도–스로틀 보정표 · 한 줄에 속도(m/s), 스로틀(0~1)
          <textarea id="throttle-calibration-points" spellcheck="false" placeholder="0.0, 0.0&#10;0.2, 0.25&#10;0.4, 0.40"></textarea>
        </label>
        <div class="settings-status autonomy-message" id="autonomy-status">상태를 확인하는 중입니다.</div>
        <div class="autonomy-actions settings-field wide">
          <button class="settings-button" id="record-start" type="button">기록 시작</button>
          <button class="settings-button secondary" id="record-stop" type="button">기록 종료</button>
          <button class="settings-button secondary" id="record-list-refresh" type="button">세션 새로고침</button>
          <button class="settings-button" id="record-label-save" type="button">설명 저장</button>
          <button class="settings-button" id="throttle-calibration-save" type="button">스로틀 보정 저장</button>
          <button class="settings-button secondary" id="record-map-show" type="button">지도에 경로 표시</button>
          <button class="settings-button danger" id="record-delete" type="button">선택 세션 삭제</button>
          <button class="settings-button" id="route-process" type="button">경로 처리</button>
          <button class="settings-button secondary" id="route-load" type="button">경로 로드</button>
          <button class="settings-button" id="auto-route-start" type="button">AUTO_ROUTE 시작</button>
          <button class="settings-button secondary" id="auto-route-stop" type="button">자동주행 중지</button>
          <button class="settings-button" id="auto-hybrid-start" type="button">카메라 보정 켜기</button>
          <button class="settings-button secondary" id="auto-hybrid-stop" type="button">카메라 보정 끄기</button>
        </div>
      </div>
    </section>
  </div>
  <script>
    const setDevice = (name, online, detail) => {
      document.getElementById(`${name}-dot`).classList.toggle("on", online);
      document.getElementById(`${name}-state`).textContent = detail;
    };

    const formatCapacity = (usedBytes, totalBytes) => {
      if (!Number.isFinite(usedBytes) || !Number.isFinite(totalBytes) || totalBytes <= 0) return "--";
      const usedGb = usedBytes / 1024 ** 3;
      const totalGb = totalBytes / 1024 ** 3;
      return `${usedGb.toFixed(1)} / ${totalGb.toFixed(1)} GB`;
    };

    const formatUptime = (seconds) => {
      if (!Number.isFinite(seconds)) return "--";
      const days = Math.floor(seconds / 86400);
      const hours = Math.floor((seconds % 86400) / 3600);
      const minutes = Math.floor((seconds % 3600) / 60);
      return days > 0 ? `${days}일 ${hours}시간` : `${hours}시간 ${minutes}분`;
    };

    const renderRaspberrySystem = (system) => {
      const load = Number.isFinite(system.cpu_load_percent) ? system.cpu_load_percent : null;
      const temperature = Number.isFinite(system.temperature_c) ? system.temperature_c : null;
      document.getElementById("pi-hostname").textContent = system.hostname || "--";
      document.getElementById("pi-cpu").textContent = load === null ? "--" : `${load.toFixed(1)}%`;
      document.getElementById("pi-temperature").textContent = temperature === null ? "--" : `${temperature.toFixed(1)} °C`;
      document.getElementById("pi-memory").textContent = formatCapacity(system.memory_used_bytes, system.memory_total_bytes);
      document.getElementById("pi-storage").textContent = formatCapacity(system.disk_used_bytes, system.disk_total_bytes);
      document.getElementById("pi-uptime").textContent = formatUptime(system.uptime_seconds);
      const warning = (load !== null && load >= 90) || (temperature !== null && temperature >= 75);
      const state = document.getElementById("pi-system-state");
      state.textContent = warning ? "점검 필요" : "정상";
      state.className = `section-state ${warning ? "warn" : "good"}`;
    };

    let unwrappedHeading = null;
    let previousHeading = null;
    let mapInstance = null;
    let positionMarker = null;
    let routeLine = null;
    let recordedRouteLine = null;
    const routePoints = [];
    let driveArmed = false;
    let driveThrottle = 0;
    let lastSentThrottle = null;
    let lastSentDeadman = false;
    let lastDriveSentAt = 0;
    let activeGamepadId = null;
    let steeringControlAvailable = false;
    let steeringCenterSupported = false;
    let gamepadSteeringNeedsNeutral = false;
    let lastGamepadEmergencyPressed = false;
    let emergencyStopRequestInFlight = false;
    let lastSentGamepadSteering = 0;
    let lastGamepadSteeringSentAt = 0;
    let steeringHoldDirection = 0;
    let steeringRepeatTimer = null;
    let steeringRequestInFlight = false;
    let steeringPendingDirection = null;
    let steeringConfigEditing = false;
    let objectDetector = null;
    let detectionStopped = false;
    let detectionFrames = 0;
    let detectionFpsStartedAt = performance.now();
    let laneProcessingCanvas = null;
    let laneProcessingContext = null;
    let visionFrameCanvas = null;
    let visionFrameContext = null;
    let laneGray = null;
    let laneFrameDivider = 0;
    let laneDetectionStopped = false;
    let laneFrames = 0;
    let laneFpsStartedAt = performance.now();
    let lidarRequestInFlight = false;
    let previousLidarDisplayCandidates = [];
    let latestSteeringAngleDegrees = 0;
    const pressedKeys = new Set();

    const clamp = (value, min, max) => Math.min(max, Math.max(min, value));

    const lidarPointsMatch = (left, right, angleTolerance = 2.2) => {
      const distanceTolerance = Math.max(
        160,
        Math.min(left.distance_mm, right.distance_mm) * 0.15
      );
      return (
        Math.abs(left.bearing_degrees - right.bearing_degrees) <= angleTolerance
        && Math.abs(left.distance_mm - right.distance_mm) <= distanceTolerance
      );
    };

    const filterLidarDisplayPoints = (points, maxDistance) => {
      const vehicleHalfWidthMm = 9.5 * 25.4;
      const vehicleFrontLengthMm = 10 * 25.4;
      const candidates = points
        .filter((point) => {
          if (
            point.distance_mm <= 0
            || point.distance_mm > maxDistance
            || point.confidence < 35
            || Math.abs(point.bearing_degrees) > 90
          ) {
            return false;
          }
          const bearingRadians = point.bearing_degrees * Math.PI / 180;
          const forwardMm = point.distance_mm * Math.cos(bearingRadians);
          const lateralMm = point.distance_mm * Math.sin(bearingRadians);
          return !(
            forwardMm <= vehicleFrontLengthMm
            && Math.abs(lateralMm) <= vehicleHalfWidthMm
          );
        })
        .sort((left, right) => left.bearing_degrees - right.bearing_degrees);

      const surfacePoints = candidates.filter((point, index) => {
        const previous = candidates[index - 1];
        const next = candidates[index + 1];
        return (
          (previous && lidarPointsMatch(point, previous))
          || (next && lidarPointsMatch(point, next))
        );
      });
      const stablePoints = previousLidarDisplayCandidates.length
        ? surfacePoints.filter((point) =>
            previousLidarDisplayCandidates.some((previous) =>
              lidarPointsMatch(point, previous, 2.6)
            )
          )
        : surfacePoints;
      previousLidarDisplayCandidates = surfacePoints;
      return stablePoints;
    };
    const applyDeadzone = (value, deadzone = 0.2) =>
      Math.abs(value) < deadzone ? 0 : value;
    const externalScriptLoads = new Map();
    const loadExternalScript = (source) => {
      if (externalScriptLoads.has(source)) return externalScriptLoads.get(source);
      const promise = new Promise((resolve, reject) => {
        const script = document.createElement("script");
        const timeout = window.setTimeout(() => {
          script.remove();
          reject(new Error(`External script timed out: ${source}`));
        }, 15000);
        script.src = source;
        script.async = true;
        script.onload = () => {
          window.clearTimeout(timeout);
          resolve();
        };
        script.onerror = () => {
          window.clearTimeout(timeout);
          reject(new Error(`External script failed: ${source}`));
        };
        document.head.appendChild(script);
      });
      externalScriptLoads.set(source, promise);
      return promise;
    };

    const drawLidarMap = (lidar) => {
      const canvas = document.getElementById("lidar-map");
      const status = document.getElementById("lidar-tag");
      const bounds = canvas.parentElement.getBoundingClientRect();
      const width = Math.max(1, Math.floor(bounds.width || canvas.clientWidth || 320));
      const height = Math.max(1, Math.floor(bounds.height || canvas.clientHeight || 260));
      canvas.style.width = `${width}px`;
      canvas.style.height = `${height}px`;
      if (canvas.width !== width || canvas.height !== height) {
        canvas.width = width;
        canvas.height = height;
      }

      const context = canvas.getContext("2d");
      context.clearRect(0, 0, width, height);
      if (!lidar.connected || !Array.isArray(lidar.points)) {
        previousLidarDisplayCandidates = [];
        status.textContent = "OFFLINE";
        context.fillStyle = "#778793";
        context.font = "700 11px ui-monospace, monospace";
        context.textAlign = "center";
        context.fillText(lidar.error || "NO DATA", width / 2, height / 2);
        return;
      }

      const maxDistance = lidar.max_overlay_distance_mm || 4000;
      const centerX = width / 2;
      const centerY = height - 4;
      const radius = Math.max(20, Math.min(width / 2 - 6, height - 8));
      const visiblePoints = filterLidarDisplayPoints(lidar.points, maxDistance);
      const frontPoints = visiblePoints.filter((point) => Math.abs(point.bearing_degrees) <= 15);
      const nearest = frontPoints.length
        ? Math.min(...frontPoints.map((point) => point.distance_mm))
        : null;

      context.save();
      context.translate(centerX, centerY);
      const cameraFovDegrees = clamp(Number(lidar.camera_fov_degrees) || 82.1, 1, 180);
      const forwardHalfAngle = cameraFovDegrees / 2 * Math.PI / 180;
      context.beginPath();
      context.moveTo(0, 0);
      context.arc(
        0,
        0,
        radius,
        -Math.PI / 2 - forwardHalfAngle,
        -Math.PI / 2 + forwardHalfAngle
      );
      context.closePath();
      context.fillStyle = "#b2222218";
      context.fill();
      context.strokeStyle = "#b2222266";
      context.setLineDash([5, 5]);
      context.beginPath();
      context.moveTo(0, 0);
      context.lineTo(
        Math.cos(-Math.PI / 2 - forwardHalfAngle) * radius,
        Math.sin(-Math.PI / 2 - forwardHalfAngle) * radius
      );
      context.moveTo(0, 0);
      context.lineTo(
        Math.cos(-Math.PI / 2 + forwardHalfAngle) * radius,
        Math.sin(-Math.PI / 2 + forwardHalfAngle) * radius
      );
      context.stroke();
      context.setLineDash([]);
      context.strokeStyle = "#542020";
      context.fillStyle = "#f2f2f2";
      context.lineWidth = 1;
      context.font = "700 10px ui-monospace, monospace";
      context.textAlign = "center";
      context.textBaseline = "middle";
      for (let ring = 1; ring <= 2; ring += 1) {
        const ringRadius = radius * ring / 2;
        context.beginPath();
        context.arc(0, 0, ringRadius, Math.PI, Math.PI * 2);
        context.stroke();
        context.save();
        context.lineWidth = 3;
        context.strokeStyle = "#000000cc";
        context.fillStyle = "#f2f2f2";
        context.strokeText(`${ring * 2}m`, 0, -ringRadius + 9);
        context.fillText(`${ring * 2}m`, 0, -ringRadius + 8);
        context.restore();
      }
      context.strokeStyle = "#542020";
      context.beginPath();
      context.moveTo(-radius, 0);
      context.lineTo(radius, 0);
      context.moveTo(0, -radius);
      context.lineTo(0, 0);
      context.stroke();
      const vehicleHalfWidthMm = 9.5 * 25.4;
      const vehicleFrontLengthMm = 10 * 25.4;
      const vehicleHalfWidth = radius * vehicleHalfWidthMm / maxDistance;
      const vehicleFrontLength = radius * vehicleFrontLengthMm / maxDistance;
      context.fillStyle = "#e9f1f433";
      context.strokeStyle = "#e9f1f4";
      context.lineWidth = 2;
      context.beginPath();
      context.moveTo(-vehicleHalfWidth * 0.58, 0);
      context.lineTo(-vehicleHalfWidth, -vehicleFrontLength * 0.22);
      context.lineTo(-vehicleHalfWidth, -vehicleFrontLength * 0.7);
      context.quadraticCurveTo(
        -vehicleHalfWidth,
        -vehicleFrontLength,
        -vehicleHalfWidth * 0.52,
        -vehicleFrontLength
      );
      context.lineTo(vehicleHalfWidth * 0.52, -vehicleFrontLength);
      context.quadraticCurveTo(
        vehicleHalfWidth,
        -vehicleFrontLength,
        vehicleHalfWidth,
        -vehicleFrontLength * 0.7
      );
      context.lineTo(vehicleHalfWidth, -vehicleFrontLength * 0.22);
      context.lineTo(vehicleHalfWidth * 0.58, 0);
      context.closePath();
      context.fill();
      context.stroke();

      const steeringRadians = -clamp(latestSteeringAngleDegrees, -45, 45) * Math.PI / 180;
      const wheelbaseMm = 530;
      const wheelOffsetX = vehicleHalfWidth * 0.72;
      const wheelCenterY = -vehicleFrontLength * 0.72;
      const wheelLength = Math.max(9, radius * 180 / maxDistance);
      const wheelWidth = Math.max(3, radius * 65 / maxDistance);
      const drawWheel = (wheelX) => {
        context.save();
        context.translate(wheelX, wheelCenterY);
        context.rotate(steeringRadians);
        context.fillStyle = "#b22222";
        context.strokeStyle = "#f3dada";
        context.lineWidth = 1;
        context.fillRect(-wheelWidth / 2, -wheelLength / 2, wheelWidth, wheelLength);
        context.strokeRect(-wheelWidth / 2, -wheelLength / 2, wheelWidth, wheelLength);
        context.restore();
      };
      drawWheel(-wheelOffsetX);
      drawWheel(wheelOffsetX);

      const trajectoryDistanceMm = maxDistance * 0.78;
      const pixelsPerMm = radius / maxDistance;
      const trajectoryColor = "#d45a5a";
      context.strokeStyle = trajectoryColor;
      context.lineWidth = 2;
      context.setLineDash([8, 6]);
      const drawTrajectory = (startX) => {
        context.beginPath();
        context.moveTo(startX, wheelCenterY);
        if (Math.abs(steeringRadians) < 0.02) {
          context.lineTo(startX, wheelCenterY - trajectoryDistanceMm * pixelsPerMm);
        } else {
          const rearAxleTurnRadiusMm = wheelbaseMm / Math.tan(steeringRadians);
          const trackOffsetMm = startX / pixelsPerMm;
          const sampleCount = 48;
          for (let sample = 1; sample <= sampleCount; sample += 1) {
            const travelMm = trajectoryDistanceMm * sample / sampleCount;
            const yawRadians = travelMm / rearAxleTurnRadiusMm;
            const centerXmm =
              rearAxleTurnRadiusMm * (1 - Math.cos(yawRadians))
              + wheelbaseMm * Math.sin(yawRadians);
            const centerForwardMm =
              rearAxleTurnRadiusMm * Math.sin(yawRadians)
              + wheelbaseMm * (Math.cos(yawRadians) - 1);
            const pathXmm = centerXmm + trackOffsetMm * Math.cos(yawRadians);
            const pathForwardMm =
              centerForwardMm - trackOffsetMm * Math.sin(yawRadians);
            context.lineTo(
              pathXmm * pixelsPerMm,
              wheelCenterY - pathForwardMm * pixelsPerMm
            );
          }
        }
        context.stroke();
      };
      drawTrajectory(-wheelOffsetX);
      drawTrajectory(wheelOffsetX);
      context.setLineDash([]);
      const obstacleLabel = nearest === null
        ? "전방 장애물 없음"
        : `전방 장애물 ${(nearest / 1000).toFixed(2)} m`;
      context.font = "700 11px ui-monospace, monospace";
      context.lineWidth = 4;
      context.strokeStyle = "#000000dd";
      context.fillStyle = "#f2f2f2";
      context.strokeText(obstacleLabel, 0, -radius + 32);
      context.fillText(obstacleLabel, 0, -radius + 32);

      const projectedPoints = visiblePoints.map((point) => {
        const angle = point.bearing_degrees * Math.PI / 180;
        const pointRadius = Math.min(radius, point.distance_mm / maxDistance * radius);
        return {
          ...point,
          x: Math.sin(angle) * pointRadius,
          y: -Math.cos(angle) * pointRadius,
        };
      });

      const pointColor = (distance, confidence = 255) => {
        if (distance < 800) return "#ff5f57";
        if (distance < 1500) return "#f0c866";
        if (confidence < 80) return "#71807d";
        return "#48d7c7";
      };
      context.lineWidth = 2.5;
      context.lineCap = "round";
      context.lineJoin = "round";
      for (let index = 1; index < projectedPoints.length; index += 1) {
        const previous = projectedPoints[index - 1];
        const point = projectedPoints[index];
        const angleGap = point.bearing_degrees - previous.bearing_degrees;
        const distanceGap = Math.abs(point.distance_mm - previous.distance_mm);
        const surfaceTolerance = Math.max(
          140,
          Math.min(point.distance_mm, previous.distance_mm) * 0.12
        );
        if (angleGap > 2.2 || distanceGap > surfaceTolerance) continue;

        context.beginPath();
        context.moveTo(previous.x, previous.y);
        context.lineTo(point.x, point.y);
        context.strokeStyle = pointColor(
          (point.distance_mm + previous.distance_mm) / 2,
          (point.confidence + previous.confidence) / 2
        );
        context.stroke();
      }

      projectedPoints.forEach((point) => {
        const danger = point.distance_mm < 800;
        const confidenceAlpha = clamp(point.confidence / 180, 0.18, 1);
        context.beginPath();
        context.arc(point.x, point.y, danger ? 3.2 : 2.4, 0, Math.PI * 2);
        context.globalAlpha = confidenceAlpha;
        context.fillStyle = pointColor(point.distance_mm, point.confidence);
        context.fill();
      });
      context.globalAlpha = 1;
      context.restore();

      const speed = Number.isFinite(lidar.rotation_hz) ? `${lidar.rotation_hz.toFixed(1)} Hz` : "-- Hz";
      status.textContent = nearest === null
        ? `${speed} · 전방 장애물 없음`
        : `${speed} · 전방 장애물 ${(nearest / 1000).toFixed(2)} m`;
    };

    const refreshLidar = async () => {
      if (lidarRequestInFlight) return;
      lidarRequestInFlight = true;
      try {
        const response = await fetch("/api/lidar", { cache: "no-store" });
        drawLidarMap(await response.json());
      } catch {
        drawLidarMap({ connected: false, error: "CONNECTION LOST" });
      } finally {
        lidarRequestInFlight = false;
      }
    };


    const detectLaneLine = (gray, side, width, height) => {
      const bins = new Map();
      const centerX = width / 2;
      const horizonY = Math.round(height * 0.54);

      for (let y = horizonY + 1; y < height - 1; y += 2) {
        for (let x = 1; x < width - 1; x += 2) {
          const top = (y - 1) * width;
          const middle = y * width;
          const bottom = (y + 1) * width;
          const gradientX =
            -gray[top + x - 1] + gray[top + x + 1]
            - 2 * gray[middle + x - 1] + 2 * gray[middle + x + 1]
            - gray[bottom + x - 1] + gray[bottom + x + 1];
          const gradientY =
            -gray[top + x - 1] - 2 * gray[top + x] - gray[top + x + 1]
            + gray[bottom + x - 1] + 2 * gray[bottom + x] + gray[bottom + x + 1];
          const strength = Math.abs(gradientX) + Math.abs(gradientY);
          if (strength < 260 || Math.abs(gradientX) < 35) continue;

          const xPerY = -gradientY / gradientX;
          const isLeft = side === "left"
            && xPerY < -0.15 && xPerY > -2.5 && x < centerX * 1.15;
          const isRight = side === "right"
            && xPerY > 0.15 && xPerY < 2.5 && x > centerX * 0.85;
          if (!isLeft && !isRight) continue;

          const intercept = x - xPerY * y;
          const key = `${Math.round(xPerY * 10)}:${Math.round(intercept / 8)}`;
          const bin = bins.get(key) || {
            score: 0,
            weightedSlope: 0,
            weightedIntercept: 0,
          };
          bin.score += strength;
          bin.weightedSlope += xPerY * strength;
          bin.weightedIntercept += intercept * strength;
          bins.set(key, bin);
        }
      }

      let best = null;
      bins.forEach((bin) => {
        if (!best || bin.score > best.score) best = bin;
      });
      if (!best || best.score < 4500) return null;

      const xPerY = best.weightedSlope / best.score;
      const intercept = best.weightedIntercept / best.score;
      const bottomY = height - 1;
      const topY = Math.round(height * 0.58);
      const bottomX = xPerY * bottomY + intercept;
      const topX = xPerY * topY + intercept;
      if (side === "left" && (bottomX >= centerX || topX > width * 0.68)) return null;
      if (side === "right" && (bottomX <= centerX || topX < width * 0.32)) return null;
      if (bottomX < 0 || bottomX >= width || topX < 0 || topX >= width) return null;
      return { bottomX, bottomY, topX, topY };
    };

    const drawLaneOverlay = (leftLane, rightLane, sourceWidth, sourceHeight) => {
      const image = document.getElementById("camera-stream");
      const canvas = document.getElementById("lane-canvas");
      const width = image.clientWidth || image.naturalWidth || 1280;
      const height = image.clientHeight || image.naturalHeight || 720;
      if (canvas.width !== width || canvas.height !== height) {
        canvas.width = width;
        canvas.height = height;
      }

      const context = canvas.getContext("2d");
      context.clearRect(0, 0, width, height);
      const scaleX = width / sourceWidth;
      const scaleY = height / sourceHeight;
      const scaleLine = (line) => line && ({
        bottomX: line.bottomX * scaleX,
        bottomY: line.bottomY * scaleY,
        topX: line.topX * scaleX,
        topY: line.topY * scaleY,
      });
      const left = scaleLine(leftLane);
      const right = scaleLine(rightLane);

      if (left && right && left.topX < right.topX) {
        context.beginPath();
        context.moveTo(left.bottomX, left.bottomY);
        context.lineTo(left.topX, left.topY);
        context.lineTo(right.topX, right.topY);
        context.lineTo(right.bottomX, right.bottomY);
        context.closePath();
        context.fillStyle = "#b222221f";
        context.fill();

        const bottomCenter = (left.bottomX + right.bottomX) / 2;
        const topCenter = (left.topX + right.topX) / 2;
        context.beginPath();
        context.setLineDash([12, 10]);
        context.moveTo(bottomCenter, height);
        context.lineTo(topCenter, left.topY);
        context.strokeStyle = "#e9f1f4aa";
        context.lineWidth = 2;
        context.stroke();
        context.setLineDash([]);
      }

      const drawLine = (line, color) => {
        if (!line) return;
        context.beginPath();
        context.moveTo(line.bottomX, line.bottomY);
        context.lineTo(line.topX, line.topY);
        context.strokeStyle = color;
        context.lineWidth = Math.max(5, width / 180);
        context.shadowColor = color;
        context.shadowBlur = 10;
        context.stroke();
        context.shadowBlur = 0;
      };
      drawLine(left, "#b22222");
      drawLine(right, "#b22222");
    };

    const runLaneDetection = (sourceCanvas) => {
      if (!laneProcessingContext || laneDetectionStopped) return;
      const status = document.getElementById("lane-meta");
      const processWidth = 320;
      const processHeight = 180;

      try {
        if (sourceCanvas) {
          laneProcessingContext.drawImage(sourceCanvas, 0, 0, processWidth, processHeight);
          const pixels = laneProcessingContext.getImageData(0, 0, processWidth, processHeight).data;
          for (let index = 0, pixel = 0; index < pixels.length; index += 4, pixel += 1) {
            laneGray[pixel] = Math.round(
              pixels[index] * 0.299 + pixels[index + 1] * 0.587 + pixels[index + 2] * 0.114
            );
          }

          const leftLane = detectLaneLine(laneGray, "left", processWidth, processHeight);
          const rightLane = detectLaneLine(laneGray, "right", processWidth, processHeight);
          drawLaneOverlay(leftLane, rightLane, processWidth, processHeight);

          laneFrames += 1;
          const now = performance.now();
          const elapsed = now - laneFpsStartedAt;
          if (elapsed >= 1000) {
            const fps = laneFrames * 1000 / elapsed;
            const laneState = leftLane && rightLane
              ? "BOTH"
              : leftLane ? "LEFT" : rightLane ? "RIGHT" : "SEARCHING";
            status.textContent = `LANE ${laneState} · ${fps.toFixed(1)} FPS`;
            laneFrames = 0;
            laneFpsStartedAt = now;
          }
        }
      } catch (error) {
        status.textContent = "LANE CV ERROR";
        console.error("Lane detection frame failed", error);
      }
    };

    const initLaneDetection = () => {
      const status = document.getElementById("lane-meta");
      laneProcessingCanvas = document.createElement("canvas");
      laneProcessingCanvas.width = 320;
      laneProcessingCanvas.height = 180;
      laneProcessingContext = laneProcessingCanvas.getContext("2d", { willReadFrequently: true });
      laneGray = new Uint8Array(320 * 180);
      visionFrameCanvas = document.createElement("canvas");
      visionFrameCanvas.width = 640;
      visionFrameCanvas.height = 360;
      visionFrameContext = visionFrameCanvas.getContext("2d");
      status.textContent = "LANE EDGE READY";
    };

    const detectionColors = {
      "person": "#41e4d2",
      "stop sign": "#ff4d5f",
      "traffic light": "#f6c760",
      "car": "#67a7ff",
      "truck": "#67a7ff",
      "bus": "#67a7ff",
      "bicycle": "#7df78b",
      "motorcycle": "#7df78b",
    };

    const drawDetections = (predictions, sourceWidth, sourceHeight) => {
      const image = document.getElementById("camera-stream");
      const canvas = document.getElementById("detection-canvas");
      const width = image.clientWidth || image.naturalWidth || 1280;
      const height = image.clientHeight || image.naturalHeight || 720;
      if (canvas.width !== width || canvas.height !== height) {
        canvas.width = width;
        canvas.height = height;
      }

      const context = canvas.getContext("2d");
      context.clearRect(0, 0, width, height);
      context.lineWidth = Math.max(3, width / 320);
      context.font = `700 ${Math.max(18, Math.round(width / 48))}px ui-monospace, monospace`;
      context.textBaseline = "top";
      const scaleX = width / sourceWidth;
      const scaleY = height / sourceHeight;

      predictions.forEach((prediction) => {
        const [sourceX, sourceY, sourceBoxWidth, sourceBoxHeight] = prediction.bbox;
        const x = sourceX * scaleX;
        const y = sourceY * scaleY;
        const boxWidth = sourceBoxWidth * scaleX;
        const boxHeight = sourceBoxHeight * scaleY;
        const color = detectionColors[prediction.class] || "#e9f1f4";
        const label = `${prediction.class.toUpperCase()} ${Math.round(prediction.score * 100)}%`;
        const labelWidth = context.measureText(label).width + 16;
        const labelHeight = Math.max(27, Math.round(width / 38));
        const labelY = Math.max(0, y - labelHeight);

        context.strokeStyle = color;
        context.fillStyle = `${color}22`;
        context.strokeRect(x, y, boxWidth, boxHeight);
        context.fillRect(x, y, boxWidth, boxHeight);
        context.fillStyle = color;
        context.fillRect(x, labelY, labelWidth, labelHeight);
        context.fillStyle = "#05080b";
        context.fillText(label, x + 8, labelY + 3);
      });
    };

    const runObjectDetection = async () => {
      if (!objectDetector || detectionStopped) return;
      const image = document.getElementById("camera-stream");
      const status = document.getElementById("detection-meta");
      if (driveArmed) {
        status.textContent = "AI PAUSED · MANUAL CONTROL";
        window.setTimeout(runObjectDetection, 250);
        return;
      }
      const startedAt = performance.now();

      try {
        if (image.naturalWidth > 0 && visionFrameContext) {
          visionFrameContext.drawImage(image, 0, 0, visionFrameCanvas.width, visionFrameCanvas.height);
          const predictions = await objectDetector.detect(visionFrameCanvas, 20, 0.45);
          drawDetections(predictions, visionFrameCanvas.width, visionFrameCanvas.height);
          laneFrameDivider += 1;
          if (laneFrameDivider >= 3) {
            laneFrameDivider = 0;
            runLaneDetection(visionFrameCanvas);
          }
          detectionFrames += 1;
          const now = performance.now();
          const elapsed = now - detectionFpsStartedAt;
          if (elapsed >= 1000) {
            const fps = detectionFrames * 1000 / elapsed;
            status.textContent = `AI ${tf.getBackend().toUpperCase()} · ${fps.toFixed(1)} FPS · ${predictions.length} OBJECTS`;
            detectionFrames = 0;
            detectionFpsStartedAt = now;
          }
        }
      } catch (error) {
        status.textContent = "AI FRAME ERROR";
      }

      const delay = Math.max(0, 100 - (performance.now() - startedAt));
      window.setTimeout(runObjectDetection, delay);
    };

    const initObjectDetection = async () => {
      const status = document.getElementById("detection-meta");
      try {
        status.textContent = "AI LIBRARY LOADING";
        await loadExternalScript(
          "https://cdn.jsdelivr.net/npm/@tensorflow/tfjs@4.22.0/dist/tf.min.js"
        );
        await loadExternalScript(
          "https://cdn.jsdelivr.net/npm/@tensorflow-models/coco-ssd@2.2.3/dist/coco-ssd.min.js"
        );
        await tf.setBackend("webgl");
        await tf.ready();
        status.textContent = "AI WEBGL · 모델 준비 중";
        objectDetector = await cocoSsd.load({ base: "lite_mobilenet_v2" });
        status.textContent = "AI WEBGL · READY";
        runObjectDetection();
      } catch (error) {
        status.textContent = "AI UNAVAILABLE";
        console.error("Object detection initialization failed", error);
      }
    };

    const initMap = () => {
      if (mapInstance || !window.L) return;
      mapInstance = L.map("leaflet-map", {
        zoomControl: true,
        attributionControl: true,
      }).setView([36.5, 127.8], 7);
      L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
        maxNativeZoom: 19,
        maxZoom: 20,
        attribution: "© OpenStreetMap",
      }).addTo(mapInstance);
      routeLine = L.polyline([], {
        color: "#b22222",
        weight: 3,
        opacity: 0.85,
      }).addTo(mapInstance);
      recordedRouteLine = L.polyline([], {
        color: "#f0c866",
        weight: 4,
        opacity: 0.95,
        dashArray: "8 5",
      }).addTo(mapInstance);
    };

    const updateMap = (gps) => {
      initMap();
      if (!mapInstance || !Number.isFinite(gps.latitude) || !Number.isFinite(gps.longitude)) {
        return;
      }

      const point = [gps.latitude, gps.longitude];
      if (!positionMarker) {
        positionMarker = L.circleMarker(point, {
          radius: 8,
          color: "#eaffff",
          weight: 2,
          fillColor: "#b22222",
          fillOpacity: 0.95,
        }).addTo(mapInstance);
        mapInstance.setView(point, 18);
      } else {
        positionMarker.setLatLng(point);
        if (!mapInstance.getBounds().pad(-0.2).contains(point)) {
          mapInstance.panTo(point);
        }
      }

      const lastPoint = routePoints[routePoints.length - 1];
      if (!lastPoint || Math.hypot(lastPoint[0] - point[0], lastPoint[1] - point[1]) > 0.000001) {
        routePoints.push(point);
        if (routePoints.length > 500) routePoints.shift();
        routeLine.setLatLngs(routePoints);
      }
    };

    const renderImu = (imu) => {
      const orientation = imu.orientation;
      const formatAngle = (value, empty) =>
        Number.isFinite(value) ? `${value.toFixed(1)}°` : empty;
      const formatSignedAngle = (value, empty) =>
        Number.isFinite(value) ? `${value >= 0 ? "+" : ""}${value.toFixed(1)}°` : empty;
      const globalHeading = Number.isFinite(orientation.global_heading_degrees)
        ? orientation.global_heading_degrees
        : orientation.heading_degrees;
      document.getElementById("lidar-global-heading").textContent =
        formatAngle(globalHeading, "---.-°");
      document.getElementById("lidar-relative-yaw").textContent =
        formatSignedAngle(orientation.relative_yaw_degrees, "--.-°");
      document.getElementById("imu-roll").textContent =
        formatAngle(orientation.roll_degrees, "--.-°");
      document.getElementById("imu-pitch").textContent =
        formatAngle(orientation.pitch_degrees, "--.-°");

      if (Number.isFinite(globalHeading)) {
        if (previousHeading === null) {
          previousHeading = globalHeading;
          unwrappedHeading = globalHeading;
        } else {
          const delta = ((globalHeading - previousHeading + 540) % 360) - 180;
          unwrappedHeading += delta;
          previousHeading = globalHeading;
        }
        document.getElementById("heading-needle").style.transform =
          `translateX(-50%) rotate(${unwrappedHeading}deg)`;
      }

      const calibration = orientation.calibration;
      const imuTag = document.getElementById("imu-tag");
      const turnDirection = orientation.turn_direction || "STRAIGHT";
      const calibrationState = orientation.calibrated ? "보정 완료" : "보정 필요";
      const turnLabel = { LEFT: "좌회전", RIGHT: "우회전", STRAIGHT: "직진" }[turnDirection] || turnDirection;
      imuTag.textContent = calibration.active
        ? `보정 중 ${Math.round(calibration.progress * 100)}%`
        : `${calibrationState} · ${turnLabel}`;
      imuTag.classList.toggle("turn-left", !calibration.active && turnDirection === "LEFT");
      imuTag.classList.toggle("turn-right", !calibration.active && turnDirection === "RIGHT");
      document.getElementById("calibration-progress").style.width =
        `${Math.round(calibration.progress * 100)}%`;
      document.getElementById("calibration-message").textContent = calibration.message;
      const button = document.getElementById("calibrate-button");
      button.disabled = !imu.connected || calibration.active;
      button.textContent = calibration.active ? "센서를 회전하세요" : "20초 IMU 보정";
      const zeroButton = document.getElementById("heading-zero-button");
      zeroButton.disabled = !imu.connected || !Number.isFinite(globalHeading);
    };

    const refreshImu = async () => {
      try {
        const response = await fetch("/api/imu", { cache: "no-store" });
        renderImu(await response.json());
      } catch {}
    };

    const renderNetwork = (network, webLatencyMs) => {
      const wifiConnected = Boolean(network.wifi.connected);
      const wifiState = document.getElementById("wifi-state");
      wifiState.textContent = wifiConnected
        ? `CONNECTED ${network.wifi.ssid ? `· ${network.wifi.ssid}` : ""}`
        : "NOT USED";
      wifiState.className = `network-value ${wifiConnected ? "good" : ""}`;

      const internetState = document.getElementById("internet-state");
      internetState.textContent = network.internet.online
        ? `${network.internet.latency_ms.toFixed(1)} ms · ${network.internet.via.toUpperCase()}`
        : "OFFLINE";
      internetState.className = network.internet.online ? "good" : "warn";

      const pcPing = document.getElementById("pc-ping");
      pcPing.textContent = network.client.ping_ok
        ? `${network.client.latency_ms.toFixed(1)} ms · ${network.client.ip}`
        : `NO ICMP · ${network.client.ip || "--"}`;
      pcPing.className = network.client.ping_ok ? "good" : "warn";

      const wifiButton = document.getElementById("wifi-settings-open");
      wifiButton.classList.toggle("active", wifiConnected);
      wifiButton.textContent = wifiConnected && network.wifi.ssid
        ? `Wi-Fi · ${network.wifi.ssid}`
        : "네트워크";
    };

    const renderNtrip = (ntrip) => {
      if (!ntrip) return;
      const button = document.getElementById("ntrip-settings-open");
      button.classList.toggle("active", Boolean(ntrip.connected));
      button.textContent = ntrip.connected ? "RTK · 연결됨" : "RTK 설정";
      const status = document.getElementById("ntrip-settings-status");
      if (!status) return;
      status.className = `settings-status ${ntrip.connected ? "good" : ntrip.error ? "warn" : ""}`;
      if (ntrip.connected) {
        status.textContent = `${ntrip.host}:${ntrip.port}/${ntrip.mountpoint} · RTCM ${ntrip.bytes_received || 0} bytes · ${ntrip.status}`;
      } else if (ntrip.error) {
        status.textContent = `${ntrip.status || "ERROR"} · ${ntrip.error}`;
      } else {
        status.textContent = ntrip.configured
          ? `${ntrip.status || "STOPPED"} · ${ntrip.host}:${ntrip.port}/${ntrip.mountpoint}`
          : "NTRIP 접속 정보를 입력하세요.";
      }
    };

    const setDriveUi = (throttle) => {
      const percent = Math.round(throttle * 100);
      document.getElementById("lidar-drive-throttle").textContent = `${percent}%`;
      const lidarFill = document.getElementById("lidar-drive-fill");
      lidarFill.style.height = `${Math.abs(percent) / 2}%`;
      lidarFill.style.top = percent < 0 ? "50%" : "auto";
      lidarFill.style.bottom = percent < 0 ? "auto" : "50%";
      lidarFill.classList.toggle("reverse", percent < 0);
      const lidarDriveButton = document.getElementById("lidar-drive-button");
      lidarDriveButton.classList.toggle("armed", driveArmed);
      lidarDriveButton.textContent = driveArmed ? "주행 중지" : "주행 활성화";
    };

    const getActiveGamepad = () => {
      const gamepads = navigator.getGamepads ? Array.from(navigator.getGamepads()).filter(Boolean) : [];
      return gamepads[0] || null;
    };

    const updateGamepadHelp = () => {
      const help = document.getElementById("drive-help");
      if (!navigator.getGamepads) {
        help.textContent = "이 브라우저는 게임패드를 지원하지 않습니다. 키보드 전후진: W/S.";
        return null;
      }
      const gamepad = getActiveGamepad();
      if (!gamepad) {
        if (activeGamepadId !== null) {
          driveArmed = false;
          sendDriveCommand(0, false);
          sendSteeringStop();
        }
        activeGamepadId = null;
        help.textContent = "게임패드를 연결한 뒤 아무 버튼이나 한 번 눌러주세요. 키보드 전후진: W/S.";
        return null;
      }
      if (activeGamepadId !== gamepad.id) {
        activeGamepadId = gamepad.id;
        help.textContent = `게임패드 준비됨: ${gamepad.id}. 왼쪽 Y축 전후진 · 오른쪽 X축 조향.`;
      }
      return gamepad;
    };

    const readDriveThrottle = () => {
      const gamepad = updateGamepadHelp();
      if (gamepad && gamepad.axes.length > 1) {
        return clamp(applyDeadzone(-gamepad.axes[1]), -1, 1);
      }
      const forward = pressedKeys.has("KeyW") || pressedKeys.has("ArrowUp") ? 1 : 0;
      const reverse = pressedKeys.has("KeyS") || pressedKeys.has("ArrowDown") ? 1 : 0;
      return forward - reverse;
    };

    const readGamepadSteering = () => {
      const gamepad = getActiveGamepad();
      if (!gamepad) return 0;
      const axisIndex = gamepad.axes.length > 2 ? 2 : 0;
      return clamp(applyDeadzone(gamepad.axes[axisIndex] || 0), -1, 1);
    };

    const readDeadman = () => {
      const gamepad = getActiveGamepad();
      if (gamepad) return Boolean(gamepad.buttons[0]?.pressed);
      return pressedKeys.has("ShiftLeft") || pressedKeys.has("ShiftRight");
    };

    const triggerSoftwareEmergencyStop = async (source = "controller") => {
      if (emergencyStopRequestInFlight) return;
      emergencyStopRequestInFlight = true;
      driveArmed = false;
      lastSentThrottle = 0;
      gamepadSteeringNeedsNeutral = true;
      setDriveUi(0);
      sendDriveCommand(0, false);
      sendSteeringStop();
      try {
        const response = await fetch("/api/safety/emergency-stop", {
          method: "POST",
          cache: "no-store",
        });
        if (!response.ok) throw new Error(`긴급정지 실패: ${response.status}`);
        setAutonomyStatus(
          source === "controller"
            ? "컨트롤러 B/○ 버튼으로 긴급정지되었습니다."
            : "소프트웨어 긴급정지가 작동했습니다.",
          "warn",
        );
        await refreshAutonomy();
      } catch (error) {
        setAutonomyStatus(error.message, "warn");
      } finally {
        emergencyStopRequestInFlight = false;
      }
    };

    const refreshGamepadEmergencyStop = () => {
      const gamepad = getActiveGamepad();
      const pressed = Boolean(gamepad?.buttons[1]?.pressed);
      if (pressed && !lastGamepadEmergencyPressed) {
        triggerSoftwareEmergencyStop("controller");
      }
      lastGamepadEmergencyPressed = pressed;
    };

    const sendDriveCommand = async (throttle, enabled, deadman = false) => {
      try {
        const response = await fetch("/api/motor", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ throttle, enabled, deadman }),
          cache: "no-store",
        });
        const result = await response.json();
        setDriveUi(result.throttle || 0);
      } catch {
        setDriveUi(0);
      }
    };

    const renderSteering = (state) => {
      const encoderOnline = Boolean(state?.encoder_connected);
      latestSteeringAngleDegrees = Number.isFinite(state?.steering_angle_degrees)
        ? state.steering_angle_degrees
        : 0;

      document.getElementById("steering-raw").textContent =
        Number.isFinite(state?.encoder_raw) ? String(state.encoder_raw) : "----";
      document.getElementById("lidar-steering-raw").textContent =
        Number.isFinite(state?.encoder_raw) ? String(state.encoder_raw) : "----";
      document.getElementById("lidar-steering-angle").textContent =
        Number.isFinite(state?.steering_angle_degrees)
          ? `${state.steering_angle_degrees >= 0 ? "+" : ""}${state.steering_angle_degrees.toFixed(2)}°`
          : "0° 미설정";
      document.getElementById("steering-zero").textContent =
        Number.isFinite(state?.encoder_zero_raw) ? String(state.encoder_zero_raw) : "미설정";
      document.getElementById("steering-range").textContent =
        Number.isFinite(state?.steer_right_raw_limit) && Number.isFinite(state?.steer_left_raw_limit)
          ? `${state.steer_right_raw_limit} ~ ${state.steer_left_raw_limit}`
          : "---- ~ ----";

      const updateConfigInput = (id, value) => {
        const input = document.getElementById(id);
        if (!steeringConfigEditing && Number.isFinite(value)) {
          input.value = String(value);
        }
      };
      updateConfigInput("steering-config-right", state?.steer_right_reference_raw);
      updateConfigInput("steering-config-center", state?.steer_center_raw);
      updateConfigInput("steering-config-left", state?.steer_left_reference_raw);
      updateConfigInput("steering-config-allowance", state?.steer_limit_allowance_raw);

      const controlsEnabled = encoderOnline;
      const configEnabled = controlsEnabled && Boolean(state?.config_supported);
      steeringControlAvailable = controlsEnabled;
      steeringCenterSupported = Boolean(state?.center_supported);
      document.getElementById("steering-config-open").disabled = !configEnabled;
      document.getElementById("steering-config-save").disabled = !configEnabled;
      for (const input of document.querySelectorAll(".steering-config input")) {
        input.disabled = !configEnabled;
      }
    };

    const drainSteeringCommands = async () => {
      if (steeringRequestInFlight) return;
      steeringRequestInFlight = true;
      try {
        while (steeringPendingDirection !== null) {
          const steeringCommand = steeringPendingDirection;
          steeringPendingDirection = null;
          const response = typeof steeringCommand === "number"
            ? await fetch("/api/steering", {
                method: "POST",
                headers: { "Content-Type": "application/json" },
                body: JSON.stringify({ direction: steeringCommand }),
                cache: "no-store",
              })
            : await fetch(`/api/steering/${steeringCommand}`, { method: "POST", cache: "no-store" });
          if (!response.ok) throw new Error(`Steering request failed: ${response.status}`);
          renderSteering(await response.json());
        }
      } catch (error) {
        console.error(error);
      } finally {
        steeringRequestInFlight = false;
        if (steeringPendingDirection !== null) drainSteeringCommands();
      }
    };

    const setSteeringInputUi = (direction) => {
      const normalizedDirection = clamp(Number(direction) || 0, -1, 1);
      const inputPercent = Math.round(normalizedDirection * 100);
      const inputElement = document.getElementById("lidar-steering-input");
      if (inputElement) {
        inputElement.textContent = `입력 ${inputPercent > 0 ? "+" : ""}${inputPercent}%`;
      }
    };

    const sendSteeringCommand = (direction) => {
      setSteeringInputUi(direction);
      steeringPendingDirection = direction;
      drainSteeringCommands();
    };

    const sendSteeringCenter = () => {
      sendSteeringCommand(0);
    };

    const sendSteeringStop = () => {
      setSteeringInputUi(0);
      steeringPendingDirection = "stop";
      drainSteeringCommands();
    };

    const clearSteeringHold = () => {
      if (steeringRepeatTimer !== null) {
        clearInterval(steeringRepeatTimer);
        steeringRepeatTimer = null;
      }
      const wasSteering = steeringHoldDirection !== 0;
      steeringHoldDirection = 0;
      return wasSteering;
    };

    const stopSteeringHold = () => {
      if (clearSteeringHold()) {
        lastSentGamepadSteering = 0;
        sendSteeringCenter();
      }
    };

    const startSteeringHold = (direction) => {
      clearSteeringHold();
      gamepadSteeringNeedsNeutral = true;
      lastSentGamepadSteering = 0;
      steeringHoldDirection = direction;
      sendSteeringCommand(direction);
      steeringRepeatTimer = setInterval(() => sendSteeringCommand(direction), 150);
    };

    const refreshSteering = async () => {
      try {
        const response = await fetch("/api/steering", { cache: "no-store" });
        renderSteering(await response.json());
      } catch {}
    };

    const refreshDrive = () => {
      refreshGamepadEmergencyStop();
      if (!driveArmed) updateGamepadHelp();
      driveThrottle = driveArmed ? readDriveThrottle() : 0;
      if (!driveArmed) lastSentDeadman = false;
      setDriveUi(driveThrottle);
      const now = performance.now();
      const deadmanPressed = driveArmed && readDeadman();
      if (driveArmed && (
        lastSentThrottle === null ||
        Math.abs(driveThrottle - lastSentThrottle) >= 0.02 ||
        deadmanPressed !== lastSentDeadman ||
        now - lastDriveSentAt > 100
      )) {
        lastSentThrottle = driveThrottle;
        lastSentDeadman = deadmanPressed;
        lastDriveSentAt = now;
        sendDriveCommand(driveThrottle, driveArmed, deadmanPressed);
      } else if (!driveArmed && lastSentThrottle !== 0) {
        lastSentThrottle = 0;
        lastSentDeadman = false;
        lastDriveSentAt = now;
        sendDriveCommand(0, false);
      }

      if (steeringHoldDirection === 0) {
        const requestedGamepadSteering = driveArmed && steeringControlAvailable
          ? readGamepadSteering()
          : 0;
        if (gamepadSteeringNeedsNeutral && requestedGamepadSteering === 0) {
          gamepadSteeringNeedsNeutral = false;
        }
        const gamepadSteering = gamepadSteeringNeedsNeutral
          ? 0
          : requestedGamepadSteering;
        const steeringChanged = Math.abs(gamepadSteering - lastSentGamepadSteering) >= 0.02;
        const steeringHeartbeatDue = gamepadSteering !== 0 && now - lastGamepadSteeringSentAt > 200;
        if (steeringChanged || steeringHeartbeatDue) {
          const previousGamepadSteering = lastSentGamepadSteering;
          lastSentGamepadSteering = gamepadSteering;
          lastGamepadSteeringSentAt = now;
          if (gamepadSteering === 0 && previousGamepadSteering !== 0) {
            sendSteeringCenter();
          } else if (gamepadSteering !== 0) {
            sendSteeringCommand(gamepadSteering);
          }
        }
      }
    };

    const refresh = async () => {
      try {
        const requestStarted = performance.now();
        const response = await fetch("/api/status", { cache: "no-store" });
        const status = await response.json();
        const webLatencyMs = performance.now() - requestStarted;
        renderRaspberrySystem(status.system || {});
        setDevice("camera", status.camera.online, status.camera.online ? "스트리밍" : "오프라인");
        setDevice("gps", status.devices.gps.connected, status.devices.gps.port || "연결 안 됨");
        setDevice(
          "imu",
          status.devices.imu.connected,
          status.devices.imu.connected
            ? status.devices.imu.addresses.join(", ")
            : status.devices.imu.bus_online ? "센서 미감지" : "I²C 오프라인"
        );
        setDevice(
          "lidar",
          status.devices.lidar.connected,
          status.devices.lidar.connected
            ? `${status.devices.lidar.point_count}개 포인트`
            : status.devices.lidar.error || "데이터 없음"
        );
        setDevice("arduino", status.devices.arduino.connected, status.devices.arduino.port || "연결 안 됨");
        renderSteering(status.devices.arduino);
        document.getElementById("gps-tag").textContent =
          status.devices.gps.connected ? "GNSS 연결됨" : "GNSS 미연결";
        const gps = status.devices.gps;
        document.getElementById("fix").textContent = gps.fix || "NO FIX";
        document.getElementById("satellites").textContent =
          Number.isFinite(gps.satellites_used) ? gps.satellites_used : "--";
        document.getElementById("latitude").textContent =
          Number.isFinite(gps.latitude) ? gps.latitude.toFixed(7) : "--.-------";
        document.getElementById("longitude").textContent =
          Number.isFinite(gps.longitude) ? gps.longitude.toFixed(7) : "---.-------";
        renderNetwork(status.network, webLatencyMs);
        renderNtrip(status.ntrip);
        updateMap(gps);
        document.getElementById("map-fix").textContent =
          gps.mode >= 2 ? `${gps.fix} · HDOP ${Number.isFinite(gps.hdop) ? gps.hdop.toFixed(2) : "--"}` : "GNSS 위치 대기 중";
        document.getElementById("map-coordinates").textContent =
          Number.isFinite(gps.latitude) && Number.isFinite(gps.longitude)
            ? `${gps.latitude.toFixed(7)}, ${gps.longitude.toFixed(7)} · ALT ${Number.isFinite(gps.altitude_m) ? gps.altitude_m.toFixed(1) : "--"} m`
            : "gpsd 연결 대기 중";
      } catch {
        const piState = document.getElementById("pi-system-state");
        piState.textContent = "연결 끊김";
        piState.className = "section-state warn";
      }
    };

    document.getElementById("device-refresh").addEventListener("click", async () => {
      const button = document.getElementById("device-refresh");
      button.disabled = true;
      button.textContent = "확인 중...";
      await Promise.all([refresh(), refreshImu(), refreshLidar(), refreshSteering()]);
      button.disabled = false;
      button.textContent = "상태 새로고침";
    });

    document.getElementById("calibrate-button").addEventListener("click", async () => {
      await fetch("/api/imu/calibrate", { method: "POST" });
      refreshImu();
    });
    document.getElementById("heading-zero-button").addEventListener("click", async () => {
      await fetch("/api/imu/reset-relative-yaw", { method: "POST" });
      refreshImu();
    });
    document.getElementById("restart-button").addEventListener("click", async () => {
      const confirmed = window.confirm("라즈베리파이를 재시작하시겠습니까?");
      if (!confirmed) return;

      const button = document.getElementById("restart-button");
      button.disabled = true;
      button.textContent = "재시작 중...";
      try {
        const response = await fetch("/api/system/reboot", {
          method: "POST",
          headers: { "X-GNSS-Confirm": "reboot" },
          cache: "no-store",
        });
        if (!response.ok) throw new Error(`Reboot request failed: ${response.status}`);
      } catch (error) {
        button.disabled = false;
        button.textContent = "재시작";
        window.alert("라즈베리파이를 재시작하지 못했습니다. 연결 상태를 확인해 주세요.");
        console.error(error);
      }
    });
    document.getElementById("power-button").addEventListener("click", async () => {
      const confirmed = window.confirm(
        "라즈베리파이를 종료하시겠습니까? 카메라 스트리밍과 모든 제어가 중지됩니다."
      );
      if (!confirmed) return;

      const button = document.getElementById("power-button");
      button.disabled = true;
      button.textContent = "종료 중...";
      try {
        const response = await fetch("/api/system/poweroff", {
          method: "POST",
          headers: { "X-GNSS-Confirm": "poweroff" },
          cache: "no-store",
        });
        if (!response.ok) throw new Error(`Power off request failed: ${response.status}`);
      } catch (error) {
        button.disabled = false;
        button.textContent = "종료";
        window.alert("라즈베리파이를 종료하지 못했습니다. 연결 상태를 확인해 주세요.");
        console.error(error);
      }
    });
    const toggleDriveArmed = () => {
      driveArmed = !driveArmed;
      if (driveArmed) {
        updateGamepadHelp();
        gamepadSteeringNeedsNeutral = true;
        lastSentThrottle = null;
        lastDriveSentAt = 0;
        lastSentGamepadSteering = 0;
        lastGamepadSteeringSentAt = 0;
      }
      if (!driveArmed) {
        driveThrottle = 0;
        lastSentThrottle = null;
        lastDriveSentAt = 0;
        lastSentGamepadSteering = 0;
        lastGamepadSteeringSentAt = performance.now();
        gamepadSteeringNeedsNeutral = false;
        sendDriveCommand(0, false);
        sendSteeringStop();
      }
      setDriveUi(0);
    };
    document.getElementById("lidar-drive-button").addEventListener("click", toggleDriveArmed);
    const ntripSettingsModal = document.getElementById("ntrip-settings-modal");
    const closeNtripSettings = () => { ntripSettingsModal.hidden = true; };
    const loadNtripSettings = async () => {
      try {
        const response = await fetch("/api/ntrip", { cache: "no-store" });
        const ntrip = await response.json();
        document.getElementById("ntrip-host").value = ntrip.host || "";
        document.getElementById("ntrip-port").value = ntrip.port || 2101;
        document.getElementById("ntrip-mountpoint").value = ntrip.mountpoint || "";
        document.getElementById("ntrip-username").value = ntrip.username || "";
        document.getElementById("ntrip-password").value = "";
        document.getElementById("ntrip-password").placeholder = ntrip.password_saved
          ? "저장된 비밀번호 유지"
          : "비밀번호 입력";
        document.getElementById("ntrip-enabled").checked = Boolean(ntrip.enabled);
        renderNtrip(ntrip);
      } catch (error) {
        const status = document.getElementById("ntrip-settings-status");
        status.className = "settings-status warn";
        status.textContent = `설정 불러오기 실패 · ${error.message}`;
      }
    };
    document.getElementById("ntrip-settings-open").addEventListener("click", () => {
      ntripSettingsModal.hidden = false;
      loadNtripSettings();
      document.getElementById("ntrip-host").focus();
    });
    document.getElementById("ntrip-settings-close").addEventListener("click", closeNtripSettings);
    ntripSettingsModal.addEventListener("click", (event) => {
      if (event.target === ntripSettingsModal) closeNtripSettings();
    });
    document.getElementById("ntrip-save").addEventListener("click", async () => {
      const button = document.getElementById("ntrip-save");
      const payload = {
        host: document.getElementById("ntrip-host").value.trim(),
        port: Number(document.getElementById("ntrip-port").value),
        mountpoint: document.getElementById("ntrip-mountpoint").value.trim(),
        username: document.getElementById("ntrip-username").value.trim(),
        password: document.getElementById("ntrip-password").value,
        enabled: document.getElementById("ntrip-enabled").checked,
        tls: false,
      };
      button.disabled = true;
      try {
        const response = await fetch("/api/ntrip/config", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify(payload),
          cache: "no-store",
        });
        const result = await response.json();
        if (!response.ok) throw new Error(result.error || `저장 실패: ${response.status}`);
        renderNtrip(result);
        document.getElementById("ntrip-password").value = "";
      } catch (error) {
        window.alert(error.message);
      } finally {
        button.disabled = false;
      }
    });
    document.getElementById("ntrip-stop").addEventListener("click", async () => {
      const response = await fetch("/api/ntrip/stop", { method: "POST", cache: "no-store" });
      const result = await response.json();
      if (!response.ok) {
        window.alert(result.error || "NTRIP 중지 실패");
        return;
      }
      document.getElementById("ntrip-enabled").checked = false;
      renderNtrip(result);
    });

    const wifiSettingsModal = document.getElementById("wifi-settings-modal");
    const closeWifiSettings = () => { wifiSettingsModal.hidden = true; };
    const setWifiStatus = (message, kind = "") => {
      const status = document.getElementById("wifi-settings-status");
      status.className = `settings-status ${kind}`;
      status.textContent = message;
    };
    const selectWifiNetwork = (network, hidden = false) => {
      const panel = document.getElementById("wifi-connect-panel");
      const ssidInput = document.getElementById("wifi-ssid");
      const passwordField = document.getElementById("wifi-password-field");
      const security = network.security || "WPA2";
      panel.hidden = false;
      panel.dataset.security = security;
      ssidInput.type = hidden ? "text" : "hidden";
      ssidInput.value = hidden ? "" : network.ssid;
      ssidInput.placeholder = hidden ? "숨겨진 네트워크 이름(SSID)" : "";
      document.getElementById("wifi-selected-name").textContent = hidden
        ? "숨겨진 네트워크"
        : network.ssid;
      document.getElementById("wifi-selected-security").textContent = security;
      passwordField.hidden = security === "OPEN";
      document.getElementById("wifi-password").value = "";
      const signal = panel.querySelector(".wifi-signal-bars");
      signal.className = `wifi-signal-bars level-${Math.max(1, Math.ceil((network.signal || 100) / 25))}`;
      if (hidden) ssidInput.focus();
      else if (security === "OPEN") document.getElementById("wifi-connect").focus();
      else document.getElementById("wifi-password").focus();
    };
    const clearWifiSelection = () => {
      document.getElementById("wifi-connect-panel").hidden = true;
      document.getElementById("wifi-ssid").value = "";
      document.getElementById("wifi-password").value = "";
      for (const item of document.querySelectorAll(".wifi-network")) {
        item.classList.remove("selected");
      }
    };
    const renderWifiNetworks = (result) => {
      const list = document.getElementById("wifi-network-list");
      list.replaceChildren();
      for (const network of result.networks || []) {
        const button = document.createElement("button");
        button.type = "button";
        button.className = `wifi-network ${network.active ? "active" : ""}`;
        const signal = document.createElement("span");
        const signalLevel = Math.max(1, Math.ceil((network.signal || 0) / 25));
        signal.className = `wifi-signal-bars level-${signalLevel}`;
        signal.append(
          document.createElement("i"),
          document.createElement("i"),
          document.createElement("i"),
          document.createElement("i")
        );
        const name = document.createElement("span");
        name.className = "wifi-network-name";
        name.textContent = network.ssid;
        const details = document.createElement("small");
        details.textContent = network.active ? "연결됨" : `${network.signal}%`;
        button.append(signal, name, details);
        if (network.security && network.security !== "OPEN") {
          const lock = document.createElement("span");
          lock.className = "wifi-lock";
          lock.setAttribute("aria-label", "보안 네트워크");
          button.appendChild(lock);
        }
        if (network.active) {
          const connected = document.createElement("span");
          connected.className = "wifi-connected-mark";
          connected.textContent = "✓";
          button.appendChild(connected);
        }
        button.addEventListener("click", () => {
          for (const item of list.querySelectorAll(".wifi-network")) item.classList.remove("selected");
          button.classList.add("selected");
          selectWifiNetwork(network);
        });
        list.appendChild(button);
      }
      if (!(result.networks || []).length) {
        const empty = document.createElement("div");
        empty.className = "wifi-empty";
        empty.textContent = "검색된 Wi-Fi 네트워크가 없습니다.";
        list.appendChild(empty);
      }
      const wifi = result.wifi || {};
      document.getElementById("wifi-interface-state").textContent = wifi.connected
        ? `${wifi.ssid || "연결됨"} · ${wifi.ipv4 || "IP 할당 중"}`
        : "켜짐 · 연결 안 됨";
      if (result.error) {
        setWifiStatus(result.error, "warn");
      } else if (wifi.connected) {
        setWifiStatus(`연결됨 · ${wifi.ssid || "Wi-Fi"} · ${wifi.ipv4 || "IP 할당 중"}`, "good");
      } else {
        setWifiStatus(`${(result.networks || []).length}개 네트워크 검색됨 · 연결할 Wi-Fi를 선택하세요.`);
      }
    };
    const scanWifi = async () => {
      const button = document.getElementById("wifi-rescan");
      button.disabled = true;
      setWifiStatus("Wi-Fi를 켜고 주변 네트워크를 검색하는 중입니다.");
      try {
        const response = await fetch("/api/network/wifi/scan", { cache: "no-store" });
        const result = await response.json();
        if (!response.ok) throw new Error(result.error || `검색 실패: ${response.status}`);
        renderWifiNetworks(result);
      } catch (error) {
        setWifiStatus(error.message, "warn");
      } finally {
        button.disabled = false;
      }
    };
    document.getElementById("wifi-settings-open").addEventListener("click", () => {
      wifiSettingsModal.hidden = false;
      scanWifi();
    });
    document.getElementById("wifi-settings-close").addEventListener("click", closeWifiSettings);
    wifiSettingsModal.addEventListener("click", (event) => {
      if (event.target === wifiSettingsModal) closeWifiSettings();
    });
    document.getElementById("wifi-rescan").addEventListener("click", scanWifi);
    document.getElementById("wifi-connect").addEventListener("click", async () => {
      const button = document.getElementById("wifi-connect");
      const payload = {
        ssid: document.getElementById("wifi-ssid").value.trim(),
        password: document.getElementById("wifi-password").value,
        security: document.getElementById("wifi-connect-panel").dataset.security || "WPA2",
      };
      if (!payload.ssid) {
        window.alert("연결할 Wi-Fi 이름을 선택하거나 입력하세요.");
        return;
      }
      button.disabled = true;
      setWifiStatus(`${payload.ssid}에 연결하는 중입니다.`);
      try {
        const response = await fetch("/api/network/wifi/connect", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify(payload),
          cache: "no-store",
        });
        const result = await response.json();
        if (!response.ok) throw new Error(result.error || `연결 실패: ${response.status}`);
        document.getElementById("wifi-password").value = "";
        setWifiStatus(`연결됨 · ${result.wifi.ssid} · ${result.wifi.ipv4 || "IP 할당 중"}`, "good");
        clearWifiSelection();
        await scanWifi();
      } catch (error) {
        setWifiStatus(error.message, "warn");
      } finally {
        button.disabled = false;
      }
    });
    document.getElementById("wifi-disconnect").addEventListener("click", async () => {
      const response = await fetch("/api/network/wifi/disconnect", { method: "POST", cache: "no-store" });
      const result = await response.json();
      if (!response.ok) {
        setWifiStatus(result.error || "연결 해제 실패", "warn");
        return;
      }
      setWifiStatus("Wi-Fi 연결이 해제되었습니다.");
      clearWifiSelection();
      scanWifi();
    });
    document.getElementById("wifi-selection-cancel").addEventListener("click", clearWifiSelection);
    document.getElementById("wifi-hidden-network").addEventListener("click", () => {
      clearWifiSelection();
      selectWifiNetwork({ security: "WPA2", signal: 100 }, true);
    });

    const autonomyModal = document.getElementById("autonomy-modal");
    const setAutonomyStatus = (message, kind = "") => {
      const status = document.getElementById("autonomy-status");
      status.className = `settings-status ${kind}`;
      status.textContent = message;
    };
    let recordingSessions = [];
    const renderRecordingSessions = (sessions) => {
      recordingSessions = Array.isArray(sessions) ? sessions : [];
      const list = document.getElementById("autonomy-session-list");
      list.replaceChildren(...recordingSessions.map((item) => {
        const option = document.createElement("option");
        option.value = item.session;
        option.label = item.label || `${(item.size_bytes / 1024 / 1024).toFixed(1)} MB`;
        return option;
      }));
      const selected = recordingSessions.find((item) => item.session === document.getElementById("autonomy-session").value.trim());
      const labelInput = document.getElementById("autonomy-session-label");
      if (document.activeElement !== labelInput) labelInput.value = selected?.label || "";
    };
    const renderRouteError = (route) => {
      const canvas = document.getElementById("route-error-chart");
      const context = canvas.getContext("2d");
      const scale = window.devicePixelRatio || 1;
      const width = Math.max(320, canvas.clientWidth);
      const height = 120;
      canvas.width = Math.round(width * scale);
      canvas.height = Math.round(height * scale);
      context.setTransform(scale, 0, 0, scale, 0, 0);
      context.clearRect(0, 0, width, height);
      context.strokeStyle = "#3d1a1a";
      context.lineWidth = 1;
      for (let row = 1; row < 4; row += 1) {
        const y = row * height / 4;
        context.beginPath(); context.moveTo(0, y); context.lineTo(width, y); context.stroke();
      }
      const history = Array.isArray(route.error_history) ? route.error_history : [];
      const summary = route.error_summary || {};
      document.getElementById("route-error-summary").textContent = history.length
        ? `현재 ${(summary.latest_m ?? 0).toFixed(3)} m · 평균 ${(summary.mean_m ?? 0).toFixed(3)} m · 최대 ${(summary.maximum_m ?? 0).toFixed(3)} m`
        : "횡오차 기록 없음";
      if (history.length < 2) return;
      const maximum = Math.max(0.3, ...history.map((item) => Number(item.cross_track_error_m) || 0));
      context.strokeStyle = "#f0c866";
      context.lineWidth = 2;
      context.beginPath();
      history.forEach((item, index) => {
        const x = index * width / (history.length - 1);
        const y = height - Math.min(maximum, Number(item.cross_track_error_m) || 0) / maximum * (height - 8) - 4;
        if (index === 0) context.moveTo(x, y); else context.lineTo(x, y);
      });
      context.stroke();
    };
    const refreshAutonomy = async () => {
      try {
        const [safetyResponse, recordingResponse, routeResponse, sessionsResponse, calibrationResponse, steeringResponse] = await Promise.all([
          fetch("/api/safety", { cache: "no-store" }),
          fetch("/api/recording", { cache: "no-store" }),
          fetch("/api/auto-route", { cache: "no-store" }),
          fetch("/api/recordings", { cache: "no-store" }),
          fetch("/api/throttle/calibration", { cache: "no-store" }),
          fetch("/api/steering", { cache: "no-store" }),
        ]);
        const safety = await safetyResponse.json();
        const recording = await recordingResponse.json();
        const route = await routeResponse.json();
        const sessions = await sessionsResponse.json();
        const calibration = await calibrationResponse.json();
        const steering = await steeringResponse.json();
        document.getElementById("autonomy-mode").textContent = safety.state_machine.mode;
        document.getElementById("autonomy-stop-reason").textContent = safety.safety.stop_reason || "정상";
        document.getElementById("autonomy-recording").textContent = recording.active
          ? `기록 중 · ${recording.frame_count} 프레임`
          : recording.session_path ? `완료 · ${recording.frame_count} 프레임` : "대기";
        document.getElementById("autonomy-route-state").textContent = route.active
          ? `주행 중 · ${route.last_command?.target_index ?? 0}`
          : route.route_path ? "경로 로드됨" : "미로드";
        document.getElementById("autonomy-software-estop").textContent =
          safety.state_machine.mode === "EMERGENCY_STOP"
            ? "정지 래치됨"
            : "B/○ 버튼 대기";
        document.getElementById("autonomy-hybrid-fallback").textContent = route.hybrid_enabled
          ? "통합 주행"
          : route.hybrid_fallback_reason || "RTK 단독";
        if (recording.session_path && !document.getElementById("autonomy-session").value) {
          document.getElementById("autonomy-session").value = recording.session_path.split(/[\\/]/).pop();
        }
        renderRecordingSessions(sessions.sessions);
        renderRouteError(route);
        const calibrationInput = document.getElementById("throttle-calibration-points");
        if (document.activeElement !== calibrationInput) {
          calibrationInput.value = (calibration.points || [])
            .map((point) => `${Number(point.speed_mps).toFixed(3)}, ${Number(point.throttle).toFixed(3)}`)
            .join("\\n");
        }
        if (safety.state_machine.mode === "EMERGENCY_STOP") {
          setAutonomyStatus("긴급정지가 래치되었습니다. 안전 확인 후 대시보드에서 해제하세요.", "warn");
        } else if (route.preflight && !route.preflight.ready) {
          setAutonomyStatus(`시작 차단 · ${(route.preflight.errors || []).join(", ")}`, "warn");
        } else if (route.route_path) {
          setAutonomyStatus("자동주행 시작 조건이 준비됐습니다.", "good");
        } else {
          setAutonomyStatus("수동 안전보조 모드에서 경로를 기록하세요.");
        }
      } catch (error) {
        setAutonomyStatus(`상태 확인 실패 · ${error.message}`, "warn");
      }
    };
    const autonomyPost = async (path, payload = null) => {
      const options = { method: "POST", cache: "no-store", headers: {} };
      if (payload !== null) {
        options.headers["Content-Type"] = "application/json";
        options.body = JSON.stringify(payload);
      }
      const response = await fetch(path, options);
      const result = await response.json();
      if (!response.ok) {
        const details = result.preflight?.errors?.join(", ");
        throw new Error(result.error || details || `요청 실패: ${response.status}`);
      }
      await refreshAutonomy();
      return result;
    };
    document.getElementById("autonomy-open").addEventListener("click", () => {
      autonomyModal.hidden = false;
      refreshAutonomy();
    });
    document.getElementById("autonomy-close").addEventListener("click", () => { autonomyModal.hidden = true; });
    autonomyModal.addEventListener("click", (event) => {
      if (event.target === autonomyModal) autonomyModal.hidden = true;
    });
    document.getElementById("record-start").addEventListener("click", async () => {
      try { await autonomyPost("/api/recording/start"); } catch (error) { setAutonomyStatus(error.message, "warn"); }
    });
    document.getElementById("record-stop").addEventListener("click", async () => {
      try { await autonomyPost("/api/recording/stop"); } catch (error) { setAutonomyStatus(error.message, "warn"); }
    });
    const selectedSession = () => document.getElementById("autonomy-session").value.trim();
    const replayNumber = (value, digits = 2) => {
      const parsed = Number(value);
      return Number.isFinite(parsed) ? parsed.toFixed(digits) : "--";
    };
    const resetRecordingReplay = () => {
      const slider = document.getElementById("record-replay-position");
      slider.max = "0";
      slider.value = "0";
      document.getElementById("record-replay-time").textContent = "0.0 / 0.0초";
      for (const id of [
        "record-replay-mode",
        "record-replay-gnss",
        "record-replay-imu",
        "record-replay-steering",
        "record-replay-throttle",
        "record-replay-lidar",
      ]) document.getElementById(id).textContent = "--";
    };
    const renderRecordingReplay = (result) => {
      const state = result.state || {};
      const vehicle = state.vehicle_state || {};
      const gnss = state.gnss || {};
      const imu = state.imu || {};
      const steering = state.steering || {};
      const control = state.control || {};
      const lidar = state.lidar_summary || {};
      const slider = document.getElementById("record-replay-position");
      slider.max = String(result.duration_seconds || 0);
      slider.value = String(result.offset_seconds || 0);
      document.getElementById("record-replay-time").textContent =
        `${replayNumber(result.offset_seconds, 1)} / ${replayNumber(result.duration_seconds, 1)}초`;
      document.getElementById("record-replay-mode").textContent =
        vehicle.mode || "--";
      document.getElementById("record-replay-gnss").textContent =
        `${gnss.rtk_status || "--"} · ${replayNumber(gnss.speed_mps, 2)}m/s`;
      document.getElementById("record-replay-imu").textContent =
        `${replayNumber(imu.yaw_degrees, 1)}°`;
      document.getElementById("record-replay-steering").textContent =
        `${replayNumber(steering.angle_degrees, 1)}° → ${replayNumber(steering.target_angle_degrees, 1)}°`;
      document.getElementById("record-replay-throttle").textContent =
        `${replayNumber(control.requested_throttle, 2)} → ${replayNumber(control.final_throttle, 2)} · ${control.stop_reason || "CLEAR"}`;
      document.getElementById("record-replay-lidar").textContent =
        `${replayNumber(lidar.front_min_distance_m, 2)}m`;
    };
    const loadRecordingReplay = async () => {
      const session = selectedSession();
      if (!session) throw new Error("재생할 기록 세션을 선택하세요.");
      const result = await autonomyPost("/api/recordings/replay", {
        session,
        offset_seconds: Number(document.getElementById("record-replay-position").value) || 0,
      });
      renderRecordingReplay(result);
      setAutonomyStatus("선택한 시점의 계기판 상태를 복원했습니다.", "good");
    };
    document.getElementById("autonomy-session").addEventListener("change", () => {
      const selected = recordingSessions.find((item) => item.session === selectedSession());
      document.getElementById("autonomy-session-label").value = selected?.label || "";
      resetRecordingReplay();
    });
    document.getElementById("record-replay-position").addEventListener("input", (event) => {
      const duration = Number(event.target.max) || 0;
      document.getElementById("record-replay-time").textContent =
        `${replayNumber(event.target.value, 1)} / ${replayNumber(duration, 1)}초`;
    });
    document.getElementById("record-replay-position").addEventListener("change", async () => {
      try { await loadRecordingReplay(); } catch (error) { setAutonomyStatus(error.message, "warn"); }
    });
    document.getElementById("record-replay-load").addEventListener("click", async () => {
      try { await loadRecordingReplay(); } catch (error) { setAutonomyStatus(error.message, "warn"); }
    });
    document.getElementById("record-list-refresh").addEventListener("click", refreshAutonomy);
    document.getElementById("record-label-save").addEventListener("click", async () => {
      try {
        await autonomyPost("/api/recordings/label", {
          session: selectedSession(),
          label: document.getElementById("autonomy-session-label").value,
        });
        setAutonomyStatus("세션 설명을 저장했습니다.", "good");
      } catch (error) { setAutonomyStatus(error.message, "warn"); }
    });
    document.getElementById("throttle-calibration-save").addEventListener("click", async () => {
      try {
        const points = document.getElementById("throttle-calibration-points").value
          .split(/\\r?\\n/)
          .map((line) => line.trim())
          .filter(Boolean)
          .map((line) => {
            const values = line.split(/[\\s,]+/).map(Number);
            if (values.length !== 2 || values.some((value) => !Number.isFinite(value))) {
              throw new Error(`보정값 형식 오류: ${line}`);
            }
            return { speed_mps: values[0], throttle: values[1] };
          });
        await autonomyPost("/api/throttle/calibration", { points });
        setAutonomyStatus("속도–스로틀 보정표를 저장했습니다.", "good");
      } catch (error) { setAutonomyStatus(error.message, "warn"); }
    });
    document.getElementById("record-map-show").addEventListener("click", async () => {
      try {
        const result = await autonomyPost("/api/recordings/route", { session: selectedSession() });
        initMap();
        recordedRouteLine.setLatLngs(result.points);
        if (result.points.length > 1) mapInstance.fitBounds(recordedRouteLine.getBounds(), { padding: [24, 24] });
        setAutonomyStatus(`지도에 ${result.points.length}개 경로점을 표시했습니다.`, "good");
      } catch (error) { setAutonomyStatus(error.message, "warn"); }
    });
    document.getElementById("record-delete").addEventListener("click", async () => {
      const session = selectedSession();
      if (!session || !window.confirm(`${session} 세션을 삭제할까요? 이 작업은 되돌릴 수 없습니다.`)) return;
      try {
        await autonomyPost("/api/recordings/delete", { session });
        document.getElementById("autonomy-session").value = "";
        document.getElementById("autonomy-session-label").value = "";
        await refreshAutonomy();
        setAutonomyStatus("선택한 세션을 삭제했습니다.", "good");
      } catch (error) { setAutonomyStatus(error.message, "warn"); }
    });
    document.getElementById("route-process").addEventListener("click", async () => {
      try { await autonomyPost("/api/routes/process", { session: selectedSession() }); } catch (error) { setAutonomyStatus(error.message, "warn"); }
    });
    document.getElementById("route-load").addEventListener("click", async () => {
      try { await autonomyPost("/api/auto-route/load", { session: selectedSession() }); } catch (error) { setAutonomyStatus(error.message, "warn"); }
    });
    document.getElementById("auto-route-start").addEventListener("click", async () => {
      try {
        driveArmed = false;
        driveThrottle = 0;
        lastSentThrottle = null;
        lastDriveSentAt = 0;
        lastSentGamepadSteering = 0;
        gamepadSteeringNeedsNeutral = false;
        setDriveUi(0);
        sendSteeringStop();
        await sendDriveCommand(0, false);
        await autonomyPost("/api/auto-route/start");
      } catch (error) { setAutonomyStatus(error.message, "warn"); }
    });
    document.getElementById("auto-route-stop").addEventListener("click", async () => {
      try { await autonomyPost("/api/auto-route/stop"); } catch (error) { setAutonomyStatus(error.message, "warn"); }
    });
    document.getElementById("auto-hybrid-start").addEventListener("click", async () => {
      try { await autonomyPost("/api/auto-hybrid/start"); } catch (error) { setAutonomyStatus(error.message, "warn"); }
    });
    document.getElementById("auto-hybrid-stop").addEventListener("click", async () => {
      try { await autonomyPost("/api/auto-hybrid/stop"); } catch (error) { setAutonomyStatus(error.message, "warn"); }
    });
    document.getElementById("emergency-stop").addEventListener("click", () => {
      triggerSoftwareEmergencyStop("dashboard");
    });
    document.getElementById("safety-reset").addEventListener("click", async () => {
      try { await autonomyPost("/api/safety/reset"); } catch (error) { setAutonomyStatus(error.message, "warn"); }
    });

    const steeringConfigModal = document.getElementById("steering-config-modal");
    const closeSteeringConfig = () => {
      steeringConfigModal.hidden = true;
      steeringConfigEditing = false;
    };
    document.getElementById("steering-config-open").addEventListener("click", () => {
      steeringConfigEditing = false;
      steeringConfigModal.hidden = false;
      document.getElementById("steering-config-right").focus();
    });
    document.getElementById("steering-config-close").addEventListener("click", closeSteeringConfig);
    steeringConfigModal.addEventListener("click", (event) => {
      if (event.target === steeringConfigModal) closeSteeringConfig();
    });
    const linksPanel = document.querySelector(".links-panel");
    const deviceToggle = document.getElementById("device-toggle");
    deviceToggle.addEventListener("click", () => {
      const expanded = linksPanel.classList.toggle("expanded");
      deviceToggle.setAttribute("aria-expanded", String(expanded));
      deviceToggle.setAttribute("aria-label", `시스템 및 설정 ${expanded ? "닫기" : "열기"}`);
    });
    for (const input of document.querySelectorAll(".steering-config input")) {
      input.addEventListener("input", () => {
        steeringConfigEditing = true;
      });
    }
    document.getElementById("steering-config-save").addEventListener("click", async () => {
      clearSteeringHold();
      gamepadSteeringNeedsNeutral = true;
      const button = document.getElementById("steering-config-save");
      const payload = {
        right_raw: Number(document.getElementById("steering-config-right").value),
        center_raw: Number(document.getElementById("steering-config-center").value),
        left_raw: Number(document.getElementById("steering-config-left").value),
        allowance_raw: Number(document.getElementById("steering-config-allowance").value),
      };
      const rawValues = [payload.right_raw, payload.center_raw, payload.left_raw];
      if (
        rawValues.some((value) => !Number.isInteger(value) || value < 0 || value > 4095)
        || !Number.isInteger(payload.allowance_raw)
        || payload.allowance_raw < 0
        || payload.allowance_raw > 300
      ) {
        window.alert("원시값은 0~4095, 안전 여유는 0~300의 정수로 입력하세요.");
        return;
      }
      button.disabled = true;
      try {
        const response = await fetch("/api/steering/config", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify(payload),
          cache: "no-store",
        });
        const result = await response.json();
        if (!response.ok) {
          throw new Error(result.error || result.config_error || `설정 저장 실패: ${response.status}`);
        }
        steeringConfigEditing = false;
        renderSteering(result);
        closeSteeringConfig();
      } catch (error) {
        window.alert(error.message);
        console.error(error);
      } finally {
        refreshSteering();
      }
    });
    window.addEventListener("keydown", (event) => {
      if (event.key === "Escape") {
        closeSteeringConfig();
        closeNtripSettings();
        closeWifiSettings();
        autonomyModal.hidden = true;
        linksPanel.classList.remove("expanded");
        deviceToggle.setAttribute("aria-expanded", "false");
      }
      if (["KeyW", "KeyS", "ArrowUp", "ArrowDown"].includes(event.code)) {
        pressedKeys.add(event.code);
        event.preventDefault();
      }
    });
    window.addEventListener("keyup", (event) => {
      pressedKeys.delete(event.code);
    });
    window.addEventListener("pointerup", stopSteeringHold);
    window.addEventListener("blur", stopSteeringHold);
    window.addEventListener("gamepadconnected", (event) => {
      activeGamepadId = event.gamepad.id;
      gamepadSteeringNeedsNeutral = true;
      document.getElementById("drive-help").textContent =
        `게임패드 준비됨: ${event.gamepad.id}. 왼쪽 Y축 전후진 · 오른쪽 X축 조향.`;
    });
    window.addEventListener("gamepaddisconnected", () => {
      driveArmed = false;
      activeGamepadId = null;
      lastSentThrottle = null;
      lastSentGamepadSteering = 0;
      gamepadSteeringNeedsNeutral = false;
      lastGamepadEmergencyPressed = false;
      sendDriveCommand(0, false);
      sendSteeringStop();
      document.getElementById("drive-help").textContent =
        "게임패드 연결이 끊겨 모든 모터를 정지했습니다.";
    });
    window.addEventListener("beforeunload", () => {
      detectionStopped = true;
      laneDetectionStopped = true;
      navigator.sendBeacon?.("/api/motor", JSON.stringify({ throttle: 0, enabled: false }));
      navigator.sendBeacon?.("/api/steering/stop");
    });
    setInterval(refreshDrive, 50);
    setInterval(refreshLidar, 250);
    setInterval(refreshImu, 250);
    setInterval(refreshSteering, 250);
    setInterval(refresh, 1000);
    setInterval(refreshAutonomy, 1000);
    setDriveUi(0);
    updateGamepadHelp();
    loadExternalScript("https://unpkg.com/leaflet@1.9.4/dist/leaflet.js")
      .then(initMap)
      .catch(() => {});
    initLaneDetection();
    initObjectDetection();
    refreshImu();
    refreshLidar();
    refreshSteering();
    refreshAutonomy();
    refresh();
  </script>
</body>
</html>
""".encode("utf-8")


class GpsMonitor:
    def __init__(self):
        self.lock = threading.Lock()
        self.state = {
            "gpsd_connected": False,
            "mode": 0,
            "fix": "NO FIX",
            "latitude": None,
            "longitude": None,
            "altitude_m": None,
            "speed_mps": None,
            "track_degrees": None,
            "satellites_visible": 0,
            "satellites_used": 0,
            "hdop": None,
            "pdop": None,
            "status": 0,
            "last_update": None,
            "received_at": None,
            "error": None,
        }

    def start(self):
        threading.Thread(target=self._read_loop, daemon=True).start()

    def snapshot(self):
        with self.lock:
            return dict(self.state)

    def _update(self, **values):
        with self.lock:
            self.state.update(values)

    def _read_loop(self):
        watch_command = b'?WATCH={"enable":true,"json":true};\n'
        while True:
            try:
                with socket.create_connection(("127.0.0.1", 2947), timeout=5) as gpsd_socket:
                    gpsd_socket.settimeout(15)
                    gpsd_socket.sendall(watch_command)
                    self._update(gpsd_connected=True, error=None)
                    with gpsd_socket.makefile("r", encoding="utf-8") as gpsd_stream:
                        for line in gpsd_stream:
                            report = json.loads(line)
                            report_class = report.get("class")
                            if report_class == "TPV":
                                mode = int(report.get("mode", 0))
                                status = int(report.get("status", 0))
                                fix = (
                                    "RTK FIXED" if status == 3
                                    else "RTK FLOAT" if status == 4
                                    else "DGPS FIX" if status == 2
                                    else "3D FIX" if mode >= 3
                                    else "2D FIX" if mode == 2
                                    else "NO FIX"
                                )
                                self._update(
                                    mode=mode,
                                    fix=fix,
                                    latitude=report.get("lat"),
                                    longitude=report.get("lon"),
                                    altitude_m=report.get("altMSL", report.get("alt")),
                                    speed_mps=report.get("speed"),
                                    track_degrees=report.get("track"),
                                    status=status,
                                    last_update=report.get("time"),
                                    received_at=time.time(),
                                )
                            elif report_class == "SKY":
                                update = {
                                    "hdop": report.get("hdop"),
                                    "pdop": report.get("pdop"),
                                }
                                visible = int(report.get("nSat", 0) or 0)
                                used = int(report.get("uSat", 0) or 0)
                                if visible > 0:
                                    update["satellites_visible"] = visible
                                if used > 0:
                                    update["satellites_used"] = used
                                self._update(**update)
                            elif report_class in {"RTCM2", "RTCM3"}:
                                ntrip_client.note_correction()
            except (OSError, ValueError, json.JSONDecodeError) as error:
                self._update(gpsd_connected=False, error=str(error))
                time.sleep(2)


class NtripClient:
    def __init__(self):
        self.lock = threading.RLock()
        self.config = self._load_config()
        self.thread = None
        self.stop_event = None
        self.active_socket = None
        self.state = {
            "connected": False,
            "status": "DISABLED" if not self.config.get("enabled") else "WAITING",
            "bytes_received": 0,
            "last_correction": None,
            "last_gga": None,
            "correction_messages": 0,
            "error": None,
        }

    @staticmethod
    def _defaults():
        return {
            "host": "",
            "port": 2101,
            "mountpoint": "",
            "username": "",
            "password": "",
            "tls": False,
            "enabled": False,
        }

    def _load_config(self):
        config = self._defaults()
        try:
            with open(NTRIP_CONFIG_PATH, "r", encoding="utf-8") as config_file:
                stored = json.load(config_file)
            if isinstance(stored, dict):
                config.update(stored)
        except (OSError, ValueError, json.JSONDecodeError):
            pass
        return config

    def _save_config_locked(self):
        directory = os.path.dirname(NTRIP_CONFIG_PATH)
        if directory:
            os.makedirs(directory, exist_ok=True)
        temporary_path = f"{NTRIP_CONFIG_PATH}.tmp"
        with open(temporary_path, "w", encoding="utf-8") as config_file:
            json.dump(self.config, config_file, indent=2)
        os.chmod(temporary_path, 0o600)
        os.replace(temporary_path, NTRIP_CONFIG_PATH)

    def snapshot(self):
        with self.lock:
            public_config = {
                "host": self.config.get("host", ""),
                "port": self.config.get("port", 2101),
                "mountpoint": self.config.get("mountpoint", ""),
                "username": self.config.get("username", ""),
                "tls": bool(self.config.get("tls")),
                "enabled": bool(self.config.get("enabled")),
                "password_saved": bool(self.config.get("password")),
                "configured": bool(
                    self.config.get("host") and self.config.get("mountpoint")
                ),
            }
            return {**public_config, **self.state}

    def configure(self, payload):
        host = str(payload.get("host", "")).strip()
        mountpoint = str(payload.get("mountpoint", "")).strip().lstrip("/")
        username = str(payload.get("username", "")).strip()
        try:
            port = int(payload.get("port", 2101))
        except (TypeError, ValueError) as error:
            raise ValueError("NTRIP port must be a number") from error
        if not host or len(host) > 255:
            raise ValueError("NTRIP host is required")
        if not 1 <= port <= 65535:
            raise ValueError("NTRIP port must be between 1 and 65535")
        if not mountpoint or len(mountpoint) > 255:
            raise ValueError("NTRIP mountpoint is required")
        if len(username) > 255:
            raise ValueError("NTRIP username is too long")
        if payload.get("tls"):
            raise ValueError("TLS NTRIP is not supported by the installed gpsd")

        with self.lock:
            password = payload.get("password")
            if password in {None, ""}:
                password = self.config.get("password", "")
            password = str(password)
            if len(password) > 512:
                raise ValueError("NTRIP password is too long")
            self.config.update(
                host=host,
                port=port,
                mountpoint=mountpoint,
                username=username,
                password=password,
                tls=bool(payload.get("tls", False)),
                enabled=bool(payload.get("enabled", True)),
            )
            self._save_config_locked()
        self.restart()
        return self.snapshot()

    def start(self):
        with self.lock:
            if not self.config.get("enabled"):
                self.state.update(connected=False, status="DISABLED", error=None)
                return
            if self.thread is not None and self.thread.is_alive():
                return
            stop_event = threading.Event()
            self.stop_event = stop_event
            self.thread = threading.Thread(
                target=self._run,
                args=(stop_event,),
                daemon=True,
            )
            self.thread.start()

    def _stop_worker(self):
        with self.lock:
            stop_event = self.stop_event
            active_socket = self.active_socket
            worker = self.thread
        if stop_event is not None:
            stop_event.set()
        if active_socket is not None:
            try:
                active_socket.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                active_socket.close()
            except OSError:
                pass
        if worker is not None and worker.is_alive():
            worker.join(timeout=2)
        with self.lock:
            self.thread = None
            self.stop_event = None
            self.active_socket = None

    def restart(self):
        self._stop_worker()
        self.start()

    def stop(self):
        with self.lock:
            self.config["enabled"] = False
            self._save_config_locked()
        self._stop_worker()
        with self.lock:
            self.state.update(connected=False, status="STOPPED", error=None)
        return self.snapshot()

    @staticmethod
    def _coordinate(value, latitude):
        degrees_width = 2 if latitude else 3
        absolute = abs(float(value))
        degrees = int(absolute)
        minutes = (absolute - degrees) * 60
        return f"{degrees:0{degrees_width}d}{minutes:09.6f}"

    def _gga_sentence(self):
        gps = gps_monitor.snapshot()
        latitude = gps.get("latitude")
        longitude = gps.get("longitude")
        if gps.get("mode", 0) < 2 or latitude is None or longitude is None:
            return None
        current_time = time.gmtime()
        time_field = time.strftime("%H%M%S", current_time) + ".00"
        latitude_direction = "N" if latitude >= 0 else "S"
        longitude_direction = "E" if longitude >= 0 else "W"
        satellites = max(0, min(99, int(gps.get("satellites_used", 0) or 0)))
        hdop = float(gps.get("hdop") or 1.0)
        altitude = float(gps.get("altitude_m") or 0.0)
        body = (
            f"GPGGA,{time_field},{self._coordinate(latitude, True)},{latitude_direction},"
            f"{self._coordinate(longitude, False)},{longitude_direction},1,{satellites:02d},"
            f"{hdop:.1f},{altitude:.3f},M,0.0,M,,"
        )
        checksum = 0
        for character in body:
            checksum ^= ord(character)
        return f"${body}*{checksum:02X}\r\n".encode("ascii")

    @staticmethod
    def _write_all(file_descriptor, data):
        view = memoryview(data)
        while view:
            written = os.write(file_descriptor, view)
            view = view[written:]

    def _connect(self, config, stop_event, output_file_descriptor=None):
        connection = socket.create_connection(
            (config["host"], config["port"]),
            timeout=10,
        )
        if config.get("tls"):
            context = ssl.create_default_context()
            connection = context.wrap_socket(
                connection,
                server_hostname=config["host"],
            )
        connection.settimeout(5)
        credentials = base64.b64encode(
            f"{config.get('username', '')}:{config.get('password', '')}".encode("utf-8")
        ).decode("ascii")
        request_lines = [
            f"GET /{config['mountpoint']} HTTP/1.1",
            f"Host: {config['host']}:{config['port']}",
            "Ntrip-Version: Ntrip/2.0",
            "User-Agent: NTRIP GNSS-Dashboard/1.0",
            "Accept: */*",
            "Connection: close",
        ]
        if config.get("username") or config.get("password"):
            request_lines.append(f"Authorization: Basic {credentials}")
        connection.sendall(("\r\n".join(request_lines) + "\r\n\r\n").encode("ascii"))

        response = b""
        payload = b""
        chunked_transfer = False
        while not stop_event.is_set() and len(response) < 16384:
            response_chunk = connection.recv(1024)
            if not response_chunk:
                raise ConnectionError("NTRIP caster closed during handshake")
            response += response_chunk
            if response.startswith(b"ICY 200 OK\r\n"):
                _, _, payload = response.partition(b"\r\n")
                break
            if b"\r\n\r\n" in response:
                header, _, payload = response.partition(b"\r\n\r\n")
                status_line = header.split(b"\r\n", 1)[0]
                if b" 200 " not in status_line:
                    raise ConnectionError(status_line.decode("ascii", errors="replace"))
                chunked_transfer = b"transfer-encoding: chunked" in header.lower()
                break
        else:
            raise ConnectionError("Invalid NTRIP caster response")

        owns_output = output_file_descriptor is None
        file_descriptor = output_file_descriptor
        if file_descriptor is None:
            file_descriptor = os.open(GPS_DEVICE, os.O_WRONLY | os.O_NOCTTY)
        chunk_buffer = bytearray()
        chunk_size = None

        def decode_corrections(data):
            nonlocal chunk_size
            if not chunked_transfer:
                return [data] if data else []
            chunk_buffer.extend(data)
            corrections = []
            while True:
                if chunk_size is None:
                    line_end = chunk_buffer.find(b"\r\n")
                    if line_end < 0:
                        break
                    size_line = bytes(chunk_buffer[:line_end]).split(b";", 1)[0]
                    del chunk_buffer[: line_end + 2]
                    try:
                        chunk_size = int(size_line, 16)
                    except ValueError as error:
                        raise ConnectionError("Invalid NTRIP chunk size") from error
                    if chunk_size == 0:
                        raise ConnectionError("NTRIP caster ended the correction stream")
                if len(chunk_buffer) < chunk_size + 2:
                    break
                corrections.append(bytes(chunk_buffer[:chunk_size]))
                del chunk_buffer[: chunk_size + 2]
                chunk_size = None
            return corrections

        try:
            with self.lock:
                self.active_socket = connection
                self.state.update(connected=True, status="CONNECTED", error=None)
            for correction in decode_corrections(payload):
                self._write_all(file_descriptor, correction)
                with self.lock:
                    self.state["bytes_received"] += len(correction)
                    self.state["last_correction"] = time.time()
            last_gga_time = 0.0
            while not stop_event.is_set():
                if time.monotonic() - last_gga_time >= 10:
                    gga = self._gga_sentence()
                    if gga:
                        connection.sendall(gga)
                        last_gga_time = time.monotonic()
                        with self.lock:
                            self.state["last_gga"] = time.time()
                try:
                    correction = connection.recv(4096)
                except socket.timeout:
                    continue
                if not correction:
                    raise ConnectionError("NTRIP caster closed the connection")
                for decoded_correction in decode_corrections(correction):
                    self._write_all(file_descriptor, decoded_correction)
                    with self.lock:
                        self.state["bytes_received"] += len(decoded_correction)
                        self.state["last_correction"] = time.time()
        finally:
            if owns_output:
                os.close(file_descriptor)
            connection.close()

    def _run(self, stop_event):
        while not stop_event.is_set():
            with self.lock:
                config = dict(self.config)
                self.state.update(connected=False, status="CONNECTING", error=None)
            try:
                self._connect(config, stop_event)
            except (OSError, ValueError, ConnectionError, ssl.SSLError) as error:
                if not stop_event.is_set():
                    with self.lock:
                        self.state.update(
                            connected=False,
                            status="RETRYING",
                            error=str(error),
                        )
            finally:
                with self.lock:
                    self.active_socket = None
                    if stop_event.is_set():
                        self.state["connected"] = False
            stop_event.wait(5)

    def _ntrip_uri(self):
        with self.lock:
            config = dict(self.config)
        username = quote(str(config.get("username", "")), safe="")
        password = quote(str(config.get("password", "")), safe="")
        authentication = f"{username}:{password}@" if username or password else ""
        mountpoint = quote(str(config.get("mountpoint", "")).lstrip("/"), safe="/")
        return (
            f"ntrip://{authentication}{config['host']}:{int(config['port'])}/"
            f"{mountpoint}"
        )

    @staticmethod
    def _gpsd_command(command):
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as control_socket:
            control_socket.settimeout(12)
            control_socket.connect(GPSD_CONTROL_SOCKET)
            control_socket.sendall((command + "\n").encode("utf-8"))
            response = control_socket.recv(4096).decode("utf-8", errors="replace").strip()
        if response and not response.upper().startswith("OK"):
            try:
                response_payload = json.loads(response)
            except json.JSONDecodeError as error:
                raise RuntimeError(response) from error
            if response_payload.get("class") != "ACK":
                raise RuntimeError("gpsd rejected the NTRIP source")
        return response or "OK"

    def _detach_gpsd_source(self):
        active_uri = getattr(self, "active_uri", None)
        if not active_uri:
            return
        try:
            self._gpsd_command(f"-{active_uri}")
        except (OSError, RuntimeError):
            pass
        self.active_uri = None

    def start(self):
        with self.lock:
            enabled = bool(self.config.get("enabled"))
            configured = bool(self.config.get("host") and self.config.get("mountpoint"))
        if not enabled:
            with self.lock:
                self.state.update(connected=False, status="DISABLED", error=None)
            return
        if not configured:
            with self.lock:
                self.state.update(
                    connected=False,
                    status="ERROR",
                    error="NTRIP configuration is incomplete",
                )
            return
        if pty is None:
            with self.lock:
                self.state.update(
                    connected=False,
                    status="ERROR",
                    error="Pseudo-terminal support is unavailable",
                )
            return
        if self.thread is not None and self.thread.is_alive():
            return
        try:
            self._gpsd_command(f"-{self._ntrip_uri()}")
        except (OSError, RuntimeError):
            pass
        stop_event = threading.Event()
        self.stop_event = stop_event
        self.thread = threading.Thread(
            target=self._relay_loop,
            args=(stop_event,),
            daemon=True,
        )
        self.thread.start()

    def _relay_loop(self, stop_event):
        while not stop_event.is_set():
            if self._gga_sentence() is None:
                with self.lock:
                    self.state.update(
                        connected=False,
                        status="WAITING FOR GNSS FIX",
                        error=None,
                    )
                stop_event.wait(2)
                continue

            master_file_descriptor = None
            relay_path = None
            registration_thread = None
            try:
                master_file_descriptor, slave_file_descriptor = pty.openpty()
                tty.setraw(slave_file_descriptor)
                relay_path = os.ttyname(slave_file_descriptor)
                if grp is not None:
                    os.chown(relay_path, -1, grp.getgrnam("dialout").gr_gid)
                os.chmod(relay_path, 0o660)
                os.close(slave_file_descriptor)
                with self.lock:
                    self.relay_master_fd = master_file_descriptor
                    self.relay_path = relay_path
                    self.state.update(
                        connected=False,
                        status="CONNECTING",
                        error=None,
                    )

                registration_error = []

                def register_relay():
                    try:
                        self._gpsd_command(f"+{relay_path}")
                    except (OSError, RuntimeError) as error:
                        registration_error.append(str(error))

                registration_thread = threading.Thread(target=register_relay, daemon=True)
                registration_thread.start()
                with self.lock:
                    config = dict(self.config)
                self._connect(config, stop_event, master_file_descriptor)
                if registration_error and not stop_event.is_set():
                    raise RuntimeError(registration_error[-1])
            except (OSError, ValueError, ConnectionError, RuntimeError, ssl.SSLError) as error:
                if not stop_event.is_set():
                    with self.lock:
                        self.state.update(
                            connected=False,
                            status="RETRYING",
                            error=str(error),
                        )
            finally:
                if relay_path:
                    try:
                        self._gpsd_command(f"-{relay_path}")
                    except (OSError, RuntimeError):
                        pass
                if master_file_descriptor is not None:
                    try:
                        os.close(master_file_descriptor)
                    except OSError:
                        pass
                if registration_thread is not None and registration_thread.is_alive():
                    registration_thread.join(timeout=1)
                with self.lock:
                    self.active_socket = None
                    self.relay_master_fd = None
                    self.relay_path = None
                    if stop_event.is_set():
                        self.state["connected"] = False
            stop_event.wait(5)

    def _stop_relay_worker(self):
        with self.lock:
            stop_event = self.stop_event
            active_socket = self.active_socket
            worker = self.thread
        if stop_event is not None:
            stop_event.set()
        if active_socket is not None:
            try:
                active_socket.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                active_socket.close()
            except OSError:
                pass
        if worker is not None and worker.is_alive():
            worker.join(timeout=3)
        with self.lock:
            self.thread = None
            self.stop_event = None
            self.active_socket = None

    def restart(self):
        self._stop_relay_worker()
        self.start()

    def stop(self):
        with self.lock:
            self.config["enabled"] = False
            self._save_config_locked()
        self._stop_relay_worker()
        with self.lock:
            self.state.update(connected=False, status="STOPPED", error=None)
        return self.snapshot()

    def note_correction(self):
        with self.lock:
            self.state["connected"] = True
            self.state["status"] = "RECEIVING RTCM"
            self.state["correction_messages"] += 1
            self.state["last_correction"] = time.time()


class ImuMonitor:
    def __init__(self):
        self.lock = threading.Lock()
        self.mag_offset = [0.0, 0.0, 0.0]
        self.mag_scale = [1.0, 1.0, 1.0]
        self.heading_samples = deque(maxlen=15)
        self.roll_samples = deque(maxlen=9)
        self.pitch_samples = deque(maxlen=9)
        self.previous_global_heading = None
        self.previous_heading_time = None
        self.relative_yaw = 0.0
        self.yaw_rate = 0.0
        self.calibration_deadline = None
        self.calibration_min = [float("inf")] * 3
        self.calibration_max = [float("-inf")] * 3
        self.state = {
            "connected": False,
            "heading_degrees": None,
            "global_heading_degrees": None,
            "raw_heading_degrees": None,
            "relative_yaw_degrees": 0.0,
            "yaw_rate_dps": 0.0,
            "turn_direction": "STRAIGHT",
            "mounting_yaw_offset_degrees": IMU_MOUNTING_YAW_OFFSET_DEGREES,
            "yaw_positive_direction": "RIGHT",
            "roll_degrees": None,
            "pitch_degrees": None,
            "acceleration_x": None,
            "acceleration_y": None,
            "acceleration_z": None,
            "calibrated": False,
            "last_update": None,
            "error": None,
            "calibration": {
                "active": False,
                "progress": 0.0,
                "message": "Rotate slowly around all 3 axes",
            },
        }
        self._load_calibration()

    def start(self):
        threading.Thread(target=self._read_loop, daemon=True).start()

    def snapshot(self):
        with self.lock:
            snapshot = dict(self.state)
            snapshot["calibration"] = dict(self.state["calibration"])
            return snapshot

    def _update(self, **values):
        with self.lock:
            self.state.update(values)

    @staticmethod
    def _smooth_angle(previous, current):
        if previous is None:
            return current
        delta = (current - previous + 180) % 360 - 180
        magnitude = abs(delta)
        if magnitude <= IMU_HEADING_DEADBAND_DEGREES:
            return previous
        if magnitude < 2:
            alpha = 0.08
        elif magnitude < 8:
            alpha = 0.22
        else:
            alpha = 0.45
        effective_delta = math.copysign(
            magnitude - IMU_HEADING_DEADBAND_DEGREES,
            delta,
        )
        return (previous + effective_delta * alpha) % 360

    @staticmethod
    def _smooth_attitude(previous, current):
        if previous is None:
            return current
        delta = current - previous
        magnitude = abs(delta)
        if magnitude <= IMU_ATTITUDE_DEADBAND_DEGREES:
            return previous
        alpha = 0.08 if magnitude < 1 else 0.24
        effective_delta = math.copysign(
            magnitude - IMU_ATTITUDE_DEADBAND_DEGREES,
            delta,
        )
        return previous + effective_delta * alpha

    def _track_turn(self, global_heading, timestamp=None):
        now = time.monotonic() if timestamp is None else timestamp
        with self.lock:
            if self.previous_global_heading is None:
                self.previous_global_heading = global_heading
                self.previous_heading_time = now
                self.yaw_rate = 0.0
                return self.relative_yaw, self.yaw_rate, "STRAIGHT"

            delta = (
                global_heading - self.previous_global_heading + 180
            ) % 360 - 180
            delta *= IMU_YAW_DIRECTION_SIGN
            elapsed = max(0.01, now - self.previous_heading_time)
            self.relative_yaw += delta
            instant_rate = max(-360.0, min(360.0, delta / elapsed))
            self.yaw_rate = self.yaw_rate * 0.6 + instant_rate * 0.4
            if abs(self.yaw_rate) < IMU_TURN_RATE_THRESHOLD_DPS:
                direction = "STRAIGHT"
            elif self.yaw_rate > 0:
                direction = "RIGHT"
            else:
                direction = "LEFT"
            self.previous_global_heading = global_heading
            self.previous_heading_time = now
            return self.relative_yaw, self.yaw_rate, direction

    def start_calibration(self, duration_seconds=20):
        with self.lock:
            if not self.state["connected"]:
                return False
            self.calibration_deadline = time.monotonic() + duration_seconds
            self.calibration_min = [float("inf")] * 3
            self.calibration_max = [float("-inf")] * 3
            self.state["calibrated"] = False
            self.state["calibration"] = {
                "active": True,
                "progress": 0.0,
                "message": "Move sensor through figure-8 and all 3 axes",
            }
            return True

    def reset_relative_yaw(self):
        with self.lock:
            if self.state.get("global_heading_degrees") is None:
                return False
            self.relative_yaw = 0.0
            self.yaw_rate = 0.0
            self.state["relative_yaw_degrees"] = 0.0
            self.state["yaw_rate_dps"] = 0.0
            self.state["turn_direction"] = "STRAIGHT"
            return True

    def _load_calibration(self):
        try:
            with open(IMU_CALIBRATION_PATH, "r", encoding="utf-8") as calibration_file:
                calibration = json.load(calibration_file)
            self.mag_offset = [float(value) for value in calibration["offset"]]
            self.mag_scale = [float(value) for value in calibration["scale"]]
            self.state["calibrated"] = True
            self.state["calibration"] = {
                "active": False,
                "progress": 1.0,
                "message": "Calibration loaded",
            }
        except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError):
            pass

    def _update_calibration(self, raw_magnetometer):
        calibration_to_save = None
        with self.lock:
            if self.calibration_deadline is None:
                return

            for axis, value in enumerate(raw_magnetometer):
                self.calibration_min[axis] = min(self.calibration_min[axis], value)
                self.calibration_max[axis] = max(self.calibration_max[axis], value)

            remaining = max(0.0, self.calibration_deadline - time.monotonic())
            progress = min(1.0, 1.0 - remaining / 20.0)
            self.state["calibration"]["progress"] = progress

            if remaining > 0:
                return

            half_ranges = [
                (maximum - minimum) / 2.0
                for minimum, maximum in zip(
                    self.calibration_min,
                    self.calibration_max,
                )
            ]
            self.calibration_deadline = None

            if min(half_ranges) < 25:
                self.state["calibrated"] = False
                self.state["calibration"] = {
                    "active": False,
                    "progress": 0.0,
                    "message": "Calibration failed: rotate through wider angles",
                }
                return

            self.mag_offset = [
                (minimum + maximum) / 2.0
                for minimum, maximum in zip(
                    self.calibration_min,
                    self.calibration_max,
                )
            ]
            average_range = sum(half_ranges) / 3.0
            self.mag_scale = [average_range / half_range for half_range in half_ranges]
            self.state["calibrated"] = True
            self.state["calibration"] = {
                "active": False,
                "progress": 1.0,
                "message": "Calibration saved",
            }
            calibration_to_save = {
                "offset": self.mag_offset,
                "scale": self.mag_scale,
                "saved_at": time.time(),
            }

        if calibration_to_save is not None:
            try:
                with open(IMU_CALIBRATION_PATH, "w", encoding="utf-8") as calibration_file:
                    json.dump(calibration_to_save, calibration_file)
            except OSError as error:
                self._update(error=f"Could not save calibration: {error}")

    def _read_loop(self):
        if qwiic_icm20948 is None:
            self._update(error="SparkFun IMU driver is not installed")
            return

        while True:
            try:
                sensor = qwiic_icm20948.QwiicIcm20948()
                if not sensor.connected or not sensor.begin():
                    raise RuntimeError("ICM-20948 did not initialize")

                self._update(connected=True, error=None)
                while True:
                    if not sensor.dataReady():
                        time.sleep(0.02)
                        continue

                    sensor.getAgmt()
                    accel_x = float(sensor.axRaw)
                    accel_y = float(sensor.ayRaw)
                    accel_z = float(sensor.azRaw)
                    acceleration_scale = 9.80665 / 16384.0
                    raw_magnetometer = [
                        float(sensor.mxRaw),
                        float(sensor.myRaw),
                        float(sensor.mzRaw),
                    ]
                    self._update_calibration(raw_magnetometer)
                    mag_x, mag_y, mag_z = [
                        (value - offset) * scale
                        for value, offset, scale in zip(
                            raw_magnetometer,
                            self.mag_offset,
                            self.mag_scale,
                        )
                    ]

                    roll_radians = math.atan2(accel_y, accel_z)
                    pitch_radians = math.atan2(
                        -accel_x,
                        math.sqrt(accel_y * accel_y + accel_z * accel_z),
                    )
                    roll_degrees = math.degrees(roll_radians)
                    pitch_degrees = math.degrees(pitch_radians)

                    horizontal_x = (
                        mag_x * math.cos(pitch_radians)
                        + mag_z * math.sin(pitch_radians)
                    )
                    horizontal_y = (
                        mag_x * math.sin(roll_radians) * math.sin(pitch_radians)
                        + mag_y * math.cos(roll_radians)
                        - mag_z * math.sin(roll_radians) * math.cos(pitch_radians)
                    )
                    heading_degrees = (
                        math.degrees(math.atan2(horizontal_y, horizontal_x)) + 360
                    ) % 360
                    raw_heading_degrees = heading_degrees
                    global_heading_degrees = (
                        raw_heading_degrees + IMU_MOUNTING_YAW_OFFSET_DEGREES
                    ) % 360

                    previous = self.snapshot()
                    if previous["heading_degrees"] is not None:
                        unfiltered_delta = (
                            global_heading_degrees
                            - previous["heading_degrees"]
                            + 180
                        ) % 360 - 180
                        if abs(unfiltered_delta) >= 8:
                            self.heading_samples.clear()

                    self.heading_samples.append(global_heading_degrees)
                    heading_sin = sum(
                        math.sin(math.radians(value))
                        for value in self.heading_samples
                    )
                    heading_cos = sum(
                        math.cos(math.radians(value))
                        for value in self.heading_samples
                    )
                    global_heading_degrees = (
                        math.degrees(math.atan2(heading_sin, heading_cos)) + 360
                    ) % 360
                    self.roll_samples.append(roll_degrees)
                    self.pitch_samples.append(pitch_degrees)
                    roll_degrees = sum(self.roll_samples) / len(self.roll_samples)
                    pitch_degrees = sum(self.pitch_samples) / len(self.pitch_samples)

                    if previous["heading_degrees"] is not None:
                        global_heading_degrees = self._smooth_angle(
                            previous["heading_degrees"],
                            global_heading_degrees,
                        )
                        roll_degrees = self._smooth_attitude(
                            previous["roll_degrees"],
                            roll_degrees,
                        )
                        pitch_degrees = self._smooth_attitude(
                            previous["pitch_degrees"],
                            pitch_degrees,
                        )

                    relative_yaw, yaw_rate, turn_direction = self._track_turn(
                        global_heading_degrees
                    )

                    self._update(
                        connected=True,
                        heading_degrees=global_heading_degrees,
                        global_heading_degrees=global_heading_degrees,
                        raw_heading_degrees=raw_heading_degrees,
                        relative_yaw_degrees=relative_yaw,
                        yaw_rate_dps=yaw_rate,
                        turn_direction=turn_direction,
                        roll_degrees=roll_degrees,
                        pitch_degrees=pitch_degrees,
                        acceleration_x=accel_x * acceleration_scale,
                        acceleration_y=accel_y * acceleration_scale,
                        acceleration_z=accel_z * acceleration_scale,
                        last_update=time.time(),
                        error=None,
                    )
                    time.sleep(0.03)
            except Exception as error:
                self._update(
                    connected=False,
                    heading_degrees=None,
                    global_heading_degrees=None,
                    roll_degrees=None,
                    pitch_degrees=None,
                    acceleration_x=None,
                    acceleration_y=None,
                    acceleration_z=None,
                    error=str(error),
                )
                time.sleep(2)


gps_monitor = GpsMonitor()
ntrip_client = NtripClient()
imu_monitor = ImuMonitor()
lidar_monitor = LidarMonitor()


def imu_status():
    state = imu_monitor.snapshot()
    connected = state.pop("connected")
    return {
        "connected": connected,
        "bus_online": os.path.exists("/dev/i2c-1"),
        "bus": "/dev/i2c-1",
        "addresses": ["0x69"] if connected else [],
        "orientation": state,
    }


def run_command(command, timeout=1.5):
    try:
        completed = subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
        return completed.returncode, completed.stdout.strip(), completed.stderr.strip()
    except (OSError, subprocess.TimeoutExpired) as error:
        return 124, "", str(error)


def interface_ipv4(interface_name):
    return_code, stdout, _ = run_command(
        ["ip", "-4", "-o", "addr", "show", "dev", interface_name]
    )
    if return_code != 0 or not stdout:
        return None

    parts = stdout.split()
    if len(parts) >= 4:
        return parts[3].split("/", 1)[0]
    return None


def wifi_status():
    state = "UNAVAILABLE"
    ssid = None
    signal = None

    return_code, stdout, _ = run_command(
        ["nmcli", "-t", "-f", "DEVICE,TYPE,STATE,CONNECTION", "device", "status"]
    )
    if return_code == 0:
        for line in stdout.splitlines():
            fields = line.split(":")
            if len(fields) >= 4 and fields[0] == "wlan0":
                state = fields[2].upper()
                ssid = fields[3] if fields[3] != "--" else None
                break

    return_code, stdout, _ = run_command(["iw", "dev", "wlan0", "link"])
    if return_code == 0:
        for line in stdout.splitlines():
            stripped = line.strip()
            if stripped.startswith("SSID:"):
                ssid = stripped.split(":", 1)[1].strip() or ssid
            elif stripped.startswith("signal:"):
                try:
                    signal = int(float(stripped.split(":", 1)[1].split()[0]))
                except ValueError:
                    signal = None

    return {
        "connected": state == "CONNECTED",
        "state": state,
        "ssid": ssid,
        "signal": signal,
        "ipv4": interface_ipv4("wlan0"),
    }


def split_nmcli_fields(line):
    fields = []
    field = []
    escaped = False
    for character in line:
        if escaped:
            field.append(character)
            escaped = False
        elif character == "\\":
            escaped = True
        elif character == ":":
            fields.append("".join(field))
            field = []
        else:
            field.append(character)
    fields.append("".join(field))
    return fields


def wifi_scan():
    radio_code, _, radio_error = run_command(
        ["nmcli", "radio", "wifi", "on"],
        timeout=5,
    )
    if radio_code != 0:
        return {
            "wifi": wifi_status(),
            "networks": [],
            "error": radio_error or "Unable to enable Wi-Fi",
        }

    for _ in range(10):
        if wifi_status()["state"] != "UNAVAILABLE":
            break
        time.sleep(0.4)
    time.sleep(1)

    return_code, stdout, stderr = run_command(
        [
            "nmcli",
            "--terse",
            "--escape",
            "yes",
            "--fields",
            "IN-USE,SSID,SIGNAL,SECURITY",
            "device",
            "wifi",
            "list",
            "ifname",
            "wlan0",
            "--rescan",
            "yes",
        ],
        timeout=15,
    )
    networks_by_ssid = {}
    if return_code == 0:
        for line in stdout.splitlines():
            fields = split_nmcli_fields(line)
            if len(fields) < 4 or not fields[1]:
                continue
            try:
                signal = int(fields[2])
            except ValueError:
                signal = 0
            network = {
                "active": fields[0] == "*",
                "ssid": fields[1],
                "signal": signal,
                "security": fields[3] or "OPEN",
            }
            existing = networks_by_ssid.get(network["ssid"])
            if existing is None or network["signal"] > existing["signal"]:
                networks_by_ssid[network["ssid"]] = network
    networks = sorted(
        networks_by_ssid.values(),
        key=lambda network: (not network["active"], -network["signal"], network["ssid"]),
    )
    return {
        "wifi": wifi_status(),
        "networks": networks,
        "error": None if return_code == 0 else stderr or "Wi-Fi scan failed",
    }


def wifi_connection_profiles(ssid):
    return_code, stdout, _ = run_command(
        [
            "nmcli",
            "--terse",
            "--escape",
            "yes",
            "--fields",
            "NAME,802-11-wireless.SSID",
            "connection",
            "show",
        ],
        timeout=8,
    )
    if return_code != 0:
        return []
    profiles = []
    for line in stdout.splitlines():
        fields = split_nmcli_fields(line)
        if len(fields) >= 2 and fields[1] == ssid:
            profiles.append(fields[0])
    return profiles


def wifi_connect(ssid, password, security=None):
    ssid = str(ssid or "").strip()
    password = str(password or "")
    security = str(security or "").upper()
    if not ssid or len(ssid) > 255:
        raise ValueError("Wi-Fi SSID is required")
    if len(password) > 512:
        raise ValueError("Wi-Fi password is too long")
    secured = bool(security and security != "OPEN") or bool(password)
    if secured and not password:
        raise ValueError("A password is required for this secured Wi-Fi network")
    if password and not (8 <= len(password) <= 63 or re.fullmatch(r"[0-9A-Fa-f]{64}", password)):
        raise ValueError("WPA Wi-Fi password must be 8-63 characters")
    if "802.1X" in security or "ENTERPRISE" in security:
        raise ValueError("Enterprise Wi-Fi is not supported by this dashboard")

    radio_code, _, radio_error = run_command(
        ["nmcli", "radio", "wifi", "on"],
        timeout=5,
    )
    if radio_code != 0:
        raise RuntimeError(radio_error or "Unable to enable Wi-Fi")

    run_command(["nmcli", "device", "disconnect", "wlan0"], timeout=8)
    for existing_profile in wifi_connection_profiles(ssid):
        run_command(["nmcli", "connection", "delete", existing_profile], timeout=8)

    profile_name = f"GNSS WiFi - {ssid}"[:120]
    add_code, _, add_error = run_command(
        [
            "nmcli",
            "connection",
            "add",
            "type",
            "wifi",
            "ifname",
            "wlan0",
            "con-name",
            profile_name,
            "ssid",
            ssid,
            "connection.autoconnect",
            "yes",
        ],
        timeout=10,
    )
    if add_code != 0:
        raise RuntimeError(add_error or "Unable to create Wi-Fi profile")

    modify_command = [
        "nmcli",
        "connection",
        "modify",
        profile_name,
        "ipv4.method",
        "auto",
        "ipv4.route-metric",
        "50",
        "ipv6.route-metric",
        "50",
    ]
    if secured:
        key_management = "sae" if "WPA3" in security and "WPA2" not in security else "wpa-psk"
        modify_command.extend(
            [
                "802-11-wireless-security.key-mgmt",
                key_management,
                "802-11-wireless-security.psk",
                password,
            ]
        )
    modify_code, _, modify_error = run_command(modify_command, timeout=10)
    if modify_code != 0:
        run_command(["nmcli", "connection", "delete", profile_name], timeout=8)
        raise RuntimeError(modify_error or "Unable to configure Wi-Fi security")

    connect_code, _, connect_error = run_command(
        [
            "nmcli",
            "--wait",
            "25",
            "connection",
            "up",
            profile_name,
            "ifname",
            "wlan0",
        ],
        timeout=30,
    )
    if connect_code != 0:
        run_command(["nmcli", "device", "disconnect", "wlan0"], timeout=8)
        run_command(["nmcli", "connection", "delete", profile_name], timeout=8)
        raise RuntimeError(connect_error or "Wi-Fi connection failed")

    with internet_status_lock:
        internet_status_cache["checked_at"] = 0.0
    return {
        "connected": True,
        "wifi": wifi_status(),
        "internet": internet_status(),
    }


def wifi_disconnect():
    return_code, _, stderr = run_command(
        ["nmcli", "device", "disconnect", "wlan0"],
        timeout=10,
    )
    if return_code != 0:
        raise RuntimeError(stderr or "Wi-Fi disconnect failed")
    with internet_status_lock:
        internet_status_cache["checked_at"] = 0.0
    return {"connected": False, "wifi": wifi_status()}


def ping_status(target_ip):
    if not target_ip:
        return {"ip": None, "ping_ok": False, "latency_ms": None}

    return_code, stdout, _ = run_command(
        ["ping", "-c", "1", "-W", "1", target_ip],
        timeout=2,
    )
    latency_ms = None
    if return_code == 0:
        match = re.search(r"time[=<]([0-9.]+)\s*ms", stdout)
        if match:
            latency_ms = float(match.group(1))

    return {
        "ip": target_ip,
        "ping_ok": return_code == 0,
        "latency_ms": latency_ms,
    }


internet_status_lock = threading.Lock()
internet_status_cache = {
    "checked_at": 0.0,
    "online": False,
    "target": "deb.debian.org:443",
    "latency_ms": None,
    "via": "ethernet",
}


def default_route_interface():
    return_code, stdout, _ = run_command(["ip", "route", "show", "default"])
    if return_code == 0:
        match = re.search(r"\bdev\s+(\S+)", stdout)
        if match:
            return match.group(1)
    return None


def internet_status():
    with internet_status_lock:
        now = time.monotonic()
        if now - internet_status_cache["checked_at"] < 5:
            return dict(internet_status_cache)

        return_code, stdout, _ = run_command(
            [
                "curl",
                "-4",
                "-sS",
                "--connect-timeout",
                "2",
                "--max-time",
                "3",
                "-o",
                "/dev/null",
                "-w",
                "%{time_connect}",
                "https://deb.debian.org/",
            ],
            timeout=4,
        )
        online = return_code == 0
        latency_ms = None
        if online:
            try:
                latency_ms = float(stdout) * 1000
            except ValueError:
                pass

        internet_status_cache.update(
            checked_at=now,
            online=online,
            latency_ms=latency_ms,
            via=default_route_interface() or "unknown",
        )
        return dict(internet_status_cache)


def network_status(client_ip):
    return {
        "wifi": wifi_status(),
        "ethernet": {
            "interface": "eth0",
            "ipv4": interface_ipv4("eth0"),
        },
        "internet": internet_status(),
        "client": ping_status(client_ip),
    }


def find_arduino_port():
    if ARDUINO_DEVICE:
        return ARDUINO_DEVICE

    for path in glob.glob("/dev/serial/by-id/*"):
        name = os.path.basename(path).lower()
        if "arduino" in name:
            return os.path.realpath(path)

    for path in glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"):
        return path

    return None


class MotorController:
    def __init__(self):
        self.lock = threading.RLock()
        self.fd = None
        self.port = None
        self.next_open_attempt = 0
        self.serial_buffer = ""
        self.enabled = False
        self.throttle = 0.0
        self.last_pwm = 0
        self.last_drive_command_time = 0
        self.drive_boost_until = 0.0
        self.drive_watchdog_stop_reason = None
        self.drive_watchdog_stopped_at = None
        self.steering_enabled = False
        self.steering_direction = 0
        self.steering_pwm = 0
        self.steering_limit = "NONE"
        self.steering_rejection = None
        self.steering_closed_loop = False
        self.steering_requested_angle_degrees = 0.0
        self.steering_target_angle_degrees = 0.0
        self.steering_control_error_degrees = None
        self.last_steering_control_time = 0
        self.centering = False
        self.center_supported = False
        self.config_supported = False
        self.last_steering_command_time = 0
        self.encoder_connected = False
        self.encoder_raw = None
        self.encoder_zero_raw = None
        self.steering_angle_degrees = None
        self.encoder_status = None
        self.encoder_error = None
        self.encoder_last_update = 0
        self.last_arduino_response_time = 0
        self.encoder_raw_min_observed = None
        self.encoder_raw_max_observed = None
        self.hardware_estop_supported = False
        self.hardware_estop_active = None
        self.steering_right_reference_raw = STEER_RIGHT_REFERENCE_RAW
        self.steering_left_reference_raw = STEER_LEFT_REFERENCE_RAW
        self.steering_limit_allowance_raw = STEER_LIMIT_ALLOWANCE_RAW
        self.steering_center_raw = STEER_CENTER_RAW
        self.steering_right_raw_limit = steering_safety_limit(
            self.steering_right_reference_raw,
            self.steering_center_raw,
            self.steering_limit_allowance_raw,
        )
        self.steering_left_raw_limit = steering_safety_limit(
            self.steering_left_reference_raw,
            self.steering_center_raw,
            self.steering_limit_allowance_raw,
        )
        self.last_serial_line = None
        self.last_error = None

    def start(self):
        threading.Thread(target=self._watchdog_loop, daemon=True).start()

    def snapshot(self):
        with self.lock:
            port = self.port or find_arduino_port()
            encoder_fresh = (
                self.encoder_connected
                and time.time() - self.encoder_last_update < 1.0
            )
            return {
                "connected": self.fd is not None,
                "device_present": port is not None,
                "port": port,
                "enabled": self.enabled,
                "throttle": self.throttle,
                "pwm": self.last_pwm,
                "drive_watchdog_stop_reason": self.drive_watchdog_stop_reason,
                "drive_watchdog_stopped_at": self.drive_watchdog_stopped_at,
                "drive_boost_remaining_seconds": max(
                    0.0,
                    self.drive_boost_until - time.time(),
                ),
                "drive_direction_sign": DRIVE_DIRECTION_SIGN,
                "steering_enabled": self.steering_enabled,
                "steering_direction": self.steering_direction,
                "steering_pwm": self.steering_pwm,
                "steer_manual_pwm": STEER_MANUAL_PWM,
                "steer_min_pwm": STEER_MIN_PWM,
                "steer_left_raw_limit": self.steering_left_raw_limit,
                "steer_right_raw_limit": self.steering_right_raw_limit,
                "steer_left_reference_raw": self.steering_left_reference_raw,
                "steer_right_reference_raw": self.steering_right_reference_raw,
                "steer_limit_allowance_raw": self.steering_limit_allowance_raw,
                "steer_center_raw": self.steering_center_raw,
                "steering_limit": self.steering_limit,
                "steering_rejection": self.steering_rejection,
                "steering_control_mode": "ANGLE" if self.steering_closed_loop else "IDLE",
                "requested_steering_angle_degrees": self.steering_requested_angle_degrees,
                "target_steering_angle_degrees": self.steering_target_angle_degrees,
                "steering_control_error_degrees": self.steering_control_error_degrees,
                "centering": self.centering,
                "center_supported": self.center_supported,
                "config_supported": self.config_supported,
                "encoder_connected": encoder_fresh,
                "encoder_raw": self.encoder_raw,
                "encoder_zero_raw": self.encoder_zero_raw,
                "steering_angle_degrees": self.steering_angle_degrees,
                "encoder_status": self.encoder_status,
                "magnet_detected": (
                    bool(self.encoder_status & 0x20)
                    if self.encoder_status is not None
                    else None
                ),
                "encoder_raw_min_observed": self.encoder_raw_min_observed,
                "encoder_raw_max_observed": self.encoder_raw_max_observed,
                "encoder_error": self.encoder_error,
                "encoder_last_update": self.encoder_last_update or None,
                "last_response_at": self.last_arduino_response_time or None,
                "response_age_seconds": (
                    time.time() - self.last_arduino_response_time
                    if self.last_arduino_response_time
                    else None
                ),
                "hardware_estop_supported": self.hardware_estop_supported,
                "hardware_estop_active": self.hardware_estop_active,
                "last_serial_line": self.last_serial_line,
                "last_error": self.last_error,
            }

    def set_drive(self, throttle, enabled):
        throttle = max(-1.0, min(1.0, float(throttle)))
        if abs(throttle) < 0.08:
            throttle = 0.0
        if not enabled:
            throttle = 0.0

        with self.lock:
            now = time.time()
            previous_direction = (
                1 if self.throttle > 0 else -1 if self.throttle < 0 else 0
            )
            requested_direction = (
                1 if throttle > 0 else -1 if throttle < 0 else 0
            )
            if (
                enabled
                and requested_direction
                and (
                    not self.enabled
                    or previous_direction == 0
                    or previous_direction != requested_direction
                )
            ):
                self.drive_boost_until = now + MOTOR_START_BOOST_SECONDS
            elif not enabled or requested_direction == 0:
                self.drive_boost_until = 0.0
            self.drive_watchdog_stop_reason = None
            self.drive_watchdog_stopped_at = None
            self.enabled = bool(enabled)
            self.throttle = throttle
            pwm = 0
            if enabled and throttle != 0.0:
                pwm = drive_pwm_magnitude(
                    throttle,
                    boost_active=now < self.drive_boost_until,
                    minimum_start_pwm=MOTOR_MIN_PWM,
                )
                if throttle < 0:
                    pwm = -pwm
                pwm *= DRIVE_DIRECTION_SIGN
            self.last_pwm = pwm
            self.last_drive_command_time = now
            command = f"DRIVE {pwm}\n"
            ok = self._write_locked(command)
            if not ok:
                self.enabled = False
                self.throttle = 0.0
                self.last_pwm = 0
                self.drive_boost_until = 0.0

            return self.snapshot()

    def set_steering(self, direction):
        direction = max(-1.0, min(1.0, float(direction)))
        if abs(direction) < 0.1:
            direction = 0.0

        with self.lock:
            encoder_fresh = (
                self.encoder_connected
                and time.time() - self.encoder_last_update < 1.0
            )
            rejection = None
            if direction and not self.enabled:
                rejection = "manual drive not armed"
            elif direction and not encoder_fresh:
                rejection = "AS5600 telemetry unavailable"

            if rejection:
                self.centering = False
                self.steering_closed_loop = False
                self.steering_enabled = False
                self.steering_direction = 0
                self.steering_pwm = 0
                self.steering_rejection = rejection
                self.last_steering_command_time = time.time()
                self._write_locked("STEER 0\n")
                return self.snapshot()

            self.centering = False
            self.steering_rejection = None
            self.last_steering_command_time = time.time()
            left_degrees, right_degrees = self._steering_angle_limits_locked()
            if direction > 0:
                requested_angle = -direction * right_degrees
            elif direction < 0:
                requested_angle = -direction * left_degrees
            else:
                requested_angle = 0.0
            self.steering_requested_angle_degrees = requested_angle
            if not self.steering_closed_loop:
                self.steering_target_angle_degrees = self.steering_angle_degrees or 0.0
                self.last_steering_control_time = time.time()
            self.steering_closed_loop = True
            return self.snapshot()

    def _steering_angle_limits_locked(self):
        left_delta = signed_steering_raw_delta(
            self.steering_center_raw,
            self.steering_left_reference_raw,
        )
        right_delta = signed_steering_raw_delta(
            self.steering_center_raw,
            self.steering_right_reference_raw,
        )
        return abs(left_delta) * 360.0 / 4096.0, abs(right_delta) * 360.0 / 4096.0

    def _update_steering_control_locked(self, now):
        if not self.steering_closed_loop:
            return
        command_timeout = (
            STEER_CENTER_TIMEOUT_SECONDS
            if self.steering_requested_angle_degrees == 0.0
            else MOTOR_TIMEOUT_SECONDS
        )
        if now - self.last_steering_command_time > command_timeout:
            self.steering_closed_loop = False
            self.steering_control_error_degrees = None
            self._stop_steering_output_locked()
            return
        encoder_fresh = self.encoder_connected and now - self.encoder_last_update < 1.0
        if not encoder_fresh or self.steering_angle_degrees is None:
            self.steering_closed_loop = False
            self.steering_rejection = "AS5600 telemetry unavailable"
            self._stop_steering_output_locked()
            return
        elapsed = max(0.0, min(0.1, now - self.last_steering_control_time))
        self.last_steering_control_time = now
        maximum_step = STEER_TARGET_RATE_DEGREES_PER_SECOND * elapsed
        target_delta = self.steering_requested_angle_degrees - self.steering_target_angle_degrees
        target_delta = max(-maximum_step, min(maximum_step, target_delta))
        self.steering_target_angle_degrees += target_delta
        error = self.steering_target_angle_degrees - self.steering_angle_degrees
        self.steering_control_error_degrees = error
        if abs(error) <= STEER_TARGET_TOLERANCE_DEGREES:
            self._stop_steering_output_locked()
            return
        magnitude = round(STEER_MIN_PWM + STEER_CONTROL_KP * abs(error))
        magnitude = max(STEER_MIN_PWM, min(STEER_MANUAL_PWM, magnitude))
        pwm = -magnitude if error > 0 else magnitude
        self.steering_enabled = True
        self.steering_direction = -1 if pwm < 0 else 1
        self.steering_pwm = pwm
        if not self._write_locked(f"STEER {pwm}\n"):
            self.steering_closed_loop = False
            self._stop_steering_output_locked()

    def _stop_steering_output_locked(self):
        was_active = self.steering_enabled or self.steering_pwm != 0
        self.steering_enabled = False
        self.steering_direction = 0
        self.steering_pwm = 0
        if was_active:
            self._write_locked("STEER 0\n")

    def center_steering(self):
        with self.lock:
            self.steering_closed_loop = False
            encoder_fresh = (
                self.encoder_connected
                and time.time() - self.encoder_last_update < 1.0
            )
            if not self.center_supported or not encoder_fresh or self.encoder_zero_raw is None:
                self.centering = False
                self.steering_rejection = "steering center unavailable"
                return self.snapshot()

            self.steering_enabled = False
            self.steering_direction = 0
            self.steering_pwm = 0
            self.centering = True
            self.steering_rejection = None
            if not self._write_locked("CENTER\n"):
                self.centering = False
                self.steering_rejection = "steering center command failed"
            return self.snapshot()

    def stop_steering(self):
        with self.lock:
            self.steering_closed_loop = False
            self.steering_enabled = False
            self.steering_direction = 0
            self.steering_pwm = 0
            self.centering = False
            self.steering_rejection = None
            self._write_locked("STEER 0\n")
            return self.snapshot()

    def reset_hardware_estop(self):
        with self.lock:
            requested = self._write_locked("RESET_ESTOP\n")
            result = self.snapshot()
            result["estop_reset_requested"] = requested
            return result

    def zero_steering_encoder(self):
        with self.lock:
            self.steering_closed_loop = False
            self.steering_enabled = False
            self.steering_direction = 0
            self.steering_pwm = 0
            self.steering_rejection = None
            self.centering = False
            stopped = self._write_locked("STEER 0\n")
            requested = stopped and self._write_locked("ZERO\n")
            result = self.snapshot()
            result["zero_requested"] = requested
            return result

    def configure_steering(self, right_raw, center_raw, left_raw, allowance_raw):
        try:
            right_raw = int(right_raw)
            center_raw = int(center_raw)
            left_raw = int(left_raw)
            allowance_raw = int(allowance_raw)
        except (TypeError, ValueError) as exc:
            raise ValueError("조향 보정값은 정수여야 합니다") from exc

        if not steering_calibration_valid(right_raw, center_raw, left_raw, allowance_raw):
            raise ValueError(
                "오른쪽과 왼쪽 기준값은 중앙값의 서로 반대 방향이어야 하며, "
                "원시값은 0~4095, 안전 여유는 0~300이어야 합니다"
            )

        with self.lock:
            self.steering_closed_loop = False
            encoder_fresh = (
                self.encoder_connected
                and time.time() - self.encoder_last_update < 1.0
            )
            if not self.config_supported or not encoder_fresh:
                result = self.snapshot()
                result["config_saved"] = False
                result["config_error"] = "새 Arduino 조향 펌웨어가 필요합니다"
                return result

            self.steering_enabled = False
            self.steering_direction = 0
            self.steering_pwm = 0
            self.centering = False
            self.steering_rejection = None
            stopped = self._write_locked("STEER 0\n")
            command = f"CONFIG {right_raw} {center_raw} {left_raw} {allowance_raw}\n"
            saved = stopped and self._write_locked(command)
            if saved:
                self.steering_right_reference_raw = right_raw
                self.steering_center_raw = center_raw
                self.encoder_zero_raw = center_raw
                self.steering_left_reference_raw = left_raw
                self.steering_limit_allowance_raw = allowance_raw
                self._refresh_steering_limits_locked()
            result = self.snapshot()
            result["config_saved"] = saved
            result["config_error"] = None if saved else "Arduino 설정 전송에 실패했습니다"
            return result

    def _refresh_steering_limits_locked(self):
        self.steering_right_raw_limit = steering_safety_limit(
            self.steering_right_reference_raw,
            self.steering_center_raw,
            self.steering_limit_allowance_raw,
        )
        self.steering_left_raw_limit = steering_safety_limit(
            self.steering_left_reference_raw,
            self.steering_center_raw,
            self.steering_limit_allowance_raw,
        )

    def stop(self):
        with self.lock:
            self.enabled = False
            self.throttle = 0.0
            self.last_pwm = 0
            self.drive_boost_until = 0.0
            self.drive_watchdog_stop_reason = None
            self.drive_watchdog_stopped_at = None
            self.steering_enabled = False
            self.steering_direction = 0
            self.steering_pwm = 0
            self.steering_rejection = None
            self.centering = False
            self.steering_closed_loop = False
            self._write_locked("STOP\n")

    def emergency_stop(self):
        with self.lock:
            self.stop()
            self._write_locked("ESTOP\n")
            return self.snapshot()

    def _open_locked(self):
        if self.fd is not None:
            return True
        if termios is None:
            self.last_error = "termios unavailable"
            self.next_open_attempt = time.time() + 2.0
            return False

        port = find_arduino_port()
        if not port:
            self.last_error = "arduino port not found"
            self.next_open_attempt = time.time() + 2.0
            return False

        fd = None
        try:
            fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            attrs = termios.tcgetattr(fd)
            attrs[0] = 0
            attrs[1] = 0
            attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
            attrs[3] = 0
            baud = getattr(termios, f"B{MOTOR_BAUD}", termios.B115200)
            attrs[4] = baud
            attrs[5] = baud
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
            time.sleep(2.0)
            self.fd = fd
            self.port = port
            self.serial_buffer = ""
            self.encoder_connected = False
            self.encoder_last_update = 0
            self.last_arduino_response_time = 0
            self.config_supported = False
            os.write(self.fd, b"STATUS\n")
            self.last_error = None
            return True
        except OSError as exc:
            self.last_error = str(exc)
            self.next_open_attempt = time.time() + 2.0
            if fd is not None:
                try:
                    os.close(fd)
                except Exception:
                    pass
            return False

    def _write_locked(self, command):
        if not self._open_locked():
            return False
        try:
            os.write(self.fd, command.encode("ascii"))
            self.last_error = None
            return True
        except OSError as exc:
            self.last_error = str(exc)
            self._close_locked()
            return False

    def _close_locked(self):
        if self.fd is not None:
            try:
                os.close(self.fd)
            except Exception:
                pass
        self.fd = None
        self.port = None
        self.encoder_connected = False
        self.next_open_attempt = time.time() + 2.0

    def _read_locked(self):
        if self.fd is None:
            return
        try:
            while True:
                chunk = os.read(self.fd, 4096)
                if not chunk:
                    break
                self.serial_buffer += chunk.decode("ascii", errors="ignore")
                if len(self.serial_buffer) > 8192:
                    self.serial_buffer = self.serial_buffer[-4096:]
        except BlockingIOError:
            pass
        except OSError as exc:
            self.last_error = str(exc)
            self._close_locked()
            return

        while "\n" in self.serial_buffer:
            line, self.serial_buffer = self.serial_buffer.split("\n", 1)
            line = line.strip()
            if line:
                self._handle_serial_line_locked(line)

    def _handle_serial_line_locked(self, line):
        self.last_arduino_response_time = time.time()
        self.last_serial_line = line
        parts = line.split()
        if not parts:
            return

        if parts[0] == "ENC":
            self.encoder_last_update = time.time()
            if len(parts) >= 2 and parts[1] == "ERR":
                self.encoder_connected = False
                self.encoder_error = "AS5600 read failed"
                return
            if len(parts) < 5:
                return
            try:
                raw = int(parts[1])
                angle = None if parts[2] == "NA" else float(parts[2])
                zero_raw = int(parts[3])
                status = int(parts[4])
            except ValueError:
                return
            self.encoder_connected = True
            self.encoder_raw = raw
            self.steering_angle_degrees = angle
            self.encoder_zero_raw = zero_raw if zero_raw >= 0 else None
            self.encoder_status = status
            self.steering_limit = parts[5] if len(parts) >= 6 else "NONE"
            self.center_supported = len(parts) >= 7
            self.centering = self.center_supported and parts[6] == "CENTERING"
            self.config_supported = len(parts) >= 12
            if self.config_supported:
                try:
                    self.steering_right_reference_raw = int(parts[7])
                    self.steering_left_reference_raw = int(parts[8])
                    self.steering_limit_allowance_raw = int(parts[9])
                    self.steering_right_raw_limit = int(parts[10])
                    self.steering_left_raw_limit = int(parts[11])
                    self.steering_center_raw = zero_raw
                except ValueError:
                    self.config_supported = False
            if len(parts) >= 13 and parts[12].startswith("ESTOP_"):
                self.hardware_estop_supported = True
                self.hardware_estop_active = parts[12] != "ESTOP_OK"
            self.encoder_error = None
            if self.encoder_raw_min_observed is None:
                self.encoder_raw_min_observed = raw
                self.encoder_raw_max_observed = raw
            else:
                self.encoder_raw_min_observed = min(self.encoder_raw_min_observed, raw)
                self.encoder_raw_max_observed = max(self.encoder_raw_max_observed, raw)
            return

        if parts[0] == "ZERO" and len(parts) >= 2:
            try:
                self.encoder_zero_raw = int(parts[1])
                self.steering_center_raw = self.encoder_zero_raw
                self._refresh_steering_limits_locked()
                self.steering_angle_degrees = 0.0
            except ValueError:
                pass
            return

        if parts[0] == "CONFIG" and len(parts) >= 5:
            try:
                self.steering_right_reference_raw = int(parts[1])
                self.steering_center_raw = int(parts[2])
                self.encoder_zero_raw = self.steering_center_raw
                self.steering_left_reference_raw = int(parts[3])
                self.steering_limit_allowance_raw = int(parts[4])
                self._refresh_steering_limits_locked()
                self.config_supported = True
                self.last_error = None
            except ValueError:
                self.last_error = line
            return

        if parts[0] == "LIMIT" and len(parts) >= 3:
            self.steering_limit = parts[1]
            self.steering_enabled = False
            self.steering_direction = 0
            self.steering_pwm = 0
            self.centering = False
            self.steering_rejection = f"{parts[1].lower()} steering limit"
            return

        if parts[0] == "CENTERED" and len(parts) >= 2:
            self.centering = False
            self.steering_enabled = False
            self.steering_direction = 0
            self.steering_pwm = 0
            self.steering_rejection = None
            return

        if parts[0] == "CENTER" and len(parts) >= 2 and parts[1] in {"TIMEOUT", "STALL"}:
            self.centering = False
            self.steering_rejection = f"steering center {parts[1].lower()}"
            return

        if parts[0] == "OK" and len(parts) >= 2 and parts[1] == "CENTER":
            self.centering = True
            self.steering_rejection = None
            return

        if parts[0] == "ERR" and len(parts) >= 2 and parts[1] == "CENTER":
            self.centering = False
            self.steering_rejection = "steering center sensor error"
            return

        if parts[0] == "ERR":
            self.last_error = line
            if len(parts) >= 3 and parts[1] == "ESTOP":
                self.hardware_estop_supported = True
                self.hardware_estop_active = True
        elif parts[0] == "OK":
            self.last_error = None
            if len(parts) >= 3 and parts[1] == "ESTOP":
                self.hardware_estop_supported = True
                self.hardware_estop_active = parts[2] == "LATCHED"

    def _watchdog_loop(self):
        while True:
            time.sleep(0.05)
            with self.lock:
                now = time.time()
                if self.fd is None:
                    if now >= self.next_open_attempt:
                        self._open_locked()
                    continue
                self._read_locked()
                self._update_steering_control_locked(now)
                if self.enabled and now - self.last_drive_command_time > MOTOR_TIMEOUT_SECONDS:
                    self.enabled = False
                    self.throttle = 0.0
                    self.last_pwm = 0
                    self.drive_boost_until = 0.0
                    self.drive_watchdog_stop_reason = "DRIVE_COMMAND_TIMEOUT"
                    self.drive_watchdog_stopped_at = now
                    self._write_locked("DRIVE 0\n")
                if (
                    self.steering_enabled
                    and now - self.last_steering_command_time
                    > (
                        STEER_CENTER_TIMEOUT_SECONDS
                        if self.steering_requested_angle_degrees == 0.0
                        else MOTOR_TIMEOUT_SECONDS
                    )
                ):
                    self.steering_enabled = False
                    self.steering_direction = 0
                    self.steering_pwm = 0
                    self._write_locked("STEER 0\n")


def device_status():
    devices = {
        "gps": {"connected": False, "port": None},
        "arduino": {"connected": False, "port": None},
        "imu": imu_status(),
        "lidar": lidar_monitor.snapshot(),
    }
    for path in glob.glob("/dev/serial/by-id/*"):
        name = os.path.basename(path).lower()
        port = os.path.realpath(path)
        if "u-blox" in name or "zed-f9p" in name:
            devices["gps"] = {"connected": True, "port": port}
        if "arduino" in name:
            devices["arduino"] = {"connected": True, "port": port}
    devices["gps"].update(gps_monitor.snapshot())
    devices["arduino"].update(motor_controller.snapshot())
    return devices


def system_status():
    cpu_count = os.cpu_count() or 1
    try:
        load_average = os.getloadavg()[0]
    except (AttributeError, OSError):
        load_average = None

    memory_total = None
    memory_available = None
    try:
        with open("/proc/meminfo", "r", encoding="utf-8") as meminfo:
            memory_values = {}
            for line in meminfo:
                key, value = line.split(":", 1)
                memory_values[key] = int(value.strip().split()[0]) * 1024
        memory_total = memory_values.get("MemTotal")
        memory_available = memory_values.get("MemAvailable")
    except (OSError, ValueError, IndexError):
        pass

    temperature_c = None
    try:
        with open("/sys/class/thermal/thermal_zone0/temp", "r", encoding="utf-8") as temperature_file:
            temperature_c = float(temperature_file.read().strip()) / 1000.0
    except (OSError, ValueError):
        pass

    uptime_seconds = None
    try:
        with open("/proc/uptime", "r", encoding="utf-8") as uptime_file:
            uptime_seconds = float(uptime_file.read().split()[0])
    except (OSError, ValueError, IndexError):
        pass

    disk = shutil.disk_usage("/")
    memory_used = (
        memory_total - memory_available
        if memory_total is not None and memory_available is not None
        else None
    )
    return {
        "hostname": socket.gethostname(),
        "time": time.time(),
        "cpu_count": cpu_count,
        "cpu_load_percent": round(load_average / cpu_count * 100, 1) if load_average is not None else None,
        "temperature_c": round(temperature_c, 1) if temperature_c is not None else None,
        "memory_total_bytes": memory_total,
        "memory_used_bytes": memory_used,
        "disk_total_bytes": disk.total,
        "disk_used_bytes": disk.used,
        "uptime_seconds": uptime_seconds,
    }


camera = Camera()
motor_controller = MotorController()
camera_calibration = CameraCalibration(CAMERA_CALIBRATION_PATH)


class PerceptionMonitor:
    def __init__(self):
        self.lock = threading.Lock()
        self.detector = ObjectDetector(
            LIDAR_CAMERA_FOV_DEGREES,
            camera_calibration=camera_calibration,
        )
        self.state = {
            "available": self.detector.available,
            "detections": [],
            "hazard": False,
            "last_update": None,
            "frame_sequence": None,
            "error": None,
        }

    def start(self):
        threading.Thread(target=self._run, daemon=True).start()

    def snapshot(self):
        with self.lock:
            return dict(self.state)

    def _run(self):
        last_sequence = -1
        while True:
            time.sleep(0.5)
            try:
                frame, sequence, frame_monotonic, _ = camera.snapshot_frame()
                if frame is None or sequence == last_sequence:
                    continue
                last_sequence = sequence
                detections = self.detector.detect_people(frame)
                fused = self.detector.fuse_lidar(
                    detections,
                    lidar_monitor.snapshot().get("points", []),
                )
                with self.lock:
                    self.state.update(
                        detections=[item.as_dict() for item in fused],
                        hazard=any(item.in_vehicle_path for item in fused),
                        last_update=time.time(),
                        frame_sequence=sequence,
                        error=None,
                    )
            except Exception as error:
                with self.lock:
                    self.state.update(hazard=False, error=str(error))


perception_monitor = PerceptionMonitor()
throttle_calibration = ThrottleCalibration(THROTTLE_CALIBRATION_PATH)
vehicle_state_machine = VehicleStateMachine()
safety_supervisor = SafetySupervisor(
    obstacle_checker=ObstacleChecker(
        half_width_m=LIDAR_SAFETY_HALF_WIDTH_M,
        lidar_to_front_bumper_m=LIDAR_TO_FRONT_BUMPER_M,
    ),
    maximum_throttle=MANUAL_MAX_THROTTLE,
    stop_distance_m=LIDAR_STOP_DISTANCE_M,
    crawl_distance_m=LIDAR_CRAWL_DISTANCE_M,
    slow_distance_m=LIDAR_SLOW_DISTANCE_M,
)


def safety_snapshot():
    snapshot = safety_supervisor.snapshot()
    lidar = lidar_monitor.snapshot()
    obstacle = safety_supervisor.obstacle_checker.check(
        lidar.get("safety_points", [])
    )
    snapshot.update(
        obstacle_detected=obstacle.distance_m is not None,
        obstacle_distance_m=obstacle.distance_m,
        obstacle_point_count=obstacle.point_count,
    )
    motor = motor_controller.snapshot()
    watchdog_reason = motor.get("drive_watchdog_stop_reason")
    if watchdog_reason:
        snapshot.update(
            final_throttle=0.0,
            allowed=False,
            stop_reason=watchdog_reason,
            command_timeout_at=motor.get("drive_watchdog_stopped_at"),
        )
    return snapshot


def safety_context(loop_delay_seconds=0.0):
    now = time.time()
    motor = motor_controller.snapshot()
    lidar = lidar_monitor.snapshot()
    return SafetyContext(
        mode=vehicle_state_machine.mode,
        arduino=SensorStatus.build(
            motor,
            motor.get("last_response_at"),
            motor["connected"],
            motor.get("last_error"),
            now,
        ),
        lidar=SensorStatus.build(
            lidar.get("safety_points", []),
            lidar.get("last_update"),
            lidar.get("connected", False),
            lidar.get("error"),
            now,
        ),
        steering=SensorStatus.build(
            motor.get("steering_angle_degrees"),
            motor.get("encoder_last_update"),
            motor.get("encoder_connected", False),
            motor.get("encoder_error"),
            now,
        ),
        emergency_stop=bool(motor.get("hardware_estop_active")),
        camera_hazard=perception_monitor.snapshot().get("hazard", False),
        loop_delay_seconds=max(0.0, float(loop_delay_seconds)),
    )


def sensor_health_snapshot():
    now = time.time()
    gps = gps_monitor.snapshot()
    imu = imu_monitor.snapshot()
    lidar = lidar_monitor.snapshot()
    motor = motor_controller.snapshot()
    _, _, camera_monotonic, camera_wall_time = camera.snapshot_frame()
    statuses = {
        "gnss": SensorStatus.build(
            {
                "latitude": gps.get("latitude"),
                "longitude": gps.get("longitude"),
                "fix": gps.get("fix"),
            },
            gps.get("received_at"),
            gps.get("gpsd_connected", False) and gps.get("latitude") is not None,
            gps.get("error"),
            now,
        ),
        "imu": SensorStatus.build(
            {
                "heading_degrees": imu.get("heading_degrees"),
                "yaw_rate_dps": imu.get("yaw_rate_dps"),
            },
            imu.get("last_update"),
            imu.get("connected", False),
            imu.get("error"),
            now,
        ),
        "lidar": SensorStatus.build(
            {"rotation_hz": lidar.get("rotation_hz"), "point_count": lidar.get("point_count")},
            lidar.get("last_update"),
            lidar.get("connected", False),
            lidar.get("error"),
            now,
        ),
        "steering": SensorStatus.build(
            motor.get("steering_angle_degrees"),
            motor.get("encoder_last_update"),
            motor.get("encoder_connected", False),
            motor.get("encoder_error"),
            now,
        ),
        "arduino": SensorStatus.build(
            {"port": motor.get("port")},
            motor.get("last_response_at"),
            motor.get("connected", False),
            motor.get("last_error"),
            now,
        ),
        "camera": SensorStatus.build(
            {"frame_monotonic": camera_monotonic},
            camera_wall_time,
            camera_wall_time is not None,
            None if camera_wall_time is not None else "CAMERA_FRAME_UNAVAILABLE",
            now,
        ),
    }
    return {name: status.as_dict() for name, status in statuses.items()}


def update_manual_mode(enabled, manual_override=False):
    mode = vehicle_state_machine.mode
    if enabled and mode in {DriveMode.AUTO_ROUTE, DriveMode.AUTO_HYBRID}:
        if not manual_override:
            return False
        if "auto_route_runtime" in globals():
            auto_route_runtime.stop("manual_override")
        mode = vehicle_state_machine.mode
    if enabled and mode == DriveMode.DISARMED:
        vehicle_state_machine.transition(DriveMode.MANUAL_ASSIST, "manual_control_armed")
    elif not enabled and mode not in {DriveMode.DISARMED, DriveMode.EMERGENCY_STOP, DriveMode.FAULT}:
        vehicle_state_machine.transition(DriveMode.DISARMED, "manual_control_disarmed")
    return True


manual_deadman_pressed = False


def apply_safe_drive(throttle, enabled, deadman=False):
    global manual_deadman_pressed
    manual_deadman_pressed = bool(deadman) and bool(enabled)
    if not update_manual_mode(enabled, manual_deadman_pressed):
        result = motor_controller.snapshot()
        result["safety"] = safety_snapshot()
        result["state_machine"] = vehicle_state_machine.snapshot()
        result["manual_command_ignored"] = True
        return result
    motor = motor_controller.snapshot()
    request = ControlRequest(
        throttle=throttle,
        steering=motor.get("steering_direction", 0.0),
        enabled=enabled,
        deadman_pressed=manual_deadman_pressed,
        source="dashboard",
    )
    decision = safety_supervisor.evaluate(request, safety_context())
    result = motor_controller.set_drive(
        decision.final_throttle,
        enabled and decision.allowed,
    )
    result["safety"] = decision.as_dict()
    result["state_machine"] = vehicle_state_machine.snapshot()
    return result


def apply_safe_steering(direction):
    if vehicle_state_machine.mode in {DriveMode.AUTO_ROUTE, DriveMode.AUTO_HYBRID}:
        result = motor_controller.snapshot()
        result["safety"] = safety_snapshot()
        result["state_machine"] = vehicle_state_machine.snapshot()
        result["manual_command_ignored"] = True
        return result
    motor = motor_controller.snapshot()
    request = ControlRequest(
        throttle=motor.get("throttle", 0.0),
        steering=direction,
        enabled=motor.get("enabled", False),
        deadman_pressed=manual_deadman_pressed,
        source="dashboard",
    )
    decision = safety_supervisor.evaluate(request, safety_context())
    result = motor_controller.set_steering(decision.final_steering)
    result["safety"] = decision.as_dict()
    result["state_machine"] = vehicle_state_machine.snapshot()
    return result


def lidar_sector_distance(points, minimum_bearing, maximum_bearing):
    distances = []
    for point in points or []:
        try:
            bearing = float(point["bearing_degrees"])
            distance = float(point["distance_mm"]) / 1000.0
        except (KeyError, TypeError, ValueError):
            continue
        if minimum_bearing <= bearing <= maximum_bearing and distance > 0:
            distances.append(distance)
    return min(distances) if distances else None


def recording_samples():
    monotonic_timestamp = time.monotonic()
    wall_time = time.time()
    gps = gps_monitor.snapshot()
    imu = imu_monitor.snapshot()
    motor = motor_controller.snapshot()
    lidar = lidar_monitor.snapshot()
    safety = safety_snapshot()
    state_machine = vehicle_state_machine.snapshot()
    perception = perception_monitor.snapshot()
    health = sensor_health_snapshot()
    points = lidar.get("points", [])
    front_left = lidar_sector_distance(points, -45, -5)
    front_center = lidar_sector_distance(points, -5, 5)
    front_right = lidar_sector_distance(points, 5, 45)
    front_values = [value for value in (front_left, front_center, front_right) if value is not None]
    common = {"monotonic": monotonic_timestamp, "wall_time": wall_time}
    route = None
    if gps.get("fix") == "RTK FIXED" and gps.get("latitude") is not None:
        route = {
            **common,
            "latitude": gps.get("latitude"),
            "longitude": gps.get("longitude"),
            "altitude_m": gps.get("altitude_m"),
            "rtk_status": gps.get("fix"),
            "speed_mps": gps.get("speed_mps"),
            "course_degrees": gps.get("track_degrees"),
            "gnss_timestamp": gps.get("last_update"),
            "is_valid": health["gnss"]["is_valid"],
            "data_age": health["gnss"]["data_age"],
            "error_code": health["gnss"]["error_code"],
        }
    auto_state = auto_route_runtime.snapshot() if "auto_route_runtime" in globals() else {}
    auto_command = auto_state.get("last_command") or {}
    lane = auto_state.get("lane") or {}
    mode = state_machine["mode"]
    system_state = (
        mode
        if mode in {
            DriveMode.DISARMED.value,
            DriveMode.EMERGENCY_STOP.value,
            DriveMode.FAULT.value,
        }
        else "ACTIVE"
    )
    manual_override = state_machine.get("reason") == "manual_override"
    input_source = safety.get("input_source")
    return {
        "vehicle_state": {
            **common,
            "mode": mode,
            "system_state": system_state,
            "manual_override": manual_override,
            "emergency_stop": mode == DriveMode.EMERGENCY_STOP.value,
            "fault_code": safety.get("stop_reason"),
        },
        "gnss": {
            **common,
            "latitude": gps.get("latitude"),
            "longitude": gps.get("longitude"),
            "altitude_m": gps.get("altitude_m"),
            "rtk_status": gps.get("fix"),
            "satellites": gps.get("satellites_used"),
            "hdop": gps.get("hdop"),
            "speed_mps": gps.get("speed_mps"),
            "course_degrees": gps.get("track_degrees"),
            "gnss_timestamp": gps.get("last_update"),
            "is_valid": health["gnss"]["is_valid"],
            "data_age": health["gnss"]["data_age"],
            "error_code": health["gnss"]["error_code"],
        },
        "imu": {
            **common,
            "yaw_degrees": imu.get("global_heading_degrees"),
            "pitch_degrees": imu.get("pitch_degrees"),
            "roll_degrees": imu.get("roll_degrees"),
            "yaw_rate_dps": imu.get("yaw_rate_dps"),
            "acceleration_x": imu.get("acceleration_x"),
            "acceleration_y": imu.get("acceleration_y"),
            "acceleration_z": imu.get("acceleration_z"),
            "imu_timestamp": imu.get("last_update"),
            "is_valid": health["imu"]["is_valid"],
            "data_age": health["imu"]["data_age"],
            "error_code": health["imu"]["error_code"],
        },
        "steering": {
            **common,
            "raw": motor.get("encoder_raw"),
            "angle_degrees": motor.get("steering_angle_degrees"),
            "target_angle_degrees": motor.get("target_steering_angle_degrees"),
            "motor_command": motor.get("steering_pwm"),
            "error_degrees": motor.get("steering_control_error_degrees"),
            "is_valid": health["steering"]["is_valid"],
            "data_age": health["steering"]["data_age"],
            "error_code": health["steering"]["error_code"],
        },
        "arduino": {
            **common,
            "port": motor.get("port"),
            "connected": motor.get("connected"),
            "enabled": motor.get("enabled"),
            "drive_pwm": motor.get("pwm"),
            "steering_pwm": motor.get("steering_pwm"),
            "hardware_estop_active": motor.get("hardware_estop_active"),
            "watchdog_stop_reason": motor.get("drive_watchdog_stop_reason"),
            "last_response_at": motor.get("last_response_at"),
            "is_valid": health["arduino"]["is_valid"],
            "data_age": health["arduino"]["data_age"],
            "error_code": health["arduino"]["error_code"],
        },
        "control": {
            **common,
            "input_source": input_source,
            "gamepad_throttle": (
                safety.get("requested_throttle")
                if input_source == "dashboard"
                else None
            ),
            "requested_throttle": safety.get("requested_throttle"),
            "limited_throttle": safety.get("throttle_limit"),
            "final_throttle": safety.get("final_throttle"),
            "requested_steering": safety.get("requested_steering"),
            "final_steering": safety.get("final_steering"),
            "stop_reason": safety.get("stop_reason"),
            "target_speed_mps": auto_command.get("target_speed_mps"),
            "cross_track_error_m": auto_command.get("cross_track_error_m"),
            "target_index": auto_command.get("target_index"),
        },
        "lidar_summary": {
            **common,
            "front_min_distance_m": min(front_values) if front_values else None,
            "front_left_distance_m": front_left,
            "front_center_distance_m": front_center,
            "front_right_distance_m": front_right,
            "obstacle_state": safety.get("stop_reason") if safety.get("stop_reason") == "OBSTACLE_STOP" else "CLEAR",
            "rotation_hz": lidar.get("rotation_hz"),
            "point_count": lidar.get("point_count"),
            "is_valid": health["lidar"]["is_valid"],
            "data_age": health["lidar"]["data_age"],
            "error_code": health["lidar"]["error_code"],
        },
        "lidar_raw": {
            "monotonic": monotonic_timestamp,
            "wall_time": wall_time,
            "rotation_hz": lidar.get("rotation_hz"),
            "points": points,
        },
        "perception": {
            **common,
            "frame_sequence": perception.get("frame_sequence"),
            "hazard": perception.get("hazard"),
            "detection_count": len(perception.get("detections") or []),
            "detections_json": json.dumps(perception.get("detections") or [], separators=(",", ":")),
            "error": perception.get("error"),
            "lane_detected": lane.get("detected"),
            "lane_confidence": lane.get("confidence"),
            "lane_lateral_error_m": lane.get("lateral_error_m"),
            "lane_heading_error_degrees": lane.get("heading_error_degrees"),
            "lane_correction_angle_degrees": lane.get("correction_angle_degrees"),
            "lane_error": lane.get("error"),
            "camera_is_valid": health["camera"]["is_valid"],
            "camera_data_age": health["camera"]["data_age"],
            "camera_error_code": health["camera"]["error_code"],
        },
        "route": route,
    }


record_manager = RecordManager(
    RECORDINGS_PATH,
    recording_samples,
    camera.snapshot_frame,
    camera_fps=RECORD_CAMERA_FPS,
)


def recording_metadata():
    return {
        "vehicle_wheelbase_m": 0.53,
        "vehicle_width_m": 0.4826,
        "lidar_position_vehicle_m": {"x": -0.254, "y": 0.0, "z": None},
        "camera_resolution": SIZE,
        "camera_calibration": camera_calibration.snapshot(),
        "imu_acceleration_unit": "m/s^2",
        "steering_center_raw": motor_controller.snapshot().get("steer_center_raw"),
        "steering_right_reference_raw": motor_controller.snapshot().get("steer_right_reference_raw"),
        "steering_left_reference_raw": motor_controller.snapshot().get("steer_left_reference_raw"),
        "software_version": "autonomy-v1",
    }


def normalized_steering_for_angle(angle_degrees):
    motor = motor_controller.snapshot()
    center = motor.get("steer_center_raw")
    left = motor.get("steer_left_reference_raw")
    right = motor.get("steer_right_reference_raw")
    if None in {center, left, right}:
        return 0.0
    left_degrees = abs(signed_steering_raw_delta(center, left)) * 360.0 / 4096.0
    right_degrees = abs(signed_steering_raw_delta(center, right)) * 360.0 / 4096.0
    if angle_degrees > 0:
        return -min(1.0, angle_degrees / max(1.0, left_degrees))
    if angle_degrees < 0:
        return min(1.0, abs(angle_degrees) / max(1.0, right_degrees))
    return 0.0


class AutoRouteRuntime:
    def __init__(self):
        self.lock = threading.RLock()
        self.active = False
        self.planner = None
        self.route_path = None
        self.last_command = None
        self.error = None
        self.thread = None
        self.hybrid_enabled = False
        self.lane_controller = LaneController(
            camera_calibration=camera_calibration,
        )
        self.lane_result = None
        self.last_lane_sequence = -1
        self.heading_estimator = HeadingEstimator()
        self.error_history = deque(maxlen=12000)
        self.started_monotonic = None
        self.owns_recording = False
        self.hybrid_fallback_guard = HybridFallbackGuard(
            minimum_lane_confidence=AUTO_LANE_MIN_CONFIDENCE,
        )
        self.lane_continuity_filter = LaneContinuityFilter()
        self.hybrid_fallback_reason = None
        self.obstacle_restart_guard = RestartDelayGuard(
            AUTO_OBSTACLE_RESTART_DELAY_SECONDS
        )
        self.steering_tracking_guard = SteeringTrackingGuard(
            AUTO_STEERING_MAX_ERROR_DEGREES,
            AUTO_STEERING_ERROR_TIMEOUT_SECONDS,
        )

    def load(self, route_path):
        route = RouteProcessor.load_json(route_path)
        if len(route.points) < 2:
            raise ValueError("Processed route requires at least two points")
        with self.lock:
            self.planner = AutoRoutePlanner(route)
            self.route_path = route_path
            self.last_command = None
            self.error = None
            self.hybrid_enabled = False
            self.lane_result = None
            self.heading_estimator.reset()
            self.error_history.clear()
            self.started_monotonic = None
            self.hybrid_fallback_guard.reset()
            self.lane_continuity_filter.reset()
            self.hybrid_fallback_reason = None
            self.obstacle_restart_guard.reset()
            self.steering_tracking_guard.reset()
        return self.snapshot()

    def preflight(self):
        with self.lock:
            if self.planner is None:
                raise ValueError("No processed route is loaded")
            motor = motor_controller.snapshot()
            result = self.planner.preflight(
                gps_monitor.snapshot(),
                imu_monitor.snapshot(),
                lidar_monitor.snapshot().get("connected", False),
                motor.get("connected", False),
                motor.get("encoder_connected", False),
                bool(motor.get("hardware_estop_active")),
            )
            return {
                "ready": result.ready,
                "errors": result.errors,
                "start_distance_m": result.start_distance_m,
                "heading_error_degrees": result.heading_error_degrees,
            }

    def start(self):
        with self.lock:
            initial_mode = vehicle_state_machine.mode
            if initial_mode not in {DriveMode.DISARMED, DriveMode.MANUAL_ASSIST}:
                raise ValueError("AUTO_ROUTE requires DISARMED or MANUAL_ASSIST mode")
            check = self.preflight()
            if not check["ready"]:
                return {**self.snapshot(), "preflight": check}
            if initial_mode == DriveMode.DISARMED:
                vehicle_state_machine.transition(
                    DriveMode.MANUAL_ASSIST,
                    "auto_route_preflight_passed",
                )
            if not record_manager.active:
                metadata = recording_metadata()
                metadata["purpose"] = "AUTO_ROUTE"
                record_manager.start(metadata)
                self.owns_recording = True
            self.active = True
            self.error = None
            self.hybrid_enabled = False
            self.started_monotonic = time.monotonic()
            self.error_history.clear()
            self.obstacle_restart_guard.reset()
            self.steering_tracking_guard.reset()
            vehicle_state_machine.transition(DriveMode.AUTO_ROUTE, "auto_route_started")
            self.thread = threading.Thread(target=self._run, daemon=True)
            self.thread.start()
            return {**self.snapshot(), "preflight": check}

    def stop(self, reason="operator_stop"):
        with self.lock:
            self.active = False
            if reason == "manual_override":
                record_manager.add_event("MANUAL_OVERRIDE", "controller_deadman")
            record_manager.add_event("AUTO_ROUTE_STOPPED", reason)
            motor_controller.stop()
            self.hybrid_enabled = False
            self.obstacle_restart_guard.reset()
            self.steering_tracking_guard.reset()
            self.hybrid_fallback_guard.reset()
            self._stop_owned_recording_locked()
            if vehicle_state_machine.mode in {DriveMode.AUTO_ROUTE, DriveMode.AUTO_HYBRID}:
                vehicle_state_machine.transition(DriveMode.MANUAL_ASSIST, reason)
            return self.snapshot()

    def enable_hybrid(self):
        with self.lock:
            if not self.active or vehicle_state_machine.mode != DriveMode.AUTO_ROUTE:
                raise ValueError("AUTO_HYBRID requires active AUTO_ROUTE")
            if not self.lane_controller.available or camera.frame is None:
                raise ValueError("Camera lane processing is unavailable")
            self.hybrid_enabled = True
            self.last_lane_sequence = -1
            self.hybrid_fallback_guard.reset()
            self.lane_continuity_filter.reset()
            self.hybrid_fallback_reason = None
            vehicle_state_machine.transition(DriveMode.AUTO_HYBRID, "auto_hybrid_started")
            return self.snapshot()

    def disable_hybrid(self):
        with self.lock:
            self.hybrid_enabled = False
            self.hybrid_fallback_guard.reset()
            self.lane_continuity_filter.reset()
            self.hybrid_fallback_reason = "OPERATOR_DISABLED"
            if vehicle_state_machine.mode == DriveMode.AUTO_HYBRID:
                vehicle_state_machine.transition(DriveMode.AUTO_ROUTE, "camera_correction_disabled")
            return self.snapshot()

    def snapshot(self):
        with self.lock:
            history = list(self.error_history)
            errors = [item[1] for item in history]
            return {
                "active": self.active,
                "route_path": self.route_path,
                "last_command": self.last_command,
                "error": self.error,
                "hybrid_enabled": self.hybrid_enabled,
                "lane": self.lane_result,
                "lane_failure_count": self.hybrid_fallback_guard.lane_failure_count,
                "minimum_lane_confidence": (
                    self.hybrid_fallback_guard.minimum_lane_confidence
                ),
                "hybrid_fallback_reason": self.hybrid_fallback_reason,
                "obstacle_hold_reason": self.obstacle_restart_guard.reason,
                "obstacle_restart_remaining_seconds": self.obstacle_restart_guard.remaining(
                    time.monotonic(), advance=False
                ),
                "steering_tracking": self.steering_tracking_guard.snapshot(),
                "error_summary": {
                    "samples": len(errors),
                    "latest_m": errors[-1] if errors else None,
                    "maximum_m": max(errors) if errors else None,
                    "mean_m": sum(errors) / len(errors) if errors else None,
                },
                "error_history": [
                    {"elapsed_seconds": elapsed, "cross_track_error_m": error}
                    for elapsed, error in history[-600:]
                ],
            }

    def _run(self):
        previous_loop_started = None
        while True:
            loop_started = time.monotonic()
            loop_delay = (
                max(0.0, loop_started - previous_loop_started)
                if previous_loop_started is not None
                else 0.0
            )
            previous_loop_started = loop_started
            with self.lock:
                if not self.active or self.planner is None:
                    return
                gps = gps_monitor.snapshot()
                imu = imu_monitor.snapshot()
                fused_heading = self.heading_estimator.update(
                    imu.get("global_heading_degrees"),
                    imu.get("yaw_rate_dps"),
                    gps.get("track_degrees"),
                    gps.get("speed_mps"),
                )
                fused_imu = dict(imu)
                fused_imu["global_heading_degrees"] = fused_heading
                command = self.planner.update(gps, fused_imu)
                motor = motor_controller.snapshot()
                steering_fault = self.steering_tracking_guard.evaluate(
                    motor.get("target_steering_angle_degrees"),
                    motor.get("steering_angle_degrees"),
                    active=motor.get("steering_control_mode") == "ANGLE",
                )
                if steering_fault:
                    self._fault_locked(steering_fault)
                    return
                base_steering_angle = command.steering_angle_degrees
                final_steering_angle = base_steering_angle
                if self.hybrid_enabled:
                    frame, sequence, frame_monotonic, _ = camera.snapshot_frame()
                    frame_age = (
                        None
                        if frame_monotonic is None
                        else time.monotonic() - frame_monotonic
                    )
                    fallback = self.hybrid_fallback_guard.evaluate(frame_age)
                    if fallback.fallback_reason:
                        self.lane_result = {
                            "detected": False,
                            "confidence": 0.0,
                            "error": fallback.fallback_reason,
                        }
                        self._fallback_to_route_locked(fallback.fallback_reason)
                    elif sequence != self.last_lane_sequence:
                        self.last_lane_sequence = sequence
                        try:
                            lane = self.lane_controller.analyze_jpeg(frame)
                            self.lane_result = self.lane_continuity_filter.filter(
                                lane.as_dict()
                            )
                        except Exception as error:
                            self.lane_result = {
                                "detected": False,
                                "confidence": 0.0,
                                "error": f"LANE_PROCESSING_ERROR: {error}",
                            }
                        fallback = self.hybrid_fallback_guard.evaluate(
                            frame_age,
                            new_frame=True,
                            lane_result=self.lane_result,
                        )
                        if fallback.fallback_reason:
                            self._fallback_to_route_locked(fallback.fallback_reason)
                    if self.hybrid_enabled and self.lane_result and self.lane_result.get("detected"):
                        confidence = float(self.lane_result.get("confidence") or 0.0)
                        correction = float(self.lane_result.get("correction_angle_degrees") or 0.0)
                        final_steering_angle += confidence * correction
                self.last_command = {
                    "base_steering_angle_degrees": base_steering_angle,
                    "steering_angle_degrees": final_steering_angle,
                    "throttle": command.throttle,
                    "cross_track_error_m": command.cross_track_error_m,
                    "nearest_index": command.nearest_index,
                    "target_index": command.target_index,
                    "finished": command.finished,
                    "fault": command.fault,
                    "fused_heading_degrees": fused_heading,
                }
                elapsed_seconds = (
                    time.monotonic() - self.started_monotonic
                    if self.started_monotonic is not None
                    else 0.0
                )
                self.error_history.append((elapsed_seconds, command.cross_track_error_m))
                if command.fault:
                    self._fault_locked(command.fault)
                    return
                if command.finished:
                    record_manager.add_event(
                        "AUTO_ROUTE_COMPLETED",
                        json.dumps(
                            {
                                "maximum_cross_track_error_m": max(
                                    (item[1] for item in self.error_history),
                                    default=0.0,
                                ),
                                "target_index": command.target_index,
                            },
                            separators=(",", ":"),
                        ),
                    )
                    self.active = False
                    motor_controller.stop()
                    self.hybrid_enabled = False
                    self.obstacle_restart_guard.reset()
                    self._stop_owned_recording_locked()
                    vehicle_state_machine.transition(DriveMode.MANUAL_ASSIST, "route_finished")
                    return
                steering = normalized_steering_for_angle(final_steering_angle)
                calibrated_throttle = throttle_calibration.throttle_for_speed(command.throttle)
                self.last_command["target_speed_mps"] = command.throttle
                self.last_command["calibrated_throttle"] = calibrated_throttle
                request = ControlRequest(
                    throttle=calibrated_throttle,
                    steering=steering,
                    enabled=True,
                    source="auto_route",
                )
                decision = safety_supervisor.evaluate(request, safety_context(loop_delay))
                if not decision.allowed:
                    motor_controller.stop_steering()
                    if decision.stop_reason in {"OBSTACLE_STOP", "CAMERA_OBJECT_STOP"}:
                        self.obstacle_restart_guard.block(decision.stop_reason)
                        motor_controller.set_drive(0.0, True)
                    else:
                        motor_controller.set_drive(0.0, False)
                        self._fault_locked(decision.stop_reason or "SAFETY_STOP")
                        return
                else:
                    restart_remaining = self.obstacle_restart_guard.remaining(time.monotonic())
                    self.last_command["obstacle_restart_delay_remaining_seconds"] = restart_remaining
                    if restart_remaining > 0:
                        motor_controller.stop_steering()
                        motor_controller.set_drive(0.0, True)
                    else:
                        motor_controller.set_drive(0.0, True)
                        steering_result = motor_controller.set_steering(
                            decision.final_steering
                        )
                        if steering_result.get("steering_rejection"):
                            motor_controller.set_drive(0.0, False)
                            self._fault_locked("STEERING_COMMAND_REJECTED")
                            return
                        motor_controller.set_drive(decision.final_throttle, True)
            elapsed = time.monotonic() - loop_started
            time.sleep(max(0.0, 0.1 - elapsed))

    def _fault_locked(self, reason):
        record_manager.add_event("AUTO_ROUTE_FAULT", reason)
        self.active = False
        self.hybrid_enabled = False
        self.error = reason
        motor_controller.stop()
        self.obstacle_restart_guard.reset()
        self.steering_tracking_guard.reset()
        self._stop_owned_recording_locked()
        vehicle_state_machine.transition(DriveMode.FAULT, reason)

    def _stop_owned_recording_locked(self):
        if not self.owns_recording:
            return
        self.owns_recording = False
        threading.Thread(target=record_manager.stop, daemon=True).start()

    def _fallback_to_route_locked(self, reason):
        record_manager.add_event("AUTO_HYBRID_FALLBACK", reason)
        self.hybrid_enabled = False
        self.hybrid_fallback_reason = reason
        self.hybrid_fallback_guard.reset()
        self.lane_continuity_filter.reset()
        if vehicle_state_machine.mode == DriveMode.AUTO_HYBRID:
            vehicle_state_machine.transition(DriveMode.AUTO_ROUTE, reason.lower())

auto_route_runtime = AutoRouteRuntime()


def resolve_recording_path(session_name, filename):
    safe_session = os.path.basename(str(session_name or ""))
    path = os.path.abspath(os.path.join(RECORDINGS_PATH, safe_session, filename))
    root = os.path.abspath(RECORDINGS_PATH)
    if os.path.commonpath([root, path]) != root:
        raise ValueError("Invalid recording path")
    return path


def recording_session_path(session_name):
    safe_session = os.path.basename(str(session_name or "").strip())
    if not safe_session or safe_session in {".", ".."} or safe_session != str(session_name).strip():
        raise ValueError("Invalid recording session")
    path = os.path.abspath(os.path.join(RECORDINGS_PATH, safe_session))
    root = os.path.abspath(RECORDINGS_PATH)
    if os.path.commonpath([root, path]) != root:
        raise ValueError("Invalid recording session")
    return path


def recording_directory_size(path):
    total = 0
    for directory, _, filenames in os.walk(path):
        for filename in filenames:
            try:
                total += os.path.getsize(os.path.join(directory, filename))
            except OSError:
                continue
    return total


def list_recording_sessions():
    os.makedirs(RECORDINGS_PATH, exist_ok=True)
    active_path = (
        os.path.abspath(record_manager.session_path)
        if record_manager.active and record_manager.session_path
        else None
    )
    sessions = []
    for entry in os.scandir(RECORDINGS_PATH):
        if not entry.is_dir(follow_symlinks=False):
            continue
        metadata = {}
        try:
            with open(os.path.join(entry.path, "metadata.json"), "r", encoding="utf-8") as file:
                metadata = json.load(file)
        except (OSError, ValueError, TypeError, json.JSONDecodeError):
            pass
        sessions.append(
            {
                "session": entry.name,
                "label": str(metadata.get("label") or ""),
                "started_wall_time": metadata.get("started_wall_time"),
                "size_bytes": recording_directory_size(entry.path),
                "active": active_path == os.path.abspath(entry.path),
                "has_route": os.path.isfile(os.path.join(entry.path, "route.csv")),
                "has_processed_route": os.path.isfile(
                    os.path.join(entry.path, "processed_route.json")
                ),
            }
        )
    sessions.sort(key=lambda item: item.get("started_wall_time") or 0, reverse=True)
    return {"sessions": sessions}


def label_recording_session(session_name, label):
    path = recording_session_path(session_name)
    if not os.path.isdir(path):
        raise FileNotFoundError("Recording session not found")
    clean_label = " ".join(str(label or "").split()).strip()
    if len(clean_label) > 80:
        raise ValueError("Recording label must be 80 characters or fewer")
    metadata_path = os.path.join(path, "metadata.json")
    metadata = {}
    try:
        with open(metadata_path, "r", encoding="utf-8") as file:
            metadata = json.load(file)
    except FileNotFoundError:
        pass
    if not isinstance(metadata, dict):
        metadata = {}
    metadata["label"] = clean_label
    temporary_path = f"{metadata_path}.tmp"
    with open(temporary_path, "w", encoding="utf-8") as file:
        json.dump(metadata, file, ensure_ascii=False, indent=2)
    os.replace(temporary_path, metadata_path)
    return {"session": os.path.basename(path), "label": clean_label}


def delete_recording_session(session_name):
    path = recording_session_path(session_name)
    if not os.path.isdir(path):
        raise FileNotFoundError("Recording session not found")
    if record_manager.active and record_manager.session_path:
        if os.path.abspath(record_manager.session_path) == path:
            raise RuntimeError("Active recording cannot be deleted")
    route_path = auto_route_runtime.snapshot().get("route_path")
    if route_path and os.path.commonpath([path, os.path.abspath(route_path)]) == path:
        raise RuntimeError("Loaded autonomous route session cannot be deleted")
    shutil.rmtree(path)
    return {"deleted": True, "session": os.path.basename(path)}


def recording_route_points(session_name):
    path = resolve_recording_path(session_name, "route.csv")
    if not os.path.isfile(path):
        raise FileNotFoundError("Recorded route not found")
    points = []
    with open(path, "r", encoding="utf-8") as file:
        for row in csv.DictReader(file):
            try:
                latitude = float(row["latitude"])
                longitude = float(row["longitude"])
            except (KeyError, TypeError, ValueError):
                continue
            if not math.isfinite(latitude) or not math.isfinite(longitude):
                continue
            points.append([latitude, longitude])
            if len(points) >= 20000:
                break
    if not points:
        raise ValueError("Recorded route has no valid GNSS points")
    return {"session": os.path.basename(os.path.dirname(path)), "points": points}


def recording_replay_state(session_name, offset_seconds=0.0):
    path = recording_session_path(session_name)
    if not os.path.isdir(path):
        raise FileNotFoundError("Recording session not found")
    replay = LogReplay(path)
    start, end = replay.time_range
    if start is None or end is None:
        raise ValueError("Recording session has no replayable samples")
    vehicle_timestamps = replay.timestamps.get("vehicle_state") or []
    if vehicle_timestamps:
        start = vehicle_timestamps[0]
    offset = float(offset_seconds or 0.0)
    if not math.isfinite(offset):
        raise ValueError("Replay offset must be finite")
    duration = max(0.0, end - start)
    offset = max(0.0, min(duration, offset))
    return {
        "session": os.path.basename(path),
        "offset_seconds": offset,
        "duration_seconds": duration,
        "monotonic": start + offset,
        "state": replay.state_at(start + offset),
    }


def poweroff_after_response():
    time.sleep(1)
    subprocess.run(
        ["sudo", "-n", "/usr/bin/systemctl", "poweroff"],
        check=False,
        timeout=10,
    )


def reboot_after_response():
    time.sleep(1)
    subprocess.run(
        ["sudo", "-n", "/usr/bin/systemctl", "reboot"],
        check=False,
        timeout=10,
    )


class CameraHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def _send_json(self, payload, status_code=200):
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status_code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _read_json(self):
        if hasattr(self, "_cached_json_payload"):
            return self._cached_json_payload
        content_length = int(self.headers.get("Content-Length", "0"))
        raw_body = self.rfile.read(content_length).decode("utf-8") if content_length else "{}"
        payload = json.loads(raw_body)
        if not isinstance(payload, dict):
            raise ValueError("JSON body must be an object")
        self._cached_json_payload = payload
        return payload

    def do_POST(self):
        if hasattr(self, "_cached_json_payload"):
            del self._cached_json_payload
        try:
            self._read_json()
        except (ValueError, json.JSONDecodeError, TypeError) as error:
            self._send_json({"error": str(error)}, 400)
            return

        if self.path == "/api/safety/emergency-stop":
            if auto_route_runtime.active:
                auto_route_runtime.stop("emergency_stop")
            vehicle_state_machine.transition(DriveMode.EMERGENCY_STOP, "dashboard_emergency_stop")
            motor_controller.emergency_stop()
            record_manager.stop()
            self._send_json(
                {
                    "state_machine": vehicle_state_machine.snapshot(),
                    "safety": safety_snapshot(),
                },
                202,
            )
            return

        if self.path == "/api/routes/process":
            try:
                payload = self._read_json()
                source = resolve_recording_path(payload.get("session"), "route.csv")
                output = resolve_recording_path(payload.get("session"), "processed_route.json")
                route = RouteProcessor().process_csv(source, output)
                self._send_json(
                    {
                        "processed_route": output,
                        "point_count": len(route.points),
                        "origin": route.origin,
                    },
                    202,
                )
            except (ValueError, OSError, json.JSONDecodeError) as error:
                self._send_json({"error": str(error)}, 400)
            return

        if self.path == "/api/auto-route/load":
            try:
                payload = self._read_json()
                path = resolve_recording_path(payload.get("session"), "processed_route.json")
                self._send_json(auto_route_runtime.load(path))
            except (ValueError, OSError, json.JSONDecodeError) as error:
                self._send_json({"error": str(error)}, 400)
            return

        if self.path == "/api/auto-route/start":
            try:
                result = auto_route_runtime.start()
                accepted = result.get("active", False)
                self._send_json(result, 202 if accepted else 409)
            except ValueError as error:
                self._send_json({"error": str(error)}, 409)
            except OSError as error:
                self._send_json({"error": f"Autonomous log could not start: {error}"}, 507)
            return

        if self.path == "/api/auto-route/stop":
            self._send_json(auto_route_runtime.stop())
            return

        if self.path == "/api/auto-hybrid/start":
            try:
                self._send_json(auto_route_runtime.enable_hybrid(), 202)
            except ValueError as error:
                self._send_json({"error": str(error)}, 409)
            return

        if self.path == "/api/auto-hybrid/stop":
            self._send_json(auto_route_runtime.disable_hybrid())
            return

        if self.path == "/api/throttle/calibration":
            try:
                payload = self._read_json()
                self._send_json(throttle_calibration.set_points(payload.get("points", [])), 202)
            except (ValueError, OSError, json.JSONDecodeError, TypeError) as error:
                self._send_json({"error": str(error)}, 400)
            return

        if self.path == "/api/camera/calibration/reload":
            if camera_calibration.load():
                self._send_json(camera_calibration.snapshot(), 202)
            else:
                self._send_json(camera_calibration.snapshot(), 409)
            return

        if self.path == "/api/recording/start":
            if vehicle_state_machine.mode != DriveMode.MANUAL_ASSIST:
                self._send_json({"error": "Recording requires MANUAL_ASSIST mode"}, 409)
                return
            try:
                result = record_manager.start(recording_metadata())
                vehicle_state_machine.transition(DriveMode.RECORD, "recording_started")
                self._send_json(
                    {"recording": result, "state_machine": vehicle_state_machine.snapshot()},
                    202,
                )
            except OSError as error:
                self._send_json({"error": str(error)}, 507)
            return

        if self.path == "/api/recording/stop":
            result = record_manager.stop()
            if vehicle_state_machine.mode == DriveMode.RECORD:
                vehicle_state_machine.transition(DriveMode.MANUAL_ASSIST, "recording_stopped")
            self._send_json(
                {"recording": result, "state_machine": vehicle_state_machine.snapshot()}
            )
            return

        if self.path == "/api/recordings/label":
            try:
                payload = self._read_json()
                self._send_json(
                    label_recording_session(payload.get("session"), payload.get("label")),
                    202,
                )
            except FileNotFoundError as error:
                self._send_json({"error": str(error)}, 404)
            except (ValueError, OSError, json.JSONDecodeError, TypeError) as error:
                self._send_json({"error": str(error)}, 400)
            return

        if self.path == "/api/recordings/delete":
            try:
                payload = self._read_json()
                self._send_json(delete_recording_session(payload.get("session")), 202)
            except FileNotFoundError as error:
                self._send_json({"error": str(error)}, 404)
            except RuntimeError as error:
                self._send_json({"error": str(error)}, 409)
            except (ValueError, OSError, TypeError) as error:
                self._send_json({"error": str(error)}, 400)
            return

        if self.path == "/api/recordings/route":
            try:
                payload = self._read_json()
                self._send_json(recording_route_points(payload.get("session")))
            except FileNotFoundError as error:
                self._send_json({"error": str(error)}, 404)
            except (ValueError, OSError, TypeError) as error:
                self._send_json({"error": str(error)}, 400)
            return

        if self.path == "/api/recordings/replay":
            try:
                payload = self._read_json()
                self._send_json(
                    recording_replay_state(
                        payload.get("session"),
                        payload.get("offset_seconds", 0.0),
                    )
                )
            except FileNotFoundError as error:
                self._send_json({"error": str(error)}, 404)
            except (ValueError, OSError, TypeError) as error:
                self._send_json({"error": str(error)}, 400)
            return

        if self.path == "/api/safety/reset":
            motor = motor_controller.snapshot()
            if motor.get("hardware_estop_supported") and motor.get("hardware_estop_active"):
                result = motor_controller.reset_hardware_estop()
                if result.get("estop_reset_requested"):
                    motor_controller.stop()
                    if vehicle_state_machine.mode in {
                        DriveMode.EMERGENCY_STOP,
                        DriveMode.FAULT,
                    }:
                        vehicle_state_machine.transition(
                            DriveMode.DISARMED,
                            "operator_safety_reset",
                        )
                self._send_json(
                    {
                        "state_machine": vehicle_state_machine.snapshot(),
                        "motor": result,
                    },
                    202 if result.get("estop_reset_requested") else 503,
                )
                return
            if vehicle_state_machine.mode not in {DriveMode.EMERGENCY_STOP, DriveMode.FAULT}:
                self._send_json({"error": "Safety reset is only valid after a stop or fault"}, 409)
                return
            motor_controller.stop()
            vehicle_state_machine.transition(DriveMode.DISARMED, "operator_safety_reset")
            self._send_json({"state_machine": vehicle_state_machine.snapshot()}, 202)
            return

        if self.path == "/api/ntrip/config":
            try:
                self._send_json(ntrip_client.configure(self._read_json()), 202)
            except (ValueError, json.JSONDecodeError, TypeError, OSError) as error:
                self._send_json({"error": str(error)}, 400)
            return

        if self.path == "/api/ntrip/stop":
            try:
                self._send_json(ntrip_client.stop())
            except OSError as error:
                self._send_json({"error": str(error)}, 500)
            return

        if self.path == "/api/network/wifi/connect":
            try:
                payload = self._read_json()
                self._send_json(
                    wifi_connect(
                        payload.get("ssid"),
                        payload.get("password"),
                        payload.get("security"),
                    ),
                    202,
                )
            except (ValueError, json.JSONDecodeError, TypeError) as error:
                self._send_json({"error": str(error)}, 400)
            except RuntimeError as error:
                self._send_json({"error": str(error)}, 409)
            return

        if self.path == "/api/network/wifi/disconnect":
            try:
                self._send_json(wifi_disconnect())
            except RuntimeError as error:
                self._send_json({"error": str(error)}, 409)
            return

        if self.path == "/api/system/poweroff":
            client_ip = self.client_address[0]
            allowed_client = (
                client_ip in {"127.0.0.1", "::1"}
                or client_ip.startswith("192.168.137.")
            )
            if not allowed_client:
                self._send_json({"error": "Power off is limited to the private control network"}, 403)
                return
            if self.headers.get("X-GNSS-Confirm") != "poweroff":
                self._send_json({"error": "Power off confirmation header is required"}, 400)
                return

            motor_controller.stop()
            self._send_json({"accepted": True, "message": "System power off scheduled"}, 202)
            threading.Thread(target=poweroff_after_response, daemon=True).start()
            return

        if self.path == "/api/system/reboot":
            client_ip = self.client_address[0]
            allowed_client = (
                client_ip in {"127.0.0.1", "::1"}
                or client_ip.startswith("192.168.137.")
            )
            if not allowed_client:
                self._send_json({"error": "Reboot is limited to the private control network"}, 403)
                return
            if self.headers.get("X-GNSS-Confirm") != "reboot":
                self._send_json({"error": "Reboot confirmation header is required"}, 400)
                return

            motor_controller.stop()
            self._send_json({"accepted": True, "message": "System reboot scheduled"}, 202)
            threading.Thread(target=reboot_after_response, daemon=True).start()
            return

        if self.path == "/api/imu/calibrate":
            started = imu_monitor.start_calibration()
            self._send_json(
                {
                    "started": started,
                    "imu": imu_status(),
                },
                202 if started else 409,
            )
            return

        if self.path in {"/api/imu/reset-relative-yaw", "/api/imu/zero-heading"}:
            reset = imu_monitor.reset_relative_yaw()
            self._send_json(
                {
                    "reset": reset,
                    "imu": imu_status(),
                },
                202 if reset else 409,
            )
            return

        if self.path == "/api/steering":
            try:
                payload = self._read_json()
                direction = payload.get("direction", 0)
                self._send_json(apply_safe_steering(direction))
            except (ValueError, json.JSONDecodeError, TypeError) as exc:
                self._send_json({"error": str(exc)}, 400)
            return

        if self.path == "/api/steering/zero":
            result = motor_controller.zero_steering_encoder()
            self._send_json(result, 202 if result["zero_requested"] else 503)
            return

        if self.path == "/api/steering/config":
            try:
                payload = self._read_json()
                result = motor_controller.configure_steering(
                    payload.get("right_raw"),
                    payload.get("center_raw"),
                    payload.get("left_raw"),
                    payload.get("allowance_raw"),
                )
                self._send_json(result, 202 if result["config_saved"] else 409)
            except (ValueError, json.JSONDecodeError, TypeError) as exc:
                self._send_json({"error": str(exc)}, 400)
            return

        if self.path == "/api/steering/center":
            result = motor_controller.center_steering()
            accepted = result["centering"] and result["steering_rejection"] is None
            self._send_json(result, 202 if accepted else 409)
            return

        if self.path == "/api/steering/stop":
            self._send_json(motor_controller.stop_steering())
            return

        if self.path == "/api/motor":
            try:
                payload = self._read_json()
                throttle = payload.get("throttle", 0)
                enabled = bool(payload.get("enabled", False))
                deadman = bool(payload.get("deadman", False))
                self._send_json(apply_safe_drive(throttle, enabled, deadman))
            except (ValueError, json.JSONDecodeError, TypeError) as exc:
                self._send_json({"error": str(exc)}, 400)
            return

        self.send_error(404)

    def do_GET(self):
        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
            self.send_header("Content-Length", str(len(INDEX_HTML)))
            self.end_headers()
            self.wfile.write(INDEX_HTML)
            return

        if self.path == "/assets/swing-logo-white.png":
            try:
                with open(LOGO_PATH, "rb") as logo_file:
                    body = logo_file.read()
            except OSError:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/png")
            self.send_header("Cache-Control", "public, max-age=86400")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if self.path == "/api/ntrip":
            self._send_json(ntrip_client.snapshot())
            return

        if self.path == "/api/network/wifi/scan":
            self._send_json(wifi_scan())
            return

        if self.path == "/api/imu":
            self._send_json(imu_status())
            return

        if self.path == "/api/lidar":
            self._send_json(lidar_monitor.snapshot())
            return

        if self.path == "/api/steering":
            self._send_json(motor_controller.snapshot())
            return

        if self.path == "/api/safety":
            self._send_json(
                {
                    "state_machine": vehicle_state_machine.snapshot(),
                    "safety": safety_snapshot(),
                }
            )
            return

        if self.path == "/api/recording":
            self._send_json(record_manager.snapshot())
            return

        if self.path == "/api/recordings":
            self._send_json(list_recording_sessions())
            return

        if self.path == "/api/auto-route":
            payload = auto_route_runtime.snapshot()
            if auto_route_runtime.planner is not None:
                payload["preflight"] = auto_route_runtime.preflight()
            self._send_json(payload)
            return

        if self.path == "/api/auto-hybrid":
            state = auto_route_runtime.snapshot()
            self._send_json(
                {
                    "active": state["hybrid_enabled"],
                    "lane": state["lane"],
                    "route_active": state["active"],
                    "opencv_available": auto_route_runtime.lane_controller.available,
                    "fallback_reason": state["hybrid_fallback_reason"],
                }
            )
            return

        if self.path == "/api/lane":
            try:
                frame, sequence, frame_monotonic, _ = camera.snapshot_frame()
                result = auto_route_runtime.lane_controller.analyze_jpeg(frame)
                payload = result.as_dict()
                payload["frame_sequence"] = sequence
                payload["data_age"] = (
                    time.monotonic() - frame_monotonic
                    if frame_monotonic is not None
                    else None
                )
                self._send_json(payload)
            except Exception as error:
                self._send_json({"error": f"{type(error).__name__}: {error}"}, 500)
            return

        if self.path == "/api/objects":
            self._send_json(perception_monitor.snapshot())
            return

        if self.path == "/api/camera/calibration":
            self._send_json(camera_calibration.snapshot())
            return

        if self.path == "/api/throttle/calibration":
            self._send_json(throttle_calibration.snapshot())
            return

        if self.path == "/api/sensors/health":
            self._send_json(sensor_health_snapshot())
            return

        if self.path == "/api/status":
            body = json.dumps(
                {
                    "system": system_status(),
                    "camera": {
                        "online": camera.frame is not None,
                        "device": DEVICE,
                        "size": SIZE,
                        "framerate": int(FRAMERATE),
                        "calibration": camera_calibration.snapshot(),
                    },
                    "devices": device_status(),
                    "network": network_status(self.client_address[0]),
                    "navigation": {
                        "mode": vehicle_state_machine.mode.value,
                        "motors_enabled": motor_controller.snapshot()["enabled"],
                        "throttle": motor_controller.snapshot()["throttle"],
                        "safety": safety_snapshot(),
                    },
                    "ntrip": ntrip_client.snapshot(),
                }
            ).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if self.path == "/health":
            body = b"ok\n" if camera.frame is not None else b"waiting for camera\n"
            self.send_response(200 if camera.frame is not None else 503)
            self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if self.path == "/snapshot.jpg":
            frame, _, _, _ = camera.snapshot_frame()
            if frame is None:
                self.send_error(503, "Camera frame unavailable")
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(frame)))
            self.end_headers()
            self.wfile.write(frame)
            return

        if self.path == "/stream.mjpg":
            self.send_response(200)
            self.send_header(
                "Content-Type", "multipart/x-mixed-replace; boundary=frame"
            )
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
            self.end_headers()
            sequence = -1

            try:
                while True:
                    frame, sequence = camera.wait_for_frame(sequence)
                    if frame is None:
                        continue
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(
                        f"Content-Length: {len(frame)}\r\n\r\n".encode("ascii")
                    )
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass
            return

        self.send_error(404)

    def log_message(self, format_string, *args):
        print(f"{self.client_address[0]} - {format_string % args}", flush=True)


if __name__ == "__main__":
    camera.start()
    gps_monitor.start()
    ntrip_client.start()
    imu_monitor.start()
    lidar_monitor.start()
    motor_controller.start()
    perception_monitor.start()
    server = ThreadingHTTPServer((HOST, PORT), CameraHandler)
    print(f"Camera stream listening on http://{HOST}:{PORT}", flush=True)
    server.serve_forever()
