"""Protect manual-control heartbeat from dashboard and RECORD best-effort work.

The Raspberry Pi keeps the motor watchdog unchanged. This module reduces false
watchdog trips by making browser motor POSTs latest-command-wins, pacing
non-critical dashboard reads while manual drive is armed, and measuring
server-side command-arrival gaps.

This layer never owns actuator authority. It wraps the existing
``legacy.apply_safe_drive`` function, which still runs SafetySupervisor and the
motor watchdog contract.
"""

from __future__ import annotations

import json
import os
import threading
import time


class ManualControlTimingMonitor:
    """Measure arrival gaps between enabled manual-drive commands."""

    def __init__(self, watchdog_seconds=0.30):
        self.lock = threading.RLock()
        self.watchdog_seconds = max(0.05, float(watchdog_seconds))
        self.last_arrival_monotonic = None
        self.last_gap_seconds = None
        self.maximum_gap_seconds = 0.0
        self.gaps_over_200ms = 0
        self.gaps_over_watchdog = 0
        self.enabled = False
        self.commands = 0
        self.last_watchdog_risk_monotonic = None

    def note(self, enabled):
        now = time.monotonic()
        with self.lock:
            gap = None
            watchdog_risk = False
            if self.enabled and self.last_arrival_monotonic is not None:
                gap = max(0.0, now - self.last_arrival_monotonic)
                self.last_gap_seconds = gap
                self.maximum_gap_seconds = max(self.maximum_gap_seconds, gap)
                if gap > 0.20:
                    self.gaps_over_200ms += 1
                if gap > self.watchdog_seconds:
                    self.gaps_over_watchdog += 1
                    self.last_watchdog_risk_monotonic = now
                    watchdog_risk = True

            self.commands += 1
            self.enabled = bool(enabled)
            self.last_arrival_monotonic = now if self.enabled else None
            return {
                "arrival_monotonic": now,
                "gap_seconds": gap,
                "watchdog_risk": watchdog_risk,
                "watchdog_seconds": self.watchdog_seconds,
            }

    def snapshot(self):
        with self.lock:
            age = (
                None
                if self.last_arrival_monotonic is None
                else max(0.0, time.monotonic() - self.last_arrival_monotonic)
            )
            return {
                "enabled": self.enabled,
                "commands": self.commands,
                "watchdog_seconds": self.watchdog_seconds,
                "last_gap_seconds": self.last_gap_seconds,
                "maximum_gap_seconds": self.maximum_gap_seconds,
                "gaps_over_200ms": self.gaps_over_200ms,
                "gaps_over_watchdog": self.gaps_over_watchdog,
                "last_command_age_seconds": age,
                "last_watchdog_risk_monotonic": self.last_watchdog_risk_monotonic,
            }


def install_manual_control_priority(legacy):
    """Wrap the existing manual drive entrypoint without changing Safety policy."""

    existing = getattr(legacy, "_swing_manual_control_timing_monitor", None)
    if existing is not None:
        return existing

    monitor = ManualControlTimingMonitor(
        getattr(legacy, "MOTOR_TIMEOUT_SECONDS", 0.30)
    )
    original = legacy.apply_safe_drive

    def apply_safe_drive_with_timing(throttle, enabled, deadman=False):
        timing = monitor.note(enabled)
        result = original(throttle, enabled, deadman)
        if isinstance(result, dict):
            result = dict(result)
            result["manual_control_timing"] = {
                **monitor.snapshot(),
                "current_arrival_gap_seconds": timing.get("gap_seconds"),
                "current_watchdog_risk": timing.get("watchdog_risk", False),
            }
        return result

    apply_safe_drive_with_timing._swing_manual_control_priority = True
    apply_safe_drive_with_timing._swing_original = original
    legacy.apply_safe_drive = apply_safe_drive_with_timing
    legacy._swing_manual_control_timing_monitor = monitor

    handler = getattr(legacy, "CameraHandler", None)
    if handler is not None:
        original_get = handler.do_GET
        if not getattr(original_get, "_swing_manual_control_timing", False):
            def do_get_with_manual_control_timing(self):
                path = str(self.path or "").split("?", 1)[0]
                if path == "/api/manual-control/timing":
                    self._send_json(monitor.snapshot())
                    return
                return original_get(self)

            do_get_with_manual_control_timing._swing_manual_control_timing = True
            handler.do_GET = do_get_with_manual_control_timing

    record_manager = getattr(legacy, "record_manager", None)
    if record_manager is not None and not getattr(
        record_manager, "_swing_priority_metrics_stop", False
    ):
        original_stop = record_manager.stop

        def stop_with_priority_metrics(*args, **kwargs):
            result = original_stop(*args, **kwargs)
            try:
                session_path = str(
                    (result or {}).get("session_path")
                    or getattr(record_manager, "session_path", "")
                    or ""
                )
                if session_path and os.path.isdir(session_path):
                    path = os.path.join(session_path, "recording_runtime.json")
                    temporary = path + ".tmp"
                    document = {
                        "recording": dict(result or {}),
                        "manual_control_timing": monitor.snapshot(),
                        "saved_wall_time": time.time(),
                    }
                    with open(temporary, "w", encoding="utf-8") as file:
                        json.dump(document, file, ensure_ascii=False, indent=2)
                        file.flush()
                        os.fsync(file.fileno())
                    os.replace(temporary, path)
            except Exception:
                # Diagnostics persistence must never turn RECORD stop into a
                # control-path failure.
                pass
            return result

        stop_with_priority_metrics._swing_original = original_stop
        record_manager.stop = stop_with_priority_metrics
        record_manager._swing_priority_metrics_stop = True

        original_run = record_manager._run

        def run_with_error_cleanup(*args, **kwargs):
            try:
                return original_run(*args, **kwargs)
            finally:
                if getattr(record_manager, "error", None) and not getattr(
                    record_manager, "active", False
                ):
                    try:
                        record_manager.stop()
                    except Exception:
                        pass

        run_with_error_cleanup._swing_original = original_run
        record_manager._run = run_with_error_cleanup
        record_manager._swing_priority_error_cleanup = True
    return monitor


MANUAL_CONTROL_HARDENING = r'''
<script id="manual-control-priority-hardening">
(function(){
  if(window.__swingManualControlPriorityInstalled)return;
  window.__swingManualControlPriorityInstalled=true;

  const nativeFetch=window.fetch.bind(window);
  let manualDriveActive=false;
  let motorInFlight=false;
  let motorPending=null;
  const pacedGets=new Map();

  function requestPath(input){
    try{
      const value=typeof input==='string'?input:input?.url;
      return new URL(value,window.location.href).pathname;
    }catch(_error){return ''}
  }

  function requestMethod(input,options){
    return String(options?.method||(typeof input!=='string'&&input?.method)||'GET').toUpperCase()
  }

  function motorPayload(options){
    try{
      const body=options?.body;
      if(typeof body==='string')return JSON.parse(body)
    }catch(_error){}
    return null
  }

  function settleMotorJob(job,response,error){
    const waiters=job?.waiters||[];
    if(error){
      waiters.forEach(waiter=>waiter.reject(error));
      return;
    }
    waiters.forEach(waiter=>waiter.resolve(response.clone()));
  }

  async function drainMotorQueue(){
    if(motorInFlight)return;
    motorInFlight=true;
    try{
      while(motorPending){
        const job=motorPending;
        motorPending=null;
        try{
          const response=await nativeFetch(job.input,job.options);
          settleMotorJob(job,response,null);
        }catch(error){
          settleMotorJob(job,null,error);
        }
      }
    }finally{
      motorInFlight=false;
      if(motorPending)queueMicrotask(drainMotorQueue);
    }
  }

  function latestMotorFetch(input,options){
    const payload=motorPayload(options);
    if(payload&&Object.prototype.hasOwnProperty.call(payload,'enabled')){
      manualDriveActive=Boolean(payload.enabled);
    }
    return new Promise((resolve,reject)=>{
      if(motorPending){
        // Preserve caller promises but replace the unsent command with the
        // newest joystick state. Stale throttle commands never build a queue.
        motorPending.input=input;
        motorPending.options=options;
        motorPending.waiters.push({resolve,reject});
      }else{
        motorPending={input,options,waiters:[{resolve,reject}]};
      }
      drainMotorQueue();
    });
  }

  const activeCadenceMs={
    '/api/lidar':500,
    '/api/status':1500,
    '/api/safety':500,
    '/api/recording':1000,
    '/api/recordings':2000,
    '/api/auto-route':1500,
    '/api/throttle/calibration':3000,
    '/api/lane':400
  };

  function pacedGet(input,options,path,minimumIntervalMs){
    let state=pacedGets.get(path);
    if(!state){
      state={lastStarted:0,inFlight:null};
      pacedGets.set(path,state);
    }
    if(state.inFlight){
      return state.inFlight.then(response=>response.clone());
    }

    const now=performance.now();
    const wait=Math.max(0,minimumIntervalMs-(now-state.lastStarted));
    state.inFlight=(async()=>{
      if(wait>0)await new Promise(resolve=>setTimeout(resolve,wait));
      state.lastStarted=performance.now();
      return nativeFetch(input,options);
    })();

    const shared=state.inFlight;
    shared.then(
      ()=>{if(state.inFlight===shared)state.inFlight=null},
      ()=>{if(state.inFlight===shared)state.inFlight=null}
    );
    return shared.then(response=>response.clone());
  }

  window.fetch=function(input,options){
    const path=requestPath(input);
    const method=requestMethod(input,options);

    if(method==='POST'&&path==='/api/motor'){
      return latestMotorFetch(input,options||{});
    }

    // Emergency and actuator writes are never delayed. Only best-effort reads
    // are paced, and only while manual propulsion is armed.
    if(method==='GET'&&manualDriveActive){
      const cadence=activeCadenceMs[path];
      if(cadence)return pacedGet(input,options,path,cadence);
    }
    return nativeFetch(input,options);
  };

  window.__swingManualControlPriorityState=()=>({
    manualDriveActive,
    motorInFlight,
    motorPending:Boolean(motorPending),
    pacedEndpoints:[...pacedGets.keys()]
  });
})();
</script>
'''.encode("utf-8")


__all__ = [
    "MANUAL_CONTROL_HARDENING",
    "ManualControlTimingMonitor",
    "install_manual_control_priority",
]
