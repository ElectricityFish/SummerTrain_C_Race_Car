# COM11 speed-loop tuning protocol

This file documents the temporary tuning transport used only on the speed-loop debug branch.

## Safety sequence

1. Put CH5 low, place the car at the start of the three-metre straight, and keep the path clear.
2. Send `CFG`, then `ARM`. `ARM` is accepted only while the receiver is online and CH5 is low.
3. Move CH5 high. Send `RUN` within 30 seconds.
4. During a run the host sends `HB` every 100 ms. Missing heartbeats for 300 ms stops the motors.
5. Time, accumulated encoder pulses, CH5 low, receiver loss, attitude, overspeed, direction faults, or `STOP` all end the run.
6. After `DONE`, put CH5 low before repositioning the car.

The firmware stops a tuning run by disabling the speed loop, clearing PI state, setting both motor PWM commands to zero, and returning to IDLE. It does not command a zero-speed closed loop, so the car coasts instead of applying reverse PI braking.

## Frame format

Host commands are ASCII payloads followed by `*HH\n`, where `HH` is the two-digit hexadecimal XOR of every payload byte.

Gains use thousandths to avoid floating-point parsing on the MCU. For example, `2500` means `2.500`.

```text
HELLO,seq
CFG,seq,kpL_milli,kiL_milli,kpR_milli,kiR_milli,targetL,targetR,maxPwmL,maxPwmR
ARM,seq
RUN,seq,max_time_ms,max_average_accumulated_pulses
HB,seq
STOP,seq
GET,seq
```

Responses do not carry a checksum:

```text
ACK,seq,command
ERR,seq,reason
HELLO,seq,SPEED_TUNE,protocol_version,115200
STATE,seq,configured,armed,running,ch5_low,ch5_high,kpL_milli,kiL_milli,kpR_milli,kiR_milli,targetL,targetR,maxPwmL,maxPwmR
D,run_id,tick,LT,LA,LO,LE,LI,RT,RA,RO,RE,RI
DONE,run_id,reason,elapsed_ms,accumulated_pulses,dropped_samples
```

## Host script

Probe without arming the motors:

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\speed_tune_com11.ps1 -Action Probe
```

Initial direction and interlock test:

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\speed_tune_com11.ps1 `
  -Action Run -LeftKp 2.0 -RightKp 2.0 -LeftKi 0 -RightKi 0 `
  -LeftTarget 80 -RightTarget 80 -LeftMaxPwm 2000 -RightMaxPwm 2000 `
  -DurationMs 300 -PulseLimit 3000
```

The script refuses to arm unless CH5 is low. After it prints `ARMED_WAIT_CH5_HIGH`, move CH5 high. It then starts automatically, saves CSV plus a JSON summary under `tools/speed_tune_logs`, and prints `RUN_FINISHED_SET_CH5_LOW` when finished.
