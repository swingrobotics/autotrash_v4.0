# AUTO_LOCAL

`AUTO_LOCAL` is the GPS-independent saved-map navigation mode for indoor, underground and GNSS-shadow environments.

## Runtime architecture

```text
LiDAR + IMU
  -> scan matching / localization
  -> saved sparse occupancy map
  -> inflated-grid A*
  -> local path following
  -> local obstacle avoidance / replan
  -> person STOP
  -> SafetySupervisor
  -> steering + throttle
```

The local map frame is defined by mapping start and stores the initial heading context so later sessions can express pose in the same map frame.

## Create a map

1. Select `MANUAL`.
2. Create/select a map from the advanced dashboard.
3. Start mapping.
4. Drive manually through the usable area.
5. Revisit observed areas when practical to improve localization/mapping consistency.
6. Stop/save mapping.

Mapping observes LiDAR + IMU. It does not take over manual steering/throttle.

Generated maps are runtime data under `maps/` and are ignored by Git.

## Refine an existing map

Refinement first attempts global relocalization against the saved occupancy map. Weak localization should not be used to inject uncertain updates into the map.

## Destinations

A localized mapping/AUTO_LOCAL session can save named destinations as local map coordinates, for example:

- entrance
- loading_dock
- charger
- workbench_a

## AUTO_LOCAL preflight

Before motion, AUTO_LOCAL requires the relevant runtime conditions, including:

- saved map asset
- selected destination
- usable LiDAR
- fresh IMU heading
- successful global localization
- a valid planned path
- Arduino/steering/common safety readiness

If required preflight cannot be satisfied, motion is refused.

## Driving behavior

Normal operation:

1. maintain pose with LiDAR/IMU localization
2. follow the saved-map path
3. use eligible bounded lane correction only as an assist where available
4. handle local obstacles with deterministic side-clearance/replan logic
5. force STOP for person hazard
6. stop when no safe bypass exists
7. stop immediately on localization loss and fault if loss persists

AUTO_LOCAL is designed to work without lane markings; lane observations are not the primary localization source.

## Top-level AUTO relationship

Top-level AUTO selects conservatively. AUTO_LOCAL is considered when a selected map/destination can localize and plan. A safety fault does not silently reset and jump to a different strategy.

The overall selector may also consider eligible GPS AI, route-independent AI and internal `PRETRAINED_ROAD` according to their own readiness/lifecycle gates.

## Validation

Software regression:

```bash
python3 -m autonomous_car.simulation.validate_auto_local_v2
```

Before field use, validate at low speed in a closed area:

- global relocalization repeatability
- map drift after repeated loops
- destination path reachability
- steering tracking
- obstacle side-clearance assumptions
- temporary bypass/replan
- localization-loss stop/fault
- person STOP
- E-STOP/watchdogs

See [../validation/FIELD_TEST.md](../validation/FIELD_TEST.md) for the current physical validation sequence.
