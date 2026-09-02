# Autonomous Agri-Aerial Swarm — Implementation Architecture

## 1. System overview

Five layers, one per drone, plus a shared ground/swarm layer:

```
Perception -> Onboard processing (Jetson) -> Flight controller (Pixhawk) -> Actuation (spray)
                        |                              |
                        v                              v
                 Ground station (QGIS/ArcGIS)     Swarm mesh (other drones)
```

Each drone is a self-contained perception-to-actuation loop; the mesh network
and ground station exist for coordination and mapping, not as a control
dependency — a drone with a lost mesh link still flies its assigned grid
strip and makes local spray decisions.

## 2. Hardware architecture (per drone)

| Subsystem | Component | Interface |
|---|---|---|
| Airframe | Quadcopter frame, sized for spray tank + battery payload | — |
| Perception | RGB camera + NoIR camera (720nm+ filter) for NIR | CSI/USB -> Jetson |
| Onboard compute | Jetson Nano (companion computer) | Runs perception + decision stack |
| Flight control | Pixhawk (PX4 or ArduPilot firmware) | MAVLink over UART/USB to Jetson |
| Positioning | GPS module (+ optional RTK base station) | Serial -> Pixhawk |
| Obstacle sensing | Ultrasonic / LiDAR rangefinder | Serial/I2C -> Pixhawk (DISTANCE_SENSOR) |
| Actuation | Solenoid micro-nozzles, PWM-modulated pump, flow sensor | GPIO/PWM, driven by an Arduino/ESP32 spray controller over serial from Jetson |
| Swarm comms | LoRa or Wi-Fi mesh radio | Serial/SPI -> Jetson |
| Power | LiPo battery, separate rails for flight vs. compute/spray | — |

## 3. Software architecture (per layer)

### 3.1 Perception -> NDVI (`ndvi_camera_loop.py`)
- Grabs synced RGB + NIR frames.
- `compute_ndvi()`: NDVI = (NIR − RED) / (NIR + RED).
- `find_stress_regions()`: thresholds + connected-component analysis -> list of `StressRegion` (pixel centroid, NDVI value, area).
- `pixel_to_gps()`: converts stressed-pixel centroids to lat/lon using altitude + camera HFOV (pinhole ground-footprint approximation).

### 3.2 Geotagging and mapping (`geotag_images.py`, `ndvi_geotiff_export.py`, `qgis_ndvi_pipeline.py`)
- `geotag_images.py`: reads GPS from MAVLink (`GLOBAL_POSITION_INT`) or a standalone NMEA module, embeds EXIF GPS tags on captured frames, appends every point to `ndvi_log.csv`.
- `ndvi_geotiff_export.py`: builds a per-frame affine geotransform from (lat, lon, altitude, heading) and camera HFOV, writes each NDVI frame as a georeferenced GeoTIFF, and merges frames into a running `mosaic.tif` — loadable directly in QGIS or ArcGIS Pro.
- `qgis_ndvi_pipeline.py`: loads `mosaic.tif`, applies a red→green NDVI ramp, reclassifies below-threshold pixels, polygonizes stress zones, and exports `spray_targets.geojson` (centroid, mean NDVI, area per zone). ArcPy equivalent (Reclassify → RasterToPolygon → FeatureToPoint) is included as a drop-in alternative.

### 3.3 Onboard smart decisions (`jetson_decision_engine.py`)
Runs continuously on the Jetson, fusing two input streams:
- **Perception**: `StressRegion` list from the NDVI loop.
- **Telemetry**: `Telemetry` (battery %, GPS satellite count, obstacle distance, position) pulled from MAVLink.

Decision logic (`decide()`):
1. `safety_check()` runs first and can unconditionally override perception:
   - battery < 20% -> `ABORT_RTL`
   - GPS satellites < 6 -> `HOLD` (don't trust position-tagged spray)
   - obstacle < 2 m -> `HOLD`
   - outside geofence radius -> `ABORT_RTL`
2. If safety passes, NDVI severity decides `SPRAY` vs `SKIP`.
3. If `SPRAY`, `dose_for_ndvi()` maps NDVI value to solenoid-open duration via a piecewise-linear dose curve — proportional dosing, not on/off.

Output: a serial command (`SPRAY <ms>` / `STOP`) to the spray micro-controller, plus a logged, geotagged decision record.

### 3.4 Flight control and coverage
- Field pre-divided into a uniform grid (slicer-style); each drone is assigned one strip and flies it as a boustrophedon (lawn-mower) waypoint mission on the Pixhawk (PX4/ArduPilot mission mode).
- `spray_targets.geojson` from the QGIS/ArcGIS pipeline can be converted into additional waypoints for a second, targeted pass over confirmed stress zones.

### 3.5 Swarm communication
- LoRa (long range, low bandwidth) or Wi-Fi mesh (shorter range, higher bandwidth) links drones for zone-split coordination: each drone broadcasts its assigned strip and completion status so no two drones re-cover the same ground, and one drone can scout ahead while another sprays behind it.
- Ground station receives the same telemetry/NDVI stream for live monitoring in QGIS/ArcGIS.

## 4. Data flow, end to end

1. Camera pair captures RGB+NIR frame on the Jetson.
2. NDVI computed; stressed regions extracted with pixel centroids.
3. Jetson reads current telemetry (position, battery, obstacle, sats) over MAVLink.
4. `decide()` fuses NDVI + telemetry -> `SPRAY` / `SKIP` / `HOLD` / `ABORT_RTL` + dose.
5. Spray command sent to the nozzle controller; decision + GPS + NDVI logged (`ndvi_log.csv`).
6. Frame also exported as a georeferenced GeoTIFF tile and merged into `mosaic.tif`.
7. Ground station's QGIS/ArcGIS session (re)loads `mosaic.tif` for the live stress map and periodically reruns the polygonize step to refresh `spray_targets.geojson`.
8. Swarm mesh exchanges zone-completion status between drones; ground station displays combined swarm progress.

## 5. Safety and failure handling

| Condition | Response |
|---|---|
| Battery < 20% | Abort mission, return-to-launch |
| GPS satellites < 6 | Hold spray decisions (position untrustworthy) — continue flying pre-planned waypoints |
| Obstacle < 2 m | Hold spray, rely on Pixhawk's own avoidance/hold behavior |
| Outside geofence | Abort, return-to-launch |
| Mesh link lost | Continue local mission and local spray decisions; resync zone status on reconnect |
| Camera desync (RGB/NIR) | Skip NDVI computation for that frame rather than compute on misaligned pixels |

## 6. File map

| File | Role |
|---|---|
| `ndvi_camera_loop.py` | Perception: NDVI computation, stress-region extraction |
| `geotag_images.py` | GPS EXIF tagging + CSV logging |
| `ndvi_geotiff_export.py` | Georeferenced GeoTIFF export + field mosaic |
| `qgis_ndvi_pipeline.py` | QGIS/ArcGIS stress-zone extraction -> spray targets |
| `jetson_decision_engine.py` | Fused perception + telemetry -> spray/hold/abort decisions, actuator control |
