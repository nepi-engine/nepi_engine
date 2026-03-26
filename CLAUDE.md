# nepi_engine — Developer Reference

## Purpose

`nepi_engine` is the core of the NEPI platform. It contains four ROS packages that together form the runtime backbone of every NEPI deployment: the API abstraction layer (`nepi_api`), the low-level SDK utilities (`nepi_sdk`), the suite of system management nodes (`nepi_managers`), and the launch and environment configuration package (`nepi_env`). All other submodules — drivers, apps, RUI, AI frameworks — depend on what is defined here.

## Architecture

```
nepi_engine/
├── nepi_api/           # Abstract base classes and interface definitions for NEPI nodes
│   ├── src/nepi_api/   # Python module: NodeConfigsIF, MsgIF, device interface classes
│   ├── scripts/        # Standalone node implementations (e.g., AI detector publisher)
│   └── setup.py / package.xml
│
├── nepi_managers/      # Nine ROS management nodes covering all platform domains
│   ├── scripts/        # One .py file per manager node
│   ├── api/            # ConnectMgrXxxIF classes for external code to query managers
│   ├── params/         # Per-manager default parameter YAML files
│   └── etc/            # Static configuration files loaded at node startup
│
├── nepi_env/           # ROS launch files and deployment utilities
│   ├── launch/         # nepi_base.launch — the primary system launch file
│   ├── etc/            # fw_version.txt and related static files
│   └── utilities/      # Setup, filesystem prep, and API table generation scripts
│
└── nepi_sdk/           # Low-level SDK: ROS wrappers, system utilities, base classes
    ├── src/nepi_sdk/   # Python module (nepi_sdk, nepi_utils, nepi_system, etc.)
    └── scripts/        # Utility scripts and the automation_script_template.py
```

## How It Works

The engine starts via `nepi_env/launch/nepi_base.launch`. This launch file sets a device namespace using `ROOTNAME` and `DEVICE_ID` environment variables, loads `system_mgr` parameters from `/opt/nepi/nepi_engine/etc/system_mgr.yaml`, and brings up `system_mgr` and `config_mgr` as the first two nodes. `rosbridge_websocket` (port 9090) and `web_video_server` (port 9091) are started optionally via launch args.

After the base launch, the remaining manager nodes are started. Each manager initializes via `nepi_sdk.init_node()`, creates a `MsgIF` for centralized logging, waits for system folders via `nepi_system.get_system_folders()`, then publishes a status message on a 1-second timer and provides query services. The nine managers and their domains:

| Node file | Domain |
|---|---|
| `system_mgr.py` | Master system status, disk monitoring, run mode, warnings |
| `config_mgr.py` | ROS param server ↔ filesystem bridge (factory/system/user config tiers) |
| `drivers_mgr.py` | Hardware driver discovery, node lifecycle, device aliasing |
| `apps_mgr.py` | Application installation, startup, status |
| `ai_models_mgr.py` | AI framework discovery, active model tracking |
| `navpose_mgr.py` | GPS, IMU, and nav/pose integration with TF2 transforms |
| `network_mgr.py` | IP address management, WiFi AP control, bandwidth throttling |
| `time_mgr.py` | NTP via Chrony, timezone management |
| `scripts_mgr.py` | Automation script execution and process monitoring |

Data flows outward from `system_mgr` (which establishes the device namespace and system folder locations) through the other managers and into drivers, apps, and the RUI. The `nepi_api` layer provides the interface classes (`NodeConfigsIF`, `MsgIF`, device-type `*IF` classes) that driver and app nodes instantiate. The `nepi_sdk` package provides the underlying ROS primitives — `create_subscriber()`, `create_publisher()`, `create_service()`, `wait_for_topic()`, `start_timer_process()` — that all nodes call instead of calling `rospy` directly.

## ROS Interface

All topics and services live under the device namespace: `/<ROOTNAME>/<DEVICE_ID>/`.

**Topics published (per manager, 1 Hz latched status):**
- `.../system_mgr/status` — `MgrSystemStatus`
- `.../ai_models_mgr/status` — `MgrAiModelsStatus`
- `.../drivers_mgr/status` — `MgrDriversStatus`
- `.../apps_mgr/status` — `MgrAppsStatus`
- `.../network_mgr/status` — `MgrNetworkStatus`
- `.../navpose_mgr/status` — `MgrNavPoseStatus`
- `.../time_sync_mgr/status` — `MgrTimeStatus`

**Services provided (per manager, query pattern):**
- `.../system_status_query` — `SystemStatusQuery`
- `.../ai_models_status_query` — `AiModelStatusQuery`
- `.../drivers_status_query` — `DriverStatusQuery`
- `.../apps_status_query` — `AppStatusQuery`
- `.../navpose_query` — `NavPoseQuery`
- `.../time_status_query` — `TimeStatusQuery`
- `.../get_scripts_query`, `.../launch_script`, `.../stop_script` — scripts_mgr control
- `.../network_mgr/wifi_query`, `.../ip_query`, `.../bandwidth_query`

All message types come from `nepi_interfaces`.

## Build and Dependencies

Built with `catkin_make` or `catkin build` as part of the `nepi_engine_ws` workspace. No separate build step is required for this submodule alone — build from the workspace root.

Runtime system dependencies (must be present on the target device):
- ROS 1 (rospy, roslib, geometry_msgs, sensor_msgs, std_msgs, nav_msgs)
- `nepi_interfaces` (custom messages/services — built as part of workspace)
- `rosbridge_suite` — for the websocket bridge used by the RUI
- `web_video_server` — for HTTP image streaming
- `chrony` — for NTP time synchronization (time_mgr)
- `wondershaper` at `/opt/nepi/nepi_engine/share/wondershaper/` — for bandwidth throttling (network_mgr)
- Python packages: `numpy`, `cv2`, `pytz`, `requests`, `psutil`, `docker`

Key filesystem paths expected at runtime:
- `/opt/nepi/nepi_engine/etc/fw_version.txt` — firmware version
- `/opt/nepi/nepi_engine/etc/system_mgr.yaml` — system_mgr parameter file
- `/opt/nepi/etc/chrony/chrony.conf` — Chrony NTP config
- `/mnt/nepi_config/factory_cfg/` — factory configuration tier
- `/mnt/nepi_config/system_cfg/` — system configuration tier
- `/mnt/nepi_storage/user_cfg/` — user configuration tier
- `/mnt/nepi_storage/ai_models/` — AI model storage
- `/mnt/nepi_storage/nepi_scripts/` — automation script storage

## Naming Conventions

Python functions and methods in `nepi_api` follow the NEPI convention:

- **Public API:** `snake_case` — `goto_tilt_ratio`, `get_ready_state`, `publish_status`. Receives docstrings.
- **Private:** `_camelCase` — `_initCb`, `_resetCb`, `_publishStatusCb`. No docstrings.
- **`Cb` suffix** — indicates a ROS callback (subscriber, publisher, or timer). Carries rename risk; audit external call sites before renaming.

## Known Constraints and Fragile Areas

**Initialization ordering is strict.** `system_mgr` must be running and publishing its status before any other manager can initialize. `config_mgr` waits for `system_mgr`. Drivers, apps, and other managers wait for config_mgr. If `system_mgr` hangs or fails, the entire platform fails to come up. Watch for this during debugging.

**`eth0` is hardcoded in `network_mgr`.** On hardware with a different interface name, network_mgr will not function correctly.

**Thread safety in manager dicts.** Managers use `threading.Lock()` to protect shared dictionaries, but lock acquisition must be explicit. `drivers_mgr` in particular manages several dicts (`drivers_running_dict`, `devices_alias_dict`, `settings_dict`) that have concurrent access from discovery threads and status publication. Review lock boundaries before modifying manager internals.

**`wait_for_topic()` has no guaranteed timeout.** Nodes that call this will hang indefinitely if the expected topic never appears. Startup failures in one component can cascade to all downstream components.

**Config file corruption is unrecoverable without intervention.** `config_mgr` reads and writes YAML files to `/mnt/nepi_config/`. A corrupted file at startup will crash the node with no automatic recovery.

**`scripts_mgr` has a 10-second stop timeout.** Automation scripts that ignore SIGTERM will be killed after 10 seconds. No zombie-process cleanup is explicitly documented beyond this timeout.

**Disk monitoring uses a fixed margin.** `system_mgr` checks for disk full using `DISK_FULL_MARGIN_MB = 250`. No automatic cleanup is performed when the threshold is crossed — only a warning is published.

**Container environment.** `time_mgr` has an `in_container` flag that bypasses some Chrony behavior. If running in Docker, time synchronization behavior differs from bare-metal deployment.

## Decision Log

- 2026-03 — CLAUDE.md created — Initial developer reference, Claude Code authoring pass.
