# trajectory_loader
A node for loading a trajectory from a CSV file and publishing it.

## Installation

```bash
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=On --packages-up-to trajectory_loader
```

## Usage

```bash
ros2 launch trajectory_loader trajectory_loader.launch.py csv_path:=<path_to_csv_file>
```

You can check example trajectory file in `data/trajectory.csv`.

## API

### Output

| Name                  | Type                                         | Description         |
| --------------------- | -------------------------------------------- | ------------------- |
| `~/output/trajectory` | autoware_auto_planning_msgs::msg::Trajectory | Vehicle trajectory. |


### Parameters

| Name               | Type   | Description                                         |
| ------------------ | ------ | --------------------------------------------------- |
| `csv_path`         | string | Path to trajectory file.                            |
| `update_rate`      | float  | Trajectory publisher frequency.                     |
| `delimiter`        | string | CSV file delimiter.                                 |
| `is_header`        | bool   | If CSV file contains header.                        |
| `yaw_offset`       | float  | Yaw offset.                                         |
| `col_x`            | int    | X coordinate column index.                          |
| `col_y`            | int    | Y coordinate column index.                          |
| `col_yaw`          | int    | Yaw column index.                                   |
| `col_vel`          | int    | Velocity column index.                              |


## References / External links
[How to generate racing trajectory](https://github.com/amadeuszsz/Raceline-Optimization)