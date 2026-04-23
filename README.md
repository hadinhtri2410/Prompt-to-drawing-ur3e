# Prompt to Drawing

A ROS2 pipeline that converts a natural-language prompt (e.g. `"draw a plane"`) into a physical drawing executed by a UR3e robotic arm. The system uses a learned embedding network to predict placement parameters, analytic inverse kinematics to convert 2D waypoints to 6-DOF joint trajectories, and a custom `ros2_control` plugin to execute them on the real robot.

---

## Code Authorship

### Written by us (ha149 / triha)

| File | Description |
| --- | --- |
| `drawing_pipeline/src/primitives.py` | `DrawingProgram` class, `sample_program()`, `apply_params()` — entirely original |
| `drawing_pipeline/src/templates.py` | All 6 shape templates (plane, bike, face, square, circle, lissajous) — entirely original |
| `drawing_pipeline/src/generate_data.py` | Dataset generation with physics-based rejection sampling — entirely original |
| `drawing_pipeline/src/train.py` | `LabelToParams` model, quality-filtered `ParamDataset`, wandb logging — entirely original |
| `drawing_pipeline/src/run_from_prompt.py` | End-to-end prompt → CSV → RViz launch entry point — entirely original |
| `drawing_pipeline/src/eval_label_detection.py` | Label detection accuracy, IK quality, embedding similarity, per-param MSE evaluation — entirely original |
| `drawing_pipeline/src/eval_drawing_quality.py` | FK-based drawing quality evaluation: Chamfer distance, Hausdorff distance, rasterized IoU — entirely original |
| `csv_controller/config/ur_controllers.yaml` | Controller config with `$(var csv_name)` launch substitution — original |

### Adapted from prior code (with edits)

| File | Original Source | What changed | Lines edited |
| --- | --- | --- | --- |
| `drawing_pipeline/src/ik_solver.py` | ECE569 course code (Modern Robotics), Purdue University — functions prefixed `ECE569_*` | Added UR3e constants (`M_HOME`, `S_AXES`, `B_AXES`, `THETA0`), `solve_trajectory()`, and `export_csv()` | Lines 1–171: unchanged course code. Lines 173–263: new. |
| `py_joint_pub/py_joint_pub/joint_publisher_csv.py` | Lab starter code from `ldihel` | Changed hardcoded filename to a ROS2 `csv_path` parameter; fixed numpy→list type error (`.tolist()`) | Lines 17–22, 32 |
| `msee22_description/launch/move_robot.launch.py` | Lab starter code from `ldihel` | Added `csv_path` launch argument and forwarded it as a node parameter | Lines 19–27, 57 |
| `csv_controller/launch/ur3e_csv.launch.py` | [UR ROS2 Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver) example launch file | Added `csv_name` launch argument and `csv_controller` spawner node | Lines 178–193, 473–479 |

### Copied from external repositories (unmodified)

| File / Directory | Source |
| --- | --- |
| `drawing_pipeline/src/ik_solver.py` lines 1–171 | ECE569 course code (Modern Robotics), Purdue University |
| `msee22_description/urdf/`, `table_description/urdf/`, `wall_description/urdf/` | Lab URDF assets from `ldihel` |
| `csv_controller/include/`, `csv_controller/src/` | Based on `ros2_control` plugin template |

---

## Code Structure

```
prompt_to_drawing/
├── csv_controller/              # C++ ros2_control plugin — runs on real UR3e
│   ├── src/                     # Controller implementation
│   ├── include/                 # Header files
│   ├── config/ur_controllers.yaml
│   ├── launch/ur3e_csv.launch.py
│   └── csv_files/               # Output CSV trajectories (written at runtime)
│
├── drawing_pipeline/            # Python ML + IK pipeline
│   └── src/
│       ├── primitives.py        # DrawingProgram, sample_program, apply_params
│       ├── templates.py         # 6 shapes: plane, bike, face, square, circle, lissajous
│       ├── ik_solver.py         # UR3e FK/IK + solve_trajectory + export_csv
│       ├── generate_data.py     # Synthetic dataset generation with rejection sampling
│       ├── train.py             # LabelToParams model, training loop, wandb logging
│       ├── run_from_prompt.py   # Entry point: prompt → CSV → RViz
│       ├── eval_label_detection.py   # Evaluates label detection accuracy + IK quality
│       ├── eval_drawing_quality.py   # Evaluates FK path vs intended shape (Hausdorff, IoU)
│       ├── model.pt             # Trained model (generated locally — see below)
│       └── data/                # Generated dataset (not tracked in git — see below)
│           ├── train/           # 300 samples/label × 6 labels (.json + .csv per sample)
│           ├── val/             # 50 samples/label
│           └── test/            # 50 samples/label
│
├── py_joint_pub/                # Python ROS2 node — replays CSV in simulation
│   ├── py_joint_pub/joint_publisher_csv.py   # Publishes JointState from CSV at 500 Hz
│   └── resource/                # Default CSV files (ha149.csv, ldihel.csv)
│
├── msee22_description/          # URDF of MSEE 22 lab room + UR3e
│   ├── launch/
│   │   ├── view_room.launch.py  # RViz + joint_state_publisher_gui (manual sliders)
│   │   └── move_robot.launch.py # RViz + py_joint_pub (CSV replay); accepts csv_path arg
│   └── urdf/msee22.urdf.xacro
│
├── table_description/           # URDF of the drawing table
└── wall_description/            # URDF of the whiteboard wall
```

---

## Dependencies

### System

- Ubuntu 22.04 or 24.04
- [ROS2 Humble or Jazzy](https://docs.ros.org/en/jazzy/Installation.html)

```bash
sudo apt-get install ros-$ROS_DISTRO-ros2-control
sudo apt-get install ros-$ROS_DISTRO-ros2-controllers
sudo apt-get install ros-$ROS_DISTRO-robot-state-publisher
sudo apt-get install ros-$ROS_DISTRO-joint-state-publisher-gui
sudo apt-get install ros-$ROS_DISTRO-xacro
```

- [UR ROS2 Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver) (real robot only)
- [generate_parameter_library](https://github.com/PickNikRobotics/generate_parameter_library) (required to build `csv_controller`)

### Python

```bash
pip install torch numpy wandb matplotlib scipy
```

---

## Dataset and Model

Neither the dataset nor the model need to be downloaded — both are generated locally.

### Generate dataset

```bash
cd drawing_pipeline/src
python generate_data.py
```

Takes ~5–10 minutes. Creates `data/{train,val,test}/` with `.json` (label + params + `min_det`) and `.csv` (joint trajectory) pairs for all 6 shape labels. Samples that fail IK, pass near singularities, or cause elbow flips are automatically rejected (~18% rejection rate).

### Train model

```bash
python train.py
```

Takes ~2–3 minutes. Saves `model.pt`. Training is logged to [Weights & Biases](https://wandb.ai) — run `wandb login` once before training to enable the dashboard. If `model.pt` is absent when running `run_from_prompt.py`, the pipeline falls back to default parameters (`scale=1.0, rot_deg=0, dx=0, dy=0`) with a printed warning.

---

## Build

```bash
mkdir -p ~/ros2ws/src
cd ~/ros2ws/src
git clone <this repo> prompt_to_drawing
cd ~/ros2ws
colcon build --symlink-install
source install/setup.bash
```

---

## Usage

### 1. Generate data and train (first time only)

```bash
cd ~/ros2ws/src/prompt_to_drawing/drawing_pipeline/src
python generate_data.py
python train.py
```

### 2. Run end-to-end in simulation (RViz)

```bash
cd ~/ros2ws/src/prompt_to_drawing/drawing_pipeline/src
source ~/ros2ws/install/setup.bash
python run_from_prompt.py "draw a plane"
```

RViz opens with the MSEE22 room and the arm replays the generated trajectory.

**Supported prompt keywords:**

| Shape | Keywords |
| --- | --- |
| Plane | `plane` |
| Bike | `bike`, `bicycle` |
| Face | `face`, `smiley` |
| Square | `square`, `rect`, `rectangle`, `box` |
| Circle | `circle` |
| Lissajous | `lissajous`, `lissajous curve`, `lissajous figure` |

### 3. Visualize the room only

```bash
# Static room with joint sliders
ros2 launch msee22_description view_room.launch.py

# Replay any CSV file
ros2 launch msee22_description move_robot.launch.py csv_path:=/absolute/path/to/file.csv
```

### 4. Run on the real UR3e

1. Connect via ethernet and configure static IPs (robot: `192.168.1.102`, PC: `192.168.1.101`).
2. Put the robot into **Remote Control** mode (upper-right corner of teaching pendant).
3. In two terminals:

```bash
# Terminal 1 — start UR driver + csv_controller
ros2 launch csv_controller ur3e_csv.launch.py launch_rviz:=false csv_name:=plane

# Terminal 2 (optional) — MoveIt visualization
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e launch_rviz:=true
```

`csv_name` is the filename without `.csv`. `run_from_prompt.py` automatically writes generated CSVs to `csv_controller/csv_files/`.

---

## Evaluation

### Label detection accuracy

Tests 30 prompts (exact names, natural phrasings, aliases, unknowns) and reports per-label accuracy, confusion matrix, and false positive/negative lists.

```bash
cd drawing_pipeline/src
python eval_label_detection.py
```

Also reports:
- **IK quality per shape** — rejection breakdown (IK fail / singularity / elbow flip) and `min|det J|` mean ± std over 50 random samples
- **Embedding cosine similarity matrix** — shows whether the model learned distinct label representations (requires `model.pt`)
- **Per-label per-parameter MSE on test set** — which params are hardest to predict per shape (requires `data/test/` and `model.pt`)

### Drawing quality (FK path vs intended shape)

For each shape with a generated CSV, runs FK on every joint row to recover the actual end-effector path, projects it back to the drawing plane, and compares it to the intended 2D waypoints.

```bash
# First generate a CSV for each shape you want to evaluate
python run_from_prompt.py "draw a plane"
python run_from_prompt.py "draw a circle"
# ... etc.

# Then evaluate
python eval_drawing_quality.py
```

Reports per-shape:

| Metric | Description |
| --- | --- |
| **Mean distance error (mm)** | Average nearest-neighbour distance between actual and intended paths |
| **Hausdorff distance (mm)** | Worst-case deviation — catches single bad waypoints |
| **Rasterized IoU** | Pixel overlap of both shapes rendered as binary images |
| **Pass/Fail** | Success if mean error < 5 mm |

Also saves `<label>_overlay.png` files showing intended (blue) vs actual FK path (red dashed) for visual inspection and inclusion in the paper.

---

## Robot Network Configuration

| Setting | Robot | PC |
| --- | --- | --- |
| IP address | 192.168.1.102 | 192.168.1.101 |
| Subnet Mask | 255.255.255.0 | 255.255.255.0 |
| Default gateway | 192.168.1.1 | 192.168.1.1 |

Install the `External Control` URCap on the robot via USB, then configure on the teaching pendant: Host IP `192.168.1.101`, Custom Port `50002`.

---

## Helpful Shell Aliases

```bash
export ROS_WS=$HOME/ros2ws
alias cb="cd $ROS_WS && colcon build --symlink-install && source $ROS_WS/install/setup.bash"
alias rosclean="cd $ROS_WS && rm -rf build install log"
alias _ws="source $ROS_WS/install/setup.bash"
export ROBOT_IP="192.168.1.102"
source /opt/ros/jazzy/setup.bash
```
