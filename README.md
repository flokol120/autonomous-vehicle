[Demo Video](https://www.youtube.com/watch?v=diwQoyUGhvM)

# Usage

## Aliases
Aliases can be used to shorten the commands used inside of the docker. Here is a list of all existing aliases:

* `help`: List all aliases with short description
* `build`: Same as `colcon build`
* `buildall`: Build while overriding all modules
* `gazebo`: Start gazebo simulation
* `driver [default|debug]`: Start the main driver. You must also start the controller and image processing nodes.
* `controller [default|debug]`: Start remote controller, optionally as debug. Will be `default` by default. See [Start Remote Controller](#start-remote-controller) for keybinds.
* `image [default|debug|image|image_debug]`: Start image processing. Refer to [Start Image Processing](#start-image-processing) for modes. Will be `default` by default.
* `sensorSteer`: Start sensor steer.

## Basic Setup

```bash
xhost +  # exposes displays
docker-compose build  # build docker
docker-compose up -d  # start docker
docker-compose exec ros bash  # enter docker terminal
colcon build
source ./install/setup.bash
```

Rebuild all applications:
```
colcon build --allow-overriding car_controller gazebo_simulation image_processing remote_controller sensor_steer
```

## Applications

### Start Simulation
```bash
ros2 launch gazebo_simulation car.launch.py
```

### Start RQT
```bash
rqt
```

### Start Autonomous Driving
```bash
ros2 run autonomous_driving <MODE>
```

### Start Remote Controller
```bash
ros2 run remote_controller <MODE>
```
Modes:
* `default` 
* `debug` - Debug logging enabled

Debug preview:
```
----------------------------------------------------
speed: 1.0
steering: 30.0
abortion: False

w: True
a: True
s: False
d: True
q: False
t (shift): False
----------------------------------------------------
```

Use WASD to steer the Vehicle. Additional Keys:
* `t` for Turbo
* `q` to switch between Driving and Remote Controller
* `f` to activate lane switching
* `r` to reset the PIDs

### Start Image Processing
```bash
ros2 run image_processing <MODE>
```
Modes:
* `default` - Everything enabled, no debug logging
* `debug` - Everything enabled, debug logging
* `image` - Image processing enabled, steering disabled, no debug logging
* `image_debug` - Same as `image` with debug logging

Debug preview:
```
----------------------------------------------------
Debug: True, Image Only: False

Cropping
        Height: 480, Width: 640

Transformer
        Height: 592, Width: 1040

Hough Transformation
        Lines: 41

Line Detection
        M Values:       Left: 0.141000, Right: -0.392000
        Offsets:        Left: 2.191000, Right: -1.342000
        Steer Values:   Left: 0.730000, Right: -0.447000
        Steering:       -1.074000
        Steering enabled: True

Time taken: 0.021237
----------------------------------------------------
```
