# car2_range

ROS 2 (Humble) nodes in Rust with [r2r](https://crates.io/crates/r2r) for car 2 ([car2_fw](https://github.com/mihoxp/car2_fw)).

- `car2_range`: prints the ultrasonic distance from `/car2/range`.
- `car2_safety`: too-close auto stop. The keyboard and gamepad programs ([keyboard_teleop](https://github.com/mihoxp/keyboard_teleop)) publish `/car2/cmd_vel_raw`; this node relays it to `/car2/cmd_vel` and drops forward motion when an obstacle is closer than 20-25 cm (more at higher motor duty). Reversing and turning still work. It beeps once when it blocks.

Build on the Pi:

    source /opt/ros/humble/setup.bash
    export LIBCLANG_PATH=/usr/lib/llvm-14/lib
    export IDL_PACKAGE_FILTER="std_msgs;sensor_msgs;geometry_msgs;builtin_interfaces;action_msgs;unique_identifier_msgs"
    cargo build --release

Run `car2_safety` at boot: `sudo install -m 644 car2-safety.service /etc/systemd/system/ && sudo systemctl enable --now car2-safety`
