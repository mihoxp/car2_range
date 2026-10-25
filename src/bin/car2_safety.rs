// Too-close auto stop for car 2.
// The keyboard and gamepad programs publish /car2/cmd_vel_raw; this node relays
// it to /car2/cmd_vel but drops forward motion while /car2/range is too close.
// Reversing and turning still work, so the car can be driven away.
use futures::{executor::LocalPool, future, stream, stream::StreamExt, task::LocalSpawnExt};
use r2r::{
    QosProfile, geometry_msgs::msg::Twist, sensor_msgs::msg::Range,
    std_msgs::msg::UInt16MultiArray,
};
use std::time::Duration;

// The car coasts further at higher speed, so it has to stop earlier. The firmware
// turns any non-zero speed into 60-100% motor duty (MOTOR_MIN_DUTY), so even a
// light stick push moves the car fast: base the distance on that duty.
// Floor tests: 60-80% duty detected at 20 cm stopped at 10-15 cm; 100% duty
// detected at 40 cm stopped at ~40 cm (the firmware brakes), so far too early.
// Stop distance = 15 cm + 10 cm x duty^2: 20 cm up to 70%, 21 cm at 80%, 25 cm at 100%.
const MAX_LINEAR: f64 = 0.5; // linear.x of 100% speed (firmware MAX_WHEEL_SPEED_MPS)
const MOTOR_MIN_DUTY: f32 = 0.60;
const STOP_BASE_M: f32 = 0.15;
const STOP_COAST_M: f32 = 0.10; // coasting distance at 100% duty
const STOP_MIN_M: f32 = 0.20; // when not driving forward
const RELEASE_EXTRA_M: f32 = 0.05; // must be this much farther than the stop distance ...
const CLEAR_READINGS: u32 = 10; // ... this many times in a row (~1 s) to release,
// because very close objects sometimes echo off the wall behind them

fn stop_distance(linear_x: f64) -> f32 {
    if linear_x <= 0.0 {
        return STOP_MIN_M;
    }
    let speed = (linear_x / MAX_LINEAR).clamp(0.0, 1.0) as f32;
    let duty = MOTOR_MIN_DUTY + (1.0 - MOTOR_MIN_DUTY) * speed;
    (STOP_BASE_M + STOP_COAST_M * duty * duty).max(STOP_MIN_M)
}

enum Event {
    Range(Range),
    Cmd(Twist),
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "car2_safety", "")?;
    let range_sub = node.subscribe::<Range>("/car2/range", QosProfile::sensor_data())?;
    let raw_sub = node.subscribe::<Twist>("/car2/cmd_vel_raw", QosProfile::default())?;
    let cmd_pub = node.create_publisher::<Twist>("/car2/cmd_vel", QosProfile::default())?;
    let beep_pub =
        node.create_publisher::<UInt16MultiArray>("/car2/buzzer", QosProfile::default())?;

    let events = stream::select(range_sub.map(Event::Range), raw_sub.map(Event::Cmd));
    let mut last_range = f32::INFINITY;
    let mut hold_release: Option<f32> = None; // Some(distance) while forward is blocked
    let mut clear_count = 0;
    let mut last_raw = Twist::default();

    let mut pool = LocalPool::new();
    pool.spawner().spawn_local(async move {
        // Block forward motion at this distance (beep, log, stop if moving)
        let block = |hold: &mut Option<f32>, range: f32, stop: f32, moving: Option<&Twist>| {
            *hold = Some(stop + RELEASE_EXTRA_M);
            println!("TOO CLOSE ({:.1} cm < {:.0} cm): forward blocked", range * 100.0, stop * 100.0);
            let _ = beep_pub.publish(&UInt16MultiArray { data: vec![1500, 150], ..Default::default() });
            if let Some(cmd) = moving {
                let stopped = Twist { linear: Default::default(), ..cmd.clone() };
                for _ in 0..3 {
                    let _ = cmd_pub.publish(&stopped);
                }
            }
        };
        events
            .for_each(|ev| {
                match ev {
                    Event::Range(msg) => {
                        let r = msg.range; // negative = no echo: neither close nor clear
                        if r >= 0.0 {
                            last_range = r;
                            let stop = stop_distance(last_raw.linear.x);
                            if let Some(release) = hold_release {
                                clear_count = if r > release { clear_count + 1 } else { 0 };
                                if clear_count >= CLEAR_READINGS {
                                    hold_release = None;
                                    println!("clear ({:.1} cm): forward allowed", r * 100.0);
                                }
                            } else if r < stop {
                                clear_count = 0;
                                let moving = (last_raw.linear.x > 0.0).then_some(&last_raw);
                                block(&mut hold_release, r, stop, moving);
                            }
                        }
                    }
                    Event::Cmd(msg) => {
                        let mut out = msg.clone();
                        if msg.linear.x > 0.0 {
                            let stop = stop_distance(msg.linear.x);
                            // A faster command needs more room than the current reading gives
                            if hold_release.is_none() && last_range < stop {
                                clear_count = 0;
                                block(&mut hold_release, last_range, stop, None);
                            }
                            if hold_release.is_some() {
                                out.linear.x = 0.0;
                            }
                        }
                        let _ = cmd_pub.publish(&out);
                        last_raw = msg;
                    }
                }
                future::ready(())
            })
            .await
    })?;

    println!(
        "relaying /car2/cmd_vel_raw -> /car2/cmd_vel, stop at {:.0}-{:.0} cm by speed",
        stop_distance(0.0) * 100.0,
        stop_distance(MAX_LINEAR) * 100.0
    );
    loop {
        node.spin_once(Duration::from_millis(20));
        pool.run_until_stalled();
    }
}
