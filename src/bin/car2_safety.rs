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

const STOP_M: f32 = 0.20; // closer than this -> no forward motion
const CLEAR_M: f32 = 0.25; // must be farther than this ...
const CLEAR_READINGS: u32 = 10; // ... this many times in a row (~1 s) to release,
// because very close objects sometimes echo off the wall behind them

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
    let mut blocked = false;
    let mut clear_count = 0;
    let mut last_cmd = Twist::default();

    let mut pool = LocalPool::new();
    pool.spawner().spawn_local(async move {
        events
            .for_each(|ev| {
                match ev {
                    Event::Range(msg) => {
                        let r = msg.range; // negative = no echo: neither close nor clear
                        if r >= 0.0 && r < STOP_M {
                            clear_count = 0;
                            if !blocked {
                                blocked = true;
                                println!("TOO CLOSE ({:.1} cm): forward blocked", r * 100.0);
                                let _ = beep_pub.publish(&UInt16MultiArray {
                                    data: vec![1500, 150],
                                    ..Default::default()
                                });
                                // Stop now instead of waiting for the next drive command
                                if last_cmd.linear.x > 0.0 {
                                    last_cmd.linear.x = 0.0;
                                    for _ in 0..3 {
                                        let _ = cmd_pub.publish(&last_cmd);
                                    }
                                }
                            }
                        } else if blocked {
                            clear_count = if r > CLEAR_M { clear_count + 1 } else { 0 };
                            if clear_count >= CLEAR_READINGS {
                                blocked = false;
                                println!("clear ({:.1} cm): forward allowed", r * 100.0);
                            }
                        }
                    }
                    Event::Cmd(mut msg) => {
                        if blocked && msg.linear.x > 0.0 {
                            msg.linear.x = 0.0;
                        }
                        let _ = cmd_pub.publish(&msg);
                        last_cmd = msg;
                    }
                }
                future::ready(())
            })
            .await
    })?;

    println!("relaying /car2/cmd_vel_raw -> /car2/cmd_vel, stop < {} cm", STOP_M * 100.0);
    loop {
        node.spin_once(Duration::from_millis(20));
        pool.run_until_stalled();
    }
}
