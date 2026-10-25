// Subscribe to car 2's ultrasonic distance (/car2/range) and print it.
use futures::{executor::LocalPool, future, stream::StreamExt, task::LocalSpawnExt};
use r2r::{QosProfile, sensor_msgs::msg::Range};
use std::time::Duration;

const TOO_CLOSE_M: f32 = 0.15; // same threshold as the firmware's obstacle warning

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "car2_range_rs", "")?;
    // The Pico publishes best-effort, so subscribe with sensor-data QoS
    let sub = node.subscribe::<Range>("/car2/range", QosProfile::sensor_data())?;

    let mut pool = LocalPool::new();
    pool.spawner().spawn_local(async move {
        sub.for_each(|msg| {
            let cm = msg.range * 100.0;
            if msg.range < 0.0 {
                println!("no echo");
            } else {
                let bar = "#".repeat((cm / 5.0).clamp(0.0, 40.0) as usize);
                let warn = if msg.range < TOO_CLOSE_M { "  !! TOO CLOSE !!" } else { "" };
                println!("{cm:6.1} cm {bar}{warn}");
            }
            future::ready(())
        })
        .await
    })?;

    println!("listening on /car2/range (Ctrl+C to quit)");
    loop {
        node.spin_once(Duration::from_millis(100));
        pool.run_until_stalled();
    }
}
