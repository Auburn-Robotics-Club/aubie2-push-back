use std::time::Duration;

use aubie2::theme::THEME_WAR_EAGLE;
use vexide::prelude::*;

#[vexide::main(banner(theme = THEME_WAR_EAGLE))]
async fn main(peripherals: Peripherals) {
    let mut controller = peripherals.primary_controller;
    let letters = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h'];
    let mut solenoids = [
        AdiDigitalOut::new(peripherals.adi_a),
        AdiDigitalOut::new(peripherals.adi_b),
        AdiDigitalOut::new(peripherals.adi_c),
        AdiDigitalOut::new(peripherals.adi_d),
        AdiDigitalOut::new(peripherals.adi_e),
        AdiDigitalOut::new(peripherals.adi_f),
        AdiDigitalOut::new(peripherals.adi_g),
        AdiDigitalOut::new(peripherals.adi_h),
    ];

    let mut port = 0;
    println!("solennoid {}", letters[port]);
    _ = controller.set_text(format!("solenoid {}", letters[port]), 1, 1).await;

    loop {
        let state = controller.state().unwrap_or_default();

        if state.button_right.is_now_pressed() {
            port = (port + 1) % solenoids.len();
            println!("solennoid {}", letters[port]);
            _ = controller.set_text(format!("solenoid {}", letters[port]), 1, 1).await;
        }
        
        if state.button_left.is_now_pressed() {
            port = (port - 1) % solenoids.len();
            println!("solennoid {}", letters[port]);
            _ = controller.set_text(format!("solenoid {}", letters[port]), 1, 1).await;
        }
        
        if state.button_a.is_now_pressed() {
            _ = solenoids[port].toggle()          ;
        }

        sleep(Duration::from_millis(10)).await;
    }
}
