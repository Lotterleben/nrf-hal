#![no_std]
#![no_main]

// Panic provider crate
use panic_halt as _;

// Used to set the program entry point
use cortex_m_rt::entry;

// Provides definitions for our development board
use dwm1001::{
    DWM1001,
};

use nrf52832_hal::{
    pwm::{self, Pwm, DecoderLoad, DecoderMode, WaveCounterMode},
};


#[entry]
fn main() -> ! {
    // instanciate board and timer
    let mut board = DWM1001::take().unwrap();
    let mut timer = board.TIMER0.constrain();

    let mut red = board.pins.SPIS_MOSI.into_floating_input().degrade();
    let mut green = board.pins.SPIS_MISO.into_floating_input().degrade();
    let mut blue = board.pins.SPIS_CLK.into_floating_input().degrade();

    let channels = pwm::Channels {
        pwm_ch0: red,
        pwm_ch1: green,
        pwm_ch2: blue,
    };

    static sequence: [u16; 4] = [9000, 15000, 10000, 0x3333];

    let mut pulse = Pwm::new(board.PWM0, channels, pwm::Prescaler::DIV_8);

    pulse.set_decoder(DecoderLoad::Individual, DecoderMode::RefreshCount)
            .set_wavecounter(WaveCounterMode::Up)
            .disable_loop()
            .set_sequence_0(sequence, 0, 0)
            .start_sequence_0();

    loop {
        // blink red LED for not ready status
        timer.delay(250_000);
        board.leds.D11.enable();
        timer.delay(250_000);
        board.leds.D11.disable();
    }
}
