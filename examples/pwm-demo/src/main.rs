#![no_std]
#![no_main]

// Panic provider crate
use panic_halt as _;

// Used to set the program entry point
use cortex_m_rt::entry;

// Provides definitions for our development board
use dwm1001::{
    prelude::*,
    DWM1001,
};

use nb::block;

use nrf52832_hal::{
    Timer,
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
    // TODO double check that this is actually unused
    let mut unused = board.pins.GPIO_12.into_floating_input().degrade();

    let channels = pwm::Channels {
        pwm_ch0: red,
        pwm_ch1: green,
        pwm_ch2: blue,
        pwm_ch3: unused, // TODO added to silence compiler; what would a suitable pin here?
    };

    static sequence: [u16; 4] = [9000, 15000, 10000, 0x3333];

    let mut pulse = Pwm::new(board.PWM0, channels, pwm::Prescaler::DIV_8);

    // TODO change functions so that they return pulse so that they can be daisy chained?
    pulse.set_decoder(DecoderLoad::Individual, DecoderMode::RefreshCount);
    pulse.set_wavecounter(WaveCounterMode::Up);
    pulse.disable_loop();
    pulse.set_sequence_0(sequence, 0, 0);
    pulse.start_sequence_0();

    // todo pick sensible value based on cycle length
    let cycles = 250_000;
    loop {
        // blink red LED for not ready status
        delay(&mut timer, cycles);
        board.leds.D11.enable();

        delay(&mut timer, cycles);
        board.leds.D11.disable();
    }
}


// stolen from blinky
fn delay<T>(timer: &mut Timer<T>, cycles: u32)
where
    T: TimerExt,
{
    timer.start(cycles);
    block!(timer.wait()).unwrap();
}