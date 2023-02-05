//! # MS8607 Example
//!
//! This application demonstrates how to use the ms8607 library to read measurements from the MS8607 on a Raspberry Pi Pico.
//!
//! The pinouts are:
//!
//! * GPIO 4 (GP2 - I2C1 SDA) - i2c SDA
//! * GPIO 5 (GP3 - I2C1 SCL) - i2c SCL
#![no_std]
#![no_main]
// The writeln! trait.
use core::fmt::Write as FmtWrite;
use cortex_m_semihosting::hprintln;
// The macro for our start-up function
use rp_pico::entry;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Some traits we need
use fugit::RateExtU32;

// Alias for our HAL crate
use rp2040_hal as hal;

// We also need this for the 'Delay' object to work.
use rp2040_hal::Clock;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use hal::pac;

// UART related types
use hal::uart::{DataBits, StopBits, UartConfig};

// Import the MS8607 library
use ms8607::{MS8607};

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then writes to the UART in
/// an infinite loop.
#[entry]
fn main() -> ! {
    // ? BOILERPLATE START ? //
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    
    // Configure the clocks
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // Lets us wait for fixed periods of time
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    
    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Configure two pins as being I²C, not GPIO
    let sda_pin = pins.gpio2.into_mode::<hal::gpio::FunctionI2C>();
    let scl_pin = pins.gpio3.into_mode::<hal::gpio::FunctionI2C>();

     // Create the I²C drive, using the two pre-configured pins. This will fail
    // at compile time if the pins are in the wrong mode, or if this I²C
    // peripheral isn't available on these pins!
    let i2c = hal::I2C::i2c1(
        pac.I2C1,
        sda_pin,
        scl_pin,
        400.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    // Allow the I²C peripheral to be shared between drivers
    let bus_i2c = shared_bus::BusManagerSimple::new(i2c);

    // ? BOILERPLATE END ? //

    hprintln!("MS8607 test!\r");
    let hz = clocks.system_clock.freq().to_Hz();
    hprintln!("System clock: {hz} Hz\r");
    // Try to initialize!
    let mut ms8607 = MS8607::new(bus_i2c.acquire_i2c());
    hprintln!("MS8607 init...\r");
    let begin = ms8607.begin(&mut delay);
    match begin {
        Ok(_) => hprintln!("MS8607 init OK\r"),
        Err(_) => {
            hprintln!("MS8607 init failed\r");
            panic!();
        }
    }

    hprintln!("MS8607 found\r");

    // wait 2s
    delay.delay_ms(2000u32);

    loop {
        let req = ms8607.get_measurements(&mut delay);
        let (pres, temp, hum) = match req {
            Ok((pres, temp, hum)) => (pres, temp, hum),
            Err(_) => {
                hprintln!("Error retrieving measurements");
                panic!();
            }
        };
        // serial print out the data with 2 decimal places
        hprintln!("Pressure: {pres:.2} Pa, Temperature: {temp:.2} C, Humidity: {hum:.2} %RH\r");
        delay.delay_ms(5000u32);
    }
}