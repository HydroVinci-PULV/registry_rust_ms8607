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


    let uart_pins = (
        // UART TX (characters sent from RP2040) on pin 1 (GPIO0)
        pins.gpio0.into_mode::<hal::gpio::FunctionUart>(),
        // UART RX (characters received by RP2040) on pin 2 (GPIO1)
        pins.gpio1.into_mode::<hal::gpio::FunctionUart>(),
    );

    // Make a UART on the given pins
    let mut uart = hal::uart::UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
        .enable(
            UartConfig::new(115200.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();

     // Create the I²C drive, using the two pre-configured pins. This will fail
    // at compile time if the pins are in the wrong mode, or if this I²C
    // peripheral isn't available on these pins!
    let mut i2c = hal::I2C::i2c1(
        pac.I2C1,
        sda_pin,
        scl_pin,
        400.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    // ? BOILERPLATE END ? //

    writeln!(uart, "MS8607 test!\r").unwrap();
    let hz = clocks.system_clock.freq().to_Hz();
    writeln!(uart, "System clock: {hz} Hz\r").unwrap();
    // Try to initialize!
    let mut ms8607 = MS8607::new();
    writeln!(uart, "MS8607 init...\r").unwrap();
    let begin = ms8607.begin(&mut i2c, &mut delay);
    match begin {
        Ok(_) => writeln!(uart, "MS8607 init OK\r").unwrap(),
        Err(_) => {
            writeln!(uart, "MS8607 init failed\r").unwrap();
            panic!();
        }
    }

    writeln!(uart, "MS8607 found\r").unwrap();

    // wait 2s
    delay.delay_ms(2000u32);

    loop {
        let req = ms8607.get_measurements(&mut i2c, &mut delay);
        let (pres, temp, hum) = match req {
            Ok((pres, temp, hum)) => (pres, temp, hum),
            Err(_) => {
                writeln!(uart, "Error retrieving measurements").unwrap();
                panic!();
            }
        };
        // serial print out the data with 2 decimal places
        writeln!(
            uart,
            "Pressure: {pres:.2} Pa, Temperature: {temp:.2} C, Humidity: {hum:.2} %RH\r"
        )
        .unwrap();
        delay.delay_ms(5000u32);
    }
}