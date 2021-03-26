#![no_main]
#![no_std]

// panic via probe
use panic_probe as _;

use cortex_m;
use cortex_m_rt::entry;
use stm32f4xx_hal::{prelude::*, crc32, stm32};

use rtt_target::{rtt_init_print, rprintln};

static BUFFER: [u8; 4] = [0x32u8, 0x42u8, 0x12u8, 0x44u8];
static BUFFER_2: [u32;1] = [0x32421244u32];

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("hello, world!");
    if let Some(dp) = stm32::Peripherals::take() {
        // Set up the system clock.
        let mut rcc = dp.RCC.constrain();

        // NOTE(unsafe) this reference will only be used for atomic writes with no side effects.
        // let rcc_raw = unsafe { &(*stm32::RCC::ptr()) };
        // Enable the CRC clock, see  https://github.com/stm32-rs/stm32f4xx-hal/issues/281
        // rcc_raw.ahb1enr.modify(|_, w| {w.crcen().set_bit()});

        let mut clocks = rcc.cfgr.freeze();

        // Initialize the CRC32 peripheral
        let mut crc = crc32::Crc32::new(dp.CRC);
        // feed the CRC peripheral bytes from a buffer that IS 4-byte alligned
        let result = crc.update_bytes(&BUFFER);
        // emit the result to rtt
        rprintln!("crc32 of {:?} = {}", BUFFER, result);
        crc.init();
        let z = u32::from_be_bytes(BUFFER);
        let result = crc.update(&[z]);
        rprintln!("crc32 of {:?} = {}", &[z], result);


        // Ok, lets see if we can feed it a U32 and have it make any more sense.
        crc.init();
        let result2 = crc.update(&BUFFER_2);
        rprintln!("crc32 of {:?} = {}", BUFFER_2, result2);

    }

    loop {
        cortex_m::asm::nop();
    }
}