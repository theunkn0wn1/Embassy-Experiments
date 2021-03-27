#![no_std]
#![no_main]
#![feature(trait_alias)]
#![feature(min_type_alias_impl_trait)]
#![feature(impl_trait_in_bindings)]
#![feature(type_alias_impl_trait)]

#[allow(unused_imports)]
use cortex_m::singleton;

use cortex_m_rt::entry;
use embassy::executor::{task, Executor, Spawner};
use embassy::time::{Duration, Timer};
use embassy::traits::uart::{Uart, IdleUart};
use embassy::util::{Forever, InterruptFuture, CriticalSectionMutex};
use embassy_stm32f4::interrupt;
use embassy_stm32f4::rtc;
use embassy_stm32f4::serial;
use embassy_stm32;
use panic_probe as _;
use rtt_target::{rprintln, rtt_init_print};
use stm32f4xx_hal::dma::{Channel4, Stream2, Stream4, StreamsTuple};
use stm32f4xx_hal::prelude::*;
use stm32f4xx_hal::serial::config::Config;
use stm32f4xx_hal::stm32;
use stm32f4xx_hal::stm32::DMA1;
use stm32f4xx_hal::crc32::Crc32;
use core::convert::TryInto;

type Uart4 = serial::Serial<stm32::UART4, Stream4<DMA1>, Stream2<DMA1>, Channel4>;

#[task]
async fn uart_worker(mut con: Uart4, mut crc32: Crc32) {
    // note: this needs to be a singleton otherwise DMA won't work correctly.
    let buf = singleton!(: [u8; 30] = [0xFF; 30]).expect("failed to create singleton");

    loop {
        crc32.init();

        rprintln!("Attempting to receive...");
        let total_read = con.receive_until_idle(buf).await.expect("failed to receive");

        let populated_slice = &buf[..total_read];
        let checksum = crc32.update_bytes(populated_slice);
        rprintln!("buffer ({:?})[{}] := {:?}",total_read, checksum,  populated_slice);
        rprintln!("writing buffer...");
        con.send(populated_slice).await.unwrap();
        match validate_crc(populated_slice, &mut crc32) {
            Ok(valid) => {
                rprintln!("data validated :: {}", match valid {
                true => {"true"}
                false => {"false"}
                });
            }
            Err(_) => {
                rprintln!("Validation error.")
            }
        }
    }
}

/// Validates a slice of bytes.
/// Note:: there must be at least 5 bytes.
/// The last 4 bytes are the sender's CRC32-Ethernet checksum, in BE encoding.
/// The payload should be 4 byte alligned, otherwise the last 4 byte chunk's MSB will be padded
/// with zeros (hardware limitation) to ensure alignment.
fn validate_crc(payload: &[u8], crc32: &mut Crc32) -> Result<bool, ()> {
    // payload must contain at least 1 data byte and 4 LE checksum bytes
    if payload.len() < 5 {
        return Err(());
    };

    crc32.init();
    // Force synchronization.
    cortex_m::asm::dsb();
    rprintln!("raw payload := {:?}", payload);

    // split off the last 4 bytes, as that will be the sender's checksum.
    let (payload_bytes, sender_checksum_bytes) = payload.split_at(payload.len() - 4);
    // let payload_u32 = u32::from_be_bytes(payload_bytes.try_into().unwrap());
    // rprintln!("CRC of [{}] (from_bytes_be) := {}", payload_u32, crc32.update(&[payload_u32]));

    // Sigh. why can't the hardware engineering world agree to one or the other?
    // this entire mess is caused by BE vs LE.

    // chunk exactly one word's worth of bytes, producing an iterator.
    let word_chunks = payload_bytes.chunks_exact(4);
    // We also need to know if there was a remainder.
    let remainder = word_chunks.remainder();

    // This has to be mut since we later iterate over the words to feed them to CRC.
    let mut device_checksum = 0;

    // For each chunk, parse it as a BE word, then convert the internal bytes to LE
    // This is necessary because the native hardware blindly reinterprets bytes as LE
    let words = word_chunks
        .map(|chunk| u32::from_be_bytes(chunk.try_into().unwrap()))
        // convert the underlying memory to LE, since these get reinterpret-cast'ed by the HAL.
        // Note: this may be a no-op since we are already on LE hardware...
        .map(|word| word.to_le());
    // Now feed each word into the CRC engine
    for word in words {
        rprintln!("[DEBUG] feeding word {:x}", word);
        device_checksum = crc32.update(&[word]);
    }
    // check but disregard the remainder.
    if !remainder.is_empty(){
        rprintln!("[warn] non-empty remainder {:?}", remainder);
    }

    rprintln!("[DEBUG]: payload bytes :: {:?} sender_checksum_bytes:: {:?}", payload_bytes, sender_checksum_bytes);
    match sender_checksum_bytes.try_into() {
        Ok(slice) => {
            let senders_checksum = u32::from_be_bytes(slice);
            rprintln!("[DEBUG]: device checksum :: {} sender checksum :: {}", device_checksum, senders_checksum);
            rprintln!("[DEBUG]: device checksum :: {:x}(le) :: {:x}(be)", device_checksum.to_le(), senders_checksum.to_be());
            Ok(device_checksum == senders_checksum)
        }
        Err(_) => {
            rprintln!("[ERROR] Error processing sender's checksum");
            Err(())
        }
    }
}

#[task]
/// Task that ticks periodically
async fn tick_periodic() -> ! {
    loop {
        // rprintln!("tick!");
        // async sleep primitive
        Timer::after(Duration::from_millis(500)).await;
    }
}

#[embassy::main]
async fn main(spawner: Spawner){
    rtt_init_print!();
    rprintln!("hello, world!");

    let dp = stm32::Peripherals::take().unwrap();
    #[allow(unused_variables)]
        let cp = cortex_m::peripheral::Peripherals::take().unwrap();

    dp.DBGMCU.cr.modify(|_, w| {
        w.dbg_sleep().set_bit();
        w.dbg_standby().set_bit();
        w.dbg_stop().set_bit()
    });
    dp.RCC.ahb1enr.modify(|_, w| w.dma1en().enabled());

    let rcc = dp.RCC.constrain();

    // https://gist.github.com/thalesfragoso/a07340c5df6eee3b04c42fdc69ecdcb1
    let gpioc = dp.GPIOC.split();

    let clocks = rcc
        .cfgr
        // .use_hse(16.mhz())
        // .sysclk(48.mhz())
        // .pclk1(24.mhz())
        .freeze();

    let streams = StreamsTuple::new(dp.DMA1);

    /*
    serial initialization
    */
    let serial = unsafe {
        serial::Serial::new(
            dp.UART4,
            (streams.4, streams.2),
            (
                gpioc.pc10.into_alternate_af8(),
                gpioc.pc11.into_alternate_af8(),
            ),
            interrupt::take!(DMA1_STREAM4),
            interrupt::take!(DMA1_STREAM2),
            interrupt::take!(UART4),
            Config::default().baudrate(9600.bps()),
            clocks,
        )
    };

    /*
    Init CRC
     */
    let crc32 = stm32f4xx_hal::crc32::Crc32::new(dp.CRC);

    /*
        Embassy config stuff.
        Borrowed from https://github.com/embassy-rs/embassy/blob/master/embassy-stm32f4-examples/src/bin/rtc_async.rs
    */
        // spawn periodic task
        spawner
            .spawn(tick_periodic())
            .expect("failed to spawn `tick_periodic`");
        // spawn UART worker
        spawner
            .spawn(uart_worker(serial, crc32))
            .expect("failed to spawn `run`");
}
