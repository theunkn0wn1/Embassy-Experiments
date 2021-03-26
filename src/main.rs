#![no_std]
#![no_main]
#![feature(trait_alias)]
#![feature(min_type_alias_impl_trait)]
#![feature(impl_trait_in_bindings)]
#![feature(type_alias_impl_trait)]

#[allow(unused_imports)]
use cortex_m::singleton;

use cortex_m_rt::entry;
use embassy::executor::{task, Executor};
use embassy::time::{Duration, Timer, Instant};
use embassy::traits::uart::{Uart, IdleUart};
use embassy::util::{Forever, InterruptFuture, CriticalSectionMutex};
use embassy_stm32f4::interrupt;
use embassy_stm32f4::rtc;
use embassy_stm32f4::serial;
use panic_probe as _;
use rtt_target::{rprintln, rtt_init_print};
use stm32f4xx_hal::dma::{Channel4, Stream2, Stream4, StreamsTuple};
use stm32f4xx_hal::prelude::*;
use stm32f4xx_hal::serial::config::Config;
use stm32f4xx_hal::stm32;
use stm32f4xx_hal::stm32::DMA1;
use stm32f4xx_hal::crc32::Crc32;

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
    }
}

#[task]
async fn validate_crc() {}

#[task]
/// Task that ticks periodically
async fn tick_periodic() -> ! {
    loop {
        // rprintln!("tick!");
        // async sleep primitive
        Timer::after(Duration::from_millis(500)).await;
    }
}

/* embassy boilerplate */
/// Embassy runtime
static EXECUTOR: Forever<Executor> = Forever::new();
/// Clock to use for Real Time Clock stuff.
/// Note: we can't use the actual stm32::RTC peripheral as doesn;t have the required accuracy.
static RTC: Forever<rtc::RTC<stm32::TIM12>> = Forever::new();
/// Alarm object for the RTC.
static RTC_ALARM: Forever<rtc::Alarm<stm32::TIM12>> = Forever::new();

#[entry]
fn main() -> ! {
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
    let mut crc32 = stm32f4xx_hal::crc32::Crc32::new(dp.CRC);

    /*
        Embassy config stuff.
        Borrowed from https://github.com/embassy-rs/embassy/blob/master/embassy-stm32f4-examples/src/bin/rtc_async.rs
    */
    let rtc = RTC.put(rtc::RTC::new(
        dp.TIM12,
        interrupt::take!(TIM8_BRK_TIM12),
        clocks,
    ));
    rtc.start();
    unsafe { embassy::time::set_clock(rtc) };
    let alarm = RTC_ALARM.put(rtc.alarm1());
    let executor = EXECUTOR.put(Executor::new());
    executor.set_alarm(alarm);
    /*
    spawn runtime stuff
     */

    executor.run(|spawner| {
        // spawn periodic task
        spawner
            .spawn(tick_periodic())
            .expect("failed to spawn `tick_periodic`");
        // spawn UART worker
        spawner
            .spawn(uart_worker(serial, crc32))
            .expect("failed to spawn `run`");
    });
}
