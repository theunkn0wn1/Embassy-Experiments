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
use embassy::traits::uart::Uart;
use embassy::util::Forever;
use embassy_stm32f4::interrupt;
use embassy_stm32f4::serial;
use embassy_stm32f4::rtc;
use embassy::time::{Duration, Timer};
use stm32f4xx_hal::dma::{StreamsTuple, Stream4, Stream2, Channel4};
use stm32f4xx_hal::prelude::*;
use stm32f4xx_hal::serial::config::Config;
use stm32f4xx_hal::stm32;
use panic_probe as _;
use stm32f4xx_hal::stm32::DMA1;
use rtt_target::{rtt_init_print, rprintln};

type Uart4 = serial::Serial<stm32::UART4, Stream4<DMA1>, Stream2<DMA1>, Channel4>;

#[task]
async fn run(mut con: Uart4) {

    Timer::after(Duration::from_secs(2)).await;
    let buf = singleton!(: [u8; 30] = [0x00; 30]).expect("failed to create singleton");

    // buf[5] = 0x01;
    // con.send(buf).await.unwrap();
    rprintln!("Attempting to receive...");
    con.receive(buf).await;
    rprintln!("buffer := {:?}", buf);
    // let foo: &str = "foobar";
    // con.send(foo.as_bytes()).await.expect("failed to send bytes")
}

/// Embassy runtime
static EXECUTOR: Forever<Executor> = Forever::new();
/// Clock to use for Real Time Clock stuff.
/// Note: we can't use the actual stm32::RTC peripheral as doesn;t have the required accuracy.
static RTC: Forever<rtc::RTC<stm32::TIM12>> = Forever::new();
/// Alarm object for the RTC.
static RTC_ALARM: Forever<rtc::Alarm<stm32::TIM12>> =Forever::new();

#[entry]
fn main() -> ! {

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

    rtt_init_print!();
    rprintln!("hello, world!");

    let streams = StreamsTuple::new(dp.DMA1);


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
        Embassy config stuff.
        Borrowed from https://github.com/embassy-rs/embassy/blob/master/embassy-stm32f4-examples/src/bin/rtc_async.rs
    */
    let rtc = RTC.put(rtc::RTC::new(dp.TIM12, interrupt::take!(TIM8_BRK_TIM12), clocks));
    rtc.start();
    unsafe {embassy::time::set_clock(rtc)};
    let alarm = RTC_ALARM.put(rtc.alarm1());
    let executor = EXECUTOR.put(Executor::new());
    executor.set_alarm(alarm);
    /*
    spawn runtime stuff
     */

    executor.run(|spawner| {
        spawner.spawn(run(serial)).expect("failed to spawn `run`");
    });
}
