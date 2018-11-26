//! Interrupt doesn't work yet. 
//! interrupt! -> #[interrupt]
//! Wait for svd2rust 0.14..

#![feature(custom_attribute)]
// #![deny(unsafe_code)]
// #![deny(warnings)]
#![no_main]
#![no_std]

extern crate cortex_m;
extern crate cortex_m_rt;
extern crate rtfm;
extern crate narc_hal;
extern crate embedded_hal;
#[macro_use]
extern crate nb;

use rtfm::app;
use rtfm::export::wfi;
use core::panic::PanicInfo;
use core::sync::atomic::{self, Ordering};

use cortex_m::asm::bkpt;

use narc_hal::stm32l052::interrupt;
use narc_hal::gpio::GpioExt;
use narc_hal::rcc::RccExt;
use narc_hal::flash::FlashExt;
use narc_hal::serial::Serial;
use narc_hal::serial::{Rx, Event, Tx};
use narc_hal::time::U32Ext;
use embedded_hal::prelude::*;
use narc_hal::gpio::{Output, PushPull, gpioa::PA5};
use narc_hal::stm32l052::USART2 as USART2_p;

use embedded_hal::digital::OutputPin;

use cortex_m::peripheral::syst::SystClkSource;

// interrupt!(USART2, USART2);

#[app(device = narc_hal::stm32l052)]
const APP: () = {
    static mut SHARED: u32 = 0;
    static mut LED: PA5<Output<PushPull>> = ();
    static mut EXTI: narc_hal::stm32l052::EXTI = ();
    static mut RX: Rx<USART2_p> = ();
    static mut TX: Tx<USART2_p> = ();
    static mut DATA: u8 = 0;

    #[init]
    fn init() {
        let device: narc_hal::stm32l052::Peripherals = device;
        
        let mut rcc = device.RCC.constrain();
        let mut gpioa = device.GPIOA.split(&mut rcc.iop);
        let mut flash = device.FLASH.constrain();
        let clocks = rcc.cfgr.freeze(&mut flash.acr);

        let led = gpioa.pa5.into_output(&mut gpioa.moder).push_pull(&mut gpioa.otyper);
        let _but = gpioa.pa4.into_input(&mut gpioa.moder).pull_up(&mut gpioa.pupdr);

        let tx = gpioa.pa2.into_alternate(&mut gpioa.moder).af4(&mut gpioa.afrl);
        let rx = gpioa.pa3.into_alternate(&mut gpioa.moder).af4(&mut gpioa.afrl);

        let serial = Serial::usart2(
            device.USART2,
            (tx, rx),
            9_600.bps(),
            clocks,
            &mut rcc.apb1,
            None
        );

        serial.listen(Event::Rxne);
        serial.listen(Event::Txe);

        // core.SYST.set_clock_source(SystClkSource::Core);
        // core.SYST.set_reload(2_000_000); // 1s
        // core.SYST.clear_current();
        // core.SYST.enable_counter();
        // core.SYST.enable_interrupt();


        device.SYSCFG_COMP.exticr2.modify(|_, w| unsafe{ w.exti4().bits(0b0000) });//PA0
        device.EXTI.imr.modify(|_, w| w.im4().bit(true));
        device.EXTI.ftsr.modify(|_, w| w.ft4().bit(true));

        let (mut tx, mut rx, mut _ri) = serial.split();

        let data = block!(rx.read()).unwrap();
        tx.write(data);

        // rtfm::pend(Interrupt::EXTI4_15);
        LED = led;
        EXTI = device.EXTI;
        RX = rx;
        TX = tx;
    }

    #[idle(resources = [SHARED, LED])]
    fn idle () -> ! {
        wfi();

        loop {
            resources.LED.set_high();
        }
    }

    // #[exception]
    // fn SysTick () {
    //     bkpt();
    // }

    #[interrupt(resources = [SHARED, EXTI])]
    fn EXTI4_15 () {
        bkpt();

        *resources.SHARED += 1;

        resources.EXTI.pr.modify(|_, w| w.pif0().bit(true));
    }

    // #[interrupt(resources = [DATA, RX, TX])]
    // fn USART2 () {
    //     *resources.DATA =  resources.RX.read().unwrap();

    //     resources.TX.write(b'i');
    // }

    #[interrupt]
    fn USART2 () {
        bkpt();
    }
};

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    bkpt();

    loop {
        atomic::compiler_fence(Ordering::SeqCst)
    }
}