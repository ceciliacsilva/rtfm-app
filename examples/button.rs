#![feature(panic_implementation)]
#![no_std]
#![no_main]

extern crate cortex_m;
extern crate cortex_m_rt;
extern crate cortex_m_rtfm as rtfm;
extern crate hal;
extern crate embedded_hal;

use core::panic::PanicInfo;
use core::sync::atomic::{self, Ordering};

use cortex_m::asm::bkpt;

use rtfm::{app, Threshold};

use embedded_hal::prelude::*;

use hal::rcc::RccExt;
use hal::gpio::GpioExt;
// use hal::flash::FlashExt;
// use hal::serial::Serial;
// use hal::serial::{Rx, Event, Tx};
// use hal::time::U32Ext;
use hal::stm32l0;
// use hal::stm32l0::stm32l0x1::USART2 as USART2_p;

use hal::gpio::{Output, Input, PushPull, PullUp, gpioa::PA5, gpioa::PA4};

app! {
    device: stm32l0::stm32l0x1,
    resources: {
        static LED: PA5<Output<PushPull>>;
        static BUTTON: PA4<Input<PullUp>>;
    },
    tasks: {
        EXTI4_15: {
            path: button_callback,
            resources: [BUTTON, LED],
        },
    }
}

fn init(mut p: init::Peripherals) -> init::LateResources {
    let mut rcc = p.device.RCC.constrain();
    let mut gpioa = p.device.GPIOA.split(&mut rcc.iop);
    
    let led = gpioa.pa5.into_output(&mut gpioa.moder).push_pull(&mut gpioa.otyper);
    let but = gpioa.pa4.into_input(&mut gpioa.moder).pull_up(&mut gpioa.pupdr);

    p.device.SYSCFG_COMP.exticr2.modify(|_, w| unsafe{ w.exti4().bits(0b0000) });//PA0

    p.device.EXTI.imr.modify(|_, w| w.im4().bit(true));
    p.device.EXTI.ftsr.modify(|_, w| w.ft4().bit(true));

    init::LateResources {
        LED: led,
        BUTTON: but,
    }
}

fn idle() -> ! {
    loop {
        rtfm::wfi();
    }
}

fn button_callback(_t: &mut Threshold, mut r: EXTI4_15::Resources) {
    if r.BUTTON.is_low() {
        r.LED.set_high();
    } else {
        r.LED.set_low();
    }
}

#[allow(deprecated)]
#[panic_implementation]
fn panic(_info: &PanicInfo) -> ! {
    bkpt();

    loop {
        atomic::compiler_fence(Ordering::SeqCst)
    }
}