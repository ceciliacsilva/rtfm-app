#![no_main]
#![no_std]

extern crate cortex_m;
extern crate cortex_m_rt;
extern crate rtfm;
extern crate narc_hal;
extern crate embedded_hal;
extern crate timer_wheels as tw;
extern crate heapless;

use rtfm::app;
use rtfm::export::wfi;
use core::panic::PanicInfo;
use core::sync::atomic::{self, Ordering};

use heapless::consts::U4;
use heapless::Vec;

use cortex_m::asm::bkpt;
use narc_hal::stm32l052::{TIM6 as TIM6_p};
use narc_hal::rcc::RccExt;
use narc_hal::flash::FlashExt;
use narc_hal::gpio::GpioExt;
use narc_hal::gpio::{gpioa::PA5, Output, PushPull};
use narc_hal::time::U32Ext;
use narc_hal::timer;
use narc_hal::timer::TimerExt;
use embedded_hal::digital::OutputPin;
use embedded_hal::timer::CountDown;
use tw::TimerWheel;

pub enum Functions {
    Callee,
}

#[app(device = narc_hal::stm32l052)]
const APP: () = {
    static mut LED: PA5<Output<PushPull>> = ();
    static mut TW: TimerWheel<Functions> = ();
    static mut STATE: bool = true;
    static mut TIM6: timer::Timer<TIM6_p> = ();

    #[init]
    fn init() {
        let mut rcc = device.RCC.constrain();
        let mut flash = device.FLASH.constrain();
        let mut gpioa = device.GPIOA.split(&mut rcc.iop);
        let mut led = gpioa.pa5.into_output(&mut gpioa.moder).push_pull(&mut gpioa.otyper);
        let clocks = rcc.cfgr.freeze(&mut flash.acr);
        let mut tim = device.TIM6.timer(1.hz(), clocks, &mut rcc.apb1);
        let mut timer_wheel = TimerWheel::new();

        timer_wheel.schedule(0, Functions::Callee);
        timer_wheel.schedule(2, Functions::Callee);
        timer_wheel.schedule(4, Functions::Callee);
        timer_wheel.schedule(6, Functions::Callee);

        tim.listen(timer::Event::TimeOut);

        LED = led;
        TW = timer_wheel;
        TIM6 = tim;
    }

    #[idle]
    fn idle() -> ! {
        loop {
            wfi();
        }
    }

    #[task(resources = [LED, STATE], priority = 1, capacity = 3)]
    fn callee() {
        if *resources.STATE {
            resources.LED.set_high();
            *resources.STATE = false;
        } else {
            resources.LED.set_low();
            *resources.STATE = true;
        }
    }

    #[interrupt(resources = [TW, LED, STATE, TIM6], priority = 1, spawn = [callee])]
    fn TIM6_DAC() {
        resources.TIM6.clear_it();
        let to_run = resources.TW.tick();

        while let Some(func) = to_run.pop() {
            match func {
                Functions::Callee => spawn.callee().unwrap(),
            }
        };
    }

    extern "C" {
        fn TIM2 ();
    }
};



#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    bkpt();

    loop {
        atomic::compiler_fence(Ordering::SeqCst)
    }
}
