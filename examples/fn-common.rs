#![no_main]
#![no_std]

extern crate cortex_m;
extern crate cortex_m_rt;
extern crate rtfm;
extern crate narc_hal;
extern crate embedded_hal;
extern crate timer_wheels;
extern crate heapless;

use rtfm::app;
use rtfm::export::wfi;
use core::panic::PanicInfo;
use core::sync::atomic::{self, Ordering};

use cortex_m::asm::bkpt;

use narc_hal::rcc::RccExt;
use narc_hal::flash::FlashExt;
use narc_hal::gpio::GpioExt;
use narc_hal::gpio::{gpioa::PA5, Output, PushPull};
use narc_hal::delay::Delay;

use embedded_hal::digital::OutputPin;
// use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::prelude::*;
use heapless::{
    consts::*,
    spsc::{Consumer, Producer, Queue},
};

#[derive(Debug)]
pub enum EventLed {
    On,
    Off,
}

#[derive(Debug)]
pub enum EventTime {
    Delay,
}

pub trait IsEvent {
    fn run(&self, spawn: idle::Spawn);
}

impl IsEvent for EventLed {
    fn run(&self, spawn: idle::Spawn) {
        match self {
            EventLed::On => spawn.led_on(true).unwrap(),
            EventLed::Off => spawn.led_off().unwrap(),
        }
    }
}

impl IsEvent for EventTime {
    fn run(&self, spawn: idle::Spawn) {
        match self {
            EventTime::Delay => spawn.one_second().unwrap(),
        }
    }
}

type EventObject = &'static IsEvent;

pub struct Event {
    e: EventObject,
}

unsafe impl Send for Event {}

#[app(device = narc_hal::stm32l052)]
const APP: () = {
    static mut EXTI: narc_hal::stm32l052::EXTI = ();
    static mut Q: Option<Queue<Event, U4>> = None;
    static mut P: Producer<'static, Event, U4> = ();
    static mut C: Consumer<'static, Event, U4> = ();
    static mut LED: PA5<Output<PushPull>> = ();
    static mut DELAY: Delay = ();

    #[init(resources = [Q])]
    fn init() {
        let mut rcc = device.RCC.constrain();
        let mut flash = device.FLASH.constrain();
        let mut gpioa = device.GPIOA.split(&mut rcc.iop);
        let clocks = rcc.cfgr.freeze(&mut flash.acr);
        let led = gpioa.pa5.into_output(&mut gpioa.moder).push_pull(&mut gpioa.otyper);
        let _but = gpioa.pa4.into_input(&mut gpioa.moder).pull_up(&mut gpioa.pupdr);
        let delay = Delay::new(core.SYST, clocks);

        device.SYSCFG_COMP.exticr2.modify(|_, w| unsafe{ w.exti4().bits(0b0000) });//PA0
        device.EXTI.imr.modify(|_, w| w.im4().bit(true));
        device.EXTI.ftsr.modify(|_, w| w.ft4().bit(true));

        *resources.Q = Some(Queue::new());
        let (p, c) = resources.Q.as_mut().unwrap().split();

        EXTI = device.EXTI;
        P = p;
        C = c;
        LED = led;
        DELAY = delay;
    }

    #[idle(resources = [C], spawn = [led_on, led_off, one_second])]
    fn idle() -> ! {
        loop {
            if let Some(event) = resources.C.dequeue(){
                event.e.run(spawn);
            }
            // wfi(); 
        }
    }

    #[interrupt(resources = [P, EXTI])]
    fn EXTI4_15() {
        resources.P.enqueue(Event{
            e: &EventLed::On
        });
        resources.EXTI.pr.modify(|_, w| w.pif0().bit(true));
    }

    #[task(resources = [P, LED], spawn = [one_second])]
    fn led_on(_msg: bool) {
        resources.LED.set_high();
        resources.P.enqueue(Event{
            e: &EventTime::Delay
        });
    }

    #[task(resources = [P, DELAY])]
    fn one_second() {
        resources.DELAY.delay_ms(1_000_u16);
        resources.P.enqueue(Event{
            e: &EventLed::Off
        });
    }

    #[task(resources = [LED])]
    fn led_off() {
        resources.LED.set_low();
    }
    extern "C" {
        fn SPI1();
    }

};

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    bkpt();

    loop {
        atomic::compiler_fence(Ordering::SeqCst)
    }
}
