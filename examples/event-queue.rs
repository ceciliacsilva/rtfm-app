#![no_main]
#![no_std]

extern crate cortex_m;
extern crate cortex_m_rt;
extern crate rtfm;
extern crate narc_hal;
extern crate embedded_hal;
extern crate sm;
extern crate timer_wheels as tw;
extern crate heapless;

use rtfm::app;
use rtfm::export::{wfi, consts::U10, consts::U8, consts::U1};
use core::panic::PanicInfo;
use core::sync::atomic::{self, Ordering};

use heapless::{
    consts::*,
    spsc::{Consumer, Producer, Queue},
    Vec,
};

use cortex_m::asm::bkpt;
use narc_hal::stm32l052::{TIM6 as TIM6_p, TIM2 as TIM2_p, USART1 as USART1_p};
use narc_hal::rcc::RccExt;
use narc_hal::gpio::GpioExt;
use narc_hal::flash::FlashExt;
use narc_hal::gpio::{gpioa::PA5, Output, PushPull, gpioa::PA4, Input, PullUp, gpioa::PA6, gpioa::PA7};
use narc_hal::time::U32Ext;
use narc_hal::timer;
use narc_hal::adc::AdcExt;
use narc_hal::timer::TimerExt;
use narc_hal::serial::{Serial, Rx, Event as UartEvent, Tx};

use embedded_hal::prelude::*;
use embedded_hal::digital::OutputPin;
use embedded_hal::digital::InputPin;

use cortex_m::peripheral::syst::SystClkSource;

use tw::TimerWheel;

#[derive(Debug)]
pub enum LedAction {
    On([Led; 2]),
    Off(Vec<Led, U2>),
    Reset(Led),
}

#[derive(Debug, PartialEq, Clone)]
pub enum Led {
    Green,
    Red,
}

#[derive(Debug)]
pub enum Uart {
    Decode,
}

pub trait IsEvent {
    fn run(&self, spawn: idle::Spawn);
}

impl IsEvent for LedAction {
    fn run(&self, spawn: idle::Spawn) {
        match self {
            LedAction::On(leds) => {
                for led in leds {
                    spawn.led_enable(led.clone()).unwrap();
                }
            },
            _ => ()
        }
    }
}

impl IsEvent for Uart {
    fn run(&self, spawn: idle::Spawn) {
        
    }
}

type EventObject = &'static IsEvent;

pub struct Event {
    e: EventObject,
}

unsafe impl Send for Event {}

#[app(device = narc_hal::stm32l052)]
const APP: () = {
    static mut Q: Option<Queue<Event, U8>> = None;
    static mut P: Producer<'static, Event, U8> = ();
    static mut C: Consumer<'static, Event, U8> = ();

    static mut Q_UART: Option<Queue<u8, U8>> = None;
    static mut P_UART: Producer<'static, u8, U8> = ();
    static mut C_UART: Consumer<'static, u8, U8> = ();

    static mut ADC: narc_hal::stm32l052::ADC = ();
    static mut ADC_VALUE: u32 = 0;
    static mut LED_GREEN: PA5<Output<PushPull>> = ();
    static mut LED_RED: PA7<Output<PushPull>> = ();
    static mut B1_COUNTER: u16 = 0;
    static mut B2_COUNTER: u16 = 0;
    static mut B1: PA4<Input<PullUp>> = ();
    static mut B2: PA6<Input<PullUp>> = ();
    static mut T_REF: u32 = 2048;
    static mut TIM6: timer::Timer<TIM6_p> = ();
    static mut TIM2: timer::Timer<TIM2_p> = ();
    static mut RX: Rx<USART1_p> = ();
    static mut TX: Tx<USART1_p> = ();

    static mut WT: TimerWheel<bool, U10, U1> = ();

    static mut TIMER_COUNTER: usize = 1;

    static mut VEC: Vec<Led, U2> = Vec::<_, U2>::new();

    #[init(resources = [Q, Q_UART])]
    fn init() {
        let mut rcc = device.RCC.constrain();
        let mut gpioa = device.GPIOA.split(&mut rcc.iop);
        let mut flash = device.FLASH.constrain();
        let led_green = gpioa.pa5.into_output(&mut gpioa.moder).push_pull(&mut gpioa.otyper);
        let led_red = gpioa.pa7.into_output(&mut gpioa.moder).push_pull(&mut gpioa.otyper);
        let mut adc: narc_hal::stm32l052::ADC = device.ADC;
        let clocks = rcc.cfgr.freeze(&mut flash.acr);
        let mut tim6 = device.TIM6.timer(200.hz(), clocks, &mut rcc.apb1);
        let mut tim2 = device.TIM2.timer(1.hz(), clocks, &mut rcc.apb1);
        let b1 = gpioa.pa4.into_input(&mut gpioa.moder).pull_up(&mut gpioa.pupdr);
        let b2 = gpioa.pa6.into_input(&mut gpioa.moder).pull_up(&mut gpioa.pupdr);
        let adc_in = gpioa.pa2.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);
        let wt = TimerWheel::<_, U10, U1>::new();

        let tx = gpioa.pa9.into_alternate(&mut gpioa.moder).af4(&mut gpioa.afrh);
        let rx = gpioa.pa10.into_alternate(&mut gpioa.moder).af4(&mut gpioa.afrh);

        let serial = Serial::usart1(
            device.USART1,
            (tx, rx),
            9_600.bps(),
            clocks,
            &mut rcc.apb2,
            None
        );

        serial.listen(UartEvent::Rxne);

        *resources.Q = Some(Queue::new());
        let (p, c) = resources.Q.as_mut().unwrap().split();

        *resources.Q_UART = Some(Queue::new());
        let (p_uart, c_uart) = resources.Q_UART.as_mut().unwrap().split();

        adc.config(adc_in, &mut rcc.apb2);
        tim6.listen(timer::Event::TimeOut);
        tim2.listen(timer::Event::TimeOut);

        core.SYST.set_clock_source(SystClkSource::Core);
        core.SYST.set_reload(2_000_000); // 1s
        core.SYST.clear_current();
        core.SYST.enable_counter();
        core.SYST.enable_interrupt();

        let (tx, rx, _ri) = serial.split();

        ADC = adc;
        LED_GREEN = led_green;
        LED_RED = led_red;
        B1 = b1;
        B2 = b2;
        TIM6 = tim6;
        TIM2 = tim2;
        WT = wt;

        P = p;
        C = c;

        P_UART = p_uart;
        C_UART = c_uart;
        RX = rx;
        TX = tx;
    }

    #[idle(spawn = [led_enable])]
    fn idle() -> ! {
        loop {
            wfi();

            // TODO dequeue of events items.
        }
    }

    #[exception(resources = [WT])]
    fn SysTick() {
        let mut has_func = false;
        {
            let to_run = resources.WT.tick();
            while let Some(_) = to_run.pop() {
                has_func = true;
            }
        }

        if has_func {
            // TODO enqueue disable led
        }
    }

    #[interrupt(resources = [P, TIM6, B1_COUNTER, B1, B2_COUNTER, B2, VEC])]
    fn TIM6_DAC() {
        resources.TIM6.clear_it();

        let b1: u16 = if resources.B1.is_high() { 1 } else { 0 };
        *resources.B1_COUNTER = (*resources.B1_COUNTER << 1) | b1 | 0xe000;
        let b2: u16 = if resources.B2.is_high() { 1 } else { 0 };
        *resources.B2_COUNTER = (*resources.B2_COUNTER << 1) | b2 | 0xe000;

        if *resources.B1_COUNTER == 0xf000 {
            // TODO enqueue enable led

            resources.P.enqueue(
                Event {
                    // e: &LedAction::On(resources.VEC),
                    e: &LedAction::On([Led::Green, Led::Red]),
                }
            );
        }

        if *resources.B2_COUNTER == 0xf000 {
            // TODO enqueue toggle state in 1 second
        }
    }

    #[interrupt(resources = [P, TIMER_COUNTER, P_UART, RX])]
    fn USART1() {
        let data = resources.RX.read().unwrap();
        resources.P_UART.enqueue(data).unwrap();
        let _ = resources.P.enqueue(
            Event {
                e: &Uart::Decode,
            }
        );
    }

    /// Capacity > 1
    #[task(resources = [WT, TIMER_COUNTER, LED_GREEN, LED_RED], capacity = 2)]
    fn led_enable(led: Led) {
        if led == Led::Green {
            resources.LED_GREEN.set_high();
        }
        if led == Led::Red {
            resources.LED_RED.set_high();
        }

        let timer_high = resources.TIMER_COUNTER.lock(|t| *t);
        let _ = resources.WT.schedule(timer_high, true);
    }

    extern "C" {
        fn SPI1 ();
    }

};


#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    bkpt();

    loop {
        atomic::compiler_fence(Ordering::SeqCst)
    }
}
