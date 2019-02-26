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
#[macro_use(block)]
extern crate nb;

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
use narc_hal::gpio::{gpioa::PA3, Output, PushPull, gpioa::PA4, Input, PullUp, gpioa::PA6, gpioa::PA7};
use narc_hal::time::U32Ext;
use narc_hal::timer;
use narc_hal::adc::AdcExt;
use narc_hal::timer::TimerExt;
use narc_hal::serial::{Serial, Rx, Event as UartEvent, Tx};
use narc_hal::pwm::PwmExt;

use embedded_hal::prelude::*;
use embedded_hal::digital::OutputPin;
use embedded_hal::digital::InputPin;

use cortex_m::peripheral::syst::SystClkSource;

use tw::TimerWheel;

pub trait IsEvent {
    fn run(&self, spawn: idle::Spawn);
}

#[derive(Debug)]
pub enum LedAction {
    On([Led; 2]),
    Off([Led; 2]),
    ChangeDuty,
}

impl IsEvent for LedAction {
    fn run(&self, spawn: idle::Spawn) {
        match self {
            LedAction::On(leds) => {
                for led in leds {
                    spawn.led_enable(led.clone()).unwrap();
                }
            },
            LedAction::Off(leds) => {
                for led in leds {
                    spawn.led_disable(led.clone()).unwrap();
                }
            },
            LedAction::ChangeDuty => spawn.led_duty().unwrap(),
        }
    }
}

#[derive(Debug, PartialEq, Clone)]
pub enum Led {
    Yellow,
    Red,
    Green,
}

#[derive(Debug)]
pub enum Uart {
    Decode,
    SendADC,
}

impl IsEvent for Uart {
    fn run(&self, spawn: idle::Spawn) {
        match self {
            Uart::Decode => spawn.uart_decode().unwrap(),
            Uart::SendADC => {
                spawn.uart_send_adc().unwrap();
            },
        }
    }
}

#[derive(Debug)]
pub enum Adc {
    Read,
}

impl IsEvent for Adc {
    fn run(&self, spawn: idle::Spawn) {
        match self {
            Adc::Read => spawn.adc_read().unwrap(),
        }
    }
}

type EventObject = &'static IsEvent;

pub struct Event {
    e: EventObject,
}

unsafe impl Send for Event {}

const END_FRAME: u8 = 10;

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
    static mut DUTY: u16 = 0;
    static mut LED_GREEN: narc_hal::pwm::Pwm<narc_hal::stm32l052::TIM2, narc_hal::pwm::C1> = ();
    static mut LED_RED: PA7<Output<PushPull>> = ();
    static mut LED_YELLOW: PA3<Output<PushPull>> = ();
    static mut B1_COUNTER: u16 = 0;
    static mut B2_COUNTER: u16 = 0;
    static mut B1: PA4<Input<PullUp>> = ();
    static mut B2: PA6<Input<PullUp>> = ();
    static mut TIM6: timer::Timer<TIM6_p> = ();
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
        let led_green = gpioa.pa5.into_alternate(&mut gpioa.moder).af5(&mut gpioa.afrl);
        let led_red = gpioa.pa7.into_output(&mut gpioa.moder).push_pull(&mut gpioa.otyper);
        let led_yellow = gpioa.pa3.into_output(&mut gpioa.moder).push_pull(&mut gpioa.otyper);
        let mut adc: narc_hal::stm32l052::ADC = device.ADC;
        let clocks = rcc.cfgr.freeze(&mut flash.acr);
        let mut tim6 = device.TIM6.timer(200.hz(), clocks, &mut rcc.apb1);
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

        core.SYST.set_clock_source(SystClkSource::Core);
        core.SYST.set_reload(2_000_000); // 1s
        core.SYST.clear_current();
        core.SYST.enable_counter();
        core.SYST.enable_interrupt();

        let (tx, rx, _ri) = serial.split();

        let mut led_green = device.TIM2
            .pwm(
                led_green,
                50.hz(),
                clocks,
                &mut rcc.apb1,
            );
        led_green.enable();

        ADC = adc;
        LED_GREEN = led_green;
        LED_RED = led_red;
        LED_YELLOW = led_yellow;
        B1 = b1;
        B2 = b2;
        TIM6 = tim6;
        WT = wt;

        P = p;
        C = c;

        P_UART = p_uart;
        C_UART = c_uart;
        RX = rx;
        TX = tx;
    }

    #[idle(resources = [C], spawn = [led_enable, led_disable, led_duty, uart_decode, uart_send_adc, adc_read])]
    fn idle() -> ! {
        loop {
            // wfi();

            // DONE dequeue of events items.
            if let Some(item) = resources.C.dequeue() {
                item.e.run(spawn);
            }
        }
    }

    #[exception(resources = [P, WT])]
    fn SysTick() {
        let mut has_func = false;
        {
            let to_run = resources.WT.tick();
            while let Some(_) = to_run.pop() {
                has_func = true;
            }
        }

        if has_func {
            // DONE enqueue disable led
            resources.P.enqueue(
                Event {
                    e: &LedAction::Off([Led::Yellow, Led::Red]),
                }
            );
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
            // DONE enqueue enable led

            resources.P.enqueue(
                Event {
                    e: &LedAction::On([Led::Yellow, Led::Red]),
                }
            );
        }

        if *resources.B2_COUNTER == 0xf000 {
            // DONE enqueue change duty
            resources.P.enqueue(
                Event {
                    e: &Adc::Read,
                }
            );
        }
    }

    #[interrupt(resources = [P, TIMER_COUNTER, P_UART, RX])]
    fn USART1() {
        let data = resources.RX.read().unwrap();
        //CR - end of frame
        if data == END_FRAME {
            let _ = resources.P.enqueue(
                Event {
                    e: &Uart::Decode,
                }
            );
        }
        resources.P_UART.enqueue(data).unwrap();
    }

    /// Capacity > 1
    #[task(resources = [WT, TIMER_COUNTER, LED_YELLOW, LED_RED], capacity = 2)]
    fn led_enable(led: Led) {
        if led == Led::Yellow {
            resources.LED_YELLOW.set_high();
        }
        if led == Led::Red {
            resources.LED_RED.set_high();
        }

        let timer_high = resources.TIMER_COUNTER.lock(|t| *t);
        let _ = resources.WT.schedule(timer_high, true);
    }

    #[task(resources = [LED_YELLOW, LED_RED])]
    fn led_disable(led: Led) {
        if led == Led::Yellow {
            resources.LED_YELLOW.set_low();
        }
        if led == Led::Red {
            resources.LED_RED.set_low();
        }
    }

    #[task(resources = [LED_GREEN, DUTY])]
    fn led_duty(){
        let duty_percent = *resources.DUTY as u32;
        let max = resources.LED_GREEN.get_max_duty();
        let duty = ((duty_percent * max as u32) / 100) as u16;
        resources.LED_GREEN.set_duty(max);
    }

    #[task(resources = [P, C_UART, TIMER_COUNTER])]
    fn uart_decode() {
        let mut vec = Vec::<u8, U3>::new();
        while let Some(c) = resources.C_UART.dequeue() {
            if c == END_FRAME {
                break;
            }
            vec.push((c));
        }
        match &vec[..]{
            [b'v', nh] => {
                resources.TIMER_COUNTER.lock(|t| *t = (*nh - 48) as usize);
            },
            [b'r'] => //DONE read adc
            {
                let _ = resources.P.enqueue(
                    Event {
                        e: &Adc::Read,
                    }
                );
            },
            _ => (),
        };
    }

    #[task(resources = [TX, ADC_VALUE])]
    fn uart_send_adc() {
        let value = *resources.ADC_VALUE;

        // memory transmute (unsafe) is an option here.
        let b3 : u8 = ((value >> 8) & 0xff) as u8;
        let b4 : u8 = (value & 0xff) as u8;

        block!(resources.TX.write(b3)).ok();
        block!(resources.TX.write(b4)).ok();
        block!(resources.TX.write(END_FRAME)).ok();
    }

    #[task(resources = [P, ADC, ADC_VALUE, DUTY])]
    fn adc_read() {
        let value = resources.ADC.read();

        *resources.ADC_VALUE = value;
        resources.P.enqueue(
            Event {
                e: &Uart::SendADC,
            }
        );


        let duty = (value * 100) / 4095;
        *resources.DUTY = duty as u16;

        resources.P.enqueue(
            Event {
                e: &LedAction::ChangeDuty,
            }
        );

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
