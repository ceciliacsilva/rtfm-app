#![no_std]
#![no_main]

#[macro_use]
extern crate cortex_m;
extern crate cortex_m_rt;
extern crate cortex_m_rtfm as rtfm;
extern crate hal;
extern crate embedded_hal;
extern crate cortex_m_semihosting as sh;
extern crate heapless;
#[macro_use(block)]
extern crate nb;

use heapless::Vec; 
use heapless::consts::U8;

use core::panic::PanicInfo;
use core::sync::atomic::{self, Ordering};

use cortex_m::asm::bkpt;

use rtfm::{app, Threshold, Resource};

use embedded_hal::prelude::*;

use hal::rcc::RccExt;
use hal::gpio::GpioExt;
use hal::flash::FlashExt;
use hal::serial::Serial;
use hal::serial::{Rx, Event, Tx, ReleaseInterupt, ClearInterupt};
use hal::time::U32Ext;
use hal::dma::DmaExt;
use hal::dma::CircBufferLinear;
use hal::dma::dma1;
use hal::stm32l052;
use hal::stm32l052::USART2 as USART2_p;
use hal::gpio::{Output, Input, PushPull, PullUp, gpioa::PA5, gpioa::PA4};

enum Command {
    Enable,
    Disable,
    Nothing,
}

const SPACE: u8 = b' ';
const LF: u8 = 10;
const SIZE_BUF: usize = 16;

pub struct FiniteMachine {
    pub state: StateType,
}
#[derive(Debug, PartialEq, Clone, Copy)]
pub enum StateType {
    Start,
    Auto,
    Manual,
    Break,
}

macro_rules! valid_transition {
    ( $self:ident, $new_state:ident, [$fromFirst:ident, $($fromX:ident),* -> $to:ident] ) => {
        if $self.state == StateType::$fromFirst $( || $self.state == StateType::$fromX)*{
            if $new_state == StateType::$to {
                $self.state = $new_state;
            }
        }
    };
    ( $self:ident, $new_state:ident, [$from:ident -> $toFisrt:ident $(,$toX:ident)*] ) => {
        if $self.state == StateType::$from {
            if $new_state == StateType::$toFisrt $( || $new_state == StateType::$toX)* {
                $self.state = $new_state;
            }
        }
    };
}

impl FiniteMachine {
    pub fn new() -> FiniteMachine {
        FiniteMachine{
            state: StateType::Start,
        }
    }

    pub fn start(&mut self) {
        self.state = StateType::Auto;
    }
    
    pub fn transition(&mut self, new_state: StateType){
        valid_transition!(self, new_state, [Auto -> Manual, Auto]);
        valid_transition!(self, new_state, [Manual -> Manual, Auto]);
        valid_transition!(self, new_state, [Manual -> Break]);
        valid_transition!(self, new_state, [Break -> Auto]);
    }
}

app! {
    device: stm32l052,
    resources: {
        static END: bool = false;
        static TX: Tx<USART2_p>;
        static RI: ReleaseInterupt<USART2_p>;
        static LED: PA5<Output<PushPull>>;
        static CB: CircBufferLinear<[u8; SIZE_BUF], dma1::C5>;
        static BUTTON: PA4<Input<PullUp>>;
        static FSM: FiniteMachine;
        static EXTI: stm32l052::EXTI;
    },
    idle: {
        resources: [LED, END, TX, CB, BUTTON, FSM],
    },
    tasks: {
        USART2: {
            priority: 1,
            path: end_frame_callback,
            resources: [END, RI],
        },
        EXTI4_15: {
            priority: 1,
            path: button_callback,
            resources: [BUTTON, LED, FSM, EXTI],
        },
    }
}

fn init(mut p: init::Peripherals, _r: init::Resources) -> init::LateResources {
    let mut rcc = p.device.RCC.constrain();
    let mut flash = p.device.FLASH.constrain();
    let mut gpioa = p.device.GPIOA.split(&mut rcc.iop);
    let mut channels = p.device.DMA1.split(&mut rcc.ahb);
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    p.device.SYSCFG_COMP.exticr2.modify(|_, w| unsafe{ w.exti4().bits(0b0000) });//PA0

    p.device.EXTI.imr.modify(|_, w| w.im4().bit(true));
    p.device.EXTI.ftsr.modify(|_, w| w.ft4().bit(true));
    p.device.EXTI.rtsr.modify(|_, w| w.rt4().bit(true));

    let mut led = gpioa.pa5.into_output(&mut gpioa.moder).push_pull(&mut gpioa.otyper);
    let mut but = gpioa.pa4.into_input(&mut gpioa.moder).pull_up(&mut gpioa.pupdr);

    let tx = gpioa.pa2.into_alternate(&mut gpioa.moder).af4(&mut gpioa.afrl);
    let rx = gpioa.pa3.into_alternate(&mut gpioa.moder).af4(&mut gpioa.afrl);

    let serial = Serial::usart2(
        p.device.USART2,
        (tx, rx),
        9_600.bps(),
        clocks,
        &mut rcc.apb1,
        Some(LF)
    );

    serial.listen(Event::Cmie);

    let (mut tx, mut rx, mut ri) = serial.split();

    let buf = singleton!(: [[u8; SIZE_BUF]; 1] = [[0; SIZE_BUF]; 1]).unwrap();
    let cb = rx.circ_buf_linear(channels.5, buf);

    let mut fsm = FiniteMachine::new();
    fsm.start();

    init::LateResources {
        LED: led,
        TX: tx,
        RI: ri,
        CB: cb,
        BUTTON: but,
        FSM: fsm,
        EXTI: p.device.EXTI,
    }
}

fn idle(mut t: &mut Threshold, mut r: idle::Resources) -> ! {
    loop {
        rtfm::wfi();

        // bkpt();

        if r.END.claim(&mut t, |end, _| *end) {
            let state = r.FSM.claim(&mut t, |fsm, _| 
                match fsm.state {
                    StateType::Auto => StateType::Auto,
                    StateType::Manual => StateType::Manual,
                    StateType::Start => StateType::Start,
                    StateType::Break => StateType::Break,
            });

            let (buf1, buf2) = r.CB.partial_peek();

            let mut vec: Vec<u8, U8> = Vec::new();

            for e in buf1.iter().chain(buf2.iter()){
                if *e != SPACE && *e != LF {
                    // let c = if *e < b'a' { *e + 32 } else { *e };
                    vec.push(*e);
                }
            }

            for v in &vec[..] {
                block!(r.TX.write(*v)).ok();
            }

            let new_state = 
                match &vec[..] {
                    [b'a', b'u', b't', b'o'] => StateType::Auto,
                    [b'm', b'a', b'n', b'u', b'a', b'l'] => StateType::Manual,
                    [b'b', b'r', b'e', b'a', b'k'] => StateType::Break,
                    _ => state,
            };
            
            match new_state {
                StateType::Auto => {
                    let op = vec.get(4);

                    let command =
                        match op {
                            Some(78) | Some (110) => Command::Enable,
                            Some(70) | Some (102) => Command::Disable,
                            _ => Command::Nothing,
                        };
                    
                    match command {
                        Command::Enable => r.LED.claim_mut(&mut t, |led, _| led.set_high()),
                        Command::Disable => r.LED.claim_mut(&mut t, |led, _| led.set_low()),
                        _ => (),
                    }
                },
                _ => (),
            }

            // r.FSM.claim_mut(&mut t, |fsm, _| fsm.state = new_state);
            r.FSM.claim_mut(&mut t, |fsm, _| fsm.transition(new_state));
        }
        
        r.END.claim_mut(&mut t, |end, _| *end = false);
    }
}

fn end_frame_callback(_t: &mut Threshold, mut r: USART2::Resources) {
    *r.END = true;
    r.RI.clear_isr_cmie();
}

fn button_callback(_t: &mut Threshold, mut r: EXTI4_15::Resources) {
    match r.FSM.state {
        StateType::Manual => {
            if r.BUTTON.is_low() {
                r.LED.set_high();
            } else {
                r.LED.set_low();
            }
        },
        _ => (),
    };

    r.EXTI.pr.modify(|_, w| w.pif0().bit(true));
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    bkpt();

    loop {
        atomic::compiler_fence(Ordering::SeqCst)
    }
}