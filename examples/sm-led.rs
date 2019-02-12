#![no_main]
#![no_std]

extern crate cortex_m;
extern crate cortex_m_rt;
extern crate rtfm;
extern crate narc_hal;
extern crate embedded_hal;
extern crate sm;
extern crate timer_wheels as tw;

use rtfm::app;
use rtfm::export::wfi;
use core::panic::PanicInfo;
use core::sync::atomic::{self, Ordering};

use rtfm::export::consts::{U10, U1};

use cortex_m::asm::bkpt;
use narc_hal::stm32l052::{TIM6 as TIM6_p};
use narc_hal::rcc::RccExt;
use narc_hal::gpio::GpioExt;
use narc_hal::flash::FlashExt;
use narc_hal::time::U32Ext;
use narc_hal::timer;
use narc_hal::timer::TimerExt;
use narc_hal::pwm::PwmExt;
use embedded_hal::PwmPin;

use tw::TimerWheel;
use sm::sm;

type TwFunctions<'a> =&'a mut tw::TimerWheel<bool, U10, U1>;
type Led<'a> = &'a mut narc_hal::pwm::Pwm<narc_hal::stm32l052::TIM2, narc_hal::pwm::C1>;

#[app(device = narc_hal::stm32l052)]
const APP: () = {
    static mut SM: Sm::Variant = ();
    static mut LED: narc_hal::pwm::Pwm<narc_hal::stm32l052::TIM2, narc_hal::pwm::C1> = ();
    static mut TIM6: timer::Timer<TIM6_p> = ();
    static mut WT: TimerWheel<bool, U10, U1> = ();

    #[init]
    fn init() {
        let mut rcc = device.RCC.constrain();
        let mut gpioa = device.GPIOA.split(&mut rcc.iop);
        let mut flash = device.FLASH.constrain();
        let clocks = rcc.cfgr.freeze(&mut flash.acr);
        let led_green = gpioa.pa5.into_alternate(&mut gpioa.moder).af5(&mut gpioa.afrl);
        let mut tim6 = device.TIM6.timer(1.hz(), clocks, &mut rcc.apb1);
        use Sm::*;
        let mut sm = Machine::new(Idle).as_enum();
        let mut wt = TimerWheel::<_, U10, U1>::new();

        tim6.listen(timer::Event::TimeOut);

        let mut led_pwm = device.TIM2
            .pwm(
                led_green,
                50.hz(),
                clocks,
                &mut rcc.apb1,
            );
        led_pwm.enable();

        sm.eval_machine(&mut wt, &mut led_pwm).unwrap();

        SM = sm;
        LED = led_pwm;
        TIM6 = tim6;
        WT = wt;
    }

    #[idle()]
    fn idle() -> ! {
        loop {
            wfi();
        }
    }

    #[interrupt(resources = [TIM6, WT, SM, LED])]
    fn TIM6_DAC() {
        use Sm::MachineEvaluation;
        resources.TIM6.clear_it();

        let mut has_func = false;
        {
            let to_run = resources.WT.tick();
            while let Some(_) = to_run.pop() {
                has_func = true;
            }
        }

        if has_func {
            resources.SM.eval_machine(&mut resources.WT, resources.LED).unwrap();
        }
    }
};

sm!{
    Sm {
        GuardResources {
        }
        ActionResources {
            { wt: TwFunctions}
            { led: Led }
        }
        InitialStates { Idle }
        Start { Idle => Low }
        Up { Low => High }
        Down { High => Low }
    }
}

impl Sm::ValidEvent for Sm::Start {
    fn is_enabled () -> bool {
        true
    }
    fn action (wt: TwFunctions, led: Led) {
        let max = led.get_max_duty();
        led.set_duty(max / 8);
        // TODO: 'fancy' error handling
        let _ = wt.schedule(1, true);
    }
}

impl Sm::ValidEvent for Sm::Down {
    fn is_enabled () -> bool {
        true
    }
    fn action (wt: TwFunctions, led: Led) {
        let max = led.get_max_duty();
        led.set_duty(max / 8);
        // TODO: 'fancy' error handling
        let _ = wt.schedule(4, true);
    }
}

impl Sm::ValidEvent for Sm::Up {
    fn is_enabled () -> bool {
        true
    }
    fn action (wt: TwFunctions, led: Led) {
        let max = led.get_max_duty();
        led.set_duty(max / 1);
        // TODO: 'fancy' error handling
        let _ = wt.schedule(4, true);
    }
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    bkpt();

    loop {
        atomic::compiler_fence(Ordering::SeqCst)
    }
}
