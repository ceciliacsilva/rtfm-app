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

use cortex_m::asm::bkpt;
use narc_hal::stm32l052::{TIM6 as TIM6_p, TIM2 as TIM2_p};
use narc_hal::rcc::RccExt;
use narc_hal::gpio::GpioExt;
use narc_hal::flash::FlashExt;
use narc_hal::gpio::{gpioa::PA5, Output, PushPull, gpioa::PA4, Input, PullUp, gpioa::PA6, gpioa::PA7};
use narc_hal::time::U32Ext;
use narc_hal::timer;
use narc_hal::adc::AdcExt;
use narc_hal::timer::TimerExt;
use embedded_hal::digital::OutputPin;
use embedded_hal::digital::InputPin;
use cortex_m::peripheral::syst::SystClkSource;

use tw::TimerWheel;
use sm::sm;

type LedGreen<'a> = &'a mut PA5<Output<PushPull>>;
type LedRed<'a> = &'a mut PA7<Output<PushPull>>;
type TwFunctions<'a> =&'a mut rtfm::Exclusive<'a, tw::TimerWheel<Functions>>;
type U32 = u32;
type BoolRTFM<'a> = &'a mut rtfm::Exclusive<'a, bool>;
type Bool = bool;

pub enum Functions {
    After5Seg,
    After6Seg,
}

#[app(device = narc_hal::stm32l052)]
const APP: () = {
    static mut SM: Sm::Variant = ();
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
    static mut WT: TimerWheel<Functions> = ();
    static mut ATLEAST6SEG: bool = false;
    static mut ATLEAST5SEG: bool = false;

    #[init]
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
        use Sm::*;
        let sm = Machine::new(Idle).as_enum();
        let wt = TimerWheel::new();

        adc.config(adc_in, &mut rcc.apb2);
        tim6.listen(timer::Event::TimeOut);
        tim2.listen(timer::Event::TimeOut);
        core.SYST.set_clock_source(SystClkSource::Core);
        core.SYST.set_reload(2_000); // 1ms
        core.SYST.clear_current();
        core.SYST.enable_counter();
        core.SYST.enable_interrupt();

        SM = sm;
        ADC = adc;
        LED_GREEN = led_green;
        LED_RED = led_red;
        B1 = b1;
        B2 = b2;
        TIM6 = tim6;
        TIM2 = tim2;
        WT = wt;
    }

    #[idle()]
    fn idle() -> ! {
        loop {
            wfi();
        }
    }

    #[exception(resources = [SM, ADC, ADC_VALUE], priority = 4)]
    fn SysTick () {
        *resources.ADC_VALUE = resources.ADC.read();

        cortex_m::peripheral::SCB::set_pendsv();
    }

    #[exception(resources = [SM, ADC_VALUE, LED_GREEN, LED_RED, T_REF, WT, ATLEAST6SEG, ATLEAST5SEG], priority = 2)]
    fn PendSV() {
        use Sm::MachineEvaluation;

        let led_green = resources.LED_GREEN;
        let led_red = resources.LED_RED;
        let mut wt = resources.WT;//.lock(|val| val);

        let mut at_least6_seg = resources.ATLEAST6SEG;//.lock(|val| val);
        let mut at_least5_seg = resources.ATLEAST5SEG;//.lock(|val| val);

        let t_amb = resources.ADC_VALUE.lock(|adc_value| *adc_value);
        let t_ref = resources.T_REF.lock(|t_ref| *t_ref);

        resources.SM.lock(|sm_old| {
            // let result = sm_old.eval_machine(t_amb, t_ref, 100, *at_least5_seg, *at_least6_seg,
            //                                  led_green, led_red, &mut wt,
            //                                  &mut at_least5_seg, &mut at_least6_seg);
            // match result {
            //     Ok(_) => (),
            //     Err(_) => {
            //         *sm_old = Sm::Machine::new(Sm::Idle).as_enum();
            //     }
            // }
            sm_old.eval_machine(t_amb, t_ref, 100, *at_least5_seg, *at_least6_seg,
                                led_green, led_red, &mut wt,
                                &mut at_least5_seg, &mut at_least6_seg).unwrap();
        });
    }

    #[interrupt(resources = [TIM6, B1_COUNTER, B1, B2_COUNTER, B2, T_REF])]
    fn TIM6_DAC() {
        resources.TIM6.clear_it();

        let b1: u16 = if resources.B1.is_high() { 1 } else { 0 };
        *resources.B1_COUNTER = (*resources.B1_COUNTER << 1) | b1 | 0xe000;
        let b2: u16 = if resources.B2.is_high() { 1 } else { 0 };
        *resources.B2_COUNTER = (*resources.B2_COUNTER << 1) | b2 | 0xe000;

        if *resources.B1_COUNTER == 0xf000 {
            resources.T_REF.lock(|t_ref| *t_ref += 10);
        }

        if *resources.B2_COUNTER == 0xf000 {
            resources.T_REF.lock(|t_ref| *t_ref -= 10);
        }
    }

    #[interrupt(resources = [TIM2, WT], spawn = [after_6_seg, after_5_seg])]
    fn TIM2() {
        resources.TIM2.clear_it();
        resources.WT.lock(
            |wt|
            {
                let to_run = wt.tick();
                while let Some(func) = to_run.pop() {
                    match func {
                        Functions::After6Seg => spawn.after_6_seg().unwrap(),
                        Functions::After5Seg => spawn.after_5_seg().unwrap(),
                    }
                };
            });
    }

    #[task(resources = [ATLEAST6SEG])]
    fn after_6_seg() {
        resources.ATLEAST6SEG.lock(|var| *var = true);
    }

    #[task(resources = [ATLEAST5SEG])]
    fn after_5_seg() {
        resources.ATLEAST5SEG.lock(|var| *var = true);
    }

    extern "C" {
        fn SPI1 ();
    }

};

sm!{
    Sm {
        GuardResources {
            { t_amb: U32 }
            { t_ref: U32 }
            { t_h: U32 }
            { after_at_least5_seg: Bool }
            { after_at_least6_seg: Bool }
        }
        ActionResources {
            { c: LedGreen }
            { h: LedRed }
            { wt: TwFunctions }
            { timer5seg: BoolRTFM }
            { timer6seg: BoolRTFM }
        }
        InitialStates { Idle }
        ToHeat { Idle => Warm }
        StopHeat { Warm => Idle }
        ToCool { Idle => Cold }
        StopCold { Cold => Idle }
    }
}

impl Sm::ValidEvent for Sm::ToHeat {
    fn is_enabled (t_amb: U32, t_ref: U32, t_h: U32, _: Bool, _: Bool) -> bool {
        t_amb < (t_ref - t_h)
    }
    fn action (c: LedGreen, h: LedRed, wt: TwFunctions, _: BoolRTFM, _: BoolRTFM) {
        c.set_high();
        h.set_low();
        // TODO: 'fancy' error handling
        let _ = wt.schedule(6, Functions::After6Seg);
    }
}

impl Sm::ValidEvent for Sm::StopHeat {
    fn is_enabled (t_amb: U32, t_ref: U32, _t_h: U32, _: Bool, after_at_least6_seg: Bool) -> bool {
        t_amb >= t_ref && after_at_least6_seg
    }
    fn action (c: LedGreen, h: LedRed, _wt: TwFunctions, _timer5seg: BoolRTFM, timer6seg: BoolRTFM) {
        c.set_low();
        h.set_low();

        use rtfm::Mutex;
        timer6seg.lock(|val| *val = false);
    }
}

impl Sm::ValidEvent for Sm::ToCool {
    fn is_enabled (t_amb: U32, t_ref: U32, t_h: U32, _: Bool, _: Bool) -> bool {
        t_amb > (t_ref + t_h)
    }
    fn action (c: LedGreen, h: LedRed, wt: TwFunctions, _: BoolRTFM, _: BoolRTFM) {
        c.set_low();
        h.set_high();
        // TODO: 'fancy' error handling
        let _ = wt.schedule(5, Functions::After5Seg);
    }
}

impl Sm::ValidEvent for Sm::StopCold {
    fn is_enabled (t_amb: U32, t_ref: U32, _t_h: U32, after_at_least5_seg: Bool, _: bool) -> bool {
        t_amb <= t_ref && after_at_least5_seg
    }
    fn action (c: LedGreen, h: LedRed, _wt: TwFunctions, timer5seg: BoolRTFM, _timer6seg: BoolRTFM) {
        c.set_low();
        h.set_low();

        use rtfm::Mutex;
        timer5seg.lock(|val| *val = false);
    }
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    bkpt();

    loop {
        atomic::compiler_fence(Ordering::SeqCst)
    }
}
