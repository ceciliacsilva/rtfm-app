#![no_main]
#![no_std]

extern crate cortex_m;
extern crate cortex_m_rt;
extern crate rtfm;
extern crate narc_hal;
extern crate embedded_hal;
extern crate sm;

use rtfm::app;
use rtfm::export::wfi;
use core::panic::PanicInfo;
use core::sync::atomic::{self, Ordering};

use cortex_m::asm::bkpt;
use narc_hal::rcc::RccExt;
use narc_hal::gpio::GpioExt;
use narc_hal::flash::FlashExt;
use narc_hal::gpio::{gpioa::PA5, Output, PushPull, gpioa::PA4, Input, PullUp};
use narc_hal::time::U32Ext;
use narc_hal::timer;
use narc_hal::adc::AdcExt;
use narc_hal::timer::TimerExt;
use embedded_hal::digital::OutputPin;
use embedded_hal::digital::InputPin;
use cortex_m::peripheral::syst::SystClkSource;

use sm::sm;

type U8<'a> = &'a mut u8;
type U32 = u32;

#[app(device = narc_hal::stm32l052)]
const APP: () = {
    static mut SM: Sm::Variant = ();
    static mut C: u8 = 0;
    static mut H: u8 = 0;
    static mut ADC: narc_hal::stm32l052::ADC = ();
    static mut ADC_VALUE: u32 = 0;
    static mut LED: PA5<Output<PushPull>> = ();
    static mut B1_COUNTER: u16 = 0;
    static mut B1: PA4<Input<PullUp>> = ();
    static mut T_REF: u32 = 2048;

    #[init]
    fn init() {
        let mut rcc = device.RCC.constrain();
        let mut gpioa = device.GPIOA.split(&mut rcc.iop);
        let mut flash = device.FLASH.constrain();
        let led = gpioa.pa5.into_output(&mut gpioa.moder).push_pull(&mut gpioa.otyper);
        let mut adc: narc_hal::stm32l052::ADC = device.ADC;
        let clocks = rcc.cfgr.freeze(&mut flash.acr);
        let mut tim = device.TIM6.timer(200.hz(), clocks, &mut rcc.apb1);
        let b1 = gpioa.pa4.into_input(&mut gpioa.moder).pull_up(&mut gpioa.pupdr);
        let adc_in = gpioa.pa2.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);
        use Sm::*;
        let sm = Machine::new(Idle).as_enum();

        adc.config(adc_in, &mut rcc.apb2);
        tim.listen(timer::Event::TimeOut);

        core.SYST.set_clock_source(SystClkSource::Core);
        core.SYST.set_reload(2_000); // 1ms
        core.SYST.clear_current();
        core.SYST.enable_counter();
        core.SYST.enable_interrupt();

        SM = sm;
        ADC = adc;
        LED = led;
        B1 = b1;
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

    #[exception(resources = [SM, C, H, ADC_VALUE, LED, T_REF])]
    fn PendSV() {
        use Sm::MachineEvaluation;

        let c = resources.C;
        let h = resources.H;

        let t_amb = resources.ADC_VALUE.lock(|adc_value| *adc_value);
        let t_ref = resources.T_REF.lock(|t_ref| *t_ref);

        resources.SM.lock(|sm_old| {
            let sm = Sm::Variant::eval_machine(sm_old, t_amb, t_ref, 5, c, h);
            *sm_old = sm;
        });

        if *c == 1 {
            resources.LED.set_high();
        } else {
            resources.LED.set_low();
        }
    }

    #[interrupt(resources = [B1_COUNTER, B1], spawn = [increase_tref])]
    fn TIM6_DAC() {
        let b1: u16 = if resources.B1.is_high() { 1 } else { 0 };
        *resources.B1_COUNTER = (*resources.B1_COUNTER << 1) | b1 | 0xe000;
        if *resources.B1_COUNTER == 0xF000 {
            spawn.increase_tref().unwrap();
         }
    }

    #[task(resources = [T_REF], capacity = 1)]
    fn increase_tref() {
        *resources.T_REF += 1;
    }

    /// TODO: spawn.decrease_tref...
    // #[task(resources = [T_REF], capacity = 1)]
    // fn decrease_ref() {
    //     *resources.T_REF += 1;
    // }

    extern "C" {
        fn TIM2();
    }
};

sm!{
    Sm {
        GuardResources {
            { t_amb: U32 }
            { t_ref: U32 }
            { t_h: U32 }
        }
        ActionResources {
            { c: U8 }
            { h: U8 }
        }
        InitialStates { Idle }
        ToHeat { Idle => Warm }
        StopHeat { Warm => Idle }
        ToCool { Idle => Cold }
        StopCold { Cold => Idle }
    }
}

impl Sm::ValidEvent for Sm::ToHeat {
    fn is_enabled (t_amb: U32, t_ref: U32, t_h: U32) -> bool {
        t_amb < (t_ref - t_h)
    }
    fn action (c: U8, h: U8) {
        *c = 1;
        *h = 0;
    }
}

impl Sm::ValidEvent for Sm::StopHeat {
    fn is_enabled (t_amb: U32, t_ref: U32, _t_h: U32) -> bool {
        t_amb > t_ref
    }
    fn action (c: U8, h: U8) {
        *c = 0;
        *h = 0;
    }
}

impl Sm::ValidEvent for Sm::ToCool {
    fn is_enabled (t_amb: U32, t_ref: U32, t_h: U32) -> bool {
        t_amb > (t_ref + t_h)
    }
    fn action (c: U8, h: U8) {
        *c = 0;
        *h = 1;
    }
}

impl Sm::ValidEvent for Sm::StopCold {
    fn is_enabled (t_amb: U32, t_ref: U32, _t_h: U32) -> bool {
        t_amb < t_ref
    }
    fn action (c: U8, h: U8) {
        *c = 0;
        *h = 0;
    }
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    bkpt();

    loop {
        atomic::compiler_fence(Ordering::SeqCst)
    }
}
