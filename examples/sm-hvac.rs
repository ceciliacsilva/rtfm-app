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
use narc_hal::adc::{adc_config, adc_read};
use narc_hal::gpio::{gpioa::PA5, Output, PushPull};
use embedded_hal::digital::OutputPin;
use cortex_m::peripheral::syst::SystClkSource;

use sm::sm;

type U8<'a> = &'a mut u8;

#[app(device = narc_hal::stm32l052)]
const APP: () = {
    static mut SM: Sm::Variant = ();
    static mut C: u8 = 0;
    static mut H: u8 = 0;
    static mut ADC: narc_hal::stm32l052::ADC = ();
    static mut ADC_VALUE: u32 = 0;
    static mut LED: PA5<Output<PushPull>> = ();

    #[init]
    fn init() {
        let mut rcc = device.RCC.constrain();
        let mut gpioa = device.GPIOA.split(&mut rcc.iop);
        let led = gpioa.pa5.into_output(&mut gpioa.moder).push_pull(&mut gpioa.otyper);
        let mut adc: narc_hal::stm32l052::ADC = device.ADC;
        gpioa.pa2.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);
        adc_config(&mut rcc.apb2, &mut adc);

        core.SYST.set_clock_source(SystClkSource::Core);
        core.SYST.set_reload(2_000_000); // 1s
        core.SYST.clear_current();
        core.SYST.enable_counter();
        core.SYST.enable_interrupt();

        use Sm::*;
        let sm = Machine::new(Idle).as_enum();

        SM = sm;
        ADC = adc;
        LED = led;
    }

    #[idle(resources = [LED])]
    fn idle() -> ! {
        // cortex_m::peripheral::SCB::set_pendst();
        loop {
            resources.LED.lock(|led| led.set_high());
            wfi();
        }
    }

    #[exception(resources = [SM, ADC, ADC_VALUE, LED], priority = 1)]
    unsafe fn SysTick () {
        *resources.ADC_VALUE = adc_read(&resources.ADC);
        // set PendSV pending
        cortex_m::peripheral::SCB::set_pendsv();
    }

    #[exception(resources = [SM, C, H, ADC_VALUE, LED])]
    unsafe fn PendSV() {
        resources.LED.set_low();

        use Sm::MachineEvaluation;

        let c = resources.C;
        let h = resources.H;

        let t_amb = resources.ADC_VALUE.lock(|adc_value| *adc_value);

        resources.SM.lock(|sm_old| {
            let sm = Sm::Variant::eval_machine(sm_old, t_amb, 2000, 20, c, h);
            *sm_old = sm;
        });
    }
};

sm!{
    Sm {
        GuardResources {
            { t_amb: u32 }
            { t_ref: u32 }
            { t_h: u32 }
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
    fn is_enabled (t_amb: u32, t_ref: u32, t_h: u32) -> bool {
        t_amb < (t_ref - t_h)
    }
    fn action (_c: U8, _h: U8) {
    }
}

impl Sm::ValidEvent for Sm::StopHeat {
    fn is_enabled (t_amb: u32, t_ref: u32, _t_h: u32) -> bool {
        t_amb > t_ref
    }
    fn action (_c: U8, _h: U8) {
    }
}

impl Sm::ValidEvent for Sm::ToCool {
    fn is_enabled (t_amb: u32, t_ref: u32, t_h: u32) -> bool {
        t_amb > (t_ref + t_h)
    }
    fn action (_c: U8, _h: U8) {
    }
}

impl Sm::ValidEvent for Sm::StopCold {
    fn is_enabled (t_amb: u32, t_ref: u32, _t_h: u32) -> bool {
        t_amb < t_ref
    }
    fn action (_c: U8, _h: U8) {
    }
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    bkpt();

    loop {
        atomic::compiler_fence(Ordering::SeqCst)
    }
}
