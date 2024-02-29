#![no_std]
#![no_main]

mod fmt;


use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, signal::Signal};
#[cfg(not(feature = "defmt"))]
use panic_halt as _;

#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_stm32::{
    adc::{Adc, SampleTime}, gpio::{AnyPin, Level, Output, OutputType, Speed}, peripherals::TIM3, time::khz, timer::{simple_pwm::{PwmPin, SimplePwm}, Channel}
};
use embassy_time::{Delay, Duration, Timer};
use fmt::info;


static PWM_DUTY_CYCLE: Signal<ThreadModeRawMutex, f32> = Signal::new();


#[embassy_executor::task]
async fn blink(led_pin: AnyPin) {
    let mut ld2 = Output::new(led_pin, Level::High /* initial level */, Speed::Low);

    loop {
        // let delay_amt = BLINK_MS.load(Ordering::Relaxed);
        let delay_amt = 500; // ms
                             // Timer::after returns a future, await yields execution to the executor
        Timer::after(Duration::from_millis(delay_amt as u64)).await;

        ld2.toggle();
    }
}

#[embassy_executor::task]
async fn pwm_out(mut pwm: SimplePwm<'static, TIM3>) {
    let max = pwm.get_max_duty() as f32;
    loop {
        let throttle = (PWM_DUTY_CYCLE.wait().await as f32)/3.3; // 0 to 1
        let duty_cycle = max*throttle;
        pwm.set_duty(Channel::Ch1, duty_cycle as u16);
    }
}


#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = embassy_stm32::Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hsi = Some(HSIPrescaler::DIV1);
        config.rcc.csi = true;
        config.rcc.pll1 = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL50,
            divp: Some(PllDiv::DIV2),
            divq: Some(PllDiv::DIV8), // SPI1 cksel defaults to pll1_q
            divr: None,
        });
        config.rcc.pll2 = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL50,
            divp: Some(PllDiv::DIV8), // 100mhz
            divq: None,
            divr: None,
        });
        config.rcc.sys = Sysclk::PLL1_P; // 400 Mhz
        config.rcc.ahb_pre = AHBPrescaler::DIV2; // 200 Mhz
        config.rcc.apb1_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.apb2_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.apb3_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.apb4_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.voltage_scale = VoltageScale::Scale1;
        config.rcc.adc_clock_source = AdcClockSource::PLL2_P;
    }
    let mut p = embassy_stm32::init(config);

    let ch1 = PwmPin::new_ch1(p.PA6, OutputType::PushPull);
    let mut pwm = SimplePwm::new(p.TIM3, Some(ch1), None, None, None, khz(10), Default::default());
    let max = pwm.get_max_duty();
    pwm.enable(Channel::Ch1);

    let mut adc2 = Adc::new(p.ADC2, &mut Delay);
    adc2.set_sample_time(SampleTime::Cycles32_5);
    let mut adc2_vrefint_channel = adc2.enable_vrefint();

    let mut adc3 = Adc::new(p.ADC3, &mut Delay);
    adc3.set_sample_time(SampleTime::Cycles32_5);
    let mut adc3_vrefint_channel = adc3.enable_vrefint();

    spawner.spawn(pwm_out(pwm)).unwrap();

    loop {
        //let vrefint = adc3.read_internal(&mut vrefint_channel);
        //info!("vrefint: {}", vrefint);
        let pot1 = adc3.read(&mut p.PC0) as f32;
        let pot1_v = (pot1/4095.0)*3.3;   // ADC3 is 12-bit

        let pot2 = adc2.read(&mut p.PC3) as f32;
        let pot2_v = (pot2/65535.0)*3.3;  // ADC2 is 16-bit
        //
        let avg = (pot1_v+pot2_v)/2.0;

        info!("\npot1: {} V\npot2: {}V\navg: {}V", pot1_v, pot2_v, avg);

        // TODO: if the current is some threshold different from last
        PWM_DUTY_CYCLE.signal(avg);
    }
}
