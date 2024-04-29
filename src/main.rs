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
    adc::{Adc, SampleTime},
    gpio::{AnyPin, Input, Level, Output, OutputType, Speed},
    peripherals::TIM3,
    time::{hz, khz},
    timer::{simple_pwm::{PwmPin, SimplePwm}, Channel}
};
use embassy_time::{Delay, Duration, Timer};
use fmt::info;


static PWM_DUTY_CYCLE: Signal<ThreadModeRawMutex, f32> = Signal::new();

/******************************************************************************
 * Async Tasks
 *****************************************************************************/

#[embassy_executor::task]
async fn blink(led_pin: AnyPin)
{

    let mut ld2 = Output::new(led_pin,
                                                        Level::High /* initial level */,
                                                        Speed::Low);

    loop
    {
        // let delay_amt = BLINK_MS.load(Ordering::Relaxed);
        let delay_amt = 500; // ms
        // Timer::after returns a future, await yields execution to the executor
        Timer::after(Duration::from_millis(delay_amt as u64)).await;
        ld2.toggle();
    }
}

#[embassy_executor::task]
async fn pwm_out(mut pwm: SimplePwm<'static, TIM3>) {
    pwm.set_frequency(hz(50));
    // let max = pwm.get_max_duty();
    pwm.enable(Channel::Ch1);
    pwm.enable(Channel::Ch1);

    let min_pos: f32 = 100.0;
    let max_pos: f32 = 6000.0;

    loop
    {
        // waits for a signal on the PWM_DUTY_CYCLE mutex
        let throttle = PWM_DUTY_CYCLE.wait().await as f32; // 0 to 1
        let duty_cycle = (min_pos + (throttle * max_pos) as f32) as u16;
        info!("setting duty cycle to {}", duty_cycle);
        pwm.set_duty(Channel::Ch1, duty_cycle);
    }
}

/// Blocking: play the speaker tone
async fn armed_tone(pin: AnyPin) {
    let mut speaker = Output::new(pin, Level::Low, Speed::Low);
    speaker.set_high();
    Timer::after(Duration::from_millis(2000)).await;
    speaker.set_low();
}

/******************************************************************************
 * Main
 *****************************************************************************/

#[embassy_executor::main]
async fn main(spawner: Spawner) {

    /* RCC configuration */
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
        config.rcc.pll2 = Some(Pll{
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

    /* Peripheral Configuration */
    let mut p = embassy_stm32::init(config);

    /* Armed: 12V -> 3.3V via relay */
    let ssok = Input::new(p.PA3, embassy_stm32::gpio::Pull::Down);
    let r2d = Input::new(p.PA8, embassy_stm32::gpio::Pull::Down);
    Timer::after(Duration::from_millis(100)).await;
    


    let mut ssok_on = false;
    while !ssok_on {
        match ssok.get_level() {
            Level::High => {
                info!("ssok!");
                ssok_on = true
            },
            Level::Low => Timer::after(Duration::from_millis(100)).await,
        }
    }
    
    let mut r2d_on = false;
    while !r2d_on {
        match r2d.get_level() {
            Level::High =>  {
                info!("rtd!");
                r2d_on = true
            },
            Level::Low => Timer::after(Duration::from_millis(100)).await,
        }
    }

    armed_tone(p.PF3.into()).await;

    let ch1 = PwmPin::new_ch1(p.PA6, OutputType::PushPull);
    let pwm = SimplePwm::new(p.TIM3, Some(ch1), None, None, None, khz(10), Default::default());

    let mut adc1 = Adc::new(p.ADC1, &mut Delay);
    adc1.set_sample_time(SampleTime::Cycles32_5);
    // let mut adc1_vrefint_channel = adc1.enable_vrefint();

    /* Tasks */
    spawner.spawn(pwm_out(pwm)).unwrap();

    /* Main Even Loop: Reading throttle position */
    loop {

        /* Read potentiometers */
        let pot1 = adc1.read(&mut p.PF11) as f32;
        let pot1_v = (pot1/65535.0)*3.3;   // ADC1 is 16-bit
        let pot1_per = pot1_v/3.3;
        //info!("pot1: {} or {}V or {}%\n", pot1, pot1_v, pot1_per);

        let pot2 = adc1.read(&mut p.PA0) as f32;
        let pot2_v = (pot2/65535.0)*3.3;  // ADC2 is 16-bit
        let pot2_per = (3.3-pot2_v)/3.3;
        //info!("pot2: {} or {}V or {}%\n", pot2, pot2_v, pot2_per);

        let avg = (pot1_per+pot2_per)/2.0;

        if percent_difference(pot1_per, pot2_per) > 0.10 {
        /* this is an invalid state: report error and close throttle? */
            //info!("Error state!")

        } 
        else {
            PWM_DUTY_CYCLE.signal(avg);
         //   info!("\npot1: {}%\npot2: {}%\navg: {}%", pot1_per, pot2_per, avg);
             /* valid state, set throttle? */
        }

        Timer::after(Duration::from_millis(100)).await;
    }
}

/******************************************************************************
 * Function Definitions
 *****************************************************************************/

pub fn percent_difference(x1: f32, x2: f32) -> f32
{
     abs_diff(x1, x2) / (x1 + x2) / 2.0
}

pub fn abs_diff(x1: f32, x2: f32) -> f32
{
    if x1 > x2 {
        x1 - x2
    } else {
        x2 - x1
    }
}
