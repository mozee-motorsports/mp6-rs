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

/// Current throttle position as percentage of max
static THROTTLE_POS: Signal<ThreadModeRawMutex, f32> = Signal::new();

/******************************************************************************
 * End stop values
 ******************************************************************************/

// Potentiometer Parameters(16 bit min-max, 0 to 65535)
const POT1_MIN: f32 = 12000.0;
const POT1_MAX: f32 = 57500.0;

const POT2_MIN: f32 = 13000.0;
const POT2_MAX: f32 = 57000.0;

const POT_THRESH: u32 = 500;

// Servo Parameters
const SERVO_MIN: f32 = 1000.0; // 1000/20000 = 5%
const SERVO_MAX: f32 = 2000.0; // 2000/20000 = 10%


/******************************************************************************
 * Async Tasks
 *****************************************************************************/

#[embassy_executor::task]
async fn blink(led_pin: AnyPin)
{

    let mut ld2 = Output::new(led_pin,
                                                        Level::High /* initial level */,
                                                        Speed::Low);
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

    // Accodring to ryan: 50Hz servo frequency
    // Duty cycle between 5% and 10%
    pwm.set_frequency(hz(50));
    pwm.enable(Channel::Ch1);

    loop {
        // waits for a signal on the PWM_DUTY_CYCLE mutex
        // duty_cycle = 1000: 5% duty cycle
        // duty_cycle = 2000: 10% duty cycle

        let throttle_pos = THROTTLE_POS.wait().await; // percentage
        let duty_cycle = (SERVO_MIN + (throttle_pos * (SERVO_MAX-SERVO_MIN))) as u16;
        info!("setting duty cycle to {} ({}%)", duty_cycle, (duty_cycle as f32)/(100.0) * 100.0);
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


    /* ssok needs to happen before anything else */
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
    
    /* ready to drive happens next */
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
    THROTTLE_POS.signal(0.0);

    let ch1 = PwmPin::new_ch1(p.PA6, OutputType::PushPull);
    let pwm = SimplePwm::new(p.TIM3, Some(ch1), None, None, None, khz(2), Default::default());
    // info!("max duty cycle: {}", pwm.get_max_duty());

    let mut adc1 = Adc::new(p.ADC1, &mut Delay);
    adc1.set_sample_time(SampleTime::Cycles32_5);
    // let mut adc1_vrefint_channel = adc1.enable_vrefint();

    /* Tasks */
    spawner.spawn(pwm_out(pwm)).unwrap();

    /* Main Even Loop: Reading throttle position */
    loop {

        /* Read potentiometers */

        /*
            Assumption: Pot1 goes from ~0 to ~65535
                                      off to  floor
         */ 
        // let mut pot1 = adc1.read(&mut p.PF11) as f32;
        let pot1 = adc1.read(&mut p.PA0) as u32; // as u32
        let pot2 = (POT2_MAX as u32 + POT2_MIN as u32) - (adc1.read(&mut p.PF11) as u32);
        //let pot2 = adc1.read(&mut p.PF11) as u32;

        let abs_diff = pot1.abs_diff(pot2);
        info!("pot1, pot2 = ({}, {}), (diff: {})", pot1, pot2, abs_diff);

       
        if abs_diff > POT_THRESH {
            info!("Error state, setting to minimum duty cycle (5%)");
            THROTTLE_POS.signal(0.0);
        } else {
            let pot1_pos = (pot1 as f32 - POT1_MIN)/(POT1_MAX - POT1_MIN);
            THROTTLE_POS.signal(pot1_pos);
        }
        // if pot1 < POT1_MIN {
        //     pot1 = POT1_MIN;
        // } else if pot1 > POT1_MAX {
        //     pot1 = POT1_MAX
        // }
        //let pot1_pos = (pot1 - POT1_MIN)/(POT1_MAX - POT1_MIN);

        /*
            Assumption: Pot2 goes from ~0 to ~65535
                                      off to floor
         */
        // let mut pot2 = adc1.read(&mut p.PA0) as f32;
        // if pot2 < POT2_MIN {
        //     pot2 = POT2_MIN;
        // } else if pot2 > POT2_MAX {
        //     pot2 = POT2_MAX
        // }
        // let pot2_pos = (pot2 - POT2_MIN)/(POT2_MAX - POT2_MIN);

        // let avg_pos = (pot1_pos + pot2_pos)/2.0;

        // info!("reading pot1: {}, pot2: {}", pot1, pot2);
        // info!("pot1_pos: {}, pot2_pos: {}, avg_pos: {}", pot1_pos, pot2_pos, avg_pos);
        

        // // maybe measure deviation from average instead of absolute difference?
        // if percent_difference(pot1, pot2) > 0.15 {
        //     /* this is an invalid state: report error and close throttle? */
        //     THROTTLE_POS.signal(0.0);  // min position is handled in PWM routine         
        //     info!("Error state!")
        // } 
        // else {
        //     info!("signaling pot1 = {}", pot1_pos);
        //     THROTTLE_POS.signal(pot1_pos);
        // }

        Timer::after(Duration::from_millis(1000)).await;
    }
}

/******************************************************************************
 * Function Definitions
 *****************************************************************************/

pub fn percent_difference(x1: f32, x2: f32) -> f32 {
    let x1 = x1 + 10.0;
    let x2 = x2 + 10.0;
    abs_diff(x1, x2) / (x1 + x2) / 2.0
}

pub fn abs_diff(x1: f32, x2: f32) -> f32 {
    if x1 > x2 {
        x1 - x2
    } else {
        x2 - x1
    }
}