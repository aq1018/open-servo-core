#![no_main]
#![no_std]

use defmt_rtt as _;
use panic_probe as _;

use core::cell::RefCell;
use cortex_m::{
    asm,
    interrupt::{free, Mutex},
};
use cortex_m_rt::entry;
use pid::Pid;
use stm32f3::stm32f301 as pac;

use pac::interrupt;

#[derive(Copy, Clone, defmt::Format)]
struct SystemState {
    // Desired Servo Position
    pub setpoint: u16,

    // Current Servo Position
    pub position: u16,

    // PWM Duty Cycle
    pub pwm_duty: i32,

    // Current in mA
    pub current: f32,

    // VDDA Voltage
    pub vdda: f32,

    // Chip Temperature in Celsius
    pub temperature: f32,
}

impl SystemState {
    fn new() -> Self {
        SystemState {
            setpoint: 0,
            position: 0,
            pwm_duty: 0,
            current: 0.0,
            vdda: 0.0,
            temperature: 0.0,
        }
    }
}

const VREFINT_CAL_ADDR: u32 = 0x1FFFF7BA;
const TS_CAL1_ADDR: u32 = 0x1FFFF7B8;
const TS_CAL2_ADDR: u32 = 0x1FFFF7C2;

// RCC parameters
// const RCC_HSE_CLOCK_HZ: u32 = 16_000_000;
const RCC_SYSCLK_HZ: u32 = 72_000_000;

// ADC parameters
const ADC_MAX_VALUE: f32 = 4095.0; // 12-bit ADC

// Current sense parameters
const ISENSE_AIPROPI: f32 = 1500.0; // uA/A
const ISENSE_RESISTOR_OHM: f32 = 2200.0; // Ohm
const ISENSE_FACTOR: f32 = ISENSE_AIPROPI / ISENSE_RESISTOR_OHM; // uA / V

// PWM parameters
const PWM_FREQUENCY: u32 = 20_000;
const PWM_MAX_DUTY: u16 = ((RCC_SYSCLK_HZ / PWM_FREQUENCY / 2) - 1) as u16;
const PWM_REVERSE: bool = false;

// PID controller parameters
const PID_KP: f32 = 40.0;
const PID_KI: f32 = 0.0;
const PID_KD: f32 = 0.0;
const PID_KP_MAX: f32 = PWM_MAX_DUTY as f32;
const PID_KI_MAX: f32 = PWM_MAX_DUTY as f32 * 0.8;
const PID_KD_MAX: f32 = PWM_MAX_DUTY as f32 * 0.8;
const PID_OUTPUT_MAX: f32 = PWM_MAX_DUTY as f32;

static ADC_DATA: Mutex<RefCell<Option<&[u16; 5]>>> = Mutex::new(RefCell::new(Option::None));
static PID_CONTROLLER: Mutex<RefCell<Option<Pid<f32>>>> = Mutex::new(RefCell::new(Option::None));
static SYSTEM_STATE: Mutex<RefCell<Option<SystemState>>> = Mutex::new(RefCell::new(Option::None));

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();

    init_rcc(&p);
    init_tim1_pwm(&p);
    init_tim2_counter(&p);
    init_led(&p);
    init_dma(&p);
    init_adc(&p);

    // intialize PID controller
    let mut pid: Pid<f32> = Pid::new(0.0, PID_OUTPUT_MAX);
    pid.p(PID_KP, PID_KP_MAX);
    pid.i(PID_KI, PID_KI_MAX);
    pid.d(PID_KD, PID_KD_MAX);
    free(|cs| PID_CONTROLLER.borrow(cs).replace(Some(pid)));

    // initialize system state
    let system_state = SystemState::new();
    free(|cs| SYSTEM_STATE.borrow(cs).replace(Some(system_state)));

    // enable interrupts
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::TIM2);
        pac::NVIC::unmask(pac::Interrupt::DMA1_CH1);
    }

    defmt::info!("Initialized");

    // start ADC conversion
    p.ADC1.cr.modify(|_, w| w.adstart().start_conversion());

    loop {}
}

// interrupt handler for TIM2
#[interrupt]
fn TIM2() {
    defmt::info!("TIM2 interrupt");

    // toggle PA3
    unsafe {
        (*pac::GPIOA::ptr())
            .odr
            .modify(|r, w| w.odr3().bit(!r.odr3().bit()));
    }

    // clone system state to local variable
    let system_state = free(|cs| SYSTEM_STATE.borrow(cs).borrow().unwrap());
    defmt::info!("{}", system_state);

    // clear interrupt flag
    unsafe {
        (*pac::TIM2::ptr()).sr.modify(|_, w| w.uif().clear());
    }
}

#[interrupt]
fn DMA1_CH1() {
    let dma1 = unsafe { &(*pac::DMA1::ptr()) };
    let [is_complete, is_error] = cortex_m::interrupt::free(|_| {
        [
            dma1.isr.read().tcif1().is_complete(),
            dma1.isr.read().teif1().is_error(),
        ]
    });

    // check interrupt flag for transfer complete
    if !is_complete {
        defmt::info!("Transfer complete interrupt flag not set");
        return;
    }

    // check interrupt flag for transfer error
    if is_error {
        defmt::error!("Transfer error interrupt flag set");
        return;
    }

    // clone ADC_DATA to local variable
    let [vref, pot, isns, mut setpoint, temp] =
        free(|cs| ADC_DATA.borrow(cs).borrow().unwrap().clone());

    // hardcode for now
    setpoint = 2048;

    // calculate pwm duty cycle using PID controller
    let pid_output = free(|cs| {
        let mut pid_option = PID_CONTROLLER.borrow(cs).borrow_mut();
        let pid = pid_option.as_mut().unwrap();
        pid.setpoint(setpoint).next_control_output(pot as f32)
    });
    let mut duty: i32 = pid_output.output as i32;
    if PWM_REVERSE {
        duty = duty * -1;
    }

    // set PWM duty cycle
    let tim1 = unsafe { &(*pac::TIM1::ptr()) };
    if duty > 0 {
        let duty: u16 = duty as u16;

        tim1.ccr1().write(|w| w.ccr().bits(duty));
        tim1.ccr4().write(|w| w.ccr().bits(0));
    } else {
        let duty: u16 = (duty * -1) as u16;

        tim1.ccr1().write(|w| w.ccr().bits(0));
        tim1.ccr4().write(|w| w.ccr().bits(duty));
    }

    // convert ADC values to physical values
    let vdda = convert_vdda(vref);
    let temp = convert_temperature(temp, vdda);
    let current = convert_current(isns, vdda);

    // update system state
    free(|cs| {
        let mut system_state_options = SYSTEM_STATE.borrow(cs).borrow_mut();
        let system_state = system_state_options.as_mut().unwrap();

        system_state.setpoint = setpoint;
        system_state.position = pot;
        system_state.pwm_duty = duty;
        system_state.vdda = vdda;
        system_state.current = current;
        system_state.temperature = temp;
    });

    // clear interrupt flag
    unsafe {
        (*pac::DMA1::ptr()).ifcr.write(|w| w.cgif1().set_bit());
    }
}

fn convert_temperature(adc_value: u16, vdda_value: f32) -> f32 {
    let ts_cal1: u16 = unsafe { core::ptr::read(TS_CAL1_ADDR as *const u16) };
    let ts_cal2: u16 = unsafe { core::ptr::read(TS_CAL2_ADDR as *const u16) };
    let ts_cal1_temp: f32 = 30.0;
    let ts_cal2_temp: f32 = 110.0;
    let ts_cal1_voltage: f32 = 3.3 * ts_cal1 as f32 / ADC_MAX_VALUE;
    let ts_cal2_voltage: f32 = 3.3 * ts_cal2 as f32 / ADC_MAX_VALUE;
    let ts_slope: f32 = (ts_cal2_temp - ts_cal1_temp) / (ts_cal2_voltage - ts_cal1_voltage);
    let ts_offset: f32 = ts_cal1_temp - ts_slope * ts_cal1_voltage;
    let temp_voltage = vdda_value * adc_value as f32 / ADC_MAX_VALUE;
    let temp = ts_slope * temp_voltage + ts_offset;
    return temp;
}

fn convert_vdda(adc_value: u16) -> f32 {
    let vrefint_cal: u16 = unsafe { core::ptr::read(VREFINT_CAL_ADDR as *const u16) };
    let vdda = 3.3 * vrefint_cal as f32 / adc_value as f32;
    return vdda;
}

fn convert_voltage(adc_value: u16, vdda_value: f32) -> f32 {
    vdda_value * adc_value as f32 / ADC_MAX_VALUE
}

fn convert_current(adc_value: u16, vdda_value: f32) -> f32 {
    let isense_voltage = convert_voltage(adc_value, vdda_value);
    let current = isense_voltage * ISENSE_FACTOR / 1000.0; // convert uA to mA
    return current;
}

fn init_dma(p: &pac::Peripherals) {
    // enable DMA1 Channel 1
    p.DMA1.ch1.cr.modify(|_, w| {
        w.pl()
            .low() // low priority
            .circ()
            .enabled() // enable circular mode
            .dir()
            .from_peripheral() // read from peripheral
            .msize()
            .bits16() // 16 bit memory size
            .psize()
            .bits16() // 16 bit peripheral size
            .tcie()
            .enabled() // enable transfer complete interrupt
            .teie()
            .enabled() // enable transfer error interrupt
            .mem2mem()
            .disabled() // memory to memory mode disabled
            .minc()
            .enabled() // increment memory address
            .pinc()
            .disabled() // do not increment peripheral address
    });

    p.DMA1
        .ch1
        .par
        .write(|w| unsafe { w.bits(p.ADC1.dr.as_ptr() as u32) }); // set peripheral address to ADC1_DR

    let adc_data = cortex_m::singleton!(: [u16; 5] = [0; 5]).unwrap();
    p.DMA1
        .ch1
        .mar
        .write(|w| unsafe { w.bits(adc_data.as_ptr() as u32) }); // set memory address to ADC_DATA

    free(|cs| ADC_DATA.borrow(cs).replace(Some(adc_data)));

    // set number of data to transfer to 5 because we are converting 5 channels
    p.DMA1.ch1.ndtr.write(|w| w.ndt().bits(5));

    // enable DMA1 Channel 1
    p.DMA1.ch1.cr.modify(|_, w| w.en().enabled());
}

fn init_adc(p: &pac::Peripherals) {
    let adc1 = &p.ADC1;

    /*
     * Configure ADC
     */
    p.ADC1_2.ccr.modify(|_, w| {
        w.ckmode()
            .sync_div1() // use synchronous clock mode
            .tsen()
            .enabled() // enable temperature sensor
            .vrefen()
            .enabled() // enable VREFINT
    });

    adc1.cfgr.modify(|_, w| {
        w.res()
            .bits12() // set ADC to 12 bit resolution
            .align()
            .right() // set ADC to right aligned
            .cont()
            .single() // set ADC to single conversion mode
            .discen()
            .disabled() // set ADC to disable discontinuous mode
            .dmacfg()
            .circular() // set ADC to circular DMA mode
            .dmaen()
            .enabled() // enable DMA
            .ovrmod()
            .preserve() // set ADC to preserve overrun
            .exten()
            .rising_edge() // set ADC to rising edge trigger
            .extsel()
            .tim1_trgo2() // set ADC to TIM1_TRGO2
    });

    /*
     * ADC calibration
     */
    // enable internal voltage regulator
    adc1.cr.modify(|_, w| w.advregen().intermediate());
    adc1.cr.modify(|_, w| w.advregen().enabled());
    // we need to wait 10 us for the voltage regulator to stabilize
    // we are running at 72 MHz, so 1 us = 72 cycles
    asm::delay(10 * 72);

    // run calibration
    adc1.cr
        .modify(|_, w| w.adcaldif().single_ended().adcal().calibration());
    while adc1.cr.read().adcal().is_calibration() {}

    // disable the internal voltage regulator, the internal analog calibration is kept
    adc1.cr.modify(|_, w| w.advregen().intermediate());
    adc1.cr.modify(|_, w| w.advregen().disabled());

    // ADEN bit cannot be set during ADCAL=1 and 4 ADC clock cycle after the ADCAL bit is
    // cleared by hardware
    // ADC clock cycle is set to be 72 MHz / 1 = 72 MHz
    asm::delay(4);

    /*
     * Configure ADC Channels
     */
    // configure PA0, PA1, PA2 as analog input
    let gpioa = &p.GPIOA;
    gpioa
        .moder
        .modify(|_, w| w.moder0().analog().moder1().analog().moder2().analog());
    gpioa.pupdr.modify(|_, w| {
        w.pupdr0()
            .floating()
            .pupdr1()
            .floating()
            .pupdr2()
            .floating()
    });

    // configure channels for conversion
    adc1.sqr1.modify(|_, w| unsafe {
        w.l()
            .bits(4) // set number of conversions to 5, this register counts from 0
            .sq1()
            .bits(18) // set vrefint ( ch 18 ) as first conversion
            .sq2()
            .bits(1) // set PA0 ( ch 1 )  as second conversion
            .sq3()
            .bits(2) // set PA1 ( ch 2 )  as third conversion
            .sq4()
            .bits(3) // set PA2 ( ch 3 ) as fourth conversion
    });

    adc1.sqr2.modify(|_, w| unsafe {
        w.sq5().bits(16) // set tempature sensor ( ch 16 ) as first conversion
    });

    // set sample time for channels
    adc1.smpr1.modify(|_, w| {
        w.smp1()
            .cycles181_5() // PA0
            .smp2()
            .cycles181_5() // PA1
            .smp3()
            .cycles181_5() // PA2
    });

    adc1.smpr2.modify(|_, w| {
        w.smp16()
            .cycles181_5() // VREFINT
            .smp18()
            .cycles181_5() // Temperature sensor
    });

    /*
     * ENABLE ADC
     */
    adc1.cr.modify(|_, w| w.aden().enabled());
    while adc1.isr.read().adrdy().is_not_ready() {}
}

fn init_led(p: &pac::Peripherals) {
    let gpioa = &p.GPIOA;

    // configure PA3 as output
    gpioa.moder.modify(|_, w| w.moder3().output());

    // configure PA3 as push-pull
    gpioa.otyper.modify(|_, w| w.ot3().push_pull());

    // configure PA3 as high speed output
    gpioa.ospeedr.modify(|_, w| w.ospeedr3().high_speed());

    // configure PA3 as no pull-up, no pull-down
    gpioa.pupdr.modify(|_, w| w.pupdr3().floating());

    // set PA3 to low
    gpioa.odr.modify(|_, w| w.odr3().low());
}

fn init_tim2_counter(p: &pac::Peripherals) {
    let tim2 = &p.TIM2;

    // set prescaler to 72, so that the counter will increment every 1 us
    tim2.psc.write(|w| w.psc().bits(72));

    // set auto-reload register to 100_000, so that the counter will overflow every 0.1 second
    tim2.arr.write(|w| unsafe { w.arr().bits(100_000) });

    // generate an update event to load the prescaler value to the counter
    tim2.egr.write(|w| w.ug().update());

    // enable update interrupt
    tim2.dier.write(|w| w.uie().enabled());

    // enable TIM2
    tim2.cr1.write(|w| w.cen().enabled());
}

fn init_tim1_pwm(p: &pac::Peripherals) {
    let tim1 = &p.TIM1;

    // Enable auto-reload preload
    tim1.cr1.modify(|_, w| w.arpe().enabled());

    // set pwm mode to center aligned count up
    tim1.cr1.modify(|_, w| w.cms().center_aligned1().dir().up());

    // set PWM frequency to 10 kHz center aligned ( 20 KHz PWM frequency)
    tim1.arr.write(|w| w.arr().bits(3599)); // 72 MHz / 3600 = 20 kHz
    tim1.psc.write(|w| w.psc().bits(0)); // 72 MHz / 1 = 72 MHz

    // generate an update event to load the prescaler value to the counter
    tim1.egr.write(|w| w.ug().update());

    // enable main output enable
    tim1.bdtr.modify(|_, w| w.moe().enabled());

    // set trigger output to output compare 2 (OC3REF)
    // this is used to trigger ADC conversions
    tim1.cr2.modify(|_, w| unsafe {
        // See STM32F301 Reference Manual section 17.4.2.
        // MMS2 register is used to trigger ADC conversions
        w.mms2().bits(0b0110) // OC3REF
    });

    // enable TIM1
    tim1.cr1.modify(|_, w| w.cen().enabled());

    init_tim1_pwm_channels(p);
}

fn init_tim1_pwm_channels(p: &pac::Peripherals) {
    let gpioa = &p.GPIOA;
    let tim1 = &p.TIM1;

    // configure PA8 ( channel 1 ) as PWM output
    {
        // configure PA8 as alternate function
        gpioa.moder.modify(|_, w| w.moder8().alternate());

        // set PA8 alternate function map to TIM1_CH1 ( AF6 )
        gpioa.afrh.modify(|_, w| w.afrh8().af6());

        // configure PA8 as high speed output
        gpioa.ospeedr.modify(|_, w| w.ospeedr8().high_speed());

        // configure PA8 as push-pull
        gpioa.otyper.modify(|_, w| w.ot8().push_pull());

        // configure PA8 as pull down
        gpioa.pupdr.modify(|_, w| w.pupdr8().pull_down());

        // configure Channel 1 as PWM mode 1 and enable preload register
        tim1.ccmr1_output().modify(|_, w| {
            w.oc1pe()
                .enabled() // enable preload register for Channel 1
                .oc1m()
                .pwm_mode1() // PWM mode 1 for Channel 1
        });

        // set PWM duty cycle to 0
        tim1.ccr1().write(|w| w.ccr().bits(0));

        // enable PWM output
        tim1.ccer.write(|w| w.cc1e().set_bit());
    }

    // configure TIM channel 3 to PWM output
    {
        // configure TIM1 as output compare and enable preload register
        tim1.ccmr2_output().modify(|_, w| {
            w.oc3pe()
                .enabled() // enable preload register for Channel 3
                .oc3m()
                .pwm_mode1() // PWM mode 1 for Channel 3
        });

        // Set CCR to center of PWM period
        tim1.ccr3().modify(|_, w| w.ccr().bits(1));

        // enable PWM output
        tim1.ccer.modify(|_, w| w.cc3e().set_bit());
    }

    // configure PA11 ( channel 4 ) as PWM output
    {
        // configure PA11 as alternate function
        gpioa.moder.modify(|_, w| w.moder11().alternate());

        // set PA11 alternate function map to TIM1_CH4 ( AF11 )
        gpioa.afrh.modify(|_, w| w.afrh11().af11());

        // configure PA11 as high speed output
        gpioa.ospeedr.modify(|_, w| w.ospeedr11().high_speed());

        // configure PA11 as push-pull
        gpioa.otyper.modify(|_, w| w.ot11().push_pull());

        // configure PA11 as pull down
        gpioa.pupdr.modify(|_, w| w.pupdr11().pull_down());

        // configure TIM1 as PWM mode 1 and enable preload register
        tim1.ccmr2_output().modify(|_, w| {
            w.oc4pe()
                .enabled() // enable preload register for Channel 4
                .oc4m()
                .pwm_mode1() // PWM mode 1 for Channel 4
        });

        // set PWM duty cycle to 0
        tim1.ccr4().modify(|_, w| w.ccr().bits(0));

        // enable PWM output
        tim1.ccer.modify(|_, w| w.cc4e().set_bit());
    }
}

fn init_rcc(p: &pac::Peripherals) {
    let rcc = &p.RCC;
    let flash = &p.FLASH;

    // enable HSI clock
    rcc.cr.modify(|_, w| w.hsion().on());
    while rcc.cr.read().hsirdy().is_not_ready() {}

    // set HSI as system clock
    rcc.cfgr.modify(|_, w| w.sw().hsi());
    while !rcc.cfgr.read().sws().is_hsi() {}

    // enable HSE clock
    rcc.cr.modify(|_, w| w.hsebyp().not_bypassed().hseon().on());
    while rcc.cr.read().hserdy().is_not_ready() {}

    // configure PLL
    rcc.cfgr2.modify(|_, w| w.prediv().div2()); // HSE is 16 MHz, so divide by 2 to get 8 MHz
    rcc.cfgr.modify(|_, w| {
        w.pllsrc()
            .hse_div_prediv() // Use HSE as PLL source
            .pllmul()
            .mul9() // 8 MHz * 9 = 72 MHz
    });

    // enable PLL
    rcc.cr.modify(|_, w| w.pllon().on());
    while rcc.cr.read().pllrdy().is_not_ready() {}

    // set Flash latency to 2 wait states since we're running at 72 MHz
    flash.acr.write(|w| w.latency().ws2());

    // set AHB prescaler
    rcc.cfgr.modify(|_, w| {
        w.hpre()
            .div1() // AHB is 72 MHz
            .mcopre()
            .div1() // MCO is 72 MHz
            .ppre1()
            .div2() // APB1 is 36 MHz
            .ppre2()
            .div1() // APB2 is 72 MHz
    });

    // Wait for the new prescalers to kick in
    // "The clocks are divided with the new prescaler factor from
    //  1 to 16 AHB cycles after write"
    cortex_m::asm::delay(16);

    // switch to PLL
    rcc.cfgr.modify(|_, w| w.sw().pll());
    while !rcc.cfgr.read().sws().is_pll() {}

    // disable HSI
    rcc.cr.modify(|_, w| w.hsion().off());
    while rcc.cr.read().hsirdy().is_ready() {}

    // reset and enable Power Control
    // Enables access to the PWR_CR register
    // rcc.apb1enr.modify(|_, w| w.pwren().enabled());
    // rcc.apb1rstr.modify(|_, w| w.pwrrst().set_bit());
    // rcc.apb1rstr.modify(|_, w| w.pwrrst().clear_bit());

    // reset and enable System Configuration Controller
    // Enables access to the EXTI and COMP registers
    // rcc.apb2enr.modify(|_, w| w.syscfgen().enabled());
    // rcc.apb2rstr.modify(|_, w| w.syscfgrst().set_bit());
    // rcc.apb2rstr.modify(|_, w| w.syscfgrst().clear_bit());

    // enable FLITF clock
    // This provides read / write access to the Flash memory
    rcc.ahbenr.modify(|_, w| w.flitfen().enabled());

    // enable TIM1
    rcc.apb2enr.modify(|_, w| w.tim1en().enabled());
    rcc.apb2rstr.modify(|_, w| w.tim1rst().set_bit());
    rcc.apb2rstr.modify(|_, w| w.tim1rst().clear_bit());

    // enable TIM2
    rcc.apb1enr.modify(|_, w| w.tim2en().enabled());
    rcc.apb1rstr.modify(|_, w| w.tim2rst().set_bit());
    rcc.apb1rstr.modify(|_, w| w.tim2rst().clear_bit());

    // enable GPIOA, GPIOB, GPIOC
    rcc.ahbenr
        .modify(|_, w| w.iopaen().enabled().iopben().enabled().iopcen().enabled());

    // enable ADC1
    rcc.ahbenr.modify(|_, w| w.adc1en().enabled());
    rcc.ahbrstr.modify(|_, w| w.adc1rst().set_bit());
    rcc.ahbrstr.modify(|_, w| w.adc1rst().clear_bit());

    // enable DMA1
    rcc.ahbenr.modify(|_, w| w.dma1en().enabled());

    // delay for a bit to let the clocks settle
    cortex_m::asm::delay(65535);
}
