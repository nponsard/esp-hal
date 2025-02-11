//! # Capacitive Touch Sensor
//!
//! ## Overview
//!
//! The touch sensor peripheral allows for cheap and robust user interfaces by
//! e.g., dedicating a part of the pcb as touch button.
//!
//! ## Examples
//!
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::touch::{Touch, TouchPad};
//! let touch_pin0 = peripherals.GPIO2;
//! let touch = Touch::continuous_mode(peripherals.TOUCH, None);
//! let mut touchpad = TouchPad::new(touch_pin0, &touch);
//! // ... give the peripheral some time for the measurement
//! let touch_val = touchpad.read();
//! # }
//! ```
//!
//! ## Implementation State:
//!
//! Mostly feature complete, missing:
//!
//! - Touch sensor slope control
//! - Deep Sleep support (wakeup from Deep Sleep)

use core::{default, marker::PhantomData};

use esp32s3::RTC_IO;

use crate::{
    gpio::TouchPin,
    interrupt::InterruptConfigurable,
    peripheral::{Peripheral, PeripheralRef},
    peripherals::{RTC_CNTL, SENS, TOUCH},
    private::{Internal, Sealed},
    rtc_cntl::{rtc, Rtc},
    Async, Blocking, DriverMode,
};

/// A marker trait describing the mode the touch pad is set to.
pub trait TouchMode: Sealed {}

/// Marker struct for the touch peripherals manual trigger mode. In the
/// technical reference manual, this is referred to as "start FSM via software".
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct OneShot;

/// Marker struct for the touch peripherals continuous reading mode. In the
/// technical reference manual, this is referred to as "start FSM via timer".
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Continuous;

impl TouchMode for OneShot {}
impl TouchMode for Continuous {}
impl Sealed for OneShot {}
impl Sealed for Continuous {}

/// Touchpad threshold type.
#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ThresholdMode {
    /// Pad is considered touched if value is greater than threshold.
    GreaterThan,
    /// Pad is considered touched if value is less than threshold.
    LessThan,
}

/// Configurations for the touch pad driver
#[derive(Debug, Copy, Clone, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TouchConfig {
    /// The [`ThresholdMode`] for the pads. Defaults to
    /// `ThresholdMode::LessThan`
    pub threshold_mode: Option<ThresholdMode>,
    /// Duration of a single measurement (in cycles of the 8 MHz touch clock).
    /// Defaults to `0x7fff`
    pub measurement_duration: Option<u16>,
    /// Sleep cycles for the touch timer in [`Continuous`]-mode. Defaults to
    /// `0x100`
    pub sleep_cycles: Option<u16>,
}

/// a
pub fn touch_ll_read_raw_data() -> [u32; 8] {
    let sens = unsafe { &*SENS::ptr() };

    unsafe {
        sens.sar_touch_conf()
            .write(|w| w.sar_touch_data_sel().bits(0));
        [
            sens.sar_touch_status0().read().sar_touch_scan_curr().bits() as u32,
            sens.sar_touch_status1().read().sar_touch_pad1_data().bits(),
            sens.sar_touch_status2().read().sar_touch_pad2_data().bits(),
            sens.sar_touch_status3().read().sar_touch_pad3_data().bits(),
            sens.sar_touch_status4().read().sar_touch_pad4_data().bits(),
            sens.sar_touch_status5().read().sar_touch_pad5_data().bits(),
            sens.sar_touch_conf().read().sar_touch_data_sel().bits() as u32,
            sens.sar_touch_conf().read().sar_touch_outen().bits() as u32,
        ]
    }
}

/// aa
pub fn outen_read() -> u32 {
    let sens = unsafe { &*SENS::ptr() };
    sens.sar_touch_conf().read().sar_touch_outen().bits() as u32
}

/// a
pub fn outen_clear() {
    let sens = unsafe { &*SENS::ptr() };
    unsafe {
        sens.sar_touch_conf().write(|w| w.sar_touch_outen().bits(0));
    }
}

/// a
pub fn touch_ll_start_fsm() {
    let rtccntl = unsafe { &*RTC_CNTL::ptr() };
    unsafe {
        rtccntl
            .touch_ctrl2()
            .write(|w| w.touch_timer_force_done().bits(0x3));
        rtccntl
            .touch_ctrl2()
            .write(|w| w.touch_timer_force_done().bits(0));
    }

    rtccntl
        .touch_ctrl2()
        .modify(|r, w| w.touch_slp_timer_en().bit(!r.touch_start_force().bit()));

    // rtccntl
    //     .touch_ctrl2()
    //     .write(|w| w.touch_start_en().set_bit());
}
/// aa
pub fn read_fsm_mode() -> bool {
    let rtccntl = unsafe { &*RTC_CNTL::ptr() };
    rtccntl.touch_ctrl2().read().touch_start_force().bit()
}

/// aa
pub fn read_started() -> bool {
    let rtccntl = unsafe { &*RTC_CNTL::ptr() };
    rtccntl.touch_ctrl2().read().touch_slp_timer_en().bit()
}

/// aa
pub fn touch_ll_set_fsm_mode(mode: bool) {
    let rtccntl = unsafe { &*RTC_CNTL::ptr() };
    rtccntl
        .touch_ctrl2()
        .write(|w| w.touch_start_force().bit(mode));
}
fn touch_hal_set_meas_mode(channel: u8, slope: u8, tie: bool) {
    touch_ll_set_slope(channel, slope);
    touch_ll_set_tie_option(channel, tie);
}

fn touch_hal_denoise_set_config(grade: u8, level: u8) {
    let rtccntl = unsafe { &*RTC_CNTL::ptr() };
    unsafe {
        //touch_ll_denoise_set_cap_level
        rtccntl.touch_ctrl2().write(|w| w.touch_refc().bits(level));
        // touch_ll_denoise_set_grade
        rtccntl
            .touch_scan_ctrl()
            .write(|w| w.touch_denoise_res().bits(grade));
    }
}

/// aa
pub fn touch_pad_denoise_set_config(grade: u8, level: u8) {
    touch_hal_set_meas_mode(0, 7, false);
    touch_hal_denoise_set_config(grade, level);
}

fn touch_hal_denoise_enable() {
    let rtccntl = unsafe { &*RTC_CNTL::ptr() };
    rtccntl
        .touch_scan_ctrl()
        .write(|w| w.touch_denoise_en().set_bit());
}

/// aa
pub fn touch_pad_denoise_enable() {
    touch_hal_clear_channel_mask(0);
    touch_hal_denoise_enable();
}

/// aa
pub fn touch_pad_config(pin: u8) {
    touch_pad_io_init(pin);
    touch_hal_config(pin);
    touch_hal_set_channel_mask(pin);
}

fn touch_ll_set_threshold(pin: u8, treshold: u32) {
    let sens = unsafe { &*SENS::ptr() };
    match pin {
        1 => unsafe {
            sens.sar_touch_thres1()
                .write(|w| w.sar_touch_out_th1().bits(treshold));
        },
        2 => unsafe {
            sens.sar_touch_thres2()
                .write(|w| w.sar_touch_out_th2().bits(treshold));
        },
        3 => unsafe {
            sens.sar_touch_thres3()
                .write(|w| w.sar_touch_out_th3().bits(treshold));
        },
        4 => unsafe {
            sens.sar_touch_thres4()
                .write(|w| w.sar_touch_out_th4().bits(treshold));
        },
        5 => unsafe {
            sens.sar_touch_thres5()
                .write(|w| w.sar_touch_out_th5().bits(treshold));
        },
        6 => unsafe {
            sens.sar_touch_thres6()
                .write(|w| w.sar_touch_out_th6().bits(treshold));
        },
        7 => unsafe {
            sens.sar_touch_thres7()
                .write(|w| w.sar_touch_out_th7().bits(treshold));
        },
        8 => unsafe {
            sens.sar_touch_thres8()
                .write(|w| w.sar_touch_out_th8().bits(treshold));
        },
        9 => unsafe {
            sens.sar_touch_thres9()
                .write(|w| w.sar_touch_out_th9().bits(treshold));
        },
        10 => unsafe {
            sens.sar_touch_thres10()
                .write(|w| w.sar_touch_out_th10().bits(treshold));
        },
        11 => unsafe {
            sens.sar_touch_thres11()
                .write(|w| w.sar_touch_out_th11().bits(treshold));
        },
        12 => unsafe {
            sens.sar_touch_thres12()
                .write(|w| w.sar_touch_out_th12().bits(treshold));
        },
        13 => unsafe {
            sens.sar_touch_thres13()
                .write(|w| w.sar_touch_out_th13().bits(treshold));
        },
        14 => unsafe {
            sens.sar_touch_thres14()
                .write(|w| w.sar_touch_out_th14().bits(treshold));
        },

        _ => {
            // panic!("Invalid pin number");
        }
    }
}
fn touch_ll_set_slope(pin: u8, slope: u8) {
    let rtccntl = unsafe { &*RTC_CNTL::ptr() };
    match pin {
        0 => unsafe {
            rtccntl
                .touch_dac()
                .write(|w| w.touch_pad0_dac().bits(slope));
        },
        1 => unsafe {
            rtccntl
                .touch_dac()
                .write(|w| w.touch_pad1_dac().bits(slope));
        },
        2 => unsafe {
            rtccntl
                .touch_dac()
                .write(|w| w.touch_pad2_dac().bits(slope));
        },
        3 => unsafe {
            rtccntl
                .touch_dac()
                .write(|w| w.touch_pad3_dac().bits(slope));
        },
        4 => unsafe {
            rtccntl
                .touch_dac()
                .write(|w| w.touch_pad4_dac().bits(slope));
        },
        5 => unsafe {
            rtccntl
                .touch_dac()
                .write(|w| w.touch_pad5_dac().bits(slope));
        },
        6 => unsafe {
            rtccntl
                .touch_dac()
                .write(|w| w.touch_pad6_dac().bits(slope));
        },
        7 => unsafe {
            rtccntl
                .touch_dac()
                .write(|w| w.touch_pad7_dac().bits(slope));
        },
        8 => unsafe {
            rtccntl
                .touch_dac()
                .write(|w| w.touch_pad8_dac().bits(slope));
        },
        9 => unsafe {
            rtccntl
                .touch_dac()
                .write(|w| w.touch_pad9_dac().bits(slope));
        },
        10 => unsafe {
            rtccntl
                .touch_dac1()
                .write(|w| w.touch_pad10_dac().bits(slope));
        },
        11 => unsafe {
            rtccntl
                .touch_dac1()
                .write(|w| w.touch_pad11_dac().bits(slope));
        },
        12 => unsafe {
            rtccntl
                .touch_dac1()
                .write(|w| w.touch_pad12_dac().bits(slope));
        },
        13 => unsafe {
            rtccntl
                .touch_dac1()
                .write(|w| w.touch_pad13_dac().bits(slope));
        },
        14 => unsafe {
            rtccntl
                .touch_dac1()
                .write(|w| w.touch_pad14_dac().bits(slope));
        },

        _ => {
            // panic!("Invalid pin number");
        }
    }
}

fn touch_ll_set_tie_option(pin: u8, option: bool) {
    let rtcio = unsafe { &*RTC_IO::ptr() };
    rtcio
        .touch_pad(pin as usize)
        .write(|w| w.tie_opt().bit(option));
}
fn touch_hal_config(pin: u8) {
    touch_ll_set_threshold(pin, 0x1FFFFF);
    touch_ll_set_slope(pin, 7);
    touch_ll_set_tie_option(pin, false);
}
fn touch_hal_set_channel_mask(pin: u8) {
    let rtccntl = unsafe { &*RTC_CNTL::ptr() };
    let sens = unsafe { &*SENS::ptr() };
    unsafe {
        rtccntl.touch_scan_ctrl().modify(|r, w| {
            w.touch_scan_pad_map()
                .bits(r.touch_scan_pad_map().bits() | (1 << pin))
        });
        sens.sar_touch_conf().modify(|r, w| {
            w.sar_touch_outen()
                .bits(r.sar_touch_outen().bits() | 1 << pin)
        });
    }
}

fn touch_hal_clear_channel_mask(pin: u8) {
    let rtccntl = unsafe { &*RTC_CNTL::ptr() };
    let sens = unsafe { &*SENS::ptr() };
    unsafe {
        rtccntl.touch_scan_ctrl().modify(|r, w| {
            w.touch_scan_pad_map()
                .bits(!(r.touch_scan_pad_map().bits() & !(((1 << 15) - 1) & 1 << pin)))
        });
        sens.sar_touch_conf().modify(|r, w| {
            w.sar_touch_outen()
                .bits(r.sar_touch_outen().bits() & !(((1 << 15) - 1) & 1 << pin))
        });
    }
}

fn touch_pad_io_init(pin: u8) {
    rtcio_ll_function_select(pin, 0);
    rtcio_hal_set_direction(pin, 3);
    rtcio_hal_pulldown_disable(pin);
    rtcio_hal_pullup_disable(pin);
}

fn rtcio_hal_pulldown_disable(pin: u8) {
    let rtcio = unsafe { &*RTC_IO::ptr() };

    rtcio.touch_pad(pin as usize).write(|w| w.rde().clear_bit());
}

fn rtcio_hal_pullup_disable(pin: u8) {
    let rtcio = unsafe { &*RTC_IO::ptr() };

    rtcio.touch_pad(pin as usize).write(|w| w.rue().clear_bit());
}
fn rtcio_ll_output_mode_set(pin: u8, od: bool) {
    let rtcio = unsafe { &*RTC_IO::ptr() };
    rtcio.pin(pin as usize).write(|w| w.pad_driver().bit(od));
}
fn rtcio_ll_output_enable(pin: u8) {
    let rtcio = unsafe { &*RTC_IO::ptr() };
    rtcio
        .rtc_gpio_enable_w1ts()
        .write(|w| unsafe { w.rtc_gpio_enable_w1ts().bits(1 << pin) });
}
fn rtcio_ll_output_disable(pin: u8) {
    let rtcio = unsafe { &*RTC_IO::ptr() };
    rtcio
        .enable_w1tc()
        .write(|w| unsafe { w.enable_w1tc().bits(1 << pin) });
}
fn rtcio_ll_input_enable(pin: u8) {
    let rtcio = unsafe { &*RTC_IO::ptr() };

    rtcio
        .touch_pad(pin as usize)
        .write(|w| w.fun_ie().set_bit());
}
fn rtcio_ll_input_disable(pin: u8) {
    let rtcio = unsafe { &*RTC_IO::ptr() };

    rtcio
        .touch_pad(pin as usize)
        .write(|w| w.fun_ie().clear_bit());
}

fn rtcio_hal_set_direction(pin: u8, mode: u8) {
    match mode {
        // RTC_GPIO_MODE_INPUT_ONLY
        0 => {
            rtcio_ll_output_mode_set(pin, false);
            rtcio_ll_output_disable(pin);
            rtcio_ll_input_enable(pin);
        }
        // RTC_GPIO_MODE_OUTPUT_ONLY
        1 => {
            rtcio_ll_output_mode_set(pin, false);
            rtcio_ll_output_enable(pin);
            rtcio_ll_input_disable(pin);
        }
        // RTC_GPIO_MODE_INPUT_OUTPU
        2 => {
            rtcio_ll_output_mode_set(pin, false);
            rtcio_ll_output_enable(pin);
            rtcio_ll_input_enable(pin);
        }
        // RTC_GPIO_MODE_DISABLED
        3 => {
            rtcio_ll_output_mode_set(pin, false);
            rtcio_ll_output_disable(pin);
            rtcio_ll_input_disable(pin);
        }
        // RTC_GPIO_MODE_OUTPUT_OD
        4 => {
            rtcio_ll_output_mode_set(pin, true);
            rtcio_ll_output_enable(pin);
            rtcio_ll_input_disable(pin);
        }

        // RTC_GPIO_MODE_INPUT_OUTPUT_OD
        5 => {
            rtcio_ll_output_mode_set(pin, true);
            rtcio_ll_output_enable(pin);
            rtcio_ll_input_enable(pin);
        }
        _ => (),
    }
}

/// a
pub fn read_fun_select(pin: u8) -> u8 {
    let rtcio = unsafe { &*RTC_IO::ptr() };
    rtcio.touch_pad(pin as usize).read().fun_sel().bits()
}

fn rtcio_ll_function_select(pin: u8, func: u8) {
    let sens = unsafe { &*SENS::ptr() };
    let rtcio = unsafe { &*RTC_IO::ptr() };

    if func == 0 {
        sens.sar_peri_clk_gate_conf()
            .write(|w| w.iomux_clk_en().set_bit());

        rtcio
            .touch_pad(pin as usize)
            .write(|w| w.mux_sel().set_bit());

        // rtcio_ll_iomux_func_sel
        rtcio
            .touch_pad(pin as usize)
            .write(|w| unsafe { w.fun_sel().bits(func) });
    } else {
        rtcio
            .touch_pad(pin as usize)
            .write(|w| w.mux_sel().clear_bit());
    }
}

/// aaa
pub fn read_scan_map() -> u16 {
    let rtccntl = unsafe { &*RTC_CNTL::ptr() };
    rtccntl.touch_scan_ctrl().read().touch_scan_pad_map().bits()
}

/// aaa
pub fn touch_hal_init() {
    let rtccntl = unsafe { &*RTC_CNTL::ptr() };
    let sens = unsafe { &*SENS::ptr() };
    // stop the fsm
    unsafe {
        rtccntl.touch_ctrl2().write(|w| {
            w.touch_start_en()
                .clear_bit()
                .touch_slp_timer_en()
                .clear_bit()
        });

        rtccntl
            .touch_ctrl2()
            .write(|w| w.touch_timer_force_done().bits(0x3));
        rtccntl
            .touch_ctrl2()
            .write(|w| w.touch_timer_force_done().bits(0x0));
    }

    // Disable touch interrupt
    rtccntl.int_ena_rtc_w1tc().write(|w| {
        w.touch_done()
            .clear_bit_by_one()
            .touch_active()
            .clear_bit_by_one()
            .touch_inactive()
            .clear_bit_by_one()
            .touch_scan_done()
            .clear_bit_by_one()
            .touch_timeout()
            .clear_bit_by_one()
            .touch_approach_loop_done()
            .clear_bit_by_one()
    });
    // Clear pending interrupts
    rtccntl.int_clr().write(|w| {
        w.touch_done()
            .clear_bit_by_one()
            .touch_active()
            .clear_bit_by_one()
            .touch_inactive()
            .clear_bit_by_one()
            .touch_scan_done()
            .clear_bit_by_one()
            .touch_timeout()
            .clear_bit_by_one()
            .touch_approach_loop_done()
            .clear_bit_by_one()
    });

    // clear channel mask
    unsafe {
        sens.sar_touch_conf().write(|w| w.sar_touch_outen().bits(0));
        rtccntl
            .touch_scan_ctrl()
            .write(|w| w.touch_scan_pad_map().bits(0));
    }

    // clear_trigger_status_mask
    sens.sar_touch_conf()
        .write(|w| w.sar_touch_status_clr().set_bit());

    // set meas time
    rtccntl
        .touch_ctrl1()
        .write(|w| unsafe { w.touch_meas_num().bits(500) });
    unsafe {
        rtccntl
            .touch_ctrl2()
            .write(|w| w.touch_xpd_wait().bits(0xff));
    }

    // set sleep time
    unsafe {
        rtccntl
            .touch_ctrl1()
            .write(|w| w.touch_sleep_cycles().bits(0xf));
    }

    // touch_ll_sleep_low_power true
    rtccntl.touch_ctrl2().write(|w| w.touch_dbias().set_bit());

    // set low and high treshold
    unsafe {
        rtccntl
            .touch_ctrl2()
            .write(|w| w.touch_drefh().bits(3).touch_drefl().bits(0));
    }

    // set voltage attenuation to 2
    unsafe {
        rtccntl.touch_ctrl2().write(|w| w.touch_drange().bits(2));
    }

    // touch_ll_set_idle_channel_connect 1
    rtccntl
        .touch_scan_ctrl()
        .write(|w| w.touch_inactive_connection().set_bit());

    // enable clock gate
    rtccntl
        .touch_ctrl2()
        .write(|w| w.touch_clkgate_en().set_bit());

    // reset benchmark
    unsafe {
        sens.sar_touch_chn_st()
            .write(|w| w.sar_touch_channel_clr().bits((1 << 15) - 1));
        rtccntl
            .touch_approach()
            .write(|w| w.touch_slp_channel_clr().set_bit());
    }
}

/// This struct marks a successfully initialized touch peripheral
pub struct Touch<'d, Tm: TouchMode, Dm: DriverMode> {
    _inner: PeripheralRef<'d, TOUCH>,
    _touch_mode: PhantomData<Tm>,
    _mode: PhantomData<Dm>,
}

impl<Tm: TouchMode, Dm: DriverMode> Touch<'_, Tm, Dm> {
    /// Reset the touch peripheral
    pub fn reset(&self) {
        Self::initialize_common_continuous(None);
    }
    /// Common initialization of the touch peripheral.
    fn initialize_common(config: Option<TouchConfig>) {
        touch_hal_init();

        let rtccntl = unsafe { &*RTC_CNTL::ptr() };

        unsafe {
            rtccntl.touch_dac().write(|w| {
                w.touch_pad0_dac()
                    .bits(7)
                    .touch_pad1_dac()
                    .bits(7)
                    .touch_pad2_dac()
                    .bits(7)
                    .touch_pad3_dac()
                    .bits(7)
                    .touch_pad4_dac()
                    .bits(7)
                    .touch_pad5_dac()
                    .bits(7)
                    .touch_pad6_dac()
                    .bits(7)
                    .touch_pad7_dac()
                    .bits(7)
                    .touch_pad8_dac()
                    .bits(7)
                    .touch_pad9_dac()
                    .bits(7)
            });
            rtccntl.touch_dac1().write(|w| {
                w.touch_pad10_dac()
                    .bits(7)
                    .touch_pad11_dac()
                    .bits(7)
                    .touch_pad12_dac()
                    .bits(7)
                    .touch_pad13_dac()
                    .bits(7)
                    .touch_pad14_dac()
                    .bits(7)
            });
        }
    }

    /// Common parts of the continuous mode initialization.
    fn initialize_common_continuous(config: Option<TouchConfig>) {
        let rtccntl = unsafe { &*RTC_CNTL::ptr() };
        let sens = unsafe { &*SENS::ptr() };

        // temp : ask for raw data
        unsafe {
            sens.sar_touch_conf()
                .write(|w| w.sar_touch_data_sel().bits(0));
        }

        // Default nr of sleep cycles from IDF
        let mut sleep_cyc = 0x1000;
        if let Some(config) = config {
            if let Some(slp) = config.sleep_cycles {
                sleep_cyc = slp;
            }
        }

        Self::initialize_common(config);
        rtccntl
            .touch_scan_ctrl()
            .write(|w| w.touch_denoise_en().set_bit());
        unsafe {
            rtccntl
                .touch_ctrl2()
                .write(|w| w.touch_timer_force_done().bits(0x3));
            rtccntl
                .touch_ctrl2()
                .write(|w| w.touch_timer_force_done().bits(0x0));
        }

        rtccntl.touch_ctrl2().write(|w| {
            w
                // Configure FSM for timer mode
                .touch_start_fsm_en()
                .clear_bit()
                .touch_start_force()
                .clear_bit()
                // start touch fsm
                .touch_slp_timer_en()
                .set_bit()
        });
        rtccntl
            .touch_ctrl1()
            .write(|w| unsafe { w.touch_sleep_cycles().bits(sleep_cyc) });
    }
}
// Async mode and OneShot does not seem to be a sensible combination....
impl<'d> Touch<'d, OneShot, Blocking> {
    /// Initializes the touch peripheral and returns this marker struct.
    /// Optionally accepts configuration options.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    #[doc = crate::before_snippet!()]
    /// # use esp_hal::touch::{Touch, TouchConfig};
    /// let touch_cfg = Some(TouchConfig {
    ///     measurement_duration: Some(0x2000),
    ///     ..Default::default()
    /// });
    /// let touch = Touch::one_shot_mode(peripherals.TOUCH, touch_cfg);
    /// # }
    /// ```
    pub fn one_shot_mode(
        touch_peripheral: impl Peripheral<P = TOUCH> + 'd,
        config: Option<TouchConfig>,
    ) -> Self {
        crate::into_ref!(touch_peripheral);
        let rtccntl = unsafe { &*RTC_CNTL::ptr() };

        // Default nr of sleep cycles from IDF
        let mut sleep_cyc = 0x1000;
        if let Some(config) = config {
            if let Some(slp) = config.sleep_cycles {
                sleep_cyc = slp;
            }
        }

        Self::initialize_common(config);

        rtccntl
            .touch_ctrl1()
            .write(|w| unsafe { w.touch_sleep_cycles().bits(sleep_cyc) });

        rtccntl.touch_ctrl2().write(|w| {
            w
                // Configure FSM for SW mode
                .touch_start_fsm_en()
                .set_bit()
                .touch_start_en()
                .clear_bit()
                .touch_start_force()
                .set_bit()
        });

        Self {
            _inner: touch_peripheral,
            _mode: PhantomData,
            _touch_mode: PhantomData,
        }
    }
}
impl<'d> Touch<'d, Continuous, Blocking> {
    /// Initializes the touch peripheral in continuous mode and returns this
    /// marker struct. Optionally accepts configuration options.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    #[doc = crate::before_snippet!()]
    /// # use esp_hal::touch::{Touch, TouchConfig};
    /// let touch_cfg = Some(TouchConfig {
    ///     measurement_duration: Some(0x3000),
    ///     ..Default::default()
    /// });
    /// let touch = Touch::continuous_mode(peripherals.TOUCH, touch_cfg);
    /// # }
    /// ```
    pub fn continuous_mode(
        touch_peripheral: impl Peripheral<P = TOUCH> + 'd,
        config: Option<TouchConfig>,
    ) -> Self {
        crate::into_ref!(touch_peripheral);

        Self::initialize_common_continuous(config);

        Self {
            _inner: touch_peripheral,
            _mode: PhantomData,
            _touch_mode: PhantomData,
        }
    }
}
impl<'d> Touch<'d, Continuous, Async> {
    /// Initializes the touch peripheral in continuous async mode and returns
    /// this marker struct.
    ///
    /// ## Warning:
    ///
    /// This uses [`RTC_CORE`](crate::peripherals::Interrupt::RTC_CORE)
    /// interrupts under the hood. So the whole async part breaks if you install
    /// an interrupt handler with [`Rtc::set_interrupt_handler()`][1].
    ///
    /// [1]: ../rtc_cntl/struct.Rtc.html#method.set_interrupt_handler
    ///
    /// ## Parameters:
    ///
    /// - `rtc`: The RTC peripheral is needed to configure the required
    ///   interrupts.
    /// - `config`: Optional configuration options.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    #[doc = crate::before_snippet!()]
    /// # use esp_hal::rtc_cntl::Rtc;
    /// # use esp_hal::touch::{Touch, TouchConfig};
    /// let mut rtc = Rtc::new(peripherals.LPWR);
    /// let touch = Touch::async_mode(peripherals.TOUCH, &mut rtc, None);
    /// # }
    /// ```
    pub fn async_mode(
        touch_peripheral: impl Peripheral<P = TOUCH> + 'd,
        rtc: &mut Rtc<'_>,
        config: Option<TouchConfig>,
    ) -> Self {
        crate::into_ref!(touch_peripheral);

        Self::initialize_common_continuous(config);

        rtc.set_interrupt_handler(asynch::handle_touch_interrupt);

        Self {
            _inner: touch_peripheral,
            _mode: PhantomData,
            _touch_mode: PhantomData,
        }
    }
}

/// A pin that is configured as a TouchPad.
pub struct TouchPad<P: TouchPin, Tm: TouchMode, Dm: DriverMode> {
    pin: P,
    _touch_mode: PhantomData<Tm>,
    _mode: PhantomData<Dm>,
}
impl<P: TouchPin> TouchPad<P, OneShot, Blocking> {
    /// (Re-)Start a touch measurement on the pin. You can get the result by
    /// calling [`read`](Self::read) once it is finished.
    pub fn start_measurement(&mut self) {
        // unsafe { &*crate::peripherals::RTC_IO::ptr() }
        //     .touch_pad(1)
        //     .write(|w| unsafe {
        //         w.start()
        //             .set_bit()
        //             .xpd()
        //             .set_bit()
        //             // clear input_enable
        //             .fun_ie()
        //             .clear_bit()
        //             // Connect pin to analog / RTC module instead of standard GPIO
        //             .mux_sel()
        //             .set_bit()
        //             // Disable pull-up and pull-down resistors on the pin
        //             .rue()
        //             .clear_bit()
        //             .rde()
        //             .clear_bit()
        //             .tie_opt()
        //             .clear_bit()
        //             // Select function "RTC function 1" (GPIO) for analog use
        //             .fun_sel()
        //             .bits(0b00)
        //     });

        unsafe { &*crate::peripherals::RTC_CNTL::PTR }
            .touch_ctrl2()
            .modify(|_, w| w.touch_start_en().clear_bit());
        unsafe { &*crate::peripherals::RTC_CNTL::PTR }
            .touch_ctrl2()
            .modify(|_, w| w.touch_start_en().set_bit());
    }
}
impl<P: TouchPin, Tm: TouchMode, Dm: DriverMode> TouchPad<P, Tm, Dm> {
    /// Construct a new instance of [`TouchPad`].
    ///
    /// ## Parameters:
    /// - `pin`: The pin that gets configured as touch pad
    /// - `touch`: The [`Touch`] struct indicating that touch is configured.
    pub fn new(pin: P) -> Self {
        // TODO revert this on drop
        pin.set_touch(Internal);

        Self {
            pin,
            _mode: PhantomData,
            _touch_mode: PhantomData,
        }
    }

    /// Read the current touch pad capacitance counter.
    ///
    /// Usually a lower value means higher capacitance, thus indicating touch
    /// event.
    ///
    /// Returns `None` if the value is not yet ready. (Note: Measurement must be
    /// started manually with [`start_measurement`](Self::start_measurement) if
    /// the touch peripheral is in [`OneShot`] mode).
    pub fn try_read(&mut self) -> Option<u32> {
        if unsafe { &*crate::peripherals::SENS::ptr() }
            .sar_touch_chn_st()
            .read()
            .sar_touch_meas_done()
            .bit_is_set()
        {
            Some(self.pin.touch_measurement(Internal))
        } else {
            None
        }
    }
}
impl<P: TouchPin, Tm: TouchMode> TouchPad<P, Tm, Blocking> {
    /// Blocking read of the current touch pad capacitance counter.
    ///
    /// Usually a lower value means higher capacitance, thus indicating touch
    /// event.
    ///
    /// ## Note for [`OneShot`] mode:
    ///
    /// This function might block forever, if
    /// [`start_measurement`](Self::start_measurement) was not called before. As
    /// measurements are not cleared, the touch values might also be
    /// outdated, if it has been some time since the last call to that
    /// function.
    pub fn read(&mut self) -> u32 {
        // while unsafe { &*crate::peripherals::SENS::ptr() }
        //     .sar_touch_chn_st()
        //     .read()
        //     .sar_touch_meas_done()
        //     .bit_is_clear()
        // {}

        self.pin.touch_measurement(Internal)
    }
    /// check if ready
    pub fn ready(&mut self) -> bool {
        unsafe { &*crate::peripherals::SENS::ptr() }
            .sar_touch_chn_st()
            .read()
            .sar_touch_meas_done()
            .bit_is_set()
    }

    /// check active
    pub fn active(&mut self) -> u16 {
        unsafe { &*crate::peripherals::SENS::ptr() }
            .sar_touch_chn_st()
            .read()
            .sar_touch_pad_active()
            .bits()
    }

    /// Enables the touch_pad interrupt.
    ///
    /// The raised interrupt is actually
    /// [`RTC_CORE`](crate::peripherals::Interrupt::RTC_CORE). A handler can
    /// be installed with [`Rtc::set_interrupt_handler()`][1].
    ///
    /// [1]: ../rtc_cntl/struct.Rtc.html#method.set_interrupt_handler
    ///
    /// ## Parameters:
    /// - `threshold`: The threshold above/below which the pin is considered
    ///   touched. Above/below depends on the configuration of `touch` in
    ///   [`new`](Self::new) (defaults to below).
    ///
    /// ## Example
    pub fn enable_interrupt(&mut self, threshold: u16) {
        self.pin.set_threshold(threshold, Internal);
        internal_enable_interrupt(self.pin.touch_nr(Internal))
    }

    /// Disables the touch pad's interrupt.
    ///
    /// If no other touch pad interrupts are active, the touch interrupt is
    /// disabled completely.
    pub fn disable_interrupt(&mut self) {
        internal_disable_interrupt(self.pin.touch_nr(Internal))
    }

    /// Clears a pending touch interrupt.
    ///
    /// ## Note on interrupt clearing behaviour:
    ///
    /// There is only a single interrupt for the touch pad.
    /// [`is_interrupt_set`](Self::is_interrupt_set) can be used to check
    /// which pins are touchted. However, this function clears the interrupt
    /// status for all pins. So only call it when all pins are handled.
    pub fn clear_interrupt(&mut self) {
        internal_clear_interrupt()
    }

    /// Checks if the pad is touched, based on the configured threshold value.
    pub fn is_interrupt_set(&mut self) -> bool {
        internal_is_interrupt_set(self.pin.touch_nr(Internal))
    }
}

fn internal_enable_interrupt(touch_nr: u8) {
    let rtccntl = unsafe { &*RTC_CNTL::ptr() };
    // enable touch interrupts
    rtccntl.int_ena().write(|w| w.touch_active().set_bit());

    let sens = unsafe { &*SENS::ptr() };
    sens.sar_touch_conf().modify(|r, w| unsafe {
        w.sar_touch_outen()
            .bits(r.sar_touch_outen().bits() | 1 << touch_nr)
    });
}

fn internal_disable_interrupt(touch_nr: u8) {
    let sens = unsafe { &*SENS::ptr() };
    sens.sar_touch_conf().modify(|r, w| unsafe {
        w.sar_touch_outen()
            .bits(r.sar_touch_outen().bits() & !(1 << touch_nr))
    });
    if sens.sar_touch_conf().read().sar_touch_outen().bits() == 0 {
        let rtccntl = unsafe { &*RTC_CNTL::ptr() };
        rtccntl.int_ena().write(|w| w.touch_active().clear_bit());
    }
}

fn internal_disable_interrupts() {
    let sens = unsafe { &*SENS::ptr() };
    sens.sar_touch_conf()
        .write(|w| unsafe { w.sar_touch_outen().bits(0) });
    let rtccntl = unsafe { &*RTC_CNTL::ptr() };
    rtccntl.int_ena().write(|w| w.touch_active().clear_bit());
}

fn internal_clear_interrupt() {
    let rtccntl = unsafe { &*RTC_CNTL::ptr() };
    rtccntl
        .int_clr()
        .write(|w| w.touch_active().clear_bit_by_one());
    let sens = unsafe { &*SENS::ptr() };

    // TODO : find how to clear measurments
    // sens.sar_touch_ctrl2()
    //     .write(|w| w.touch_meas_en_clr().set_bit());
}

fn internal_pins_touched() -> u16 {
    let sens = unsafe { &*SENS::ptr() };

    sens.sar_touch_chn_st().read().sar_touch_pad_active().bits()
}

fn internal_is_interrupt_set(touch_nr: u8) -> bool {
    internal_pins_touched() & (1 << touch_nr) != 0
}

mod asynch {
    use core::{
        sync::atomic::{AtomicU16, Ordering},
        task::{Context, Poll},
    };

    use super::*;
    use crate::{
        asynch::AtomicWaker,
        macros::{handler, ram},
        Async,
    };

    const NUM_TOUCH_PINS: usize = 10;

    static TOUCH_WAKERS: [AtomicWaker; NUM_TOUCH_PINS] =
        [const { AtomicWaker::new() }; NUM_TOUCH_PINS];

    // Helper variable to store which pins need handling.
    static TOUCHED_PINS: AtomicU16 = AtomicU16::new(0);

    #[must_use = "futures do nothing unless you `.await` or poll them"]
    pub struct TouchFuture {
        touch_nr: u8,
    }

    impl TouchFuture {
        pub fn new(touch_nr: u8) -> Self {
            Self { touch_nr }
        }
    }

    impl core::future::Future for TouchFuture {
        type Output = ();

        fn poll(self: core::pin::Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
            TOUCH_WAKERS[self.touch_nr as usize].register(cx.waker());

            let pins = TOUCHED_PINS.load(Ordering::Acquire);

            if pins & (1 << self.touch_nr) != 0 {
                // clear the pin to signal that this pin was handled.
                TOUCHED_PINS.fetch_and(!(1 << self.touch_nr), Ordering::Release);
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        }
    }

    #[handler]
    #[ram]
    pub(super) fn handle_touch_interrupt() {
        let touch_pads = internal_pins_touched();
        for (i, waker) in TOUCH_WAKERS.iter().enumerate() {
            if touch_pads & (1 << i) != 0 {
                waker.wake();
            }
        }
        TOUCHED_PINS.store(touch_pads, Ordering::Relaxed);
        internal_clear_interrupt();
        internal_disable_interrupts();
    }

    impl<P: TouchPin, Tm: TouchMode> TouchPad<P, Tm, Async> {
        /// Wait for the pad to be touched.
        pub async fn wait_for_touch(&mut self, threshold: u16) {
            self.pin.set_threshold(threshold, Internal);
            let touch_nr = self.pin.touch_nr(Internal);
            internal_enable_interrupt(touch_nr);
            TouchFuture::new(touch_nr).await;
        }
    }
}
