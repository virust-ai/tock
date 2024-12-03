// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2022.

//! Board file for Nuen Motor Sona board
//!
//! - <https://nuenmoto.com/en>

#![no_std]
// Disable this attribute when documenting, as a workaround for
// https://github.com/rust-lang/rust/issues/62184.
#![cfg_attr(not(doc), no_main)]
#![deny(missing_docs)]
use core::ptr::{addr_of, addr_of_mut};

use capsules_core::virtualizers::virtual_alarm::VirtualMuxAlarm;
use components::rng::RngComponent;
use kernel::capabilities;
use kernel::component::Component;
use kernel::hil::i2c::I2CMaster;
use kernel::hil::led::LedLow;
use kernel::platform::{KernelResources, SyscallDriverLookup};
use kernel::scheduler::round_robin::RoundRobinSched;
use kernel::{create_capability, debug, static_init};
use stm32f412rg::chip_specs::Stm32f412Specs;
use stm32f412rg::clocks::hsi::HSI_FREQUENCY_MHZ;
use stm32f412rg::interrupt_service::Stm32f412rgDefaultPeripherals;
use stm32f412rg::rcc::PllSource;
/// Support routines for debugging I/O.
pub mod io;

// Number of concurrent processes this platform supports.
const NUM_PROCS: usize = 4;

// Actual memory for holding the active process structures.
static mut PROCESSES: [Option<&'static dyn kernel::process::Process>; NUM_PROCS] =
    [None, None, None, None];

static mut CHIP: Option<&'static stm32f412rg::chip::Stm32f4xx<Stm32f412rgDefaultPeripherals>> =
    None;
static mut PROCESS_PRINTER: Option<&'static capsules_system::process_printer::ProcessPrinterText> =
    None;

// How should the kernel respond when a process faults.
const FAULT_RESPONSE: capsules_system::process_policies::PanicFaultPolicy =
    capsules_system::process_policies::PanicFaultPolicy {};

/// Dummy buffer that causes the linker to reserve enough space for the stack.
#[no_mangle]
#[link_section = ".stack_buffer"]
pub static mut STACK_MEMORY: [u8; 0x2000] = [0; 0x2000];

type TemperatureSTMSensor = components::temperature_stm::TemperatureSTMComponentType<
    capsules_core::virtualizers::virtual_adc::AdcDevice<'static, stm32f412rg::adc::Adc<'static>>,
>;
type TemperatureDriver = components::temperature::TemperatureComponentType<TemperatureSTMSensor>;
type RngDriver = components::rng::RngComponentType<stm32f412rg::trng::Trng<'static>>;

/// A structure representing this platform that holds references to all
/// capsules for this platform.
struct STM32F412GDiscovery {
    console: &'static capsules_core::console::Console<'static>,
    ipc: kernel::ipc::IPC<{ NUM_PROCS as u8 }>,
    alarm: &'static capsules_core::alarm::AlarmDriver<
        'static,
        VirtualMuxAlarm<'static, stm32f412rg::tim2::Tim2<'static>>,
    >,
    leds: &'static capsules_core::led::LedDriver<
        'static,
        LedLow<'static, stm32f412rg::gpio::Pin<'static>>,
        2,
    >,

    temperature: &'static TemperatureDriver,
    rng: &'static RngDriver,
    i2c: &'static capsules_core::i2c_master::I2CMasterDriver<
        'static,
        stm32f412rg::i2c::I2C<'static>,
    >,
    can: &'static capsules_extra::can::CanCapsule<'static, stm32f412rg::can::Can<'static>>,
    scheduler: &'static RoundRobinSched<'static>,
    systick: cortexm4::systick::SysTick,
}

/// Mapping of integer syscalls to objects that implement syscalls.
impl SyscallDriverLookup for STM32F412GDiscovery {
    fn with_driver<F, R>(&self, driver_num: usize, f: F) -> R
    where
        F: FnOnce(Option<&dyn kernel::syscall::SyscallDriver>) -> R,
    {
        match driver_num {
            capsules_core::console::DRIVER_NUM => f(Some(self.console)),
            capsules_core::alarm::DRIVER_NUM => f(Some(self.alarm)),
            kernel::ipc::DRIVER_NUM => f(Some(&self.ipc)),
            capsules_extra::can::DRIVER_NUM => f(Some(self.can)),
            capsules_core::i2c_master::DRIVER_NUM => f(Some(self.i2c)),
            capsules_core::led::DRIVER_NUM => f(Some(self.leds)),
            capsules_extra::temperature::DRIVER_NUM => f(Some(self.temperature)),
            capsules_core::rng::DRIVER_NUM => f(Some(self.rng)),
            _ => f(None),
        }
    }
}

impl
    KernelResources<
        stm32f412rg::chip::Stm32f4xx<
            'static,
            stm32f412rg::interrupt_service::Stm32f412rgDefaultPeripherals<'static>,
        >,
    > for STM32F412GDiscovery
{
    type SyscallDriverLookup = Self;
    type SyscallFilter = ();
    type ProcessFault = ();
    type Scheduler = RoundRobinSched<'static>;
    type SchedulerTimer = cortexm4::systick::SysTick;
    type WatchDog = ();
    type ContextSwitchCallback = ();

    fn syscall_driver_lookup(&self) -> &Self::SyscallDriverLookup {
        self
    }
    fn syscall_filter(&self) -> &Self::SyscallFilter {
        &()
    }
    fn process_fault(&self) -> &Self::ProcessFault {
        &()
    }
    fn scheduler(&self) -> &Self::Scheduler {
        self.scheduler
    }
    fn scheduler_timer(&self) -> &Self::SchedulerTimer {
        &self.systick
    }
    fn watchdog(&self) -> &Self::WatchDog {
        &()
    }
    fn context_switch_callback(&self) -> &Self::ContextSwitchCallback {
        &()
    }
}

/// Helper function called during bring-up that configures DMA.
unsafe fn setup_dma(
    dma: &stm32f412rg::dma::Dma2,
    dma_streams: &'static [stm32f412rg::dma::Stream<stm32f412rg::dma::Dma2>; 8],
    usart1: &'static stm32f412rg::usart::Usart<stm32f412rg::dma::Dma2>,
) {
    use stm32f412rg::dma::Dma2Peripheral;
    use stm32f412rg::usart;

    dma.enable_clock();

    let usart1_tx_stream = &dma_streams[Dma2Peripheral::USART1_TX.get_stream_idx()];
    let usart1_rx_stream = &dma_streams[Dma2Peripheral::USART1_RX.get_stream_idx()];

    usart1.set_dma(
        usart::TxDMA(usart1_tx_stream),
        usart::RxDMA(usart1_rx_stream),
    );

    usart1_tx_stream.set_client(usart1);
    usart1_rx_stream.set_client(usart1);

    usart1_tx_stream.setup(Dma2Peripheral::USART1_TX);
    usart1_rx_stream.setup(Dma2Peripheral::USART1_RX);

    cortexm4::nvic::Nvic::new(Dma2Peripheral::USART1_TX.get_stream_irqn()).enable();
    cortexm4::nvic::Nvic::new(Dma2Peripheral::USART1_RX.get_stream_irqn()).enable();
}

/// Helper function called during bring-up that configures multiplexed I/O.
unsafe fn set_pin_primary_functions(
    syscfg: &stm32f412rg::syscfg::Syscfg,
    i2c1: &stm32f412rg::i2c::I2C,
    gpio_ports: &'static stm32f412rg::gpio::GpioPorts<'static>,
    peripheral_clock_frequency: usize,
) {
    use kernel::hil::gpio::Configure;
    use stm32f412rg::gpio::{AlternateFunction, Mode, PinId, PortId};

    syscfg.enable_clock();

    // Enable clocks for GPIO Ports
    // Disable some of them if you don't need some of the GPIOs
    gpio_ports.get_port_from_port_id(PortId::A).enable_clock();
    gpio_ports.get_port_from_port_id(PortId::B).enable_clock();
    gpio_ports.get_port_from_port_id(PortId::C).enable_clock();
    gpio_ports.get_port_from_port_id(PortId::D).enable_clock();
    gpio_ports.get_port_from_port_id(PortId::E).enable_clock();
    gpio_ports.get_port_from_port_id(PortId::F).enable_clock();
    gpio_ports.get_port_from_port_id(PortId::G).enable_clock();
    gpio_ports.get_port_from_port_id(PortId::H).enable_clock();

    // User LED1 is connected to PC10. Configure PC10 as `debug_gpio!(0, ...)`
    // Configure kernel debug gpios as early as possible
    kernel::debug::assign_gpios(
        Some(
            gpio_ports
                .get_pin(PinId::PC10)
                .expect("Can't request user led1"),
        ),
        Some(
            gpio_ports
                .get_pin(PinId::PB05)
                .expect("Can't request user led2"),
        ),
        None,
    );

    //pa2 and pa3 (USART2) is connected to ST-LINK virtual COM port
    gpio_ports.get_pin(PinId::PA09).map(|pin| {
        pin.set_mode(Mode::AlternateFunctionMode);
        // AF7 is USART2_TX
        pin.set_alternate_function(AlternateFunction::AF7);
    });
    gpio_ports.get_pin(PinId::PA10).map(|pin| {
        pin.set_mode(Mode::AlternateFunctionMode);
        // AF7 is USART2_RX
        pin.set_alternate_function(AlternateFunction::AF7);
    });

    // I2C1 has the TouchPanel connected
    gpio_ports.get_pin(PinId::PB06).map(|pin| {
        // pin.make_output();
        pin.set_mode_output_opendrain();
        pin.set_mode(Mode::AlternateFunctionMode);
        pin.set_floating_state(kernel::hil::gpio::FloatingState::PullNone);
        // AF4 is I2C
        pin.set_alternate_function(AlternateFunction::AF4);
    });
    gpio_ports.get_pin(PinId::PB07).map(|pin| {
        // pin.make_output();
        pin.set_mode_output_opendrain();
        pin.set_floating_state(kernel::hil::gpio::FloatingState::PullNone);
        pin.set_mode(Mode::AlternateFunctionMode);
        // AF4 is I2C
        pin.set_alternate_function(AlternateFunction::AF4);
    });

    i2c1.enable_clock();
    i2c1.set_speed(
        stm32f412rg::i2c::I2CSpeed::Speed100k,
        peripheral_clock_frequency,
    );
}

/// Helper function for miscellaneous peripheral functions
unsafe fn setup_peripherals(
    tim2: &stm32f412rg::tim2::Tim2,
    fsmc: &stm32f412rg::fsmc::Fsmc,
    trng: &stm32f412rg::trng::Trng,
    i2c: &stm32f412rg::i2c::I2C,
) {
    // USART2 IRQn is 38
    cortexm4::nvic::Nvic::new(stm32f412rg::nvic::USART1).enable();

    // TIM2 IRQn is 28
    tim2.enable_clock();
    tim2.start();
    cortexm4::nvic::Nvic::new(stm32f412rg::nvic::TIM2).enable();

    i2c.enable_clock();
    cortexm4::nvic::Nvic::new(stm32f412rg::nvic::I2C1_EV).enable();
    cortexm4::nvic::Nvic::new(stm32f412rg::nvic::I2C1_ER).enable();

    // FSMC
    fsmc.enable();

    // RNG
    trng.enable_clock();
}

/// Main function.
///
/// This is in a separate, inline(never) function so that its stack frame is
/// removed when this function returns. Otherwise, the stack space used for
/// these static_inits is wasted.
#[inline(never)]
unsafe fn start() -> (
    &'static kernel::Kernel,
    STM32F412GDiscovery,
    &'static stm32f412rg::chip::Stm32f4xx<'static, Stm32f412rgDefaultPeripherals<'static>>,
) {
    stm32f412rg::init();

    let rcc = static_init!(stm32f412rg::rcc::Rcc, stm32f412rg::rcc::Rcc::new());
    let clocks = static_init!(
        stm32f412rg::clocks::Clocks<Stm32f412Specs>,
        stm32f412rg::clocks::Clocks::new(rcc)
    );

    let syscfg = static_init!(
        stm32f412rg::syscfg::Syscfg,
        stm32f412rg::syscfg::Syscfg::new(clocks)
    );

    let exti = static_init!(
        stm32f412rg::exti::Exti,
        stm32f412rg::exti::Exti::new(syscfg)
    );

    let dma1 = static_init!(stm32f412rg::dma::Dma1, stm32f412rg::dma::Dma1::new(clocks));
    let dma2 = static_init!(stm32f412rg::dma::Dma2, stm32f412rg::dma::Dma2::new(clocks));

    let peripherals = static_init!(
        Stm32f412rgDefaultPeripherals,
        Stm32f412rgDefaultPeripherals::new(clocks, exti, dma1, dma2)
    );
    peripherals.init();

    let _ = clocks.set_ahb_prescaler(stm32f412rg::rcc::AHBPrescaler::DivideBy1);
    let _ = clocks.set_apb1_prescaler(stm32f412rg::rcc::APBPrescaler::DivideBy4);
    let _ = clocks.set_apb2_prescaler(stm32f412rg::rcc::APBPrescaler::DivideBy2);
    let _ = clocks.set_pll_frequency_mhz(PllSource::HSI, 100);
    let _ = clocks.pll.enable();
    let _ = clocks.set_sys_clock_source(stm32f412rg::rcc::SysClockSource::PLL);
    let apb1_frequence_mhz = clocks.get_apb1_frequency_mhz();

    let base_peripherals = &peripherals.stm32f4;
    setup_peripherals(
        &base_peripherals.tim2,
        &base_peripherals.fsmc,
        &peripherals.trng,
        &base_peripherals.i2c1,
    );

    set_pin_primary_functions(
        syscfg,
        &base_peripherals.i2c1,
        &base_peripherals.gpio_ports,
        apb1_frequence_mhz,
    );

    setup_dma(
        dma2,
        &base_peripherals.dma2_streams,
        &base_peripherals.usart1,
    );

    let board_kernel = static_init!(kernel::Kernel, kernel::Kernel::new(&*addr_of!(PROCESSES)));

    let chip = static_init!(
        stm32f412rg::chip::Stm32f4xx<Stm32f412rgDefaultPeripherals>,
        stm32f412rg::chip::Stm32f4xx::new(peripherals)
    );
    CHIP = Some(chip);

    // USER LEDs
    let leds = components::led::LedsComponent::new().finalize(components::led_component_static!(
        LedLow<'static, stm32f412rg::gpio::Pin>,
        // User Led1
        LedLow::new(
            base_peripherals
                .gpio_ports
                .get_pin(stm32f412rg::gpio::PinId::PC10)
                .unwrap()
        ),
        // User Led2
        LedLow::new(
            base_peripherals
                .gpio_ports
                .get_pin(stm32f412rg::gpio::PinId::PB05)
                .unwrap()
        ),
    ));

    // UART
    // Create a shared UART channel for kernel debug.
    base_peripherals.usart1.enable_clock();
    let uart_mux = components::console::UartMuxComponent::new(&base_peripherals.usart1, 115200)
        .finalize(components::uart_mux_component_static!());

    (*addr_of_mut!(io::WRITER)).set_initialized();

    // Create capabilities that the board needs to call certain protected kernel
    // functions.
    let memory_allocation_capability = create_capability!(capabilities::MemoryAllocationCapability);
    let process_management_capability =
        create_capability!(capabilities::ProcessManagementCapability);

    // Setup the console.
    let console = components::console::ConsoleComponent::new(
        board_kernel,
        capsules_core::console::DRIVER_NUM,
        uart_mux,
    )
    .finalize(components::console_component_static!());
    // Create the debugger object that handles calls to `debug!()`.
    components::debug_writer::DebugWriterComponent::new(uart_mux)
        .finalize(components::debug_writer_component_static!());

    // LEDs

    // ALARM

    let tim2 = &base_peripherals.tim2;
    let mux_alarm = components::alarm::AlarmMuxComponent::new(tim2).finalize(
        components::alarm_mux_component_static!(stm32f412rg::tim2::Tim2),
    );

    let alarm = components::alarm::AlarmDriverComponent::new(
        board_kernel,
        capsules_core::alarm::DRIVER_NUM,
        mux_alarm,
    )
    .finalize(components::alarm_component_static!(stm32f412rg::tim2::Tim2));

    // RNG
    let rng = RngComponent::new(
        board_kernel,
        capsules_core::rng::DRIVER_NUM,
        &peripherals.trng,
    )
    .finalize(components::rng_component_static!(stm32f412rg::trng::Trng));

    // I2C Master
    // initialize capabilities
    let memory_allocation_cap = create_capability!(capabilities::MemoryAllocationCapability);
    // Init the I2C device attached via Qwiic
    let i2c_master_buffer: &mut [u8; 64] = static_init!(
        [u8; capsules_core::i2c_master::BUFFER_LENGTH],
        [0; capsules_core::i2c_master::BUFFER_LENGTH]
    );
    let i2c = static_init!(
        capsules_core::i2c_master::I2CMasterDriver<'static, stm32f412rg::i2c::I2C<'static>>,
        capsules_core::i2c_master::I2CMasterDriver::new(
            &base_peripherals.i2c1,
            i2c_master_buffer,
            board_kernel.create_grant(
                capsules_core::i2c_master::DRIVER_NUM,
                &memory_allocation_cap
            )
        )
    );

    base_peripherals.i2c1.set_master_client(i2c);
    base_peripherals.i2c1.enable();

    // CAN
    let can = components::can::CanComponent::new(
        board_kernel,
        capsules_extra::can::DRIVER_NUM,
        &peripherals.can1,
    )
    .finalize(components::can_component_static!(
        stm32f412rg::can::Can<'static>
    ));

    // ADC
    let adc_mux = components::adc::AdcMuxComponent::new(&base_peripherals.adc1)
        .finalize(components::adc_mux_component_static!(stm32f412rg::adc::Adc));

    let temp_sensor = components::temperature_stm::TemperatureSTMComponent::new(
        adc_mux,
        stm32f412rg::adc::Channel::Channel18,
        2.5,
        0.76,
    )
    .finalize(components::temperature_stm_adc_component_static!(
        stm32f412rg::adc::Adc
    ));

    let temp = components::temperature::TemperatureComponent::new(
        board_kernel,
        capsules_extra::temperature::DRIVER_NUM,
        temp_sensor,
    )
    .finalize(components::temperature_component_static!(
        TemperatureSTMSensor
    ));

    let process_printer = components::process_printer::ProcessPrinterTextComponent::new()
        .finalize(components::process_printer_text_component_static!());
    PROCESS_PRINTER = Some(process_printer);

    // PROCESS CONSOLE
    let process_console = components::process_console::ProcessConsoleComponent::new(
        board_kernel,
        uart_mux,
        mux_alarm,
        process_printer,
        Some(cortexm4::support::reset),
    )
    .finalize(components::process_console_component_static!(
        stm32f412rg::tim2::Tim2
    ));
    let _ = process_console.start();

    let scheduler = components::sched::round_robin::RoundRobinComponent::new(&*addr_of!(PROCESSES))
        .finalize(components::round_robin_component_static!(NUM_PROCS));

    let stm32f412g = STM32F412GDiscovery {
        console,
        ipc: kernel::ipc::IPC::new(
            board_kernel,
            kernel::ipc::DRIVER_NUM,
            &memory_allocation_capability,
        ),
        alarm,
        leds,
        temperature: temp,
        rng,
        can,
        i2c,
        scheduler,
        systick: cortexm4::systick::SysTick::new_with_calibration(
            (HSI_FREQUENCY_MHZ * 1_000_000) as u32,
        ),
    };
    debug!("\n\rSystem Clock Infor: ");
    debug!(" => APB  : {}Mhz", clocks.get_ahb_frequency_mhz());
    debug!(" => APB1 : {}Mhz", clocks.get_apb1_frequency_mhz());
    debug!(" => APB2 : {}Mhz", clocks.get_apb2_frequency_mhz());
    debug!(" => SYS  : {}Mhz", clocks.get_sys_clock_frequency_mhz());

    debug!("Initialization complete. Entering main loop");

    extern "C" {
        /// Beginning of the ROM region containing app images.
        ///
        /// This symbol is defined in the linker script.
        static _sapps: u8;

        /// End of the ROM region containing app images.
        ///
        /// This symbol is defined in the linker script.
        static _eapps: u8;

        /// Beginning of the RAM region for app memory.
        ///
        /// This symbol is defined in the linker script.
        static mut _sappmem: u8;

        /// End of the RAM region for app memory.
        ///
        /// This symbol is defined in the linker script.
        static _eappmem: u8;
    }

    kernel::process::load_processes(
        board_kernel,
        chip,
        core::slice::from_raw_parts(
            core::ptr::addr_of!(_sapps),
            core::ptr::addr_of!(_eapps) as usize - core::ptr::addr_of!(_sapps) as usize,
        ),
        core::slice::from_raw_parts_mut(
            core::ptr::addr_of_mut!(_sappmem),
            core::ptr::addr_of!(_eappmem) as usize - core::ptr::addr_of!(_sappmem) as usize,
        ),
        &mut *addr_of_mut!(PROCESSES),
        &FAULT_RESPONSE,
        &process_management_capability,
    )
    .unwrap_or_else(|err| {
        debug!("Error loading processes!");
        debug!("{:?}", err);
    });

    //Uncomment to run multi alarm test
    /*components::test::multi_alarm_test::MultiAlarmTestComponent::new(mux_alarm)
    .finalize(components::multi_alarm_test_component_buf!(stm32f412rg::tim2::Tim2))
    .run();*/

    (board_kernel, stm32f412g, chip)
}

/// Main function called after RAM initialized.
#[no_mangle]
pub unsafe fn main() {
    let main_loop_capability = create_capability!(capabilities::MainLoopCapability);

    let (board_kernel, platform, chip) = start();
    board_kernel.kernel_loop(&platform, chip, Some(&platform.ipc), &main_loop_capability);
}
