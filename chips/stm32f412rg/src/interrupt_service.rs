// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2022.

use crate::{can_registers, chip_specs::Stm32f412Specs};
use stm32f4xx::chip::Stm32f4xxDefaultPeripherals;

use crate::{stm32f412rg_nvic, trng_registers};

pub struct Stm32f412rgDefaultPeripherals<'a> {
    pub stm32f4: Stm32f4xxDefaultPeripherals<'a, Stm32f412Specs>,
    // Once implemented, place Stm32f412g specific peripherals here
    pub trng: stm32f4xx::trng::Trng<'a>,
    pub can1: stm32f4xx::can::Can<'a>,
}

impl<'a> Stm32f412rgDefaultPeripherals<'a> {
    pub unsafe fn new(
        clocks: &'a crate::clocks::Clocks<'a, Stm32f412Specs>,
        exti: &'a crate::exti::Exti<'a>,
        dma1: &'a crate::dma::Dma1<'a>,
        dma2: &'a crate::dma::Dma2<'a>,
    ) -> Self {
        Self {
            stm32f4: Stm32f4xxDefaultPeripherals::new(clocks, exti, dma1, dma2),
            trng: stm32f4xx::trng::Trng::new(trng_registers::RNG_BASE, clocks),
            can1: stm32f4xx::can::Can::new(clocks, can_registers::CAN1_BASE),
        }
    }
    // Necessary for setting up circular dependencies & registering deferred calls
    pub fn init(&'static self) {
        self.stm32f4.setup_circular_deps();
    }
}
impl kernel::platform::chip::InterruptService for Stm32f412rgDefaultPeripherals<'_> {
    unsafe fn service_interrupt(&self, interrupt: u32) -> bool {
        match interrupt {
            // put Stm32f412g specific interrupts here
            stm32f412rg_nvic::RNG => {
                self.trng.handle_interrupt();
                true
            }
            _ => self.stm32f4.service_interrupt(interrupt),
        }
    }
}
