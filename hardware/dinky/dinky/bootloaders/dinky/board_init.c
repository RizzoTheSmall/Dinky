/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.
  Copyright (c) 2015 Atmel Corporation/Thibaut VIARD.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <sam.h>
#include <stdbool.h>
#include "board_definitions.h"

#define SYSCTRL_FUSES_OSC32K_CAL_ADDR   (NVMCTRL_OTP4 + 4)
#define SYSCTRL_FUSES_OSC32K_CAL_Pos   6
#define   SYSCTRL_FUSES_OSC32K_ADDR   SYSCTRL_FUSES_OSC32K_CAL_ADDR
#define   SYSCTRL_FUSES_OSC32K_Pos   SYSCTRL_FUSES_OSC32K_CAL_Pos
#define   SYSCTRL_FUSES_OSC32K_Msk   (0x7Fu << SYSCTRL_FUSES_OSC32K_Pos)

volatile bool g_interrupt_enabled = true;

static void gclk_sync(void) {
    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY)
        ;
}

static void dfll_sync(void) {
    while ((SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY) == 0)
        ;
}


void board_init(void) {


  /* Set 1 Flash Wait State for 48MHz, cf tables 20.9 and 35.27 in SAMD21 Datasheet */
  NVMCTRL->CTRLB.bit.RWS = 1;//NVMCTRL_CTRLB_RWS_HALF_Val;

  /* Turn on the digital interface clock */
  PM->APBAMASK.reg |= PM_APBAMASK_GCLK;




  /* ----------------------------------------------------------------------------------------------
   * 1) Enable OSC32K clock (Internal on-board 32.768Hz oscillator)
   */
  SYSCTRL->OSC32K.bit.CALIB =
			((*(uint32_t *)SYSCTRL_FUSES_OSC32K_ADDR >> 
			SYSCTRL_FUSES_OSC32K_Pos) & 0x7Ful);

  SYSCTRL->OSC32K.reg = SYSCTRL_OSC32K_STARTUP( 0x6u ) | /* cf table 15.10 of product datasheet in chapter 15.8.6 */
                         SYSCTRL_OSC32K_EN32K;
  SYSCTRL->OSC32K.bit.CALIB =
			((*(uint32_t *)SYSCTRL_FUSES_OSC32K_ADDR >> 
			SYSCTRL_FUSES_OSC32K_Pos) & 0x7Ful);
                       
  SYSCTRL->OSC32K.bit.ENABLE = 1; /* separate call, as described in chapter 15.6.3 */

  while (  (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_OSC32KRDY) == 0 )
  {
    /* Wait for oscillator stabilization */
  }
  
  /* Software reset the module to ensure it is re-initialized correctly */
  /* Note: Due to synchronization, there is a delay from writing CTRL.SWRST until the reset is complete.
   * CTRL.SWRST and STATUS.SYNCBUSY will both be cleared when the reset is complete, as described in chapter 13.8.1
   */
  GCLK->CTRL.reg = GCLK_CTRL_SWRST;

  while ( (GCLK->CTRL.reg & GCLK_CTRL_SWRST) && (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY) )
  {
    /* Wait for reset to complete */
  }



  NVMCTRL->CTRLB.bit.RWS = 1;

  /* Configure OSC8M as source for GCLK_GEN 2 */
  GCLK->GENDIV.reg = GCLK_GENDIV_ID(2);  // Read GENERATOR_ID - GCLK_GEN_2
  gclk_sync();

  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(2) | GCLK_GENCTRL_SRC_OSC8M_Val | GCLK_GENCTRL_GENEN;
  gclk_sync();

  // Turn on DFLL with USB correction and sync to internal 8 mhz oscillator
  SYSCTRL->DFLLCTRL.reg = SYSCTRL_DFLLCTRL_ENABLE;
  dfll_sync();

  SYSCTRL_DFLLVAL_Type dfllval_conf = {0};
  uint32_t coarse =( *((uint32_t *)(NVMCTRL_OTP4)
           + (NVM_SW_CALIB_DFLL48M_COARSE_VAL / 32))
         >> (NVM_SW_CALIB_DFLL48M_COARSE_VAL % 32))
    & ((1 << 6) - 1);
  if (coarse == 0x3f) {
    coarse = 0x1f;
  }
  dfllval_conf.bit.COARSE  = coarse;
  // TODO(tannewt): Load this from a well known flash location so that it can be
  // calibrated during testing.
  dfllval_conf.bit.FINE    = 0x1ff;

  SYSCTRL->DFLLMUL.reg = SYSCTRL_DFLLMUL_CSTEP( 0x1f / 4 ) | // Coarse step is 31, half of the max value
                         SYSCTRL_DFLLMUL_FSTEP( 10 ) |
                         48000;
  SYSCTRL->DFLLVAL.reg = dfllval_conf.reg;
  SYSCTRL->DFLLCTRL.reg = 0;
  dfll_sync();
  SYSCTRL->DFLLCTRL.reg = SYSCTRL_DFLLCTRL_MODE |
                          SYSCTRL_DFLLCTRL_CCDIS |
                          SYSCTRL_DFLLCTRL_USBCRM | /* USB correction */
                          SYSCTRL_DFLLCTRL_BPLCKC;
  dfll_sync();
  SYSCTRL->DFLLCTRL.reg |= SYSCTRL_DFLLCTRL_ENABLE ;
  dfll_sync();

  GCLK_CLKCTRL_Type clkctrl={0};
  uint16_t temp;
  GCLK->CLKCTRL.bit.ID = 2; // GCLK_ID - DFLL48M Reference
  temp = GCLK->CLKCTRL.reg;
  clkctrl.bit.CLKEN = 1;
  clkctrl.bit.WRTLOCK = 0;
  clkctrl.bit.GEN = GCLK_CLKCTRL_GEN_GCLK0_Val;
  GCLK->CLKCTRL.reg = (clkctrl.reg | temp);

    // Configure DFLL48M as source for GCLK_GEN 0
    GCLK->GENDIV.reg = GCLK_GENDIV_ID(0);
    gclk_sync();

    // Add GCLK_GENCTRL_OE below to output GCLK0 on the SWCLK pin.
    GCLK->GENCTRL.reg =
        GCLK_GENCTRL_ID(0) | GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN;
    gclk_sync();

    SysTick_Config(1000);

    // Uncomment these two lines to output GCLK0 on the SWCLK pin.
    // PORT->Group[0].PINCFG[30].bit.PMUXEN = 1;
    // Set the port mux mask for odd processor pin numbers, PA30 = 30 is even number, PMUXE = PMUX Even
    // PORT->Group[0].PMUX[30 / 2].reg |= PORT_PMUX_PMUXE_H;
}
