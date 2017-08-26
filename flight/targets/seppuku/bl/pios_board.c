/**
 ******************************************************************************
 * @addtogroup Bootloader Bootloaders
 * @{
 * @addtogroup Seppuku DTFAIR Seppuku
 * @{
 *
 * @file       seppuku/bl/pios_board.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2012-2013
 * @author     dRonin, http://dRonin.org/, Copyright (C) 2016
 * @brief      Board specific initialization for the bootloader
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>
 *
 * Additional note on redistribution: The copyright and license notices above
 * must be maintained in each individual source file that is a derivative work
 * of this source file; otherwise redistribution is prohibited.
 */

/* Pull in the board-specific static HW definitions.
 * Including .c files is a bit ugly but this allows all of
 * the HW definitions to be const and static to limit their
 * scope.
 *
 * NOTE: THIS IS THE ONLY PLACE THAT SHOULD EVER INCLUDE THIS FILE
 */
#include "board_hw_defs.c"

#include <pios_board_info.h>
#include <pios.h>

#ifdef PIOS_INCLUDE_WS2811
#include <pios_ws2811.h>
#endif

uintptr_t pios_com_telem_usb_id;

void PIOS_Board_Init() {
	/* Absolute first thing-- drive the video mask pin low */
	/*
	GPIO_Init(video_mask_pin.gpio, (GPIO_InitTypeDef *)&video_mask_pin.init);
	GPIO_WriteBit(video_mask_pin.gpio,
			video_mask_pin.init.GPIO_Pin, Bit_RESET);*/

	/* Delay system */
	PIOS_DELAY_Init();

	const struct pios_board_info * bdinfo = &pios_board_info_blob;

#if defined(PIOS_INCLUDE_ANNUNC)
	const struct pios_annunc_cfg * led_cfg = PIOS_BOARD_HW_DEFS_GetLedCfg(bdinfo->board_rev);
	PIOS_Assert(led_cfg);
	PIOS_ANNUNC_Init(led_cfg);
#endif	/* PIOS_INCLUDE_ANNUNC */

#ifdef PIOS_INCLUDE_WS2811
	PIOS_WS2811_init(&pios_ws2811, &pios_ws2811_cfg, 1);
	PIOS_WS2811_trigger_update(pios_ws2811);
#endif

	if ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS ) != RCC_CFGR_SWS_PLL) {
		/* Oh no.  We're not clocked from PLL, that is DEFINITELY
		 * not what we expect. */

		while (true) {
#ifdef PIOS_INCLUDE_ANNUNC
#if defined(PIOS_LED_HEARTBEAT)
			PIOS_ANNUNC_Off(PIOS_LED_HEARTBEAT);
			for (int i=0; i < 500000; i++) {
				asm volatile ("nop"::);
			}
			PIOS_ANNUNC_On(PIOS_LED_HEARTBEAT);
			for (int i=0; i < 500000; i++) {
				asm volatile ("nop"::);
			}
#endif	/* PIOS_LED_HEARTBEAT */
#endif  /* PIOS_INCLUDE_ANNUNC */
		}
	}

#if defined(PIOS_INCLUDE_FLASH)
	/* Inititialize all flash drivers */
	PIOS_Flash_Internal_Init(&pios_internal_flash_id, &flash_internal_cfg);

	/* Register the partition table */
	PIOS_FLASH_register_partition_table(pios_flash_partition_table, NELEMENTS(pios_flash_partition_table));
#endif	/* PIOS_INCLUDE_FLASH */

#if defined(PIOS_INCLUDE_USB)
	/* Initialize board specific USB data */
	PIOS_USB_BOARD_DATA_Init();

	/* Activate the HID-only USB configuration */
	PIOS_USB_DESC_HID_ONLY_Init();

	uintptr_t pios_usb_id;
	PIOS_USB_Init(&pios_usb_id, PIOS_BOARD_HW_DEFS_GetUsbCfg(bdinfo->board_rev));

#if defined(PIOS_INCLUDE_USB_HID) && defined(PIOS_INCLUDE_COM_MSG)
	uintptr_t pios_usb_hid_id;
	if (PIOS_USB_HID_Init(&pios_usb_hid_id, &pios_usb_hid_cfg, pios_usb_id)) {
		PIOS_Assert(0);
	}
	if (PIOS_COM_MSG_Init(&pios_com_telem_usb_id, &pios_usb_hid_com_driver, pios_usb_hid_id)) {
		PIOS_Assert(0);
	}
#endif	/* PIOS_INCLUDE_USB_HID && PIOS_INCLUDE_COM_MSG */

	PIOS_USBHOOK_Activate();

#endif	/* PIOS_INCLUDE_USB */
}

/**
 * @}
 * @}
 */
