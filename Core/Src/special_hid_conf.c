/*
 * special_hid_conf.c
 *
 *  Created on: Jul 21, 2025
 *      Author: zuidec
 */

#include "special_hid_conf.h"
uint8_t ENABLE_USBD_HID_CUSTOM2 = 0;

uint8_t is_enabled_hid2(void)	{
	return ENABLE_USBD_HID_CUSTOM2;
}
