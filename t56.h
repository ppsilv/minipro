/*
 * t56.h - Low level ops for T56 declarations and definitions
 *
 * This file is a part of Minipro.
 *
 * Minipro is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * Minipro is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __T56_H
#define __T56_H

#define T56_FIRMWARE_VERSION 0x147
#define T56_FIRMWARE_STRING  "01.1.71"

/* T56 low level functions. */
int t56_begin_transaction(minipro_handle_t *handle);
int t56_end_transaction(minipro_handle_t *handle);
int t56_get_chip_id(minipro_handle_t *handle, uint8_t *type,
			uint32_t *device_id);
int t56_get_ovc_status(minipro_handle_t *handle,
			minipro_status_t *status, uint8_t *ovc);

#endif
