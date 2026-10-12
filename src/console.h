// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Florian Frankenberger
//
// printf over USB CDC (there is no UART on this board).
#pragma once

void console_init(void);

// Run tud_task(), unless we are already inside it (USB callbacks).
void usb_task(void);
