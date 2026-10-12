// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Florian Frankenberger
//
// Activity LED on GP19: solid while a disk is present, blinking on transfer.
#pragma once
#include <stdbool.h>

void led_init(void);
void led_media(bool present);
void led_activity(void);
void led_task(void);
