// SPDX-License-Identifier: GPL-2.0+
/*
 * PinePhone keyboard case: an MCU at 0x15 on the pogo-pin I2C bus that reports its key matrix.
 *
 * Ported from the Linux driver by Samuel Holland; polled here, since U-Boot has no interrupts.
 */

#include <dm.h>
#include <i2c.h>
#include <input.h>
#include <keyboard.h>
#include <log.h>
#include <stdio_dev.h>
#include <time.h>
#include <u-boot/crc.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <power/regulator.h>

#define PPKB_DEVICE_ID_HI		0x00
#define PPKB_DEVICE_ID_LO		0x01
#define PPKB_MATRIX_SIZE		0x06
#define PPKB_SCAN_CRC			0x07
#define PPKB_SYS_CONFIG			0x20
#define PPKB_SYS_CONFIG_DISABLE_SCAN	BIT(0)

#define PPKB_ROWS			6
#define PPKB_COLS			12

/* The CRC byte comes first, then one byte of row bits per column. */
#define PPKB_BUF_LEN			(1 + PPKB_COLS)

#define PPKB_POLL_MS			20

/* The second six rows are the Fn layer. */
static const u16 ppkb_keymap[2 * PPKB_ROWS][PPKB_COLS] = {
	{ KEY_ESC, KEY_1, KEY_2, KEY_3, KEY_4, KEY_5, KEY_6, KEY_7, KEY_8, KEY_9, KEY_0, KEY_BACKSPACE },
	{ KEY_TAB, KEY_Q, KEY_W, KEY_E, KEY_R, KEY_T, KEY_Y, KEY_U, KEY_I, KEY_O, KEY_P, KEY_ENTER },
	{ KEY_LEFTMETA, KEY_A, KEY_S, KEY_D, KEY_F, KEY_G, KEY_H, KEY_J, KEY_K, KEY_L, KEY_SEMICOLON },
	{ KEY_LEFTSHIFT, KEY_Z, KEY_X, KEY_C, KEY_V, KEY_B, KEY_N, KEY_M, KEY_COMMA, KEY_DOT, KEY_SLASH },
	{ [1] = KEY_LEFTCTRL, [4] = KEY_SPACE, [6] = KEY_APOSTROPHE, [8] = KEY_RIGHTBRACE, [9] = KEY_LEFTBRACE },
	{ [2] = KEY_FN, [3] = KEY_LEFTALT, [5] = KEY_RIGHTALT },

	{ KEY_ESC, KEY_F1, KEY_F2, KEY_F3, KEY_F4, KEY_F5, KEY_F6, KEY_F7, KEY_F8, KEY_F9, KEY_F10, KEY_DELETE },
	{ [10] = KEY_PAGEUP },
	{ [0] = KEY_SYSRQ, [9] = KEY_PAGEDOWN, [10] = KEY_INSERT },
	{ [0] = KEY_LEFTSHIFT, [8] = KEY_HOME, [9] = KEY_UP, [10] = KEY_END },
	{ [1] = KEY_LEFTCTRL, [6] = KEY_LEFT, [8] = KEY_RIGHT, [9] = KEY_DOWN },
	{ [3] = KEY_LEFTALT, [5] = KEY_RIGHTALT },
};

struct ppkb_priv {
	u8 cols[PPKB_COLS];
	u8 fn_state[PPKB_COLS];
	bool fn_pressed;
	ulong next_poll;
};

/* Reads the matrix and reports every key whose state changed since the last read. */
static int ppkb_read_keys(struct input_config *input)
{
	struct udevice *dev = input->dev;
	struct ppkb_priv *priv = dev_get_priv(dev);
	u8 buf[PPKB_BUF_LEN];
	int col, row, changes = 0;

	if (get_timer(priv->next_poll) < PPKB_POLL_MS)
		return 0;
	priv->next_poll = get_timer(0);

	if (dm_i2c_read(dev, PPKB_SCAN_CRC, buf, sizeof(buf)))
		return 0;
	if (crc8(0xff, &buf[1], PPKB_COLS) != buf[0])
		return 0;

	for (col = 0; col < PPKB_COLS; col++) {
		u8 changed = priv->cols[col] ^ buf[1 + col];

		for (row = 0; row < PPKB_ROWS; row++) {
			u8 mask = BIT(row);
			bool pressed = buf[1 + col] & mask;
			bool fn;
			u16 code;

			if (!(changed & mask))
				continue;

			/* A release takes the layer its press was in, so Fn let go first cannot strand a key. */
			fn = pressed ? priv->fn_pressed : priv->fn_state[col] & mask;
			if (fn)
				priv->fn_state[col] ^= mask;

			code = ppkb_keymap[fn ? PPKB_ROWS + row : row][col];
			if (code == KEY_FN)
				priv->fn_pressed = pressed;
			else if (code)
				input_add_keycode(input, code, !pressed);
			changes++;
		}
		priv->cols[col] = buf[1 + col];
	}

	return changes > 0;
}

/* Sets or clears the MCU's scan-disable bit, which Linux leaves set whenever nothing has the keyboard open. */
static int ppkb_set_scan(struct udevice *dev, bool enable)
{
	int val = dm_i2c_reg_read(dev, PPKB_SYS_CONFIG);

	if (val < 0)
		return val;
	if (enable)
		val &= ~PPKB_SYS_CONFIG_DISABLE_SCAN;
	else
		val |= PPKB_SYS_CONFIG_DISABLE_SCAN;

	return dm_i2c_reg_write(dev, PPKB_SYS_CONFIG, val);
}

/* Powers the case, checks it is there, and registers it as a console input only if it answered. */
static int ppkb_probe(struct udevice *dev)
{
	struct keyboard_priv *uc_priv = dev_get_uclass_priv(dev);
	struct stdio_dev *sdev = &uc_priv->sdev;
	struct input_config *input = &uc_priv->input;
	struct udevice *vbat;
	u8 info[PPKB_MATRIX_SIZE + 1];
	int ret, tries;

	if (!device_get_supply_regulator(dev, "vbat-supply", &vbat))
		regulator_set_enable_if_allowed(vbat, true);

	for (tries = 0; tries < 10; tries++) {
		ret = dm_i2c_read(dev, PPKB_DEVICE_ID_HI, info, sizeof(info));
		if (!ret)
			break;
		mdelay(10);
	}

	/* A phone out of its case is the normal case, so absence registers nothing rather than failing the probe and printing on the panel. */
	if (ret || info[PPKB_DEVICE_ID_HI] != 'K' || info[PPKB_DEVICE_ID_LO] != 'B' ||
	    info[PPKB_MATRIX_SIZE] != (PPKB_COLS << 4 | PPKB_ROWS)) {
		log_debug("no keyboard case (ret=%d)\n", ret);
		return 0;
	}

	ret = ppkb_set_scan(dev, true);
	if (ret)
		return ret;

	input_add_tables(input, false);
	input->dev = dev;
	input->read_keys = ppkb_read_keys;
	strcpy(sdev->name, "pinephone-kbd");

	return input_stdio_register(sdev);
}

static const struct udevice_id ppkb_ids[] = {
	{ .compatible = "pine64,pinephone-keyboard" },
	{ }
};

U_BOOT_DRIVER(pinephone_kbd) = {
	.name		= "pinephone_kbd",
	.id		= UCLASS_KEYBOARD,
	.of_match	= ppkb_ids,
	.probe		= ppkb_probe,
	.priv_auto	= sizeof(struct ppkb_priv),
};
