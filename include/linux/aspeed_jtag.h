/*
 * Copyright (c) 2017 Mellanox Technologies. All rights reserved.
 * Copyright (c) 2017 Oleksandr Shamray <oleksandrs@mellanox.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the names of the copyright holders nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * Alternatively, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef	__ASPEED_JTAG_H__
#define	__ASPEED_JTAG_H__

enum aspeed_jtag_xfer_mode {
	JTAG_XFER_HW_MODE = 0,
	JTAG_XFER_SW_MODE = 1,
};

enum aspeed_jtag_enstate {
	JTAG_STATE_IDLE = 0,
	JTAG_STATE_IRPAUSE = 1,
	JTAG_STATE_DRPAUSE = 2,
};

struct runtest_idle {
	enum aspeed_jtag_xfer_mode	mode;	/* 0 :HW mode,
						 * 1: SW mode
						 */
	unsigned char			reset;	/* Test Logic Reset */
	unsigned char			end;	/* o: idle,
						 * 1: ir pause,
						 * 2: drpause
						 */
	unsigned char			tck;	 /* keep tck */
};

struct sir_xfer {
	enum aspeed_jtag_xfer_mode	mode;	/* 0 :HW mode, 1: SW mode */
	unsigned short			length;	/* bits */
	unsigned int			tdi;
	unsigned int			tdo;
	enum aspeed_jtag_enstate endir;	/* 0: idle, 1:pause */
};

struct sdr_xfer {
	enum aspeed_jtag_xfer_mode	mode;	/* 0 :HW mode,
						 * 1: SW mode
						 */
	unsigned char			direct; /* 0 ; read ,
						 * 1 : write
						 */
	unsigned short			length;	/* bits */
	unsigned int			*tdio;
	enum aspeed_jtag_enstate enddr;	/* 0: idle, 1:pause */
};

#define JTAGIOC_BASE       'T'

#define AST_JTAG_IOCRUNTEST	_IOW(JTAGIOC_BASE, 0, struct runtest_idle)
#define AST_JTAG_IOCSIR		_IOWR(JTAGIOC_BASE, 1, struct sir_xfer)
#define AST_JTAG_IOCSDR		_IOWR(JTAGIOC_BASE, 2, struct sdr_xfer)
#define AST_JTAG_SIOCFREQ	_IOW(JTAGIOC_BASE, 3, unsigned int)
#define AST_JTAG_GIOCFREQ	_IOR(JTAGIOC_BASE, 4, unsigned int)

#endif /* __ASPEED_JTAG_H__ */
