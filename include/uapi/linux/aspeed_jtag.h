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

#ifndef	_UAPI_LINUX_ASPEED_JTAG_H
#define	_UAPI_LINUX_ASPEED_JTAG_H

/*
 * enum aspeed_jtag_xfer_mode:
 *
 * @ASPEED_JTAG_XFER_HW_MODE: hardware mode transfer;
 * @ASPEED_JTAG_XFER_SW_MODE: software mode transfer;
 */
enum aspeed_jtag_xfer_mode {
	ASPEED_JTAG_XFER_HW_MODE = 0,
	ASPEED_JTAG_XFER_SW_MODE = 1,
};

/*
 *  enum aspeed_jtag_endstate:
 * @ASPEED_JTAG_STATE_IDLE: JTAG sm state IDLE
 * @ASPEED_JTAG_STATE_IRPAUSE: JTAG sm state PAUSE_IR
 * @ASPEED_JTAG_STATE_DRPAUSE: JTAG sm state PAUSE_DR
 */
enum aspeed_jtag_endstate {
	ASPEED_JTAG_STATE_IDLE = 0,
	ASPEED_JTAG_STATE_IRPAUSE = 1,
	ASPEED_JTAG_STATE_DRPAUSE = 2,
};

/*
 * struct aspeed_jtag_runtest_idle jtag idle test:
 *
 * @mode: access mode: 0 - HW, 1 - SW;
 * @reset: logic reset;
 * @end: completion flag: 0 - idle, 1 - ir pause, 2 - dr pause;
 * @tck: clock counter;
 *
 * Structure represents interface to Aspeed JTAG device for jtag idle
 * execution.
 */
struct aspeed_jtag_runtest_idle {
	enum aspeed_jtag_xfer_mode	mode;
	unsigned char			reset;
	unsigned char			end;
	unsigned char			tck;
};

/*
 * struct aspeed_jtag_sir_xfer  jtag SIR xfer:
 *
 * @mode: access mode: 0 - HW, 1 - SW;
 * @length: xfer bits len;
 * @tdi : TDI xfer data;
 * @tdo : TDO xfer data;
 * @endir: xfer end state 0 - idle, 1 - pause;
 *
 * Structure represents interface to Aspeed JTAG device for jtag sir xfer
 * execution.
 */
struct aspeed_jtag_sir_xfer {
	enum aspeed_jtag_xfer_mode	mode;
	unsigned short			length;
	unsigned int			tdi;
	unsigned int			tdo;
	enum aspeed_jtag_endstate endir;
};

/*
 * struct aspeed_jtag_sdr_xfer jtag SDR xfer:
 *
 * @mode: access mode: 0 - HW, 1 - SW;
 * @direct: xfer direction: 0 -read, 1 - write;
 * @length: xfer bits len;
 * @*tdio : xfer data array;
 * @endir: xfer end state 0 - idle, 1 - pause;
 *
 * Structure represents interface to Aspeed JTAG device for jtag sdr xfer
 * execution.
 */
struct aspeed_jtag_sdr_xfer {
	enum aspeed_jtag_xfer_mode	mode;
	unsigned char			direct;
	unsigned short			length;
	unsigned int			*tdio;
	enum aspeed_jtag_endstate enddr;
};

#define __ASPEED_JTAG_IOCTL_MAGIC	0xb2

#define ASPEED_JTAG_IOCRUNTEST	_IOW(__ASPEED_JTAG_IOCTL_MAGIC, 0,\
				     struct aspeed_jtag_runtest_idle)
#define ASPEED_JTAG_IOCSIR	_IOWR(__ASPEED_JTAG_IOCTL_MAGIC, 1,\
				      struct aspeed_jtag_sir_xfer)
#define ASPEED_JTAG_IOCSDR	_IOWR(__ASPEED_JTAG_IOCTL_MAGIC, 2,\
				      struct aspeed_jtag_sdr_xfer)
#define ASPEED_JTAG_SIOCFREQ	_IOW(__ASPEED_JTAG_IOCTL_MAGIC, 3, unsigned int)
#define ASPEED_JTAG_GIOCFREQ	_IOR(__ASPEED_JTAG_IOCTL_MAGIC, 4, unsigned int)

#endif /* _UAPI_LINUX_ASPEED_JTAG_H */
