/*
 * Copyright (c) 2017 Mellanox Technologies. All rights reserved.
 * Copyright (c) 2017 Vadim Pasternak <vadimp@mellanox.com>
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

#ifndef	_UAPI_LINUX_ASPEED_MCTP_H
#define	_UAPI_LINUX_ASPEED_MCTP_H

#define ASPEED_MCTP_XFER_BUFF_SIZE	1024

/*
 * struct aspeed_mctp_xfer - ioctl structure to transfer data to Aspeed SoC
 * MCTP controller:
 *
 * @xfer_buff: transfer data buffer;
 * @xfer_len: transfer data length;
 * @ep_id: destination end-point id;
 * @pci_bus: destination PCIe bus number;
 * @pci_dev: destination PCIe device number;
 * @pci_fun: destination PCIe function number;
 * @ret: return code;
 *
 * Structure represents interface to Aspeed SoC device for MCTP data transfer
 * execution. It allows MCTP packets receiving through PCIe from a host and
 * sending MCTP packets through PCI a host.
 */
struct aspeed_mctp_xfer {
	unsigned char	xfer_buff[ASPEED_MCTP_XFER_BUFF_SIZE];
	unsigned int	xfer_len;
	unsigned int	ep_id;
	unsigned int	pci_bus;
	unsigned int	pci_dev;
	unsigned int	pci_fun;
	unsigned char	ret;
};

#define ASPEED_MCTP_IO_BASE     0x4d
#define ASPEED_MCTP_IOCRX       _IOWR(ASPEED_MCTP_IO_BASE, 0x10, \
				      struct aspeed_mctp_xfer)
#define ASPEED_MCTP_IOCTX       _IOW(ASPEED_MCTP_IO_BASE, 0x11, \
				     struct aspeed_mctp_xfer)

#endif /* _UAPI_LINUX_ASPEED_MCTP_H */
