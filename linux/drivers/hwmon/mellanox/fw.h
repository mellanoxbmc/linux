/*
 * Copyright (C) Mellanox Technologies, Ltd. 2010-2015 ALL RIGHTS RESERVED.
 *
 * This software product is a proprietary product of Mellanox Technologies, Ltd.
 * (the "Company") and all right, title, and interest in and to the software product,
 * including all associated intellectual property rights, are and shall
 * remain exclusively with the Company.
 *
 * This software product is governed by the End User License Agreement
 * provided with the software product.
 *
 */

#ifndef SX_FW_H
#define SX_FW_H

#include <linux/mlx_sx/device.h>
#include "icm.h"

int sx_SET_PROFILE(struct sx_dev *dev, struct ku_profile *profile);
int sx_QUERY_FW_2(struct sx_dev *dev, int sx_dev_id);
int sx_MAP_FA(struct sx_dev *dev, struct sx_icm *icm);
int sx_UNMAP_FA(struct sx_dev *dev);

#endif /* SX_FW_H */
