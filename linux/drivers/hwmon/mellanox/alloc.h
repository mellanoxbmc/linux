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


#ifndef ALLOC_H_
#define ALLOC_H_

/************************************************
 * Includes
 ***********************************************/

#include "sx.h"

/************************************************
 * Function prototypes
 ***********************************************/
u32 sx_bitmap_test(struct sx_bitmap *bitmap, u32 obj);
u32 sx_bitmap_set(struct sx_bitmap *bitmap, u32 obj);
u32  sx_bitmap_alloc(struct sx_bitmap *bitmap);
void sx_bitmap_free(struct sx_bitmap *bitmap, u32 obj);
int sx_buf_alloc(struct sx_dev *dev, int size, int max_direct,
		 struct sx_buf *buf, int init_val);
void sx_fill_page_list(__be64 *dma_addrs, struct sx_buf *buf);
void sx_buf_free(struct sx_dev *dev, int size, struct sx_buf *buf);
void sx_bitmap_free(struct sx_bitmap *bitmap, u32 obj);
int sx_bitmap_init(struct sx_bitmap *bitmap, u32 num);

#endif /* ALLOC_H_ */

/************************************************
 *                  EOF                         *
 ***********************************************/
