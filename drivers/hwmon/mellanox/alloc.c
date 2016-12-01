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


/************************************************
 * Includes
 ***********************************************/

#include <asm/atomic.h>
#include <linux/pci.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/bitmap.h>
#include <linux/dma-mapping.h>
#include <linux/vmalloc.h>
#include <linux/mlx_sx/device.h>
#include "alloc.h"
#include "sx.h"

/************************************************
 * Functions
 ***********************************************/

u32 sx_bitmap_test(struct sx_bitmap *bitmap, u32 obj)
{
	u32 ret;
	unsigned long flags;

	spin_lock_irqsave(&bitmap->lock, flags);

	if (obj >= bitmap->max) {
		ret = 0;
	} else if (test_bit(obj, bitmap->table)) {
		ret = 1;
	} else {
		ret = 0;
	}

	spin_unlock_irqrestore(&bitmap->lock, flags);

	return ret;
}

u32 sx_bitmap_set(struct sx_bitmap *bitmap, u32 obj)
{
	u32 ret = obj;
	unsigned long flags;

	spin_lock_irqsave(&bitmap->lock, flags);

	if (obj >= bitmap->max) {
		ret = -1;
	} else if (test_bit(obj, bitmap->table)) {
		ret = -1;
	} else {
		set_bit(obj, bitmap->table);
	}

	spin_unlock_irqrestore(&bitmap->lock, flags);

	return ret;
}

u32 sx_bitmap_alloc(struct sx_bitmap *bitmap)
{
	u32 obj;
	unsigned long flags;

	spin_lock_irqsave(&bitmap->lock, flags);

	obj = find_next_zero_bit(bitmap->table, bitmap->max, 0);
	if (obj >= bitmap->max) {
		obj = -1;
	} else {
		set_bit(obj, bitmap->table);
	}

	spin_unlock_irqrestore(&bitmap->lock, flags);

	return obj;
}

void sx_bitmap_free(struct sx_bitmap *bitmap, u32 obj)
{
	unsigned long flags;

	WARN_ON(obj >= bitmap->max);

	spin_lock_irqsave(&bitmap->lock, flags);
	clear_bit(obj, bitmap->table);
	spin_unlock_irqrestore(&bitmap->lock, flags);
}

/*
 * Handling for queue buffers -- we allocate a bunch of memory and
 * register it in a memory region at Switch virtual address 0.  If the
 * requested size is > max_direct, we split the allocation into
 * multiple pages, so we don't require too much contiguous memory.
 */
int sx_buf_alloc(struct sx_dev *dev, int size, int max_direct,
		 struct sx_buf *buf, int init_val)
{
	dma_addr_t t = 0;

	if (size <= max_direct) {
		buf->nbufs        = 1;
		buf->npages       = (size + PAGE_SIZE - 1) / PAGE_SIZE;
		buf->page_shift   = PAGE_SHIFT;
		if (!dev->pdev) {
			buf->u.direct.buf = kmalloc(size, GFP_KERNEL);
		} else {
			buf->u.direct.buf = dma_alloc_coherent(&dev->pdev->dev, size, &t, GFP_KERNEL);
		}

		if (!buf->u.direct.buf) {
			return -ENOMEM;
		}

		buf->u.direct.map = t;


		memset(buf->u.direct.buf, init_val, size);
	} else {
		int i;

		buf->nbufs       = (size + PAGE_SIZE - 1) / PAGE_SIZE;
		buf->npages      = buf->nbufs;
		buf->page_shift  = PAGE_SHIFT;
		buf->u.page_list = kzalloc(buf->nbufs*sizeof(*buf->u.page_list), GFP_KERNEL);
		if (!buf->u.page_list) {
			return -ENOMEM;
		}

		for (i = 0; i < buf->nbufs; ++i) {
			if (!dev->pdev) {
				buf->u.page_list[i].buf =
						kmalloc(PAGE_SIZE, GFP_KERNEL);
			} else {
				buf->u.page_list[i].buf =
						dma_alloc_coherent(&dev->pdev->dev, PAGE_SIZE, &t, GFP_KERNEL);
			}

			if (!buf->u.page_list[i].buf) {
				goto err_free;
			}

			buf->u.page_list[i].map = t;

			memset(buf->u.page_list[i].buf, init_val, PAGE_SIZE);
		}

		if (BITS_PER_LONG == 64) {
			struct page **pages;
			pages = kmalloc(sizeof *pages * buf->nbufs, GFP_KERNEL);
			if (!pages)
				goto err_free;
			for (i = 0; i < buf->nbufs; ++i) {
				pages[i] =
					virt_to_page(buf->u.page_list[i].buf);
			}

			buf->u.direct.buf = vmap(pages, buf->nbufs,
						 VM_MAP, PAGE_KERNEL);
			kfree(pages);
			if (!buf->u.direct.buf) {
				goto err_free;
			}
		}
	}

	return 0;

err_free:
	sx_buf_free(dev, size, buf);

	return -ENOMEM;
}


void sx_fill_page_list(__be64 *dma_addrs, struct sx_buf *buf)
{
	int i;
	dma_addr_t map;

	if (buf->nbufs == 1) {
		map = buf->u.direct.map;
		for (i = 0; i < buf->npages; i++) {
			*dma_addrs++ = cpu_to_be64(map & 0xfffffffffffff000ULL);
			map += PAGE_SIZE;
		}
	} else {
		for (i = 0; i < buf->nbufs; ++i) {
			*dma_addrs++ = cpu_to_be64(buf->u.page_list[i].map
					& 0xfffffffffffff000ULL);
		}
	}
}

void sx_buf_free(struct sx_dev *dev, int size, struct sx_buf *buf)
{
	int i;

	if (buf->nbufs == 1) {
		if (!dev->pdev) {
			kfree(buf->u.direct.buf);
		} else {
			dma_free_coherent(&dev->pdev->dev, size, buf->u.direct.buf,
					buf->u.direct.map);
		}
	} else {
		if (BITS_PER_LONG == 64)
			vunmap(buf->u.direct.buf);

		for (i = 0; i < buf->nbufs; ++i) {
			if (buf->u.page_list[i].buf) {
				if (!dev->pdev) {
					kfree(buf->u.page_list[i].buf);
				} else {
					dma_free_coherent(&dev->pdev->dev, PAGE_SIZE,
							buf->u.page_list[i].buf,
							buf->u.page_list[i].map);
				}
			}
		}

		kfree(buf->u.page_list);
	}
}


int sx_bitmap_init(struct sx_bitmap *bitmap, u32 num)
{
	if (num > (8 * sizeof bitmap->table))
		return -EINVAL;

	bitmap->max  = num;
	memset(bitmap->table, 0, sizeof bitmap->table);
	spin_lock_init(&bitmap->lock);

	return 0;
}

/************************************************
 *                  EOF                         *
 ***********************************************/

