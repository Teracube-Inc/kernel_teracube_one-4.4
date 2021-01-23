/*
 * Copyright (C) 2006-2017 ILITEK TECHNOLOGY CORP.
 *
 * Description: ILITEK I2C touchscreen driver for linux platform.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see the file COPYING, or write
 * to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * Author: Johnson Yeh
 * Maintain: Luca Hsu, Tigers Huang, Dicky Chiang
 */
 
#ifndef __ILITEK_DRV_ILIPROC_H__
#define __ILITEK_DRV_ILIPROC_H__

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 36)
long ilitek_file_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
#else
int  ilitek_file_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg);
#endif
ssize_t ilitek_file_read(struct file *filp, char *buf, size_t count, loff_t *f_pos);
ssize_t ilitek_file_write(struct file *filp, const char *buf, size_t count, loff_t *f_pos);
int ilitek_file_close(struct inode *inode, struct file *filp);
int ilitek_file_open(struct inode *inode, struct file *filp);

#endif //__ILITEK_DRV_ILIPROC_H__
