/*
 * Toshiba TC358762 DSI-to-DPI Bridge chip driver
 *
 * Copyright (C) Texas Instruments
 * Author: Tomi Valkeinen <tomi.valkeinen@ti.com>
 *
 * Based on original version from Jerry Alexander <x0135174@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/backlight.h>
#include <linux/fb.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/mutex.h>

#include <video/omapdss.h>
#include <video/omap-panel-tc358762.h>
#include <video/mipi_display.h>

#include "panel-tc358762.h"

static int tc358762_power_on(struct omap_dss_device *dssdev);
static void tc358762_power_off(struct omap_dss_device *dssdev);

struct tc762_i2c {
        struct i2c_client *client;
};
static struct tc762_i2c *i2c_data;

/* DISPC timings */
static const struct omap_video_timings tc358762_timings = {
        .x_res                    = 320,
        .y_res                    = 240,
        .pixel_clock              = 25600,
        .hfp                      = 44,
        .hsw                      = 4,
        .hbp                      = 112,
        .vfp                      = 2,
        .vsw                      = 5,
        .vbp                      = 7,
};

/* DSI timings */
static const struct omap_dss_dsi_videomode_data videomode_data = {
        .hfp                      = 106,
        .hsa                      = 68,
        .hbp                      = 172,
        .vfp                      = 3,
        .vsa                      = 4,
        .vbp                      = 7,

        .line_buffer              = 0,

        .vp_de_pol                = 1,
        .vp_vsync_pol             = 0,
        .vp_hsync_pol             = 0,
        .vp_hsync_start           = true,
        .vp_hsync_end             = true,
        .vp_vsync_start           = true,
        .vp_vsync_end             = true,
        .vp_eot_enable            = true,

        .blanking_mode            = 0,
        .hsa_blanking_mode        = 0,
        .hfp_blanking_mode        = 0,
        .hbp_blanking_mode        = 0,

        .ddr_clk_always_on        = true,
        .window_sync              = 4,

        .enter_hs_mode_latency    = 4,
        .exit_hs_mode_latency     = 3,
};

/* device private data structure */
struct tc358762_data {
        struct mutex lock;

        struct omap_dss_device *dssdev;
        struct kobject kobj;

        int pixel_channel;
};

/*** SYSFS interface ***/
static ssize_t tc358762_dispc_hfp_show(struct tc358762_data *tc_data, char *buf) {
        return snprintf(buf, PAGE_SIZE, "%d\n",
                        tc_data->dssdev->panel.timings.hfp);
}
static ssize_t tc358762_dispc_hbp_show(struct tc358762_data *tc_data, char *buf) {
        return snprintf(buf, PAGE_SIZE, "%d\n",
                        tc_data->dssdev->panel.timings.hbp);
}
static ssize_t tc358762_dispc_hsw_show(struct tc358762_data *tc_data, char *buf) {
        return snprintf(buf, PAGE_SIZE, "%d\n",
                        tc_data->dssdev->panel.timings.hsw);
}
static ssize_t tc358762_dispc_vfp_show(struct tc358762_data *tc_data, char *buf) {
        return snprintf(buf, PAGE_SIZE, "%d\n",
                        tc_data->dssdev->panel.timings.vfp);
}
static ssize_t tc358762_dispc_vbp_show(struct tc358762_data *tc_data, char *buf) {
        return snprintf(buf, PAGE_SIZE, "%d\n",
                        tc_data->dssdev->panel.timings.vbp);
}
static ssize_t tc358762_dispc_vsw_show(struct tc358762_data *tc_data, char *buf) {
        return snprintf(buf, PAGE_SIZE, "%d\n",
                        tc_data->dssdev->panel.timings.vsw);
}
static ssize_t tc358762_dispc_hfp_store(struct tc358762_data *tc_data,
                                     const char *buf, size_t size) {
        int r, value;
        r = kstrtoint(buf, 0, &value);
        if (r)
                return r;
        tc_data->dssdev->panel.timings.hfp = value;
        return size;
}
static ssize_t tc358762_dispc_hbp_store(struct tc358762_data *tc_data,
                                     const char *buf, size_t size) {
        int r, value;
        r = kstrtoint(buf, 0, &value);
        if (r)
                return r;
        tc_data->dssdev->panel.timings.hbp = value;
        return size;
}
static ssize_t tc358762_dispc_hsw_store(struct tc358762_data *tc_data,
                                     const char *buf, size_t size) {
        int r, value;
        r = kstrtoint(buf, 0, &value);
        if (r)
                return r;
        tc_data->dssdev->panel.timings.hsw = value;
        return size;
}
static ssize_t tc358762_dispc_vfp_store(struct tc358762_data *tc_data,
                                     const char *buf, size_t size) {
        int r, value;
        r = kstrtoint(buf, 0, &value);
        if (r)
                return r;
        tc_data->dssdev->panel.timings.vfp = value;
        return size;
}
static ssize_t tc358762_dispc_vbp_store(struct tc358762_data *tc_data,
                                     const char *buf, size_t size) {
        int r, value;
        r = kstrtoint(buf, 0, &value);
        if (r)
                return r;
        tc_data->dssdev->panel.timings.vbp = value;
        return size;
}
static ssize_t tc358762_dispc_vsw_store(struct tc358762_data *tc_data,
                                     const char *buf, size_t size) {
        int r, value;
        r = kstrtoint(buf, 0, &value);
        if (r)
                return r;
        tc_data->dssdev->panel.timings.vsw = value;
        return size;
}

/* DSI vm timings */
static ssize_t tc358762_dsi_vm_de_pol_show(struct tc358762_data *tc_data,
                                                char *buf) {
        return snprintf(buf, PAGE_SIZE, "%d\n",
                        tc_data->dssdev->panel.dsi_vm_data.vp_de_pol);
}
static ssize_t tc358762_dsi_vm_hsync_pol_show(struct tc358762_data *tc_data,
                                                char *buf) {
        return snprintf(buf, PAGE_SIZE, "%d\n",
                        tc_data->dssdev->panel.dsi_vm_data.vp_hsync_pol);
}
static ssize_t tc358762_dsi_vm_vsync_pol_show(struct tc358762_data *tc_data,
                                                char *buf) {
        return snprintf(buf, PAGE_SIZE, "%d\n",
                        tc_data->dssdev->panel.dsi_vm_data.vp_vsync_pol);
}
static ssize_t tc358762_dsi_vm_line_buffer_show(struct tc358762_data *tc_data,
                                                char *buf) {
        return snprintf(buf, PAGE_SIZE, "%d\n",
                        tc_data->dssdev->panel.dsi_vm_data.line_buffer);
}
static ssize_t tc358762_dsi_vm_window_sync_show(struct tc358762_data *tc_data,
                                                char *buf) {
        return snprintf(buf, PAGE_SIZE, "%d\n",
                        tc_data->dssdev->panel.dsi_vm_data.window_sync);
}
static ssize_t tc358762_dsi_vm_hfp_show(struct tc358762_data *tc_data,
                                        char *buf) {
        return snprintf(buf, PAGE_SIZE, "%d\n",
                        tc_data->dssdev->panel.dsi_vm_data.hfp);
}
static ssize_t tc358762_dsi_vm_hbp_show(struct tc358762_data *tc_data,
                                        char *buf) {
        return snprintf(buf, PAGE_SIZE, "%d\n",
                        tc_data->dssdev->panel.dsi_vm_data.hbp);
}
static ssize_t tc358762_dsi_vm_hsa_show(struct tc358762_data *tc_data,
                                        char *buf) {
        return snprintf(buf, PAGE_SIZE, "%d\n",
                        tc_data->dssdev->panel.dsi_vm_data.hsa);
}
static ssize_t tc358762_dsi_vm_vfp_show(struct tc358762_data *tc_data,
                                        char *buf) {
        return snprintf(buf, PAGE_SIZE, "%d\n",
                        tc_data->dssdev->panel.dsi_vm_data.vfp);
}
static ssize_t tc358762_dsi_vm_vbp_show(struct tc358762_data *tc_data,
                                        char *buf) {
        return snprintf(buf, PAGE_SIZE, "%d\n",
                        tc_data->dssdev->panel.dsi_vm_data.vbp);
}
static ssize_t tc358762_dsi_vm_vsa_show(struct tc358762_data *tc_data,
                                        char *buf) {
        return snprintf(buf, PAGE_SIZE, "%d\n",
                        tc_data->dssdev->panel.dsi_vm_data.vsa);
}
static ssize_t tc358762_dsi_vm_de_pol_store(struct tc358762_data *tc_data,
                                     const char *buf, size_t size) {
        int r, value;
        r = kstrtoint(buf, 0, &value);
        if (r)
                return r;
        value = !!value;
        tc_data->dssdev->panel.dsi_vm_data.vp_de_pol = value;
        return size;
}
static ssize_t tc358762_dsi_vm_hsync_pol_store(struct tc358762_data *tc_data,
                                     const char *buf, size_t size) {
        int r, value;
        r = kstrtoint(buf, 0, &value);
        if (r)
                return r;
        value = !!value;
        tc_data->dssdev->panel.dsi_vm_data.vp_hsync_pol = value;
        return size;
}
static ssize_t tc358762_dsi_vm_vsync_pol_store(struct tc358762_data *tc_data,
                                     const char *buf, size_t size) {
        int r, value;
        r = kstrtoint(buf, 0, &value);
        if (r)
                return r;
        value = !!value;
        tc_data->dssdev->panel.dsi_vm_data.vp_vsync_pol = value;
        return size;
}
static ssize_t tc358762_dsi_vm_line_buffer_store(struct tc358762_data *tc_data,
                                     const char *buf, size_t size) {
        int r, value;
        r = kstrtoint(buf, 0, &value);
        if (r)
                return r;
        if (value < 0 || value > 2)
                return -EINVAL;
        tc_data->dssdev->panel.dsi_vm_data.line_buffer = value;
        return size;
}
static ssize_t tc358762_dsi_vm_window_sync_store(struct tc358762_data *tc_data,
                                     const char *buf, size_t size) {
        int r, value;
        r = kstrtoint(buf, 0, &value);
        if (r)
                return r;
        tc_data->dssdev->panel.dsi_vm_data.window_sync = value;
        return size;
}
static ssize_t tc358762_dsi_vm_hfp_store(struct tc358762_data *tc_data,
                                     const char *buf, size_t size) {
        int r, value;
        r = kstrtoint(buf, 0, &value);
        if (r)
                return r;
        tc_data->dssdev->panel.dsi_vm_data.hfp = value;
        return size;
}
static ssize_t tc358762_dsi_vm_hbp_store(struct tc358762_data *tc_data,
                                     const char *buf, size_t size) {
        int r, value;
        r = kstrtoint(buf, 0, &value);
        if (r)
                return r;
        tc_data->dssdev->panel.dsi_vm_data.hbp = value;
        return size;
}
static ssize_t tc358762_dsi_vm_hsa_store(struct tc358762_data *tc_data,
                                     const char *buf, size_t size) {
        int r, value;
        r = kstrtoint(buf, 0, &value);
        if (r)
                return r;
        tc_data->dssdev->panel.dsi_vm_data.hsa = value;
        return size;
}
static ssize_t tc358762_dsi_vm_vfp_store(struct tc358762_data *tc_data,
                                     const char *buf, size_t size) {
        int r, value;
        r = kstrtoint(buf, 0, &value);
        if (r)
                return r;
        tc_data->dssdev->panel.dsi_vm_data.vfp = value;
        return size;
}
static ssize_t tc358762_dsi_vm_vbp_store(struct tc358762_data *tc_data,
                                     const char *buf, size_t size) {
        int r, value;
        r = kstrtoint(buf, 0, &value);
        if (r)
                return r;
        tc_data->dssdev->panel.dsi_vm_data.vbp = value;
        return size;
}
static ssize_t tc358762_dsi_vm_vsa_store(struct tc358762_data *tc_data,
                                     const char *buf, size_t size) {
        int r, value;
        r = kstrtoint(buf, 0, &value);
        if (r)
                return r;
        tc_data->dssdev->panel.dsi_vm_data.vsa = value;
        return size;
}

static ssize_t tc358762_sysfs_reset(struct tc358762_data *tc_data,
                                    const char *buf, size_t size) {
        tc358762_power_off(tc_data->dssdev);
        msleep(200);
        tc358762_power_on(tc_data->dssdev);
        return size;
}

struct tc358762_attribute {
        struct attribute attr;
        ssize_t (*show)(struct tc358762_data *, char *);
        ssize_t (*store)(struct tc358762_data *, const char *, size_t);
};

#define OVERLAY_ATTR(_name, _mode, _show, _store) \
        struct tc358762_attribute tc358762_attr_##_name = \
        __ATTR(_name, _mode, _show, _store)


static OVERLAY_ATTR(reset, S_IWUSR, NULL, tc358762_sysfs_reset);

static OVERLAY_ATTR(dispc_hfp, S_IRUGO|S_IWUSR,
                tc358762_dispc_hfp_show, tc358762_dispc_hfp_store);
static OVERLAY_ATTR(dispc_hbp, S_IRUGO|S_IWUSR,
                tc358762_dispc_hbp_show, tc358762_dispc_hbp_store);
static OVERLAY_ATTR(dispc_hsw, S_IRUGO|S_IWUSR,
                tc358762_dispc_hsw_show, tc358762_dispc_hsw_store);
static OVERLAY_ATTR(dispc_vfp, S_IRUGO|S_IWUSR,
                tc358762_dispc_vfp_show, tc358762_dispc_vfp_store);
static OVERLAY_ATTR(dispc_vbp, S_IRUGO|S_IWUSR,
                tc358762_dispc_vbp_show, tc358762_dispc_vbp_store);
static OVERLAY_ATTR(dispc_vsw, S_IRUGO|S_IWUSR,
                tc358762_dispc_vsw_show, tc358762_dispc_vsw_store);

static OVERLAY_ATTR(dsi_vm_de_pol, S_IRUGO|S_IWUSR,
                tc358762_dsi_vm_de_pol_show,
                tc358762_dsi_vm_de_pol_store);
static OVERLAY_ATTR(dsi_vm_hsync_pol, S_IRUGO|S_IWUSR,
                tc358762_dsi_vm_hsync_pol_show,
                tc358762_dsi_vm_hsync_pol_store);
static OVERLAY_ATTR(dsi_vm_vsync_pol, S_IRUGO|S_IWUSR,
                tc358762_dsi_vm_vsync_pol_show,
                tc358762_dsi_vm_vsync_pol_store);
static OVERLAY_ATTR(dsi_vm_line_buffer, S_IRUGO|S_IWUSR,
                tc358762_dsi_vm_line_buffer_show,
                tc358762_dsi_vm_line_buffer_store);
static OVERLAY_ATTR(dsi_vm_window_sync, S_IRUGO|S_IWUSR,
                tc358762_dsi_vm_window_sync_show,
                tc358762_dsi_vm_window_sync_store);
static OVERLAY_ATTR(dsi_vm_hfp, S_IRUGO|S_IWUSR,
                tc358762_dsi_vm_hfp_show, tc358762_dsi_vm_hfp_store);
static OVERLAY_ATTR(dsi_vm_hbp, S_IRUGO|S_IWUSR,
                tc358762_dsi_vm_hbp_show, tc358762_dsi_vm_hbp_store);
static OVERLAY_ATTR(dsi_vm_hsa, S_IRUGO|S_IWUSR,
                tc358762_dsi_vm_hsa_show, tc358762_dsi_vm_hsa_store);
static OVERLAY_ATTR(dsi_vm_vfp, S_IRUGO|S_IWUSR,
                tc358762_dsi_vm_vfp_show, tc358762_dsi_vm_vfp_store);
static OVERLAY_ATTR(dsi_vm_vbp, S_IRUGO|S_IWUSR,
                tc358762_dsi_vm_vbp_show, tc358762_dsi_vm_vbp_store);
static OVERLAY_ATTR(dsi_vm_vsa, S_IRUGO|S_IWUSR,
                tc358762_dsi_vm_vsa_show, tc358762_dsi_vm_vsa_store);

static struct attribute *tc358762_sysfs_attrs[] = {
        &tc358762_attr_reset.attr,
        &tc358762_attr_dispc_hfp.attr,
        &tc358762_attr_dispc_hbp.attr,
        &tc358762_attr_dispc_hsw.attr,
        &tc358762_attr_dispc_vfp.attr,
        &tc358762_attr_dispc_vbp.attr,
        &tc358762_attr_dispc_vsw.attr,
        &tc358762_attr_dsi_vm_de_pol.attr,
        &tc358762_attr_dsi_vm_hsync_pol.attr,
        &tc358762_attr_dsi_vm_vsync_pol.attr,
        &tc358762_attr_dsi_vm_line_buffer.attr,
        &tc358762_attr_dsi_vm_window_sync.attr,
        &tc358762_attr_dsi_vm_hfp.attr,
        &tc358762_attr_dsi_vm_hbp.attr,
        &tc358762_attr_dsi_vm_hsa.attr,
        &tc358762_attr_dsi_vm_vfp.attr,
        &tc358762_attr_dsi_vm_vbp.attr,
        &tc358762_attr_dsi_vm_vsa.attr,
        NULL
};

static ssize_t tc358762_attr_show(struct kobject *kobj, struct attribute *attr,
                char *buf)
{
        struct tc358762_data *tc358762;
        struct tc358762_attribute *tc358762_attr;

        tc358762 = container_of(kobj, struct tc358762_data, kobj);
        tc358762_attr = container_of(attr, struct tc358762_attribute, attr);

        if (!tc358762_attr->show)
                return -ENOENT;

        return tc358762_attr->show(tc358762, buf);
}

static ssize_t tc358762_attr_store(struct kobject *kobj, struct attribute *attr,
                const char *buf, size_t size)
{
        struct tc358762_data *tc358762;
        struct tc358762_attribute *tc358762_attr;

        tc358762 = container_of(kobj, struct tc358762_data, kobj);
        tc358762_attr = container_of(attr, struct tc358762_attribute, attr);

        if (!tc358762_attr->store)
                return -ENOENT;

        return tc358762_attr->store(tc358762, buf, size);
}

static const struct sysfs_ops tc358762_sysfs_ops = {
        .show = tc358762_attr_show,
        .store = tc358762_attr_store,
};

static struct kobj_type tc358762_ktype = {
        .sysfs_ops = &tc358762_sysfs_ops,
        .default_attrs = tc358762_sysfs_attrs,
};

static struct tc358762_board_data *get_board_data(struct omap_dss_device *dssdev)
{
        return (struct tc358762_board_data *)dssdev->data;
}

static int tc358762_write_register(struct omap_dss_device *dssdev, u16 reg,
                u32 value)
{
        u8 buf[6];
        int r;

        /* Send commands via I2C */
        struct i2c_msg msgs[1];

        if (!i2c_data || !i2c_data->client) {
                dev_err(&dssdev->dev, "No I2C data set for tc'762\n");
                return -1;
        }

        buf[0] = (reg >> 8) & 0xff;
        buf[1] = (reg >> 0) & 0xff;
        buf[2] = (value >> 0) & 0xff;
        buf[3] = (value >> 8) & 0xff;
        buf[4] = (value >> 16) & 0xff;
        buf[5] = (value >> 24) & 0xff;

        msgs[0].addr = 0xb;
        msgs[0].flags = 0;
        msgs[0].len = sizeof(buf);
        msgs[0].buf = buf;

        r = i2c_transfer(i2c_data->client->adapter, msgs, 1);
        if (r < 0) {
                dev_err(&dssdev->dev, "Failed i2c write to tc'762\n");
                return r;
        }

        return 0;
}

#if 0
/****************************
********* DEBUG *************
****************************/
static void dump_regs(struct omap_dss_device *dssdev)
{
#define DUMPREG(r) do {                               \
  u32 val = 0;                                        \
  int ret = tc358762_read_register(dssdev, r, &val);       \
  if (ret) {                                          \
    printk("%-35s : Read Error (%i)\n", #r, ret);     \
  } else {                                            \
    printk("%-35s : %08x\n", #r, val);                \
  }                                                   \
} while (0);                                          \

        printk(KERN_ALERT "TC'762 Toshiba Registers\n");

        printk(KERN_ALERT "DSI D-PHY Layer Registers\n");
        DUMPREG(D0W_DPHYCONTTX);
        DUMPREG(CLW_DPHYCONTRX);
        DUMPREG(D0W_DPHYCONTRX);
        DUMPREG(D1W_DPHYCONTRX);
        DUMPREG(COM_DPHYCONTRX);
        DUMPREG(MON_DPHYRX);
        DUMPREG(CLW_CNTRL);
        DUMPREG(D0W_CNTRL);
        DUMPREG(D1W_CNTRL);
        DUMPREG(DFTMODE_CNTRL);

        printk(KERN_ALERT "DSI PPI Layer Registers\n");
        DUMPREG(PPI_STARTPPI);
        DUMPREG(PPI_BUSYPPI);
        DUMPREG(PPI_LINEINITCNT);
        DUMPREG(PPI_LPTXTIMECNT);
        DUMPREG(PPI_CLS_ATMR);
        DUMPREG(PPI_D0S_ATMR);
        DUMPREG(PPI_D1S_ATMR);
        DUMPREG(PPI_D0S_CLRSIPOCOUNT);
        DUMPREG(PPI_D1S_CLRSIPOCOUNT);
        DUMPREG(CLS_PRE);
        DUMPREG(D0S_PRE);
        DUMPREG(D1S_PRE);
        DUMPREG(CLS_PREP);
        DUMPREG(D0S_PREP);
        DUMPREG(D1S_PREP);
        DUMPREG(CLS_ZERO);
        DUMPREG(D0S_ZERO);
        DUMPREG(D1S_ZERO);
        DUMPREG(PPI_CLRFLG);
        DUMPREG(PPI_CLRSIPO);
        DUMPREG(PPI_HSTimeout);
        DUMPREG(PPI_HSTimeoutEnable);

        printk(KERN_ALERT "DSI Protocol Layer Registers\n");
        DUMPREG(DSI_STARTDSI);
        DUMPREG(DSI_BUSYDSI);
        DUMPREG(DSI_LANEENABLE);
        DUMPREG(DSI_LANESTATUS0);
        DUMPREG(DSI_LANESTATUS1);
        DUMPREG(DSI_INTSTATUS);
        DUMPREG(DSI_INTMASK);
        DUMPREG(DSI_INTCLR);
        DUMPREG(DSI_LPTXTO);
        DUMPREG(DSI_MODE);
        DUMPREG(DSI_PAYLOAD0);
        DUMPREG(DSI_PAYLOAD1);
        DUMPREG(DSI_BTASTAT);
        DUMPREG(DSI_BTACLR);

        printk(KERN_ALERT "DSI General Registers\n");
        DUMPREG(DSIERRCNT);
        DUMPREG(DSISIGMOD);

        printk(KERN_ALERT "DSI Application Layer Registers\n");
        DUMPREG(APLCTRL);
        DUMPREG(APLSTAT);
        DUMPREG(APLERR);
        DUMPREG(PWRMOD);
        DUMPREG(RDPKTLN);
        DUMPREG(PXLFMT);
        DUMPREG(MEMWRCMD);

        printk(KERN_ALERT "LCDC/DPI Host Registers\n");
        DUMPREG(LCDCTRL_PORT);
        DUMPREG(HSR_HBPR);
        DUMPREG(HDISPR_HFPR);
        DUMPREG(VSR_VBPR);
        DUMPREG(VDISPR_VFPR);
        DUMPREG(VFUEN);

        printk(KERN_ALERT "System Controller Registers\n");
        DUMPREG(SYSSTAT);
        DUMPREG(SYSCTRL);
        DUMPREG(SYSPLL1);
        DUMPREG(SYSPLL2);
        DUMPREG(SYSPLL3);
        DUMPREG(SYSPMCTRL);

        printk(KERN_ALERT "GPIO Registers\n");
        DUMPREG(GPIOC);
        DUMPREG(GPIOO);
        DUMPREG(GPIOI);

        printk(KERN_ALERT "Chip Revision Registers\n");
        DUMPREG(IDREG);

        printk(KERN_ALERT "Debug Registers\n");
        DUMPREG(DEBUG00);

        printk(KERN_ALERT "Command Queue\n");
        DUMPREG(WCMDQUE);
        DUMPREG(RCMDQUE);
#undef DUMPREG
}
EXPORT_SYMBOL(dump_regs);
#endif


static void tc358762_get_timings(struct omap_dss_device *dssdev,
                struct omap_video_timings *timings)
{
        *timings = dssdev->panel.timings;
}

static void tc358762_set_timings(struct omap_dss_device *dssdev,
                struct omap_video_timings *timings)
{
}

static int tc358762_check_timings(struct omap_dss_device *dssdev,
                struct omap_video_timings *timings)
{
        if (tc358762_timings.x_res != timings->x_res ||
                        tc358762_timings.y_res != timings->y_res ||
                        tc358762_timings.pixel_clock != timings->pixel_clock ||
                        tc358762_timings.hsw != timings->hsw ||
                        tc358762_timings.hfp != timings->hfp ||
                        tc358762_timings.hbp != timings->hbp ||
                        tc358762_timings.vsw != timings->vsw ||
                        tc358762_timings.vfp != timings->vfp ||
                        tc358762_timings.vbp != timings->vbp)
                return -EINVAL;

        return 0;
}

static void tc358762_get_resolution(struct omap_dss_device *dssdev,
                u16 *xres, u16 *yres)
{
        *xres = tc358762_timings.x_res;
        *yres = tc358762_timings.y_res;
}

static int tc358762_hw_reset(struct omap_dss_device *dssdev)
{
        struct tc358762_board_data *board_data = get_board_data(dssdev);

        if (board_data == NULL || board_data->reset_gpio == -1) {
                dev_err(&dssdev->dev, "Failed to reset tc'762, no reset_gpio "
                        "configured.\n");
                return 0;
        }

        msleep(200);

        /* reset the panel */
        gpio_set_value(board_data->reset_gpio, 0);
        udelay(100);
        gpio_set_value(board_data->reset_gpio, 1);

        msleep(10);

        return 0;
}

static int tc358762_probe(struct omap_dss_device *dssdev)
{
        struct tc358762_data *tc_drv_data;
        int r = 0;

        dev_dbg(&dssdev->dev, "tc358762_probe\n");

        dssdev->panel.config = OMAP_DSS_LCD_TFT;
        dssdev->panel.timings = tc358762_timings;
        dssdev->panel.dsi_vm_data = videomode_data;
        dssdev->ctrl.pixel_size = 18;

        dssdev->panel.acbi = 0;
        dssdev->panel.acb = 40;

        tc_drv_data = kzalloc(sizeof(*tc_drv_data), GFP_KERNEL);
        if (!tc_drv_data) {
                r = -ENOMEM;
                goto err0;
        }

        tc_drv_data->dssdev = dssdev;

        mutex_init(&tc_drv_data->lock);

        dev_set_drvdata(&dssdev->dev, tc_drv_data);

        r = omap_dsi_request_vc(dssdev, &tc_drv_data->pixel_channel);
        if (r) {
                dev_err(&dssdev->dev, "failed to get virtual channel for"
                        " transmitting pixel data\n");
                goto err0;
        }

        r = omap_dsi_set_vc_id(dssdev, tc_drv_data->pixel_channel, 0);
        if (r) {
                dev_err(&dssdev->dev, "failed to set VC_ID for pixel data"
                        " virtual channel\n");
                goto err1;
        }

        r = kobject_init_and_add(&tc_drv_data->kobj, &tc358762_ktype,
                        &dssdev->manager->kobj, "tc358762");
        if (r) {
                dev_err(&dssdev->dev, "failed to create sysfs directory\n");
                goto err1;
        }

        dev_dbg(&dssdev->dev, "tc358762_probe done\n");

        return 0;

err1:
        omap_dsi_release_vc(dssdev, tc_drv_data->pixel_channel);
err0:
        kfree(tc_drv_data);

        return r;
}

static void tc358762_remove(struct omap_dss_device *dssdev)
{
        struct tc358762_data *tc_drv_data = dev_get_drvdata(&dssdev->dev);

        omap_dsi_release_vc(dssdev, tc_drv_data->pixel_channel);

        kobject_del(&tc_drv_data->kobj);
        kobject_put(&tc_drv_data->kobj);
        kfree(tc_drv_data);
}

static struct
{
        u16 reg;
        u32 data;
} tc358762_init_seq[] = {
        { SYSPMCTRL,                 0x00000000 },
        /* Delay for 5 ms to allow the '762 to come out of sleep mode. */
        { DELAY_INIT_SEQ,                     5 },
        { DSI_LANEENABLE,            0x00000003 },
        { PPI_D0S_CLRSIPOCOUNT,      0x00000005 },
        { PPI_D0S_ATMR,              0x00000000 },
        { PPI_STARTPPI,              0x00000001 },
        { DSI_STARTDSI,              0x00000001 },
        { LCDCTRL_PORT,              LCDCTRL_DPI_ENABLE |
                                     LCDCTRL_RGB666_PACKED },
        { VFUEN,                     0x00000001 },
};

static int tc358762_write_init_config(struct omap_dss_device *dssdev)
{
        int i;
        int r;

        for (i = 0; i < ARRAY_SIZE(tc358762_init_seq); ++i) {
                u16 reg = tc358762_init_seq[i].reg;
                u32 data = tc358762_init_seq[i].data;

                if (reg == DELAY_INIT_SEQ) {
                  msleep(data);
                  continue;
                }

                r = tc358762_write_register(dssdev, reg, data);
                if (r) {
                        dev_err(&dssdev->dev, "Failed to write initial config"
                                " to tc'762 while writing 0x%08x to 0x%04x\n",
                                data, reg);
                        return r;
                }
        }

        return 0;
}

static int tc358762_power_on(struct omap_dss_device *dssdev)
{
        struct tc358762_data *tc_drv_data = dev_get_drvdata(&dssdev->dev);
        int r;

        dev_dbg(&dssdev->dev, "power_on\n");

        if (dssdev->platform_enable)
                dssdev->platform_enable(dssdev);

        r = omapdss_dsi_display_enable(dssdev);
        if (r) {
                dev_err(&dssdev->dev, "Failed to enable DSI for tc'762\n");
                goto err_disp_enable;
        }

        omapdss_dsi_vc_enable_hs(dssdev, tc_drv_data->pixel_channel, true);
        dsi_videomode_panel_preinit(dssdev, tc_drv_data->pixel_channel);

        /* reset tc358762 bridge */
        tc358762_hw_reset(dssdev);

        /* configure D2L chip DSI-RX configuration registers */
        tc358762_write_init_config(dssdev);

        dsi_video_mode_enable(dssdev, MIPI_DSI_PACKED_PIXEL_STREAM_18,
                tc_drv_data->pixel_channel);

        dev_dbg(&dssdev->dev, "power_on done\n");

        return 0;

err_disp_enable:
        if (dssdev->platform_disable)
                dssdev->platform_disable(dssdev);

        return r;
}

static void tc358762_power_off(struct omap_dss_device *dssdev)
{
        struct tc358762_data *tc_drv_data = dev_get_drvdata(&dssdev->dev);
        struct tc358762_board_data *board_data = get_board_data(dssdev);

        /* Hold the chip in reset while it's powered off. */
        if (board_data && board_data->reset_gpio > -1) {
                gpio_set_value(board_data->reset_gpio, 0);
        }

        dsi_video_mode_disable(dssdev, tc_drv_data->pixel_channel);
        omapdss_dsi_display_disable(dssdev, false, false);

        if (dssdev->platform_disable)
                dssdev->platform_disable(dssdev);
}

static void tc358762_disable(struct omap_dss_device *dssdev)
{
        struct tc358762_data *tc_drv_data = dev_get_drvdata(&dssdev->dev);

        dev_dbg(&dssdev->dev, "disable\n");

        if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE)
        {
                mutex_lock(&tc_drv_data->lock);
                dsi_bus_lock(dssdev);

                tc358762_power_off(dssdev);

                dsi_bus_unlock(dssdev);
                mutex_unlock(&tc_drv_data->lock);
        }

        dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
}

static int tc358762_enable(struct omap_dss_device *dssdev)
{
        struct tc358762_data *tc_drv_data = dev_get_drvdata(&dssdev->dev);
        int r = 0;

        dev_dbg(&dssdev->dev, "enable\n");

        if (dssdev->state != OMAP_DSS_DISPLAY_DISABLED)
                return -EINVAL;

        mutex_lock(&tc_drv_data->lock);
        dsi_bus_lock(dssdev);

        r = tc358762_power_on(dssdev);

        dsi_bus_unlock(dssdev);

        if (r) {
                dev_dbg(&dssdev->dev, "enable failed\n");
                dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
        } else {
                dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
        }

        mutex_unlock(&tc_drv_data->lock);

        return r;
}

static struct omap_dss_driver tc358762_driver = {
        .probe                 = tc358762_probe,
        .remove                = tc358762_remove,

        .enable                = tc358762_enable,
        .disable               = tc358762_disable,

        .get_resolution        = tc358762_get_resolution,
        .get_recommended_bpp   = omapdss_default_get_recommended_bpp,

        .get_timings           = tc358762_get_timings,
        .set_timings           = tc358762_set_timings,
        .check_timings         = tc358762_check_timings,

        .driver                = {
                .name                  = "tc358762",
                .owner                 = THIS_MODULE,
        },
};

static int __devinit i2c_probe(struct i2c_client *client,
                               const struct i2c_device_id *id)
{
        i2c_data = kzalloc(sizeof(struct tc762_i2c), GFP_KERNEL);
        if (i2c_data == NULL)
                return -ENOMEM;

        /* store i2c_client pointer on private data structure */
        i2c_data->client = client;

        /* store private data structure pointer on i2c_client structure */
        i2c_set_clientdata(client, i2c_data);

        /* init mutex */
//        mutex_init(&sd1->xfer_lock);

        return 0;
}

/* driver remove function */
static int __devexit i2c_remove(struct i2c_client *client)
{
        struct tc762_i2c *drv_data = i2c_get_clientdata(client);

        /* remove client data */
        i2c_set_clientdata(client, NULL);

        /* free private data memory */
        i2c_data = NULL;
        kfree(drv_data);

        return 0;
}

static const struct i2c_device_id i2c_idtable[] = {
        {"tc358762-i2c", 0},
        {},
};

static struct i2c_driver i2c_driver = {
        .probe = i2c_probe,
        .remove = __exit_p(i2c_remove),
        .id_table = i2c_idtable,
        .driver = {
                   .name  = "tc358762-i2c",
                   .owner = THIS_MODULE,
        },
};

static int __init tc358762_init(void)
{
        int r = 0;
        r = i2c_add_driver(&i2c_driver);
        if (r < 0) {
                printk(KERN_WARNING "tc'762 i2c driver registration failed\n");
                return r;
        }

        omap_dss_register_driver(&tc358762_driver);;
        return 0;
}

static void __exit tc358762_exit(void)
{
        omap_dss_unregister_driver(&tc358762_driver);
        i2c_del_driver(&i2c_driver);
}

module_init(tc358762_init);
module_exit(tc358762_exit);

MODULE_DESCRIPTION("TC358762 DSI-2-DPI Bridge Driver");
MODULE_LICENSE("GPL");
