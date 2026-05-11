// SPDX-License-Identifier: GPL-2.0+
/*
 * comedi/drivers/ni_usb6009.c
 * Comedi driver for National Instruments USB-6009
 *
 * COMEDI - Linux Control and Measurement Device Interface
 * Copyright (C) 2026 Sergio Hidalgo Gamborino <sergiohg.dev@gmail.com>
 */

/*
 * Driver: ni_usb6009
 * Description: National Instruments USB-6009 module
 * Devices: [National Instruments] USB-6009 (ni_usb6009)
 * Author: Sergio Hidalgo Gamborino <sergiohg.dev@gmail.com>
 * Updated: 25 Feb 2026
 * Status: in progress
 *
 *
 * Configuration Options:
 * none
 */

/*
 * Experimental Comedi driver skeleton for National Instruments USB-6009.
 *
 * This file was adapted from a ni_usb6501-based prototype and now follows
 * observed USB-6009 traffic captured from NI-DAQmx Base sessions.
 *
 * Current scope:
 * - USB transport and endpoint wiring for 0x3923:0x717b
 * - Capture-derived AI one-shot reads for ai0..ai7 single-ended
 * - Capture-derived AI differential scan/read path for ai0..ai3
 * - Capture-derived finite buffered AI command path for ai0..ai7 scans
 * - Capture-derived AO write path for ao0..ao1
 * - Capture-derived DIO port0/port1 read/write path
 * - Capture-derived counter edge-count path
 *
 * TODO:
 * - Pulse output counter programming
 * - Higher-order AI command modes beyond the captured finite scan path
 * - AO streaming/command support
 */

#include <linux/init.h>
#include <linux/comedidev.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/unaligned.h>
#include <linux/usb.h>

/*
 * The comedi.org tree uses typedef-style COMEDI types with different
 * structure tags from the in-kernel tree. Keep the driver source in the
 * familiar in-kernel "struct comedi_*" style by mapping those tags here.
 */
#define comedi_device comedi_device_struct
#define comedi_subdevice comedi_subdevice_struct
#define comedi_driver comedi_driver_struct
#define comedi_lrange comedi_lrange_struct
#define comedi_insn comedi_insn_struct
#define comedi_cmd comedi_cmd_struct
#define comedi_devconfig comedi_devconfig_struct

#define NI6009_TIMEOUT_MS 1000
#define NI6009_MIN_PKT 12
#define NI6009_AI_MAXDATA 0x3fff
#define NI6009_AI_COMMON_ZERO_CODE 12224
#define NI6009_AI_COMMON_SPAN_CODE 1964
#define NI6009_AI_SCAN_CHANLIST_LEN 8
#define NI6009_AI_SCAN_BASE_CLOCK_HZ 24000000ULL

/* Capture-observed USB-6009 command templates */
static const u8 NI6009_REQ_HELLO[] = {
    0x00, 0x00, 0x00, 0x0C, 0x00, 0x08, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
};
static const u8 NI6009_REQ_AI_0B[] = {
    0x00, 0x00, 0x00, 0x0C, 0x00, 0x08, 0x01, 0x0B, 0x02, 0x00, 0x00, 0x00,
};
static const u8 NI6009_REQ_AI_0C[] = {
    0x00, 0x00, 0x00, 0x0C, 0x00, 0x08, 0x01, 0x0C, 0x02, 0x00, 0x00, 0x00,
};
static const u8 NI6009_REQ_AI_0D[] = {
    0x00, 0x00, 0x00, 0x0C, 0x00, 0x08, 0x01, 0x0D, 0x02, 0x00, 0x00, 0x00,
};
static const u8 NI6009_REQ_AI_0C_G3[] = {
    0x00, 0x00, 0x00, 0x0C, 0x00, 0x08, 0x01, 0x0C, 0x02, 0x03, 0x00, 0x00,
};
static const u8 NI6009_REQ_AI_0F_G2_4[] = {
    0x00, 0x00, 0x00, 0x10, 0x00, 0x0C, 0x01, 0x0F,
    0x02, 0x02, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00,
};
static const u8 NI6009_REQ_AI_0E_G2_3[] = {
    0x00, 0x00, 0x00, 0x10, 0x00, 0x0C, 0x01, 0x0E,
    0x02, 0x02, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00,
};
static const u8 NI6009_REQ_AI_14_SETUP[] = {
    0x00, 0x00, 0x00, 0x14, 0x00, 0x10, 0x01, 0x14, 0x02, 0x04,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x08, 0x00, 0x00,
};
static const u8 NI6009_REQ_AI_10[] = {
    0x00, 0x00, 0x00, 0x1C, 0x00, 0x18, 0x01, 0x10, 0x02, 0x02,
    0x00, 0x00, 0x00, 0x00, 0x27, 0x10, 0xFF, 0xFF, 0xD8, 0xF0,
    0xFD, 0xFD, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00,
};
static const u8 NI6009_REQ_AI_0E_G0[] = {
    0x00, 0x00, 0x00, 0x10, 0x00, 0x0C, 0x01, 0x0E,
    0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};
static const u8 NI6009_REQ_AI_13[] = {
    0x00, 0x00, 0x00, 0x0C, 0x00, 0x08, 0x01, 0x13, 0x02, 0x00, 0x00, 0x00,
};
static const u8 NI6009_REQ_AI_15[] = {
    0x00, 0x00, 0x00, 0x0C, 0x00, 0x08, 0x01, 0x15, 0x02, 0x00, 0x00, 0x00,
};
static const u8 NI6009_REQ_AI_DIFF_0F[] = {
    0x00, 0x00, 0x00, 0x18, 0x00, 0x14, 0x01, 0x0F, 0x02, 0x02,
    0x00, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x01, 0x00, 0x02,
    0x00, 0x03, 0x00, 0x00,
};
static const u8 NI6009_REQ_AI_DIFF_0E[] = {
    0x00, 0x00, 0x00, 0x14, 0x00, 0x10, 0x01, 0x0E, 0x02, 0x02,
    0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};
static const u8 NI6009_REQ_AI_START_0F_G3[] = {
    0x00, 0x00, 0x00, 0x0C, 0x00, 0x08, 0x01, 0x0F, 0x02, 0x03, 0x00, 0x00,
};
static const u8 NI6009_REQ_AI_START_09_G3[] = {
    0x00, 0x00, 0x00, 0x0C, 0x00, 0x08, 0x01, 0x09, 0x02, 0x03, 0x00, 0x00,
};
static const u8 NI6009_REQ_AI_START_09_G0[] = {
    0x00, 0x00, 0x00, 0x0C, 0x00, 0x08, 0x01, 0x09, 0x02, 0x00, 0x00, 0x00,
};
static const u8 NI6009_REQ_AI_SCAN_0F[] = {
    0x00, 0x00, 0x00, 0x20, 0x00, 0x1C, 0x01, 0x0F, 0x02, 0x02,
    0x00, 0x00, 0x00, 0x12, 0x00, 0x00, 0x00, 0x01, 0x00, 0x02,
    0x00, 0x03, 0x00, 0x04, 0x00, 0x05, 0x00, 0x06, 0x00, 0x07,
    0x00, 0x00,
};
static const u8 NI6009_REQ_AI_SCAN_0E[] = {
    0x00, 0x00, 0x00, 0x18, 0x00, 0x14, 0x01, 0x0E, 0x02, 0x02,
    0x00, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x02, 0x02,
    0x02, 0x02, 0x00, 0x00,
};
static const u8 NI6009_REQ_AI_SCAN_0E_ARM[] = {
    0x00, 0x00, 0x00, 0x10, 0x00, 0x0C, 0x01, 0x0E,
    0x02, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
};
static const u8 NI6009_REQ_AI_SCAN_16_CLOCK[] = {
    0x00, 0x00, 0x00, 0x10, 0x00, 0x0C, 0x01, 0x16,
    0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5D, 0xC0,
};
static const u8 NI6009_REQ_AI_SCAN_0F_COUNT[] = {
    0x00, 0x00, 0x00, 0x10, 0x00, 0x0C, 0x01, 0x0F,
    0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xE8,
};
static const u8 NI6009_REQ_AI_SCAN_12_BYTES[] = {
    0x00, 0x00, 0x00, 0x10, 0x00, 0x0C, 0x01, 0x12,
    0x02, 0x03, 0x00, 0x00, 0x00, 0x00, 0x3E, 0x80,
};
static const u8 NI6009_REQ_AI_START_0A_G0[] = {
    0x00, 0x00, 0x00, 0x0C, 0x00, 0x08, 0x01, 0x0A, 0x02, 0x00, 0x00, 0x00,
};
static const u8 NI6009_REQ_AI_START_10_G3[] = {
    0x00, 0x00, 0x00, 0x0C, 0x00, 0x08, 0x01, 0x10, 0x02, 0x03, 0x00, 0x00,
};

static const u8 NI6009_REQ_AO_START_14_A[] = {
    0x00, 0x00, 0x00, 0x14, 0x00, 0x10, 0x01, 0x14, 0x02, 0x04,
    0x00, 0x00, 0x00, 0x00, 0x01, 0x54, 0x00, 0x04, 0x00, 0x00,
};
static const u8 NI6009_REQ_AO_START_14_B[] = {
    0x00, 0x00, 0x00, 0x14, 0x00, 0x10, 0x01, 0x14, 0x02, 0x04,
    0x00, 0x00, 0x00, 0x00, 0x01, 0x50, 0x00, 0x04, 0x00, 0x00,
};
static const u8 NI6009_REQ_AO_WRITE[] = {
    0x00, 0x00, 0x00, 0x10, 0x00, 0x0C, 0x01, 0x0E, 0x02, 0x21,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static const u8 NI6009_REQ_DIO_SET_DIR[] = {
    0x00, 0x00, 0x00, 0x14, 0x00, 0x10, 0x01, 0x12, 0x02, 0x10,
    0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00,
};
static const u8 NI6009_REQ_DIO_READ_PORT[] = {
    0x00, 0x00, 0x00, 0x10, 0x00, 0x0C, 0x01, 0x0E, 0x02, 0x10,
    0x00, 0x00, 0x00, 0x03, 0x00, 0x00,
};
static const u8 NI6009_REQ_DIO_WRITE_PORT[] = {
    0x00, 0x00, 0x00, 0x14, 0x00, 0x10, 0x01, 0x0F, 0x02, 0x10,
    0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00,
};

static const u8 NI6009_REQ_CTR_START_0F[] = {
    0x00, 0x00, 0x00, 0x10, 0x00, 0x0C, 0x01, 0x0F, 0x02, 0x20,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};
static const u8 NI6009_REQ_CTR_START_09[] = {
    0x00, 0x00, 0x00, 0x0C, 0x00, 0x08, 0x01, 0x09, 0x02, 0x20,
    0x00, 0x00,
};
static const u8 NI6009_REQ_CTR_STOP[] = {
    0x00, 0x00, 0x00, 0x0C, 0x00, 0x08, 0x01, 0x0C, 0x02, 0x20,
    0x00, 0x00,
};
static const u8 NI6009_REQ_CTR_READ[] = {
    0x00, 0x00, 0x00, 0x0C, 0x00, 0x08, 0x01, 0x0E, 0x02, 0x20,
    0x00, 0x00,
};

static const u8 NI6009_REQ_AI_READ_ARM[] = {
    0x00, 0x00, 0x00, 0x10, 0x00, 0x0C, 0x01, 0x0F,
    0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
};
static const u8 NI6009_REQ_AI_READ_TRIGGER[] = {
    0x00, 0x00, 0x00, 0x10, 0x00, 0x0C, 0x01, 0x14,
    0x02, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
};
static const u8 NI6009_RSP_GENERIC[] = {
    0x00, 0x00, 0x00, 0x0C, 0x00, 0x08, 0x01, 0x00, 0x00, 0x00, 0x00, 0x02,
};
static const u8 NI6009_RSP_HEADER_HELLO[] = {
    0x00, 0x00, 0x00, 0x1C, 0x00, 0x18, 0x01, 0x00, 0x00, 0x00, 0x00, 0x02,
};
static const u8 NI6009_RSP_HEADER_AI_14_SETUP[] = {
    0x00, 0x00, 0x00, 0x18, 0x00, 0x14, 0x01, 0x00, 0x00, 0x00, 0x00, 0x02,
};
static const u8 NI6009_RSP_AO_START_PREFIX[] = {
    0x00, 0x00, 0x00, 0x14, 0x00, 0x10, 0x01, 0x00, 0x00, 0x00, 0x00, 0x02,
    0x00, 0x06,
};
static const u8 NI6009_RSP_DIO_READ_PORT_PREFIX[] = {
    0x00, 0x00, 0x00, 0x10, 0x00, 0x0C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x02,
    0x00, 0x03,
};
static const u8 NI6009_RSP_CTR_READ_PREFIX[] = {
    0x00, 0x00, 0x00, 0x10, 0x00, 0x0C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x02,
};

static const struct comedi_lrange ni6009_ai_ranges = {
    3, {
           BIP_RANGE(10),
           BIP_RANGE(5),
           UNI_RANGE(5),
       }
};

struct ni6009_private {
  struct usb_endpoint_descriptor *ep_cmd_out; /* 0x01 (interrupt out) */
  struct usb_endpoint_descriptor *ep_cmd_in;  /* 0x81 (interrupt in)  */
  struct usb_endpoint_descriptor *ep_data_in; /* 0x82 (interrupt in)  */

  struct mutex mut;

  u8 *cmd_tx_buf;
  u8 *cmd_rx_buf;
  u8 *data_rx_buf;

  bool ai_session_ready;
  bool ai_task_started;
  bool ao_session_ready;
  bool ao_task_started;
  lsampl_t ao_readback[2];
  bool dio_session_ready;
  bool ctr_session_ready;
  bool ctr_task_started;
};

static u16 ni6009_ai_decode_sample(const u8 *buf);

static int ni6009_check_trigger_src(unsigned int *src, unsigned int flags) {
  unsigned int orig = *src;

  *src &= flags;
  if (!*src)
    *src = flags & -flags;

  return orig != *src;
}

static int ni6009_check_trigger_is_unique(unsigned int src) {
  return (src && !(src & (src - 1))) ? 0 : 1;
}

static int ni6009_check_trigger_arg_is(unsigned int *arg, unsigned int val) {
  if (*arg != val) {
    *arg = val;
    return 1;
  }
  return 0;
}

static int ni6009_check_trigger_arg_min(unsigned int *arg, unsigned int val) {
  if (*arg < val) {
    *arg = val;
    return 1;
  }
  return 0;
}

static int ni6009_dio_insn_config_local(struct comedi_subdevice *s,
                                        struct comedi_insn *insn,
                                        unsigned int *data) {
  unsigned int chan = CR_CHAN(insn->chanspec);
  unsigned int mask = 1U << chan;

  switch (data[0]) {
  case INSN_CONFIG_DIO_OUTPUT:
    s->io_bits |= mask;
    break;
  case INSN_CONFIG_DIO_INPUT:
    s->io_bits &= ~mask;
    break;
  case INSN_CONFIG_DIO_QUERY:
    data[1] = (s->io_bits & mask) ? COMEDI_OUTPUT : COMEDI_INPUT;
    return insn->n;
  default:
    return -EINVAL;
  }

  return insn->n;
}

static unsigned int ni6009_dio_update_state_local(struct comedi_subdevice *s,
                                                  unsigned int *data) {
  unsigned int mask = data[0];

  if (mask) {
    s->state &= ~mask;
    s->state |= mask & data[1];
  }

  return mask;
}

static int ni6009_xfer_cmd(struct comedi_device *dev, const u8 *req,
                           int req_len, const u8 *expected_rsp,
                           int expected_rsp_len) {
  struct usb_device *usb = comedi_to_usb_dev(dev);
  struct ni6009_private *devpriv = dev->private;
  int actual = 0;
  int ret;

  if (req_len <= 0 || req_len > usb_endpoint_maxp(devpriv->ep_cmd_out))
    return -EINVAL;

  memcpy(devpriv->cmd_tx_buf, req, req_len);

  ret = usb_bulk_msg(
      usb, usb_sndbulkpipe(usb, devpriv->ep_cmd_out->bEndpointAddress),
      devpriv->cmd_tx_buf, req_len, &actual, NI6009_TIMEOUT_MS);
  if (ret)
    return ret;
  if (actual != req_len)
    return -EIO;

  ret = usb_bulk_msg(
      usb, usb_rcvbulkpipe(usb, devpriv->ep_cmd_in->bEndpointAddress),
      devpriv->cmd_rx_buf, usb_endpoint_maxp(devpriv->ep_cmd_in), &actual,
      NI6009_TIMEOUT_MS);
  if (ret)
    return ret;

  if (!expected_rsp || expected_rsp_len <= 0)
    return -EINVAL;

  if (actual != expected_rsp_len)
    return -EIO;

  if (memcmp(devpriv->cmd_rx_buf, expected_rsp, expected_rsp_len))
    return -EIO;

  return 0;
}

static int ni6009_xfer_cmd_prefix(struct comedi_device *dev, const u8 *req,
                                  int req_len, int expected_rsp_len,
                                  const u8 *expected_prefix,
                                  int expected_prefix_len) {
  struct usb_device *usb = comedi_to_usb_dev(dev);
  struct ni6009_private *devpriv = dev->private;
  int actual = 0;
  int ret;

  if (req_len <= 0 || req_len > usb_endpoint_maxp(devpriv->ep_cmd_out))
    return -EINVAL;
  if (!expected_prefix || expected_prefix_len <= 0)
    return -EINVAL;

  memcpy(devpriv->cmd_tx_buf, req, req_len);

  ret = usb_bulk_msg(
      usb, usb_sndbulkpipe(usb, devpriv->ep_cmd_out->bEndpointAddress),
      devpriv->cmd_tx_buf, req_len, &actual, NI6009_TIMEOUT_MS);
  if (ret)
    return ret;
  if (actual != req_len)
    return -EIO;

  ret = usb_bulk_msg(
      usb, usb_rcvbulkpipe(usb, devpriv->ep_cmd_in->bEndpointAddress),
      devpriv->cmd_rx_buf, usb_endpoint_maxp(devpriv->ep_cmd_in), &actual,
      NI6009_TIMEOUT_MS);
  if (ret)
    return ret;
  if (actual != expected_rsp_len)
    return -EIO;
  if (memcmp(devpriv->cmd_rx_buf, expected_prefix, expected_prefix_len))
    return -EIO;

  return 0;
}

static int ni6009_ai_session_init(struct comedi_device *dev) {
  int ret;

  /* Sequence observed in captures before first AI read. */
  ret = ni6009_xfer_cmd_prefix(dev, NI6009_REQ_HELLO,
                               sizeof(NI6009_REQ_HELLO), 0x1c,
                               NI6009_RSP_HEADER_HELLO,
                               sizeof(NI6009_RSP_HEADER_HELLO));
  if (ret)
    return ret;

  ret = ni6009_xfer_cmd(dev, NI6009_REQ_AI_0B, sizeof(NI6009_REQ_AI_0B),
                        NI6009_RSP_GENERIC, sizeof(NI6009_RSP_GENERIC));
  if (ret)
    return ret;

  ret = ni6009_xfer_cmd(dev, NI6009_REQ_AI_0C, sizeof(NI6009_REQ_AI_0C),
                        NI6009_RSP_GENERIC, sizeof(NI6009_RSP_GENERIC));
  if (ret)
    return ret;

  ret = ni6009_xfer_cmd(dev, NI6009_REQ_AI_0D, sizeof(NI6009_REQ_AI_0D),
                        NI6009_RSP_GENERIC, sizeof(NI6009_RSP_GENERIC));
  if (ret)
    return ret;

  return 0;
}

static void ni6009_ai_apply_scale(u8 *req_10, unsigned int range)
{
  if (range == 2) {
    req_10[14] = 0x13;
    req_10[15] = 0x88;
    req_10[18] = 0xEC;
    req_10[19] = 0x78;
  }
}

static int ni6009_ai_single_ended_setup_code(unsigned int chan,
                                             unsigned int range,
                                             unsigned int aref,
                                             u16 *setup_code)
{
  if (chan > 7 || !setup_code)
    return -EINVAL;

  if (aref == AREF_COMMON) {
    if (chan != 0 || range != 2)
      return -EINVAL;
    /*
     * The explicit RSE capture used a 0..5 V userspace request, but the
     * device still programmed the default scale words.
     */
    *setup_code = 0x0110;
  } else if (range == 2) {
    if (chan != 0)
      return -EINVAL;
    *setup_code = 0x0050;
  } else if (chan < 4) {
    *setup_code = 0x0030 + (chan * 0x08);
  } else {
    *setup_code = 0x0118 + ((chan - 4) * 0x08);
  }

  return 0;
}

static int ni6009_ai_program_single(struct comedi_device *dev,
                                    unsigned int chan,
                                    unsigned int range,
                                    unsigned int aref) {
  u8 req_0f[sizeof(NI6009_REQ_AI_0F_G2_4)];
  u8 req_0e[sizeof(NI6009_REQ_AI_0E_G2_3)];
  u8 req_14[sizeof(NI6009_REQ_AI_14_SETUP)];
  u8 req_10[sizeof(NI6009_REQ_AI_10)];
  u16 setup_code;
  int ret;

  if (chan > 7)
    return -EINVAL;

  ret = ni6009_ai_single_ended_setup_code(chan, range, aref, &setup_code);
  if (ret)
    return ret;

  ret = ni6009_xfer_cmd(dev, NI6009_REQ_AI_0C_G3, sizeof(NI6009_REQ_AI_0C_G3),
                        NI6009_RSP_GENERIC, sizeof(NI6009_RSP_GENERIC));
  if (ret)
    return ret;

  memcpy(req_0f, NI6009_REQ_AI_0F_G2_4, sizeof(req_0f));
  req_0f[15] = chan & 0xff;
  ret = ni6009_xfer_cmd(dev, req_0f, sizeof(req_0f), NI6009_RSP_GENERIC,
                        sizeof(NI6009_RSP_GENERIC));
  if (ret)
    return ret;

  memcpy(req_0e, NI6009_REQ_AI_0E_G2_3, sizeof(req_0e));
  if (setup_code >= 0x0100)
    req_0e[14] = 0x02;
  ret = ni6009_xfer_cmd(dev, req_0e, sizeof(req_0e), NI6009_RSP_GENERIC,
                        sizeof(NI6009_RSP_GENERIC));
  if (ret)
    return ret;

  memcpy(req_14, NI6009_REQ_AI_14_SETUP, sizeof(req_14));
  put_unaligned_be16(setup_code, &req_14[14]);
  ret = ni6009_xfer_cmd_prefix(dev, req_14, sizeof(req_14), 0x18,
                               NI6009_RSP_HEADER_AI_14_SETUP,
                               sizeof(NI6009_RSP_HEADER_AI_14_SETUP));
  if (ret)
    return ret;

  memcpy(req_10, NI6009_REQ_AI_10, sizeof(req_10));
  if (aref != AREF_COMMON)
    ni6009_ai_apply_scale(req_10, range);
  ret = ni6009_xfer_cmd(dev, req_10, sizeof(req_10), NI6009_RSP_GENERIC,
                        sizeof(NI6009_RSP_GENERIC));
  if (ret)
    return ret;

  ret = ni6009_xfer_cmd(dev, NI6009_REQ_AI_0E_G0, sizeof(NI6009_REQ_AI_0E_G0),
                        NI6009_RSP_GENERIC, sizeof(NI6009_RSP_GENERIC));
  if (ret)
    return ret;

  ret = ni6009_xfer_cmd(dev, NI6009_REQ_AI_13, sizeof(NI6009_REQ_AI_13),
                        NI6009_RSP_GENERIC, sizeof(NI6009_RSP_GENERIC));
  if (ret)
    return ret;

  return ni6009_xfer_cmd(dev, NI6009_REQ_AI_15, sizeof(NI6009_REQ_AI_15),
                         NI6009_RSP_GENERIC, sizeof(NI6009_RSP_GENERIC));
}

static int ni6009_ai_program_diff(struct comedi_device *dev) {
  u8 req_14[sizeof(NI6009_REQ_AI_14_SETUP)];
  u8 req_10[sizeof(NI6009_REQ_AI_10)];
  unsigned int chan;
  int ret;

  ret = ni6009_xfer_cmd(dev, NI6009_REQ_AI_0C_G3, sizeof(NI6009_REQ_AI_0C_G3),
                        NI6009_RSP_GENERIC, sizeof(NI6009_RSP_GENERIC));
  if (ret)
    return ret;

  ret = ni6009_xfer_cmd(dev, NI6009_REQ_AI_DIFF_0F,
                        sizeof(NI6009_REQ_AI_DIFF_0F), NI6009_RSP_GENERIC,
                        sizeof(NI6009_RSP_GENERIC));
  if (ret)
    return ret;

  ret = ni6009_xfer_cmd(dev, NI6009_REQ_AI_DIFF_0E,
                        sizeof(NI6009_REQ_AI_DIFF_0E), NI6009_RSP_GENERIC,
                        sizeof(NI6009_RSP_GENERIC));
  if (ret)
    return ret;

  for (chan = 0; chan < 4; chan++) {
    memcpy(req_14, NI6009_REQ_AI_14_SETUP, sizeof(req_14));
    req_14[15] = 0x50 + (chan * 0x08);
    ret = ni6009_xfer_cmd_prefix(dev, req_14, sizeof(req_14), 0x18,
                                 NI6009_RSP_HEADER_AI_14_SETUP,
                                 sizeof(NI6009_RSP_HEADER_AI_14_SETUP));
    if (ret)
      return ret;

    memcpy(req_10, NI6009_REQ_AI_10, sizeof(req_10));
    ni6009_ai_apply_scale(req_10, 2);
    req_10[26] = chan & 0xff;
    ret = ni6009_xfer_cmd(dev, req_10, sizeof(req_10), NI6009_RSP_GENERIC,
                          sizeof(NI6009_RSP_GENERIC));
    if (ret)
      return ret;
  }

  ret = ni6009_xfer_cmd(dev, NI6009_REQ_AI_0E_G0, sizeof(NI6009_REQ_AI_0E_G0),
                        NI6009_RSP_GENERIC, sizeof(NI6009_RSP_GENERIC));
  if (ret)
    return ret;

  ret = ni6009_xfer_cmd(dev, NI6009_REQ_AI_13, sizeof(NI6009_REQ_AI_13),
                        NI6009_RSP_GENERIC, sizeof(NI6009_RSP_GENERIC));
  if (ret)
    return ret;

  return ni6009_xfer_cmd(dev, NI6009_REQ_AI_15, sizeof(NI6009_REQ_AI_15),
                         NI6009_RSP_GENERIC, sizeof(NI6009_RSP_GENERIC));
}

static int ni6009_ao_session_init(struct comedi_device *dev) {
  struct ni6009_private *devpriv = dev->private;
  int ret;

  if (devpriv->ao_session_ready)
    return 0;

  ret = ni6009_xfer_cmd_prefix(dev, NI6009_REQ_HELLO,
                               sizeof(NI6009_REQ_HELLO), 0x1c,
                               NI6009_RSP_HEADER_HELLO,
                               sizeof(NI6009_RSP_HEADER_HELLO));
  if (ret)
    return ret;

  devpriv->ao_session_ready = true;
  return 0;
}

static int ni6009_ao_start_task(struct comedi_device *dev, unsigned int chan) {
  u8 req_a[sizeof(NI6009_REQ_AO_START_14_A)];
  u8 req_b[sizeof(NI6009_REQ_AO_START_14_B)];
  int ret;

  if (chan > 1)
    return -EINVAL;

  memcpy(req_a, NI6009_REQ_AO_START_14_A, sizeof(req_a));
  memcpy(req_b, NI6009_REQ_AO_START_14_B, sizeof(req_b));
  req_a[15] += chan * 0x08;
  req_b[15] += chan * 0x08;

  ret = ni6009_xfer_cmd_prefix(dev, req_a, sizeof(req_a), 0x14,
                               NI6009_RSP_AO_START_PREFIX,
                               sizeof(NI6009_RSP_AO_START_PREFIX));
  if (ret)
    return ret;

  ret = ni6009_xfer_cmd_prefix(dev, req_b, sizeof(req_b), 0x14,
                               NI6009_RSP_AO_START_PREFIX,
                               sizeof(NI6009_RSP_AO_START_PREFIX));
  return ret;
}

static int ni6009_ao_write_sample(struct comedi_device *dev, unsigned int chan,
                                  unsigned int data) {
  struct ni6009_private *devpriv = dev->private;
  struct usb_device *usb = comedi_to_usb_dev(dev);
  int actual = 0;
  int ret;

  if (chan > 1)
    return -EINVAL;

  memcpy(devpriv->cmd_tx_buf, NI6009_REQ_AO_WRITE, sizeof(NI6009_REQ_AO_WRITE));
  devpriv->cmd_tx_buf[11] = chan & 0xff;
  devpriv->cmd_tx_buf[12] = (data >> 8) & 0xff;
  devpriv->cmd_tx_buf[13] = data & 0xff;

  ret = usb_bulk_msg(
      usb, usb_sndbulkpipe(usb, devpriv->ep_cmd_out->bEndpointAddress),
      devpriv->cmd_tx_buf, sizeof(NI6009_REQ_AO_WRITE), &actual,
      NI6009_TIMEOUT_MS);
  if (ret)
    return ret;
  if (actual != sizeof(NI6009_REQ_AO_WRITE))
    return -EIO;

  ret = usb_bulk_msg(
      usb, usb_rcvbulkpipe(usb, devpriv->ep_cmd_in->bEndpointAddress),
      devpriv->cmd_rx_buf, usb_endpoint_maxp(devpriv->ep_cmd_in), &actual,
      NI6009_TIMEOUT_MS);
  if (ret)
    return ret;
  if (actual != sizeof(NI6009_RSP_GENERIC))
    return -EIO;
  if (memcmp(devpriv->cmd_rx_buf, NI6009_RSP_GENERIC,
             sizeof(NI6009_RSP_GENERIC)))
    return -EIO;

  return 0;
}

static int ni6009_ao_insn_write(struct comedi_device *dev,
                                struct comedi_subdevice *s,
                                struct comedi_insn *insn, unsigned int *data) {
  struct ni6009_private *devpriv = dev->private;
  unsigned int chan = CR_CHAN(insn->chanspec);
  unsigned int i;
  int ret;

  if (chan > 1)
    return -EINVAL;

  mutex_lock(&devpriv->mut);

  ret = ni6009_ao_session_init(dev);
  if (ret)
    goto out;

  ret = ni6009_ao_start_task(dev, chan);
  if (ret)
    goto out;

  for (i = 0; i < insn->n; i++) {
    ret = ni6009_ao_write_sample(dev, chan, data[i] & 0x0fff);
    if (ret)
      goto out;
    devpriv->ao_readback[chan] = data[i] & 0x0fff;
  }

  ret = insn->n;
out:
  mutex_unlock(&devpriv->mut);
  return ret;
}

static int ni6009_ao_insn_read(struct comedi_device *dev,
                               struct comedi_subdevice *s,
                               struct comedi_insn *insn, unsigned int *data) {
  struct ni6009_private *devpriv = dev->private;
  unsigned int chan = CR_CHAN(insn->chanspec);
  unsigned int i;

  if (chan > 1)
    return -EINVAL;

  for (i = 0; i < insn->n; i++)
    data[i] = devpriv->ao_readback[chan];

  return insn->n;
}

static int ni6009_dio_session_init(struct comedi_device *dev) {
  struct ni6009_private *devpriv = dev->private;
  int ret;

  if (devpriv->dio_session_ready)
    return 0;

  ret = ni6009_xfer_cmd_prefix(dev, NI6009_REQ_HELLO,
                               sizeof(NI6009_REQ_HELLO), 0x1c,
                               NI6009_RSP_HEADER_HELLO,
                               sizeof(NI6009_RSP_HEADER_HELLO));
  if (ret)
    return ret;

  devpriv->dio_session_ready = true;
  return 0;
}

static int ni6009_dio_set_port_dir(struct comedi_device *dev,
                                   unsigned int io_bits) {
  struct ni6009_private *devpriv = dev->private;
  struct usb_device *usb = comedi_to_usb_dev(dev);
  int actual = 0;
  int ret;

  memcpy(devpriv->cmd_tx_buf, NI6009_REQ_DIO_SET_DIR,
         sizeof(NI6009_REQ_DIO_SET_DIR));
  devpriv->cmd_tx_buf[14] = io_bits & 0xff;
  devpriv->cmd_tx_buf[15] = (io_bits >> 8) & 0x1f;

  ret = usb_bulk_msg(
      usb, usb_sndbulkpipe(usb, devpriv->ep_cmd_out->bEndpointAddress),
      devpriv->cmd_tx_buf, sizeof(NI6009_REQ_DIO_SET_DIR), &actual,
      NI6009_TIMEOUT_MS);
  if (ret)
    return ret;
  if (actual != sizeof(NI6009_REQ_DIO_SET_DIR))
    return -EIO;

  ret = usb_bulk_msg(
      usb, usb_rcvbulkpipe(usb, devpriv->ep_cmd_in->bEndpointAddress),
      devpriv->cmd_rx_buf, usb_endpoint_maxp(devpriv->ep_cmd_in), &actual,
      NI6009_TIMEOUT_MS);
  if (ret)
    return ret;
  if (actual != sizeof(NI6009_RSP_GENERIC))
    return -EIO;
  if (memcmp(devpriv->cmd_rx_buf, NI6009_RSP_GENERIC,
             sizeof(NI6009_RSP_GENERIC)))
    return -EIO;

  return 0;
}

static int ni6009_dio_read_port(struct comedi_device *dev, unsigned int port,
                                u8 *bitmap) {
  struct ni6009_private *devpriv = dev->private;
  u8 req[sizeof(NI6009_REQ_DIO_READ_PORT)];
  int ret;

  if (port > 1)
    return -EINVAL;

  memcpy(req, NI6009_REQ_DIO_READ_PORT, sizeof(req));
  req[14] = port & 0xff;

  ret = ni6009_xfer_cmd_prefix(dev, req, sizeof(req), 0x10,
                               NI6009_RSP_DIO_READ_PORT_PREFIX,
                               sizeof(NI6009_RSP_DIO_READ_PORT_PREFIX));
  if (ret)
    return ret;

  *bitmap = devpriv->cmd_rx_buf[14];
  return 0;
}

static int ni6009_dio_write_port(struct comedi_device *dev, unsigned int port,
                                 u8 bitmap) {
  struct ni6009_private *devpriv = dev->private;
  struct usb_device *usb = comedi_to_usb_dev(dev);
  int actual = 0;
  int ret;

  if (port > 1)
    return -EINVAL;

  memcpy(devpriv->cmd_tx_buf, NI6009_REQ_DIO_WRITE_PORT,
         sizeof(NI6009_REQ_DIO_WRITE_PORT));
  devpriv->cmd_tx_buf[14] = port & 0xff;
  devpriv->cmd_tx_buf[17] = bitmap;

  ret = usb_bulk_msg(
      usb, usb_sndbulkpipe(usb, devpriv->ep_cmd_out->bEndpointAddress),
      devpriv->cmd_tx_buf, sizeof(NI6009_REQ_DIO_WRITE_PORT), &actual,
      NI6009_TIMEOUT_MS);
  if (ret)
    return ret;
  if (actual != sizeof(NI6009_REQ_DIO_WRITE_PORT))
    return -EIO;

  ret = usb_bulk_msg(
      usb, usb_rcvbulkpipe(usb, devpriv->ep_cmd_in->bEndpointAddress),
      devpriv->cmd_rx_buf, usb_endpoint_maxp(devpriv->ep_cmd_in), &actual,
      NI6009_TIMEOUT_MS);
  if (ret)
    return ret;
  if (actual != sizeof(NI6009_RSP_GENERIC))
    return -EIO;
  if (memcmp(devpriv->cmd_rx_buf, NI6009_RSP_GENERIC,
             sizeof(NI6009_RSP_GENERIC)))
    return -EIO;

  return 0;
}

static int ni6009_dio_insn_config(struct comedi_device *dev,
                                  struct comedi_subdevice *s,
                                  struct comedi_insn *insn,
                                  unsigned int *data) {
  struct ni6009_private *devpriv = dev->private;
  int ret;

  mutex_lock(&devpriv->mut);

  ret = ni6009_dio_session_init(dev);
  if (ret)
    goto out;

  ret = ni6009_dio_insn_config_local(s, insn, data);
  if (ret)
    goto out;

  ret = ni6009_dio_set_port_dir(dev, s->io_bits);
  if (ret)
    goto out;

  ret = insn->n;
out:
  mutex_unlock(&devpriv->mut);
  return ret;
}

static int ni6009_dio_insn_bits(struct comedi_device *dev,
                                struct comedi_subdevice *s,
                                struct comedi_insn *insn, unsigned int *data) {
  struct ni6009_private *devpriv = dev->private;
  unsigned int mask;
  unsigned int port;
  int ret;

  mutex_lock(&devpriv->mut);

  ret = ni6009_dio_session_init(dev);
  if (ret)
    goto out;

  mask = ni6009_dio_update_state_local(s, data);

  for (port = 0; port < 2; port++) {
    unsigned int port_mask = (port == 0) ? 0x00ff : 0x1f00;
    u8 bitmap;

    if ((mask & s->io_bits & port_mask) == 0)
      continue;

    bitmap = (s->state >> (port * 8)) & 0xff;
    if (port == 1)
      bitmap &= 0x1f;

    ret = ni6009_dio_write_port(dev, port, bitmap);
    if (ret)
      goto out;
  }

  data[1] = 0;
  for (port = 0; port < 2; port++) {
    u8 bitmap;

    ret = ni6009_dio_read_port(dev, port, &bitmap);
    if (ret)
      goto out;
    if (port == 1)
      bitmap &= 0x1f;
    data[1] |= bitmap << (port * 8);
  }

  ret = insn->n;
out:
  mutex_unlock(&devpriv->mut);
  return ret;
}

static int ni6009_ctr_session_init(struct comedi_device *dev) {
  struct ni6009_private *devpriv = dev->private;
  int ret;

  if (devpriv->ctr_session_ready)
    return 0;

  ret = ni6009_xfer_cmd_prefix(dev, NI6009_REQ_HELLO,
                               sizeof(NI6009_REQ_HELLO), 0x1c,
                               NI6009_RSP_HEADER_HELLO,
                               sizeof(NI6009_RSP_HEADER_HELLO));
  if (ret)
    return ret;

  devpriv->ctr_session_ready = true;
  return 0;
}

static int ni6009_ctr_start(struct comedi_device *dev) {
  struct ni6009_private *devpriv = dev->private;
  int ret;

  if (devpriv->ctr_task_started)
    return 0;

  ret = ni6009_xfer_cmd(dev, NI6009_REQ_CTR_START_0F,
                        sizeof(NI6009_REQ_CTR_START_0F), NI6009_RSP_GENERIC,
                        sizeof(NI6009_RSP_GENERIC));
  if (ret)
    return ret;

  ret = ni6009_xfer_cmd(dev, NI6009_REQ_CTR_START_09,
                        sizeof(NI6009_REQ_CTR_START_09), NI6009_RSP_GENERIC,
                        sizeof(NI6009_RSP_GENERIC));
  if (ret)
    return ret;

  devpriv->ctr_task_started = true;
  return 0;
}

static int ni6009_ctr_stop(struct comedi_device *dev) {
  struct ni6009_private *devpriv = dev->private;
  int ret;

  ret = ni6009_xfer_cmd(dev, NI6009_REQ_CTR_STOP, sizeof(NI6009_REQ_CTR_STOP),
                        NI6009_RSP_GENERIC, sizeof(NI6009_RSP_GENERIC));
  if (!ret)
    devpriv->ctr_task_started = false;
  return ret;
}

static int ni6009_ctr_read_count(struct comedi_device *dev, u32 *val) {
  struct ni6009_private *devpriv = dev->private;
  int ret;

  ret = ni6009_xfer_cmd_prefix(dev, NI6009_REQ_CTR_READ,
                               sizeof(NI6009_REQ_CTR_READ), 0x10,
                               NI6009_RSP_CTR_READ_PREFIX,
                               sizeof(NI6009_RSP_CTR_READ_PREFIX));
  if (ret)
    return ret;

  *val = get_unaligned_be32(&devpriv->cmd_rx_buf[12]);
  return 0;
}

static int ni6009_ctr_insn_config(struct comedi_device *dev,
                                  struct comedi_subdevice *s,
                                  struct comedi_insn *insn,
                                  unsigned int *data) {
  struct ni6009_private *devpriv = dev->private;
  int ret;

  mutex_lock(&devpriv->mut);

  ret = ni6009_ctr_session_init(dev);
  if (ret)
    goto out;

  switch (data[0]) {
  case INSN_CONFIG_ARM:
    ret = ni6009_ctr_start(dev);
    break;
  case INSN_CONFIG_DISARM:
    ret = ni6009_ctr_stop(dev);
    break;
  case INSN_CONFIG_RESET:
    ret = ni6009_ctr_stop(dev);
    break;
  default:
    ret = -EINVAL;
    break;
  }

  if (!ret)
    ret = insn->n;
out:
  mutex_unlock(&devpriv->mut);
  return ret;
}

static int ni6009_ctr_insn_read(struct comedi_device *dev,
                                struct comedi_subdevice *s,
                                struct comedi_insn *insn, unsigned int *data) {
  struct ni6009_private *devpriv = dev->private;
  unsigned int i;
  int ret;
  u32 val;

  mutex_lock(&devpriv->mut);

  ret = ni6009_ctr_session_init(dev);
  if (ret)
    goto out;

  if (!devpriv->ctr_task_started) {
    ret = ni6009_ctr_start(dev);
    if (ret)
      goto out;
  }

  for (i = 0; i < insn->n; i++) {
    ret = ni6009_ctr_read_count(dev, &val);
    if (ret)
      goto out;
    data[i] = val;
  }

  ret = insn->n;
out:
  mutex_unlock(&devpriv->mut);
  return ret;
}

static int ni6009_ai_start_task(struct comedi_device *dev) {
  int ret;

  ret = ni6009_xfer_cmd(dev, NI6009_REQ_AI_START_0F_G3,
                        sizeof(NI6009_REQ_AI_START_0F_G3),
                        NI6009_RSP_GENERIC, sizeof(NI6009_RSP_GENERIC));
  if (ret)
    return ret;

  ret = ni6009_xfer_cmd(dev, NI6009_REQ_AI_START_09_G3,
                        sizeof(NI6009_REQ_AI_START_09_G3),
                        NI6009_RSP_GENERIC, sizeof(NI6009_RSP_GENERIC));
  if (ret)
    return ret;

  ret = ni6009_xfer_cmd(dev, NI6009_REQ_AI_START_09_G0,
                        sizeof(NI6009_REQ_AI_START_09_G0),
                        NI6009_RSP_GENERIC, sizeof(NI6009_RSP_GENERIC));
  return ret;
}

static unsigned int ni6009_ai_scan_divisor(unsigned int scan_begin_ns)
{
  u64 divisor;

  divisor = DIV_ROUND_CLOSEST_ULL(NI6009_AI_SCAN_BASE_CLOCK_HZ *
                                      (u64)scan_begin_ns,
                                  NSEC_PER_SEC);
  if (!divisor)
    divisor = 1;
  if (divisor > U32_MAX)
    divisor = U32_MAX;
  return divisor;
}

static unsigned int ni6009_ai_scan_period_ns(unsigned int divisor)
{
  return DIV_ROUND_CLOSEST_ULL((u64)NSEC_PER_SEC * divisor,
                               NI6009_AI_SCAN_BASE_CLOCK_HZ);
}

static int ni6009_ai_scan_chanlist_valid(struct comedi_cmd *cmd)
{
  unsigned int i;

  if (cmd->chanlist_len != NI6009_AI_SCAN_CHANLIST_LEN)
    return -EINVAL;

  for (i = 0; i < cmd->chanlist_len; i++) {
    unsigned int chanspec = cmd->chanlist[i];

    if (CR_CHAN(chanspec) != i || CR_RANGE(chanspec) != 0 ||
        CR_AREF(chanspec) != AREF_GROUND)
      return -EINVAL;
  }

  return 0;
}

static int ni6009_ai_cmdtest(struct comedi_device *dev,
                             struct comedi_subdevice *s,
                             struct comedi_cmd *cmd)
{
  unsigned int divisor;
  int err = 0;

  err |= ni6009_check_trigger_src(&cmd->start_src, TRIG_NOW);
  err |= ni6009_check_trigger_src(&cmd->scan_begin_src, TRIG_TIMER);
  err |= ni6009_check_trigger_src(&cmd->convert_src, TRIG_NOW);
  err |= ni6009_check_trigger_src(&cmd->scan_end_src, TRIG_COUNT);
  err |= ni6009_check_trigger_src(&cmd->stop_src, TRIG_COUNT);
  if (err)
    return 1;

  err |= ni6009_check_trigger_is_unique(cmd->start_src);
  err |= ni6009_check_trigger_is_unique(cmd->scan_begin_src);
  err |= ni6009_check_trigger_is_unique(cmd->convert_src);
  err |= ni6009_check_trigger_is_unique(cmd->scan_end_src);
  err |= ni6009_check_trigger_is_unique(cmd->stop_src);
  if (err)
    return 2;

  err |= ni6009_check_trigger_arg_is(&cmd->start_arg, 0);
  err |= ni6009_check_trigger_arg_min(&cmd->scan_begin_arg, 1);
  err |= ni6009_check_trigger_arg_is(&cmd->convert_arg, 0);
  err |= ni6009_check_trigger_arg_is(&cmd->scan_end_arg, cmd->chanlist_len);
  err |= ni6009_check_trigger_arg_min(&cmd->stop_arg, 1);
  if (err)
    return 3;

  divisor = ni6009_ai_scan_divisor(cmd->scan_begin_arg);
  err |= ni6009_check_trigger_arg_is(&cmd->scan_begin_arg,
                                     ni6009_ai_scan_period_ns(divisor));
  if (err)
    return 4;

  if (ni6009_ai_scan_chanlist_valid(cmd))
    return 5;

  return 0;
}

static int ni6009_ai_program_scan(struct comedi_device *dev,
                                  struct comedi_cmd *cmd)
{
  u8 req_14[sizeof(NI6009_REQ_AI_14_SETUP)];
  u8 req_10[sizeof(NI6009_REQ_AI_10)];
  u8 req_16[sizeof(NI6009_REQ_AI_SCAN_16_CLOCK)];
  u8 req_0f_count[sizeof(NI6009_REQ_AI_SCAN_0F_COUNT)];
  u8 req_12_bytes[sizeof(NI6009_REQ_AI_SCAN_12_BYTES)];
  u32 divisor = ni6009_ai_scan_divisor(cmd->scan_begin_arg);
  u64 total_bytes = (u64)cmd->stop_arg * cmd->chanlist_len *
                    comedi_bytes_per_sample(dev->read_subdev);
  unsigned int chan;
  int ret;

  if (total_bytes > U32_MAX)
    return -EINVAL;

  ret = ni6009_xfer_cmd(dev, NI6009_REQ_AI_0C_G3, sizeof(NI6009_REQ_AI_0C_G3),
                        NI6009_RSP_GENERIC, sizeof(NI6009_RSP_GENERIC));
  if (ret)
    return ret;

  ret = ni6009_xfer_cmd(dev, NI6009_REQ_AI_SCAN_0F,
                        sizeof(NI6009_REQ_AI_SCAN_0F), NI6009_RSP_GENERIC,
                        sizeof(NI6009_RSP_GENERIC));
  if (ret)
    return ret;

  ret = ni6009_xfer_cmd(dev, NI6009_REQ_AI_SCAN_0E,
                        sizeof(NI6009_REQ_AI_SCAN_0E), NI6009_RSP_GENERIC,
                        sizeof(NI6009_RSP_GENERIC));
  if (ret)
    return ret;

  for (chan = 0; chan < NI6009_AI_SCAN_CHANLIST_LEN; chan++) {
    u16 setup_code;

    ret = ni6009_ai_single_ended_setup_code(chan, 0, AREF_GROUND,
                                            &setup_code);
    if (ret)
      return ret;

    memcpy(req_14, NI6009_REQ_AI_14_SETUP, sizeof(req_14));
    put_unaligned_be16(setup_code, &req_14[14]);
    ret = ni6009_xfer_cmd_prefix(dev, req_14, sizeof(req_14), 0x18,
                                 NI6009_RSP_HEADER_AI_14_SETUP,
                                 sizeof(NI6009_RSP_HEADER_AI_14_SETUP));
    if (ret)
      return ret;

    memcpy(req_10, NI6009_REQ_AI_10, sizeof(req_10));
    req_10[26] = chan & 0xff;
    ret = ni6009_xfer_cmd(dev, req_10, sizeof(req_10), NI6009_RSP_GENERIC,
                          sizeof(NI6009_RSP_GENERIC));
    if (ret)
      return ret;
  }

  ret = ni6009_xfer_cmd(dev, NI6009_REQ_AI_SCAN_0E_ARM,
                        sizeof(NI6009_REQ_AI_SCAN_0E_ARM),
                        NI6009_RSP_GENERIC, sizeof(NI6009_RSP_GENERIC));
  if (ret)
    return ret;

  memcpy(req_16, NI6009_REQ_AI_SCAN_16_CLOCK, sizeof(req_16));
  put_unaligned_be32(divisor, &req_16[12]);
  ret = ni6009_xfer_cmd(dev, req_16, sizeof(req_16), NI6009_RSP_GENERIC,
                        sizeof(NI6009_RSP_GENERIC));
  if (ret)
    return ret;

  ret = ni6009_xfer_cmd(dev, NI6009_REQ_AI_13, sizeof(NI6009_REQ_AI_13),
                        NI6009_RSP_GENERIC, sizeof(NI6009_RSP_GENERIC));
  if (ret)
    return ret;

  ret = ni6009_xfer_cmd(dev, NI6009_REQ_AI_15, sizeof(NI6009_REQ_AI_15),
                        NI6009_RSP_GENERIC, sizeof(NI6009_RSP_GENERIC));
  if (ret)
    return ret;

  memcpy(req_0f_count, NI6009_REQ_AI_SCAN_0F_COUNT, sizeof(req_0f_count));
  put_unaligned_be32(cmd->stop_arg, &req_0f_count[12]);
  ret = ni6009_xfer_cmd(dev, req_0f_count, sizeof(req_0f_count),
                        NI6009_RSP_GENERIC, sizeof(NI6009_RSP_GENERIC));
  if (ret)
    return ret;

  memcpy(req_12_bytes, NI6009_REQ_AI_SCAN_12_BYTES, sizeof(req_12_bytes));
  put_unaligned_be32((u32)total_bytes, &req_12_bytes[12]);
  return ni6009_xfer_cmd(dev, req_12_bytes, sizeof(req_12_bytes),
                         NI6009_RSP_GENERIC, sizeof(NI6009_RSP_GENERIC));
}

static int ni6009_ai_start_scan_task(struct comedi_device *dev)
{
  int ret;

  ret = ni6009_xfer_cmd(dev, NI6009_REQ_AI_START_0F_G3,
                        sizeof(NI6009_REQ_AI_START_0F_G3),
                        NI6009_RSP_GENERIC, sizeof(NI6009_RSP_GENERIC));
  if (ret)
    return ret;

  ret = ni6009_xfer_cmd(dev, NI6009_REQ_AI_START_09_G3,
                        sizeof(NI6009_REQ_AI_START_09_G3),
                        NI6009_RSP_GENERIC, sizeof(NI6009_RSP_GENERIC));
  if (ret)
    return ret;

  ret = ni6009_xfer_cmd(dev, NI6009_REQ_AI_START_09_G0,
                        sizeof(NI6009_REQ_AI_START_09_G0),
                        NI6009_RSP_GENERIC, sizeof(NI6009_RSP_GENERIC));
  if (ret)
    return ret;

  ret = ni6009_xfer_cmd(dev, NI6009_REQ_AI_START_0A_G0,
                        sizeof(NI6009_REQ_AI_START_0A_G0),
                        NI6009_RSP_GENERIC, sizeof(NI6009_RSP_GENERIC));
  if (ret)
    return ret;

  return ni6009_xfer_cmd(dev, NI6009_REQ_AI_START_10_G3,
                         sizeof(NI6009_REQ_AI_START_10_G3),
                         NI6009_RSP_GENERIC, sizeof(NI6009_RSP_GENERIC));
}

static int ni6009_ai_do_cmd(struct comedi_device *dev,
                            struct comedi_subdevice *s)
{
  struct ni6009_private *devpriv = dev->private;
  struct comedi_cmd *cmd = &s->async->cmd;
  u64 bytes_left = (u64)cmd->stop_arg * cmd->chanlist_len *
                   comedi_bytes_per_sample(s);
  int ret = 0;

  mutex_lock(&devpriv->mut);

  ret = ni6009_ai_session_init(dev);
  if (ret)
    goto out;

  ret = ni6009_ai_program_scan(dev, cmd);
  if (ret)
    goto out;

  ret = ni6009_ai_start_scan_task(dev);
  if (ret)
    goto out;

  while (bytes_left) {
    struct usb_device *usb = comedi_to_usb_dev(dev);
    unsigned int nsamples;
    unsigned int written_bytes;
    unsigned int i;
    int actual = 0;

    ret = usb_bulk_msg(
        usb, usb_rcvbulkpipe(usb, devpriv->ep_data_in->bEndpointAddress),
        devpriv->data_rx_buf, usb_endpoint_maxp(devpriv->ep_data_in), &actual,
        NI6009_TIMEOUT_MS);
    if (ret)
      goto out;
    if (actual <= 0 || actual > bytes_left || (actual & 0x1)) {
      ret = -EIO;
      goto out;
    }

    for (i = 0; i < actual; i += 2) {
      u16 sample = ni6009_ai_decode_sample(&devpriv->data_rx_buf[i]);

      put_unaligned(sample, (u16 *)&devpriv->data_rx_buf[i]);
    }

    nsamples = comedi_bytes_to_samples(s, actual);
    written_bytes =
        comedi_buf_write_samples(s, devpriv->data_rx_buf, nsamples);
    if (written_bytes != (unsigned int)actual) {
      s->async->events |= COMEDI_CB_OVERFLOW;
      ret = -EOVERFLOW;
      goto out;
    }

    bytes_left -= actual;
  }

out:
  if (ret)
    s->async->events |= COMEDI_CB_ERROR;
  else
    s->async->events |= COMEDI_CB_EOA | COMEDI_CB_BLOCK;
  comedi_handle_events(dev, s);
  mutex_unlock(&devpriv->mut);
  return ret;
}

static int ni6009_ai_cancel(struct comedi_device *dev, struct comedi_subdevice *s)
{
  return 0;
}

static u16 ni6009_ai_decode_sample(const u8 *buf)
{
  /*
   * Captured USB payloads carry the 14-bit AI sample in a 16-bit big-endian
   * word, left-justified by 2 bits.
   */
  return get_unaligned_be16(buf) >> 2;
}

static u16 ni6009_ai_normalize_common_sample(u16 sample)
{
  int code = sample - NI6009_AI_COMMON_ZERO_CODE;

  if (code <= 0)
    return 0;
  if (code >= NI6009_AI_COMMON_SPAN_CODE)
    return NI6009_AI_MAXDATA;

  return DIV_ROUND_CLOSEST(code * NI6009_AI_MAXDATA,
                           NI6009_AI_COMMON_SPAN_CODE);
}

static int ni6009_ai_read_sample(struct comedi_device *dev, u16 *sample) {
  struct usb_device *usb = comedi_to_usb_dev(dev);
  struct ni6009_private *devpriv = dev->private;
  int actual = 0;
  int ret;

  ret = ni6009_xfer_cmd(dev, NI6009_REQ_AI_READ_ARM,
                        sizeof(NI6009_REQ_AI_READ_ARM), NI6009_RSP_GENERIC,
                        sizeof(NI6009_RSP_GENERIC));
  if (ret)
    return ret;

  ret = ni6009_xfer_cmd(dev, NI6009_REQ_AI_READ_TRIGGER,
                        sizeof(NI6009_REQ_AI_READ_TRIGGER),
                        NI6009_RSP_GENERIC, sizeof(NI6009_RSP_GENERIC));
  if (ret)
    return ret;

  ret = usb_bulk_msg(
      usb, usb_rcvbulkpipe(usb, devpriv->ep_data_in->bEndpointAddress),
      devpriv->data_rx_buf, usb_endpoint_maxp(devpriv->ep_data_in), &actual,
      NI6009_TIMEOUT_MS);
  if (ret)
    return ret;

  if (actual < 2)
    return -EIO;

  *sample = ni6009_ai_decode_sample(devpriv->data_rx_buf);
  return 0;
}

static int ni6009_ai_read_diff_sample(struct comedi_device *dev,
                                      unsigned int chan, u16 *sample) {
  struct usb_device *usb = comedi_to_usb_dev(dev);
  struct ni6009_private *devpriv = dev->private;
  int actual = 0;
  int ret;

  if (chan > 3)
    return -EINVAL;

  ret = ni6009_xfer_cmd(dev, NI6009_REQ_AI_READ_ARM,
                        sizeof(NI6009_REQ_AI_READ_ARM), NI6009_RSP_GENERIC,
                        sizeof(NI6009_RSP_GENERIC));
  if (ret)
    return ret;

  ret = ni6009_xfer_cmd(dev, NI6009_REQ_AI_READ_TRIGGER,
                        sizeof(NI6009_REQ_AI_READ_TRIGGER),
                        NI6009_RSP_GENERIC, sizeof(NI6009_RSP_GENERIC));
  if (ret)
    return ret;

  ret = usb_bulk_msg(
      usb, usb_rcvbulkpipe(usb, devpriv->ep_data_in->bEndpointAddress),
      devpriv->data_rx_buf, usb_endpoint_maxp(devpriv->ep_data_in), &actual,
      NI6009_TIMEOUT_MS);
  if (ret)
    return ret;

  if (actual < 8)
    return -EIO;

  *sample = ni6009_ai_decode_sample(&devpriv->data_rx_buf[chan * 2]);
  return 0;
}

static int ni6009_ai_insn_read(struct comedi_device *dev,
                               struct comedi_subdevice *s,
                               struct comedi_insn *insn, unsigned int *data) {
  unsigned int chan = CR_CHAN(insn->chanspec);
  unsigned int range = CR_RANGE(insn->chanspec);
  unsigned int aref = CR_AREF(insn->chanspec);
  struct ni6009_private *devpriv = dev->private;
  unsigned int i;
  int ret;

  if (aref == AREF_DIFF) {
    if (chan > 3 || range != 1)
      return -EINVAL;
  } else if (aref == AREF_COMMON) {
    if (chan != 0 || range != 2)
      return -EINVAL;
  } else {
    if (aref != AREF_GROUND)
      return -EINVAL;
    if (range == 0) {
      if (chan > 7)
        return -EINVAL;
    } else if (range == 2) {
      if (chan != 0)
        return -EINVAL;
    } else {
      return -EINVAL;
    }
  }

  mutex_lock(&devpriv->mut);

  ret = ni6009_ai_session_init(dev);
  if (ret)
    goto out;

  if (aref == AREF_DIFF)
    ret = ni6009_ai_program_diff(dev);
  else
    ret = ni6009_ai_program_single(dev, chan, range, aref);
  if (ret)
    goto out;

  ret = ni6009_ai_start_task(dev);
  if (ret)
    goto out;

  for (i = 0; i < insn->n; i++) {
    u16 sample;

    if (aref == AREF_DIFF)
      ret = ni6009_ai_read_diff_sample(dev, chan, &sample);
    else
      ret = ni6009_ai_read_sample(dev, &sample);
    if (ret)
      goto out;

    if (aref == AREF_COMMON)
      sample = ni6009_ai_normalize_common_sample(sample);

    data[i] = sample;
  }

  ret = insn->n;
out:
  mutex_unlock(&devpriv->mut);
  return ret;
}

static int ni6009_alloc_usb_buffers(struct comedi_device *dev) {
  struct ni6009_private *devpriv = dev->private;

  devpriv->cmd_tx_buf =
      kzalloc(usb_endpoint_maxp(devpriv->ep_cmd_out), GFP_KERNEL);
  if (!devpriv->cmd_tx_buf)
    return -ENOMEM;

  devpriv->cmd_rx_buf =
      kzalloc(usb_endpoint_maxp(devpriv->ep_cmd_in), GFP_KERNEL);
  if (!devpriv->cmd_rx_buf)
    return -ENOMEM;

  devpriv->data_rx_buf =
      kzalloc(usb_endpoint_maxp(devpriv->ep_data_in), GFP_KERNEL);
  if (!devpriv->data_rx_buf)
    return -ENOMEM;

  return 0;
}

static int ni6009_find_endpoints(struct comedi_device *dev) {
  struct usb_interface *intf = comedi_to_usb_interface(dev);
  struct ni6009_private *devpriv = dev->private;
  struct usb_host_interface *iface_desc = intf->cur_altsetting;
  struct usb_endpoint_descriptor *ep_desc;
  int i;

  for (i = 0; i < iface_desc->desc.bNumEndpoints; i++) {
    ep_desc = &iface_desc->endpoint[i].desc;

    if (!usb_endpoint_xfer_bulk(ep_desc))
      continue;

    if (usb_endpoint_dir_out(ep_desc) && ep_desc->bEndpointAddress == 0x01 &&
        !devpriv->ep_cmd_out) {
      devpriv->ep_cmd_out = ep_desc;
      continue;
    }

    if (usb_endpoint_dir_in(ep_desc) && ep_desc->bEndpointAddress == 0x81 &&
        !devpriv->ep_cmd_in) {
      devpriv->ep_cmd_in = ep_desc;
      continue;
    }

    if (usb_endpoint_dir_in(ep_desc) && ep_desc->bEndpointAddress == 0x82 &&
        !devpriv->ep_data_in) {
      devpriv->ep_data_in = ep_desc;
      continue;
    }
  }

  if (!devpriv->ep_cmd_out || !devpriv->ep_cmd_in || !devpriv->ep_data_in)
    return -ENODEV;

  if (usb_endpoint_maxp(devpriv->ep_cmd_out) < NI6009_MIN_PKT ||
      usb_endpoint_maxp(devpriv->ep_cmd_in) < NI6009_MIN_PKT ||
      usb_endpoint_maxp(devpriv->ep_data_in) < 2)
    return -ENODEV;

  return 0;
}

static int ni6009_auto_attach(struct comedi_device *dev,
                              unsigned long context) {
  struct usb_interface *intf = comedi_to_usb_interface(dev);
  struct ni6009_private *devpriv;
  struct comedi_subdevice *s;
  int ret;

  devpriv = comedi_alloc_devpriv(dev, sizeof(*devpriv));
  if (!devpriv)
    return -ENOMEM;

  mutex_init(&devpriv->mut);
  usb_set_intfdata(intf, devpriv);

  ret = ni6009_find_endpoints(dev);
  if (ret)
    return ret;

  ret = ni6009_alloc_usb_buffers(dev);
  if (ret)
    return ret;

  ret = comedi_alloc_subdevices(dev, 4);
  if (ret)
    return ret;

  /* Capture-derived AI single-ended and differential read paths. */
  s = &dev->subdevices[0];
  s->type = COMEDI_SUBD_AI;
  s->subdev_flags = SDF_READABLE | SDF_GROUND | SDF_COMMON | SDF_DIFF |
                    SDF_CMD | SDF_CMD_READ;
  s->n_chan = 8;
  s->len_chanlist = NI6009_AI_SCAN_CHANLIST_LEN;
  s->maxdata = NI6009_AI_MAXDATA;
  s->range_table = &ni6009_ai_ranges;
  s->insn_read = ni6009_ai_insn_read;
  s->do_cmd = ni6009_ai_do_cmd;
  s->do_cmdtest = ni6009_ai_cmdtest;
  s->cancel = ni6009_ai_cancel;
  dev->read_subdev = s;

  /* Capture-derived AO write path for ao0/ao1. */
  s = &dev->subdevices[1];
  s->type = COMEDI_SUBD_AO;
  s->subdev_flags = SDF_WRITABLE | SDF_GROUND;
  s->n_chan = 2;
  s->maxdata = 0x0fff;
  s->range_table = &range_unipolar5;
  s->insn_read = ni6009_ao_insn_read;
  s->insn_write = ni6009_ao_insn_write;
  dev->write_subdev = s;

  /* Capture-derived DIO port0/port1 path. */
  s = &dev->subdevices[2];
  s->type = COMEDI_SUBD_DIO;
  s->subdev_flags = SDF_READABLE | SDF_WRITABLE;
  s->n_chan = 13;
  s->maxdata = 1;
  s->range_table = &range_digital;
  s->insn_bits = ni6009_dio_insn_bits;
  s->insn_config = ni6009_dio_insn_config;

  /* Capture-derived counter edge-count path. */
  s = &dev->subdevices[3];
  s->type = COMEDI_SUBD_COUNTER;
  s->subdev_flags = SDF_READABLE | SDF_LSAMPL;
  s->n_chan = 1;
  s->maxdata = 0xffffffff;
  s->insn_read = ni6009_ctr_insn_read;
  s->insn_config = ni6009_ctr_insn_config;

  dev_warn(dev->class_dev,
           "ni_usb6009: capture-derived driver loaded\n");

  return 0;
}

static int ni6009_detach(struct comedi_device *dev) {
  struct usb_interface *intf = comedi_to_usb_interface(dev);
  struct ni6009_private *devpriv = dev->private;

  if (!devpriv)
    return 0;

  mutex_destroy(&devpriv->mut);
  usb_set_intfdata(intf, NULL);

  kfree(devpriv->data_rx_buf);
  kfree(devpriv->cmd_rx_buf);
  kfree(devpriv->cmd_tx_buf);

  return 0;
}

static struct comedi_driver ni6009_driver = {
    .module = THIS_MODULE,
    .driver_name = "ni_usb6009",
    .auto_attach = ni6009_auto_attach,
    .detach = ni6009_detach,
};

static int ni6009_usb_probe(struct usb_interface *intf,
                            const struct usb_device_id *id) {
  return comedi_usb_auto_config(intf, &ni6009_driver, id->driver_info);
}

static const struct usb_device_id ni6009_usb_table[] = {
    {USB_DEVICE(0x3923, 0x717b)},
    {},
};
MODULE_DEVICE_TABLE(usb, ni6009_usb_table);

static struct usb_driver ni6009_usb_driver = {
    .name = "ni_usb6009",
    .id_table = ni6009_usb_table,
    .probe = ni6009_usb_probe,
    .disconnect = comedi_usb_auto_unconfig,
};

static int __init ni6009_init_module(void)
{
  int ret;

  ret = comedi_driver_register(&ni6009_driver);
  if (ret < 0)
    return ret;

  ret = usb_register(&ni6009_usb_driver);
  if (ret < 0) {
    comedi_driver_unregister(&ni6009_driver);
    return ret;
  }

  return 0;
}

static void __exit ni6009_cleanup_module(void)
{
  usb_deregister(&ni6009_usb_driver);
  comedi_driver_unregister(&ni6009_driver);
}

module_init(ni6009_init_module);
module_exit(ni6009_cleanup_module);

MODULE_AUTHOR("Sergio Hidalgo");
MODULE_DESCRIPTION("Experimental Comedi driver for National Instruments"
                   "USB-6009, adapted from NI USB-6501 driver");
MODULE_LICENSE("GPL");
