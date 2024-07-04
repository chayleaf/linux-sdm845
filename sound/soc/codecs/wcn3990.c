// SPDX-License-Identifier: GPL-2.0-
// Copyright (c) 2024, Dylan Van Assche

#include <linux/module.h>
#include <linux/device.h>
#include <linux/slimbus.h>
#include <linux/of.h>
#include "wcn3990.h"

struct wcn3990_codec {
	struct device *dev;
	struct slim_device *slim;
	struct slim_device *slim_ifc_dev;
	uint32_t sample_rate;
};

struct wcn3990_slim_channel {
	u32 id;
	char *name;
	u16 port;
	u8 port_type;
	u8 channels;
	bool grouped;
};

/* TX channels */
static const struct wcn3990_slim_channel wcn3990_channels_tx[] = {
	{
		.id = WCN3990_FM_SLIM_TX,
		.name = "wcn3990_fm_slim_tx1_port",
		.port = WCN3990_FM_SLIM_TX1_PORT,
		.port_type = WCN3990_PORT_TYPE_TX,
		.channels = 2,
		.grouped = true,
	},
	{
		.id = WCN3990_FM_SLIM_TX,
		.name = "wcn3990_fm_slim_tx2_port",
		.port = WCN3990_FM_SLIM_TX2_PORT,
		.port_type = WCN3990_PORT_TYPE_TX,
		.channels = 2,
		.grouped = false,
	},
	{
		.id = WCN3990_BT_SLIM_TX,
		.name = "wcn3990_bt_slim_tx_port",
		.port = WCN3990_BT_SLIM_TX_PORT,
		.port_type = WCN3990_PORT_TYPE_TX,
		.channels = 1,
		.grouped = false,
	},

};

/* RX channels */
static const struct wcn3990_slim_channel wcn3990_channels_rx[] = {
	{
		.id = WCN3990_BT_SLIM_RX,
		.name = "wcn3990_bt_slim_rx_port",
		.port = WCN3990_BT_SLIM_RX_PORT,
		.port_type = WCN3990_PORT_TYPE_RX,
		.channels = 1,
		.grouped = false,
	},
	{
		.id = WCN3990_BT_SPLIT_SLIM_RX,
		.name = "wcn3990_bt_split_slim_rx_port",
		.port = WCN3990_BT_SLIM_SPLIT_RX_PORT,
		.port_type = WCN3990_PORT_TYPE_RX,
		.channels = 1,
		.grouped = false,
	},

};

/* Find a SLIMBus audio channel based on DAI ID */
static wcn3990_slim_channel *find_channel(struct snd_soc_dai *dai) {
	struct device *dev = dai->dev;
	struct wcn3990_slim_channel *channel;
	bool found = false;

	/* Determine TX or RX channels */
	switch (dai->id) {
		case WCN3990_FM_SLIM_TX:
		case WCN3990_BT_SLIM_TX:
			channel = &wcn3990_slim_channels_tx[0];
			break;
		case WCN3990_BT_SLIM_RX:
		case WCN3990_BT_SPLIT_SLIM_RX:
			channel = &wcn3990_slim_channels_rx[0];
			break;
		default:
			dev_err(wcn->dev, "Unsupported DAI %s (%d)", dai->name, dai->id);
			return -EINVAL;
	}

	/* Get SLIMBus channel for DAI */
	for (int i = 0; i < WCN3990_NUM_OF_DAIS; i++) {
		if (channel->id == dai->id) {
			found = true;
			break;
		}
		port++;
	}

	if (found) {
		dev_error(dev, "No port found for DAI %s (%d)", dai->name, dai->id);
		return NULL;
	}

	dev_info(dev, "Found port: %s (%d)", channel->name, channel->id);

	return channel;
}

static int wcn3990_slim_enable_channel(struct wcn3990_codec *wcn,
				       struct wcn3990_channel *channel,
				       uint8_t rxport, uint32_t rates, 
				       uint8_t grp, uint8_t number_of_channels)
{
	struct device *dev = wcn->dev;

	dev_info(dev, "wcn3990_slim_enable_channel\n");

	// TODO

	return 0;
}

static int wcn3990_slim_disable_channel(struct wcn3990_codec *wcn
				        struct wcn3990_channel *channel,
				        uint8_t rxport, uint32_t rates, 
				        uint8_t grp, uint8_t number_of_channels)
{
	struct device *dev = wcn->dev;

	dev_info(dev, "wcn3990_slim_disable_channel\n");

	// TODO

	return 0;
}

static int wcn3990_alloc_port(struct wcn3990_codec *wcn)
{
	struct device *dev = wcn->dev;

	dev_info(dev, "wcn3990_slim_alloc_port\n");

	// TODO

	return 0;
}

static int wcn3990_dai_startup(struct snd_pcm_substream *substream,
			       struct snd_soc_dai *dai)
{
	printk("wcn3990_dai_startup\n");

	/* No intialisation needed as SLIM device is already brought up */

	return 0;
}

static void wcn3990_dai_shutdown(struct snd_pcm_substream *substream,
				 struct snd_soc_dai *dai)
{
	struct wcn3990_codec *wcn = dai->dev->platform_data;
	struct wcn3990_slim_channel *channel;

	printk("wcn3990_dai_shutdown\n");

	wcn->sample_rate = 0;
	channel = find_channel(dai);

	//btfm_slim_disable_ch
	//btfm_slim_hw_deinit
	return 0;
}

static int wcn3990_dai_hw_params(struct snd_pcm_substream *substream,
			 	 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	printk("wcn3990_dai_hw_params\n");

	/* No HW params to configure as they are fixed anyway */

	return 0;
}

static int wcn3990_dai_prepare(struct snd_pcm_substream *substream,
		    	       struct snd_soc_dai *dai)
{
	struct wcn3990_codec *wcn = dai->dev->platform_data;
	struct wcn3990_slim_channel *channel;
	printk("wcn3990_dai_prepare\n");

	wcn->sample_rate = dai->rate;
	channel = find_channel(dai);

	//btfm_slim_enable_ch
	return 0;
}

static int wcn3990_dai_set_channel_map(struct snd_soc_dai *dai,
		   		       unsigned int tx_num, unsigned int *tx_slot,
				       unsigned int rx_num, unsigned int *rx_slot)
{
	struct wcn3990_codec *wcn = dai->dev->platform_data;
	printk("wcn3990_set_channelmap\n");

	// TODO: create channels with SLIM 

	return 0;
}

static int wcn3990_dai_get_channel_map(struct snd_soc_dai *dai,
			    	       unsigned int *tx_num, unsigned int *tx_slot,
				       unsigned int *rx_num, unsigned int *rx_slot)
{
	struct wcn3990_codec *wcn = dai->dev->platform_data;
	printk("wcn3990_get_channelmap\n");

	switch (dai->id) {
		case WCN3990_FM_SLIM_TX:
			num = 2;
		case WCN3990_BT_SLIM_TX:
			// TODO
			break;
		case WCN3990_BT_SLIM_RX:
		case WCN3990_BT_SPLIT_SLIM_RX:
			// TODO
			break;
		default:
			dev_err(wcn->dev, "Unsupported DAI %s (%d)", dai->name, dai->id);
			return -EINVAL;
	
	}

	// TODO

	return 0;
}

static struct snd_soc_dai_ops wcn3990_dai_ops = {
	.startup = wcn3990_dai_startup,
	.shutdown = wcn3990_dai_shutdown,
	.hw_params = wcn3990_dai_hw_params,
	.prepare = wcn3990_dai_prepare,
	.set_channel_map = wcn3990_set_channel_map,
	.get_channel_map = wcn3990_get_channel_map,
}

static struct snd_soc_dai_driver wcn3990_dai[] = {
	{	/* FM radio audio: FM -> QDSP */
		.name = "wcn3990_fm_slim_tx",
		.id = WCN3990_FM_SLIM_TX,
		.capture = {
			.stream_name = "FM radio TX Capture",
			.rates = SNDRV_PCM_RATE_48000, /* 48 KHz */
			.formats = SNDRV_PCM_FMTBIT_S16_LE, /* 16 bits */
			.rate_max = 48000,
			.rate_min = 48000,
			.channels_min = 1,
			.channels_max = 2,
		},
		.ops = &wcn3990_dai_ops,
	},
	{	/* Bluetooth SCO voice source: Bluetooth SCO -> Modem */
		.name = "wcn3990_bt_slim_tx",
		.id = WCN3990_BT_SLIM_TX,
		.capture = {
			.stream_name = "Bluetooth TX Capture",
			/* 8 KHz or 16 KHz */
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000
				| SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000
				| SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE, /* 16 bits */
			.rate_max = 96000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 1,
		},
		.ops = &wcn3990_dai_ops,
	},
	{	/* Bluetooth SCO voice sink: Modem -> Bluetooth SCO or A2DP */
		.name = "wcn3990_bt_slim_rx",
		.id = WCN3990_BT_SLIM_RX,
		.playback = {
			.stream_name = "Bluetooth RX Playback",
			/* 8/16/44.1/48/88.2/96 Khz */
			.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000
				| SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000
				| SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE, /* 16 bits */
			.rate_max = 96000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 1,
		},
		.ops = &wcn3990_dai_ops,
	},
	{	/* Bluetooth Split A2DP data: QDSP -> Bluetooth */
		.name = "wcn3990_bt_split_slim_rx",
		.id = WCN3990_BT_SPLIT_SLIM_RX,
		.playback = {
			.stream_name = "Bluetooth Split RX Playback",
			.rates = SNDRV_PCM_RATE_48000, /* 48 KHz */
			.formats = SNDRV_PCM_FMTBIT_S16_LE, /* 16 bits */
			.rate_max = 48000,
			.rate_min = 48000,
			.channels_min = 1,
			.channels_max = 1,
		},
		.ops = &wcn3990_dai_ops,
	},
};

static int wcn3990_parse_dt(struct wcn3990_codec *wcn)
{
	struct device *dev = wcn->dev;

	dev_info(dev, "wcn3990_parse_dt\n");

	/* No additional properties to parse yet besides builtin SLIMBus device */

	return 0;
}

static void wcn3990_codec_probe(struct snd_soc_component *component)
{
	struct device *dev = component->dev;

	/* ALSA ASoC component specific probe */
	dev_info(dev, "wcn3990_codec_probe\n");

	/* No ALSA controls yet, so nothing to do here */
}


static void wcn3990_codec_remove(struct snd_soc_component *component)
{
	struct device *dev = component->dev;

	/* ALSA ASoC component specific remove */
	dev_info(dev, "wcn3990_codec_remove\n");

	/* No cleanup necessary here, done by the SLIMBus driver */
}

static const struct snd_soc_component_driver wcn3990_codec_component_drv = {
	.probe = wcn3990_codec_probe,
	.remove = wcn3990_codec_remove,
}

static int wcn3990_slim_probe(struct slim_device *slim)
{
	struct device *dev = &slim->dev;
	struct wcn3990_codec *wcn;
	struct slim_val_inf msg_msb;
	struct slim_val_inf msg_lsb;
	uint16_t hw_revision;
	uint16_t hw_subrevision;
	int ret;

	/* SLIM device probe */
	dev_info(dev, "wcn3990_slim_probe\n");

	wcn = devm_kzalloc(dev, sizeof(*wcn), GFP_KERNEL);
	if (!wcn)
		return -ENOMEM;

	wcn->dev = dev;
	ret = wcn3990_parse_dt(wcn);
	if (ret) {
		dev_err(dev, "Error parsing DT: %d\n", ret);
		return ret;
	}

	dev_set_drvdata(dev, wcn);

	return 0;
}

static int wcn3990_slim_status(struct slim_device *sdev,
			       enum slim_device_status status)
{
	struct device *dev = &sdev->dev;
	struct device_node *ifc_dev_np;
	struct wcn3990_codec *wcn;

	/* SLIM device status reporting when device is up, down, etc. */
	dev_info(dev, "wcn3990_slim_status\n");

	wcn = dev_get_drvdata(dev);

	ifc_dev_np = of_parse_phandle(dev->of_node, "slim-ifc-dev", 0);
	if (!ifc_dev_np) {
		dev_err(dev, "No Interface device found\n");
		return -EINVAL;
	}

	/* Downstream only uses SLIMBus Interface Device for WCN3990, PGD mode not used */
	wcn->slim = sdev;
	wcn->slim_ifc_dev = of_slim_get_device(wcn->slim->ctrl, ifc_dev_np);
	of_node_put(ifc_dev_np);
	if (!wcn->slim_ifc_dev) {
		dev_err(dev, "Unable to get SLIM Interface device\n");
		return -EINVAL;
	}

	dev_info(dev, "Getting logical addr of SLIM\n");
	slim_get_logical_addr(wcn->slim);

	dev_info(dev, "Getting logical addr of SLIM IFC DEV\n");
	slim_get_logical_addr(wcn->slim_ifc_dev);

	/* Get hardware revision */
	msg_msb.start_offset = WCN3990_HW_REV_MSB;
	msg_msb.num_bytes = 1;
	msg_msb.rbuf = &hw_revision;
	msg_msb.wbuf = NULL;
	msg_msb.comp = NULL;

	ret = slim_xfer_msg(wcn->slim, &msg_msb, SLIM_MSG_MC_REQUEST_INFORMATION);
	if (!ret) {
		dev_err(dev, "HW revision retrieval failed: %d\n", ret);
		return ret;
	}
	dev_info(dev, "HW version: v0x%x.0x%x",
		 (hw_revision & 0xF0) >> 4, (hw_revision & 0x0F));

	/* Get hardware subrevision */
	msg_lsb.start_offset = WCN3990_HW_REV_LSB;
	msg_lsb.num_bytes = 1;
	msg_lsb.rbuf = &hw_revision_lsb;
	msg_lsb.wbuf = NULL;
	msg_lsb.comp = NULL;

	slim_xfer_msg(wcn->slim, &msg_lsb, SLIM_MSG_MC_REQUEST_INFORMATION);
	if (!ret) {
		dev_err(dev, "HW subrevision retrieval failed: %d\n", ret);
		return ret;
	}
	dev_info(dev, "HW subversion: v0x%x", hw_subrevision);

	/* Register ALSA ASoC DAIs when SLIM device is up */
	ret = devm_snd_soc_register_component(dev, &wcn3990_codec_component_drv,
					      wcn3990_dai, ARRAY_SIZE(wcn3990_dai));
	if (ret) {
		dev_err(dev, "Registering WCN3990 codec failed: %d\n", ret);
		return ret;
	}


	return 0;
}

static const struct slim_device_id wcn3990_slim_id[] = {
	{SLIM_MANF_ID_QCOM, SLIM_PROD_CODE_WCN3990, 0x1, 0x0},
	{}
};
MODULE_DEVICE_TABLE(slim, wcn3990_slim_id);

static struct slim_driver wcn3990_slim_driver = {
	.driver = {
		.name = "wcn3990-slim",
	},
	.probe = wcn3990_slim_probe,
	.device_status = wcn3990_slim_status,
	.id_table = wcn3990_slim_id,
};

module_slim_driver(wcn3990_slim_driver);
MODULE_DESCRIPTION("WCN3990 slim driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("slim:217:220:*");
