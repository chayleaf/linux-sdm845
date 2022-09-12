// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2017, The Linux Foundation
 * Copyright (c) 2023, Linaro Limited
 *
 * QMI Thermal Mitigation Device (TMD) client driver.
 * This driver provides an in-kernel client to handle hot and cold thermal
 * mitigations for remote subsystems (modem and DSPs). It doesn't implement
 * any handling of reports from remote subsystems.
 */

#define DEBUG

#include <linux/err.h>
#include <linux/module.h>
#include <linux/net.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/remoteproc/qcom_rproc.h>
#include <linux/slab.h>
#include <linux/soc/qcom/qmi.h>
#include <linux/thermal.h>
#include <dt-bindings/thermal/qcom,tmd.h>

#include "qmi-cooling.h"

#define QMI_TMD_RESP_TIMEOUT msecs_to_jiffies(100)

/**
 * struct qmi_tmd_client - TMD client state
 * @dev:	Device associated with this client
 * @name:	Friendly name for the remote TMD service
 * @handle:	QMI connection handle
 * @mutex:	Lock to synchronise QMI communication
 * @id:		The QMI TMD service instance ID
 * @cdev_list:	The list of cooling devices (controls) enabled for this instance
 * @svc_arrive_work: Work item for initialising the client when the TMD service
 *		     starts.
 * @connection_active: Whether or not we're connected to the QMI TMD service.
 *		This property is not protected by the mutex because it's possible
 *		for the service to dissappear at any point, instead we just ensure
 *		that any pending state changes are cached, to be sent when the
 *		service is available again.
 */
struct qmi_tmd_client {
	struct device *dev;
	const char *name;
	struct qmi_handle handle;
	struct mutex mutex;
	u32 id;
	struct list_head cdev_list;
	struct work_struct svc_arrive_work;
	bool connection_active;
};

/**
 * struct qmi_tmd_control - A cooling device (control)
 * @np:		OF node associated with this control
 * @name:	The control type (exposed via sysfs)
 * @qmi_name:	The common name of this control shared by the remote subsystem
 * @cdev:	Thermal framework cooling device handle
 * @cur_state:	The current cooling/warming/mitigation state
 * @max_state:	The maximum state
 * @client:	The TMD client instance this control is associated with
 */
struct qmi_tmd_control {
	struct device_node *np;
	const char *type;
	char qmi_name[QMI_TMD_MITIGATION_DEV_ID_LENGTH_MAX_V01 + 1];
	struct list_head node;
	struct thermal_cooling_device *cdev;
	unsigned int cur_state;
	unsigned int max_state;
	struct qmi_tmd_client *client;
};

static int qmi_get_max_state(struct thermal_cooling_device *cdev, unsigned long *state)
{
	struct qmi_tmd_control *tmd_ctrl = cdev->devdata;

	if (!tmd_ctrl)
		return -EINVAL;

	*state = tmd_ctrl->max_state;

	return 0;
}

static int qmi_get_cur_state(struct thermal_cooling_device *cdev, unsigned long *state)
{
	struct qmi_tmd_control *tmd_ctrl = cdev->devdata;

	if (!tmd_ctrl)
		return -EINVAL;

	*state = tmd_ctrl->cur_state;

	return 0;
}

static int qmi_tmd_send_state_request(struct qmi_tmd_control *tmd_ctrl)
{
	struct tmd_set_mitigation_level_resp_msg_v01 tmd_resp;
	struct tmd_set_mitigation_level_req_msg_v01 req;
	struct qmi_tmd_client *client;
	struct qmi_txn txn;
	int ret = 0;

	client = tmd_ctrl->client;

	if (!client->connection_active)
		return 0;

	memset(&req, 0, sizeof(req));
	memset(&tmd_resp, 0, sizeof(tmd_resp));

	strscpy(req.mitigation_dev_id.mitigation_dev_id, tmd_ctrl->qmi_name,
		QMI_TMD_MITIGATION_DEV_ID_LENGTH_MAX_V01 + 1);
	req.mitigation_level = tmd_ctrl->cur_state;

	mutex_lock(&client->mutex);

	ret = qmi_txn_init(&client->handle, &txn, tmd_set_mitigation_level_resp_msg_v01_ei,
			   &tmd_resp);
	if (ret < 0) {
		dev_err(client->dev, "qmi set state %d txn init failed for %s ret %d\n",
			tmd_ctrl->cur_state, tmd_ctrl->type, ret);
		goto qmi_send_exit;
	}

	ret = qmi_send_request(&client->handle, NULL, &txn, QMI_TMD_SET_MITIGATION_LEVEL_REQ_V01,
			       TMD_SET_MITIGATION_LEVEL_REQ_MSG_V01_MAX_MSG_LEN,
			       tmd_set_mitigation_level_req_msg_v01_ei, &req);
	if (ret < 0) {
		dev_err(client->dev, "qmi set state:%d txn send failed for %s ret %d\n",
			tmd_ctrl->cur_state, tmd_ctrl->type, ret);
		qmi_txn_cancel(&txn);
		goto qmi_send_exit;
	}

	ret = qmi_txn_wait(&txn, QMI_TMD_RESP_TIMEOUT);
	if (ret < 0) {
		dev_err(client->dev, "qmi set state %d txn wait failed for %s ret %d\n",
			tmd_ctrl->cur_state, tmd_ctrl->type, ret);
		goto qmi_send_exit;
	}
	ret = 0;

	if (tmd_resp.resp.result != QMI_RESULT_SUCCESS_V01) {
		ret = tmd_resp.resp.result;
		dev_err(client->dev, "qmi set state %d NOT success for %s ret %d\n",
			tmd_ctrl->cur_state, tmd_ctrl->type, ret);
		goto qmi_send_exit;
	}

	dev_dbg(client->dev, "Requested state %d/%d for %s\n", tmd_ctrl->cur_state,
		tmd_ctrl->max_state, tmd_ctrl->type);

qmi_send_exit:
	mutex_unlock(&client->mutex);
	return ret;
}

static int qmi_set_cur_state(struct thermal_cooling_device *cdev, unsigned long state)
{
	struct qmi_tmd_control *tmd_ctrl = cdev->devdata;

	if (!tmd_ctrl)
		return -EINVAL;

	if (state > tmd_ctrl->max_state)
		return -EINVAL;

	if (tmd_ctrl->cur_state == state)
		return 0;

	tmd_ctrl->cur_state = state;

	return qmi_tmd_send_state_request(tmd_ctrl);
}

static struct thermal_cooling_device_ops qmi_device_ops = {
	.get_max_state = qmi_get_max_state,
	.get_cur_state = qmi_get_cur_state,
	.set_cur_state = qmi_set_cur_state,
};

static int qmi_register_cooling_device(struct qmi_tmd_control *tmd_ctrl)
{
	struct thermal_cooling_device *cdev;
	cdev = thermal_of_cooling_device_register(tmd_ctrl->np, tmd_ctrl->type, tmd_ctrl,
						  &qmi_device_ops);

	if (IS_ERR(cdev))
		return dev_err_probe(tmd_ctrl->client->dev, PTR_ERR(tmd_ctrl->cdev),
				     "Failed to register cooling device %s\n", tmd_ctrl->qmi_name);

	tmd_ctrl->cdev = cdev;
	return 0;
}

/*
 * Init a single TMD control by registering a cooling device for it, or
 * synchronising state with the remote subsystem if recovering from a service
 * restart.
 */
static int qmi_tmd_init_control(struct qmi_tmd_client *client, const char *label)
{
	struct qmi_tmd_control *tmd_ctrl = NULL;
	int ret;

	list_for_each_entry(tmd_ctrl, &client->cdev_list, node) {
		if ((strncasecmp(tmd_ctrl->qmi_name, label,
				 QMI_TMD_MITIGATION_DEV_ID_LENGTH_MAX_V01 + 1)))
			continue;

		break;
	}

	if (!tmd_ctrl) {
		dev_dbg(client->dev,
			"TMD control '%s' available in firmware but not specified in DT\n", label);
		return 0;
	}

	/*
	 * If the cooling device already exists then the QMI service went away and
	 * came back. So make sure the current cooling device state is reflected
	 * on the remote side.
	 */
	if (!tmd_ctrl->cdev)
		ret = qmi_register_cooling_device(tmd_ctrl);
	else
		ret = qmi_tmd_send_state_request(tmd_ctrl);

	return ret;
}

static void qmi_tmd_svc_arrive(struct work_struct *work)
{
	struct qmi_tmd_client *client = container_of(work, struct qmi_tmd_client, svc_arrive_work);

	struct tmd_get_mitigation_device_list_req_msg_v01 req;
	struct tmd_get_mitigation_device_list_resp_msg_v01 *resp;
	int ret = 0, i;
	struct qmi_txn txn;

	memset(&req, 0, sizeof(req));
	/* resp struct is 1.1kB, allocate it on the heap. */
	resp = kzalloc(sizeof(*resp), GFP_KERNEL);
	if (!resp) {
		dev_err(client->dev, "No memory to allocate resp\n");
		return;
	}

	mutex_lock(&client->mutex);
	ret = qmi_txn_init(&client->handle, &txn, tmd_get_mitigation_device_list_resp_msg_v01_ei,
			   resp);
	if (ret < 0) {
		dev_err(client->dev, "Transaction init error for instance_id: %#x ret %d\n",
			client->id, ret);
		goto reg_exit;
	}

	ret = qmi_send_request(&client->handle, NULL, &txn,
			       QMI_TMD_GET_MITIGATION_DEVICE_LIST_REQ_V01,
			       TMD_GET_MITIGATION_DEVICE_LIST_REQ_MSG_V01_MAX_MSG_LEN,
			       tmd_get_mitigation_device_list_req_msg_v01_ei, &req);
	if (ret < 0) {
		qmi_txn_cancel(&txn);
		goto reg_exit;
	}

	ret = qmi_txn_wait(&txn, QMI_TMD_RESP_TIMEOUT);
	if (ret < 0) {
		dev_err(client->dev, "Transaction wait error for client %#x ret:%d\n", client->id,
			ret);
		goto reg_exit;
	}
	if (resp->resp.result != QMI_RESULT_SUCCESS_V01) {
		ret = resp->resp.result;
		dev_err(client->dev, "Failed to get device list for client %#x ret:%d\n",
			client->id, ret);
		goto reg_exit;
	}
	mutex_unlock(&client->mutex);

	client->connection_active = true;

	for (i = 0; i < resp->mitigation_device_list_len; i++) {
		struct tmd_mitigation_dev_list_type_v01 *device = &resp->mitigation_device_list[i];

		ret = qmi_tmd_init_control(client, device->mitigation_dev_id.mitigation_dev_id);
		if (ret)
			break;
	}

	kfree(resp);
	return;

reg_exit:
	mutex_unlock(&client->mutex);
	kfree(resp);

	return;
}

static void thermal_qmi_net_reset(struct qmi_handle *qmi)
{
	struct qmi_tmd_client *client = container_of(qmi, struct qmi_tmd_client, handle);
	struct qmi_tmd_control *tmd_ctrl = NULL;

	list_for_each_entry(tmd_ctrl, &client->cdev_list, node) {
		qmi_tmd_send_state_request(tmd_ctrl);
	}
}

static void thermal_qmi_del_server(struct qmi_handle *qmi, struct qmi_service *service)
{
	struct qmi_tmd_client *client = container_of(qmi, struct qmi_tmd_client, handle);

	client->connection_active = false;
}

static int thermal_qmi_new_server(struct qmi_handle *qmi, struct qmi_service *service)
{
	struct qmi_tmd_client *client = container_of(qmi, struct qmi_tmd_client, handle);
	struct sockaddr_qrtr sq = { AF_QIPCRTR, service->node, service->port };

	mutex_lock(&client->mutex);
	kernel_connect(qmi->sock, (struct sockaddr *)&sq, sizeof(sq), 0);
	mutex_unlock(&client->mutex);
	queue_work(system_highpri_wq, &client->svc_arrive_work);

	return 0;
}

static struct qmi_ops thermal_qmi_event_ops = {
	.new_server = thermal_qmi_new_server,
	.del_server = thermal_qmi_del_server,
	.net_reset = thermal_qmi_net_reset,
};

static void qmi_tmd_cleanup(struct qmi_tmd_client *client)
{
	struct qmi_tmd_control *tmd_ctrl, *c_next;

	client->connection_active = false;

	mutex_lock(&client->mutex);
	qmi_handle_release(&client->handle);
	cancel_work(&client->svc_arrive_work);
	list_for_each_entry_safe(tmd_ctrl, c_next, &client->cdev_list, node) {
		if (tmd_ctrl->cdev)
			thermal_cooling_device_unregister(tmd_ctrl->cdev);

		list_del(&tmd_ctrl->node);
	}
	mutex_unlock(&client->mutex);
}

static int qmi_tmd_parse_controls(struct qmi_tmd_client *client)
{
	struct device *dev = client->dev;
	struct qmi_tmd_control *tmd_ctrl;
	struct device_node *subnode, *node = dev->of_node;
	int ret;

	for_each_available_child_of_node(node, subnode) {
		const char *name;

		tmd_ctrl = devm_kzalloc(dev, sizeof(*tmd_ctrl), GFP_KERNEL);
		if (!tmd_ctrl)
			return dev_err_probe(client->dev, -ENOMEM, "Couldn't allocate tmd_ctrl\n");

		tmd_ctrl->type = devm_kasprintf(client->dev, GFP_KERNEL, "%s.%s", client->name,
						subnode->name);
		if (!tmd_ctrl->type)
			return dev_err_probe(dev, -ENOMEM, "Cooling device name\n");

		if (of_property_read_string(subnode, "label", &name)) {
			dev_err(client->dev, "Fail to parse dev name for %s\n", subnode->name);
			of_node_put(subnode);
			break;
		}

		ret = strscpy(tmd_ctrl->qmi_name, name,
			      QMI_TMD_MITIGATION_DEV_ID_LENGTH_MAX_V01 + 1);
		if (ret == -E2BIG) {
			return dev_err_probe(dev, -EINVAL, "TMD label %s is too long\n", name);
		}

		tmd_ctrl->client = client;
		tmd_ctrl->np = subnode;
		tmd_ctrl->cur_state = 0;
		list_add(&tmd_ctrl->node, &client->cdev_list);
	}

	if (list_empty(&client->cdev_list))
		return dev_err_probe(client->dev, -EINVAL,
				     "No cooling devices specified for client %s (%#x)\n",
				     client->name, client->id);

	return 0;
}

static int qmi_tmd_client_register(struct qmi_tmd_client *client)
{
	int ret;

	ret = qmi_handle_init(&client->handle,
			      TMD_GET_MITIGATION_DEVICE_LIST_RESP_MSG_V01_MAX_MSG_LEN,
			      &thermal_qmi_event_ops, NULL);
	if (ret < 0) {
		dev_err_probe(client->dev, ret, "QMI handle init failed for client %#x\n",
			      client->id);
		return ret;
	}

	ret = qmi_add_lookup(&client->handle, TMD_SERVICE_ID_V01, TMD_SERVICE_VERS_V01, client->id);
	if (ret < 0) {
		dev_err_probe(client->dev, ret, "QMI register failed for client 0x%x\n",
			      client->id);
		return ret;
	}

	return 0;
}

static int qmi_tmd_client_probe(struct platform_device *pdev)
{
	struct device *dev;
	struct qmi_tmd_client *client;
	int ret;

	dev = &pdev->dev;

	client = devm_kzalloc(dev, sizeof(*client), GFP_KERNEL);
	if (!client)
		return -ENOMEM;

	client->dev = dev;

	ret = of_property_read_u32(dev->of_node, "qcom,instance-id", &client->id);
	if (ret) {
		dev_err(client->dev, "error reading qcom,instance-id (%d)\n", ret);
		return ret;
	}

	switch (client->id) {
	case MODEM0_INSTANCE_ID:
		client->name = "tmd-modem";
		break;
	case ADSP_INSTANCE_ID:
		client->name = "tmd-adsp";
		break;
	case CDSP_INSTANCE_ID:
		client->name = "tmd-cdsp";
		break;
	case SLPI_INSTANCE_ID:
		client->name = "tmd-slpi";
		break;
	default:
		return dev_err_probe(dev, -EINVAL, "Invalid instance ID %#x\n", client->id);
	}

	mutex_init(&client->mutex);
	INIT_LIST_HEAD(&client->cdev_list);
	INIT_WORK(&client->svc_arrive_work, qmi_tmd_svc_arrive);

	ret = qmi_tmd_parse_controls(client);
	if (ret) {
		return ret;
	}

	platform_set_drvdata(pdev, client);

	ret = qmi_tmd_client_register(client);
	if (ret)
		goto probe_err;

	return 0;

probe_err:
	qmi_tmd_cleanup(client);
	return ret;
}

static int qmi_tmd_client_remove(struct platform_device *pdev)
{
	struct qmi_tmd_client *client = platform_get_drvdata(pdev);

	qmi_tmd_cleanup(client);

	return 0;
}

static const struct of_device_id qmi_tmd_device_table[] = { { .compatible = "qcom,qmi-cooling" },
							    {} };
MODULE_DEVICE_TABLE(of, qmi_tmd_device_table);

static struct platform_driver qmi_tmd_device_driver = {
	.probe          = qmi_tmd_client_probe,
	.remove         = qmi_tmd_client_remove,
	.driver         = {
		.name   = "qcom-qmi-cooling",
		.of_match_table = qmi_tmd_device_table,
	},
};

module_platform_driver(qmi_tmd_device_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Qualcomm QMI Thermal Mitigation Device driver");
