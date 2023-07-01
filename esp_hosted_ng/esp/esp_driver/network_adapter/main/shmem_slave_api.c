// Copyright 2015-2021 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "sdkconfig.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <rom/rtc.h>
#include "esp.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "interface.h"
#include "endian.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/periph_defs.h"
#include "stats.h"

static const char TAG[] = "FW_SHMEM";

#define QUEUE_SIZE 64

typedef uint32_t u32;

struct esp_wifi_shmem_queue
{
	volatile u32 write;
	volatile u32 read;
	u32 offset;
	u32 mask;
};

#define ESP_SHMEM_READ_HW_Q		1
#define ESP_SHMEM_WRITE_HW_Q		0

#define ESP_SHMEM_IRQ_FROM_HOST_REG	(4 * CONFIG_ESP_SHMEM_IRQ_FROM_HOST_IDX)
#define ESP_SHMEM_IRQ_TO_HOST_REG	(4 * CONFIG_ESP_SHMEM_IRQ_TO_HOST_IDX)

static struct esp_wifi_shmem_queue hw_queue[2];
static void *queue_data[2][QUEUE_SIZE];

static u32 tx_postprocessed;

#define SPI_RX_QUEUE_SIZE      20
#define SPI_TX_QUEUE_SIZE      20

static interface_context_t context;
static interface_handle_t if_handle_g;

static SemaphoreHandle_t hw_queue_read_semaphore;
static SemaphoreHandle_t read_semaphore;
static QueueHandle_t shmem_rx_queue[MAX_PRIORITY_QUEUES];
static QueueHandle_t shmem_tx_queue[MAX_PRIORITY_QUEUES];
static QueueHandle_t shmem_tx_postprocess_queue;

static void esp_shmem_write_irq(u32 reg, u32 v)
{
	*(volatile u32*)(0x600c0030 + reg) = v;
}

static void esp_shmem_buffer_done(void *p)
{
	ESP_LOGD(TAG, "%s", __func__);
	free(p);
}

static void esp_shmem_hw_queue_write(u32 irq)
{
	struct esp_wifi_shmem_queue *hw_q = hw_queue + ESP_SHMEM_WRITE_HW_Q;
	void * volatile *data = (void *)hw_queue + hw_q->offset;
	bool changed = false;

	for (;;) {
		interface_buffer_handle_t buf_handle;
		u32 r = hw_q->read;
		u32 w = hw_q->write;
		u32 p = tx_postprocessed;
		BaseType_t ret;

		while (p != r) {
			ret = xQueueReceive(shmem_tx_postprocess_queue, &buf_handle, 0);

			if (!ret) {
				ESP_LOGE(TAG, "%s: xQueueReceive failed for postprocessing queue", __func__);
				break;
			}
			if (buf_handle.priv_buffer_handle &&
			    buf_handle.free_buf_handle)
				buf_handle.free_buf_handle(buf_handle.priv_buffer_handle);
			++p;
		}
		tx_postprocessed = p;

		if (w - r == hw_q->mask)
			break;

		if (uxQueueMessagesWaiting(shmem_tx_queue[PRIO_Q_HIGH]))
			ret = xQueueReceive(shmem_tx_queue[PRIO_Q_HIGH], &buf_handle, portMAX_DELAY);
		else if (uxQueueMessagesWaiting(shmem_tx_queue[PRIO_Q_MID]))
			ret = xQueueReceive(shmem_tx_queue[PRIO_Q_MID], &buf_handle, portMAX_DELAY);
		else if (uxQueueMessagesWaiting(shmem_tx_queue[PRIO_Q_LOW]))
			ret = xQueueReceive(shmem_tx_queue[PRIO_Q_LOW], &buf_handle, portMAX_DELAY);
		else
			break;

		if (!ret) {
			ESP_LOGE(TAG, "%s: xQueueReceive failed for tx queue", __func__);
			break;
		}

		data[w & hw_q->mask] = buf_handle.payload;
		hw_q->write = ++w;
		changed = true;
		ESP_LOGD(TAG, "%s: write_queue->write = %d", __func__, w);

		ret = xQueueSend(shmem_tx_postprocess_queue, &buf_handle, 0);
		if (!ret) {
			ESP_LOGE(TAG, "%s: xQueueSend failed", __func__);
			break;
		}
	}

	if (changed)
		esp_shmem_write_irq(ESP_SHMEM_IRQ_TO_HOST_REG, irq);
}

static int32_t esp_shmem_write(interface_handle_t *handle, interface_buffer_handle_t *buf_handle)
{
	esp_err_t ret = ESP_OK;
	int32_t total_len = 0;
	uint16_t offset = 0;
	struct esp_payload_header *header = NULL;
	interface_buffer_handle_t tx_buf_handle = {
		.free_buf_handle = esp_shmem_buffer_done,
	};

	ESP_LOGD(TAG, "%s", __func__);
	if (!handle || !buf_handle) {
		ESP_LOGE(TAG , "Invalid arguments\n");
		return ESP_FAIL;
	}

	if (!buf_handle->payload_len || !buf_handle->payload) {
		ESP_LOGE(TAG , "Invalid arguments, len:%d\n", buf_handle->payload_len);
		return ESP_FAIL;
	}

	total_len = buf_handle->payload_len + sizeof (struct esp_payload_header);

	if (total_len > RX_BUF_SIZE) {
		ESP_LOGE(TAG, "Max frame length exceeded %d.. drop it\n", total_len);
		return ESP_FAIL;
	}

	header = malloc(total_len);
	if (!header) {
		ESP_LOGE(TAG, "couldn't allocate packet copy\n");
		return ESP_FAIL;
	}
	tx_buf_handle.payload = (uint8_t *)header;
	tx_buf_handle.priv_buffer_handle = header,

	offset = sizeof(struct esp_payload_header);
	/* Initialize header */
	*header = (struct esp_payload_header){
		.if_type = buf_handle->if_type,
		.if_num = buf_handle->if_num,
		.flags = buf_handle->flag,
		.packet_type = buf_handle->pkt_type,
		.len = htole16(buf_handle->payload_len),
		.offset = htole16(offset),
	};

	/* copy the data from caller */
	memcpy(tx_buf_handle.payload + offset, buf_handle->payload, buf_handle->payload_len);

	if (header->if_type == ESP_INTERNAL_IF)
		ret = xQueueSend(shmem_tx_queue[PRIO_Q_HIGH], &tx_buf_handle, portMAX_DELAY);
	else if (header->if_type == ESP_HCI_IF)
		ret = xQueueSend(shmem_tx_queue[PRIO_Q_MID], &tx_buf_handle, portMAX_DELAY);
	else
		ret = xQueueSend(shmem_tx_queue[PRIO_Q_LOW], &tx_buf_handle, portMAX_DELAY);

	if (ret != pdTRUE) {
		free(header);
		return ESP_FAIL;
	}

	esp_shmem_hw_queue_write(1);

	return buf_handle->payload_len;
}

static bool esp_shmem_hw_queue_read(void)
{
	struct esp_wifi_shmem_queue *hw_q = hw_queue + ESP_SHMEM_READ_HW_Q;
	void * volatile *data = (void *)hw_queue + hw_q->offset;
	u32 r, w;

	r = hw_q->read;
	w = hw_q->write;

	if (r == w)
		return false;

	while (r != w) {
		struct esp_payload_header *header = NULL;
		uint16_t len = 0, offset = 0;

		header = data[r & hw_q->mask];
		len = le16toh(header->len);
		offset = le16toh(header->offset);

		if (!len || (len > RX_BUF_SIZE)) {
			ESP_LOGE(TAG, "%s: bad len = %d", __func__, len);
		} else {
			/* Buffer is valid */
			void *copy = malloc(len + offset);
			interface_buffer_handle_t buf_handle = {
				.if_type = header->if_type,
				.if_num = header->if_num,
				.payload = copy,
				.payload_len = len + offset,
				.free_buf_handle = esp_shmem_buffer_done,
				.priv_buffer_handle = copy,
			};
			BaseType_t ret;

			memcpy(copy, header, len + offset);

			if (header->if_type == ESP_INTERNAL_IF)
				ret = xQueueSend(shmem_rx_queue[PRIO_Q_HIGH], &buf_handle, portMAX_DELAY);
			else if (header->if_type == ESP_HCI_IF)
				ret = xQueueSend(shmem_rx_queue[PRIO_Q_MID], &buf_handle, portMAX_DELAY);
			else
				ret = xQueueSend(shmem_rx_queue[PRIO_Q_LOW], &buf_handle, portMAX_DELAY);

			if (!ret) {
				ESP_LOGE(TAG, "%s: xQueueSend failed", __func__);
				free(copy);
				break;
			}
		}
		hw_q->read = ++r;
		ESP_LOGD(TAG, "%s: read_queue->read = %d write = %d", __func__, r, w);
	}
	return true;
}

static void esp_shmem_hw_queue_task(void *p)
{
	for (;;) {
		if (esp_shmem_hw_queue_read())
			xSemaphoreGive(read_semaphore);
		else
			xSemaphoreTake(hw_queue_read_semaphore, portMAX_DELAY);
	}
}

static int esp_shmem_read(interface_handle_t *if_handle, interface_buffer_handle_t *buf_handle)
{
	esp_err_t ret = ESP_OK;

	ESP_LOGD(TAG, "%s", __func__);
	if (!if_handle) {
		ESP_LOGE(TAG, "Invalid arguments to esp_spi_read\n");
		return ESP_FAIL;
	}

	while (1) {
		esp_shmem_hw_queue_read();

		if (uxQueueMessagesWaiting(shmem_rx_queue[PRIO_Q_HIGH])) {
			ret = xQueueReceive(shmem_rx_queue[PRIO_Q_HIGH], buf_handle, portMAX_DELAY);
			break;
		} else if (uxQueueMessagesWaiting(shmem_rx_queue[PRIO_Q_MID])) {
			ret = xQueueReceive(shmem_rx_queue[PRIO_Q_MID], buf_handle, portMAX_DELAY);
			break;
		} else if (uxQueueMessagesWaiting(shmem_rx_queue[PRIO_Q_LOW])) {
			ret = xQueueReceive(shmem_rx_queue[PRIO_Q_LOW], buf_handle, portMAX_DELAY);
			break;
		} else {
			xSemaphoreTake(read_semaphore, portMAX_DELAY);
		}
	}

	if (ret != pdTRUE) {
		return ESP_FAIL;
	}
	return buf_handle->payload_len;
}

static esp_err_t esp_shmem_reset(interface_handle_t *handle)
{
	ESP_LOGD(TAG, "%s", __func__);
	esp_err_t ret = ESP_OK;
	return ret;
}

esp_err_t send_bootup_event_to_host(uint8_t cap)
{
	struct esp_payload_header *header = NULL;
	struct esp_internal_bootup_event *event = NULL;
	struct fw_data * fw_p = NULL;
	interface_buffer_handle_t buf_handle = {0};
	uint8_t * pos = NULL;
	uint16_t len = 0;
	uint8_t raw_tp_cap = 0;
	raw_tp_cap = debug_get_raw_tp_conf();

	ESP_LOGD(TAG, "%s", __func__);

	buf_handle.payload = malloc(RX_BUF_SIZE);
	assert(buf_handle.payload);
	memset(buf_handle.payload, 0, RX_BUF_SIZE);

	header = (struct esp_payload_header *) buf_handle.payload;

	header->if_type = ESP_INTERNAL_IF;
	header->if_num = 0;
	header->offset = htole16(sizeof(struct esp_payload_header));

	event = (struct esp_internal_bootup_event*) (buf_handle.payload + sizeof(struct esp_payload_header));

	event->header.event_code = ESP_INTERNAL_BOOTUP_EVENT;
	event->header.status = 0;

	pos = event->data;

	/* TLVs start */

	/* TLV - Board type */
	*pos = ESP_BOOTUP_FIRMWARE_CHIP_ID;   pos++;len++;
	*pos = LENGTH_1_BYTE;                 pos++;len++;
	*pos = CONFIG_IDF_FIRMWARE_CHIP_ID;   pos++;len++;

	/* TLV - Capability */
	*pos = ESP_BOOTUP_CAPABILITY;         pos++;len++;
	*pos = LENGTH_1_BYTE;                 pos++;len++;
	*pos = cap;                           pos++;len++;

	*pos = ESP_BOOTUP_TEST_RAW_TP;        pos++;len++;
	*pos = LENGTH_1_BYTE;                 pos++;len++;
	*pos = raw_tp_cap;                    pos++;len++;

	/* TLV - FW data */
	*pos = ESP_BOOTUP_FW_DATA;            pos++; len++;
	*pos = sizeof(struct fw_data);        pos++; len++;
	fw_p = (struct fw_data *) pos;

	/* core0 sufficient now */
	fw_p->last_reset_reason = htole32(rtc_get_reset_reason(0));
	fw_p->version.major1 = PROJECT_VERSION_MAJOR_1;
	fw_p->version.major2 = PROJECT_VERSION_MAJOR_2;
	fw_p->version.minor  = PROJECT_VERSION_MINOR;
	pos += sizeof(struct fw_data);
	len += sizeof(struct fw_data);

	/* TLVs end */
	event->len = len;
	buf_handle.payload_len = len + sizeof(struct esp_internal_bootup_event) + sizeof(struct esp_payload_header);
	/*print_reset_reason(event->last_reset_reason);*/

	/* payload len = Event len + sizeof(event len) */
	len += 1;
	event->header.len = htole16(len);

	header->len = htole16(buf_handle.payload_len - sizeof(struct esp_payload_header));

	xQueueSend(shmem_tx_queue[PRIO_Q_HIGH], &buf_handle, portMAX_DELAY);

	esp_shmem_hw_queue_write(0);

	return ESP_OK;
}

static void esp_shmem_isr(void *p)
{
	esp_shmem_write_irq(ESP_SHMEM_IRQ_FROM_HOST_REG, 0);
	xSemaphoreGiveFromISR(hw_queue_read_semaphore, NULL);
}

static interface_handle_t * esp_shmem_init(void)
{
	int i;

	ESP_LOGD(TAG, "%s", __func__);

	hw_queue_read_semaphore = xSemaphoreCreateBinary();
	read_semaphore = xSemaphoreCreateBinary();
	for (i = 0; i < MAX_PRIORITY_QUEUES; ++i) {
		shmem_rx_queue[i] = xQueueCreate(SPI_RX_QUEUE_SIZE, sizeof(interface_buffer_handle_t));
		assert(shmem_rx_queue[i] != NULL);

		shmem_tx_queue[i] = xQueueCreate(SPI_TX_QUEUE_SIZE, sizeof(interface_buffer_handle_t));
		assert(shmem_tx_queue[i] != NULL);
	}
	shmem_tx_postprocess_queue = xQueueCreate(MAX_PRIORITY_QUEUES * SPI_TX_QUEUE_SIZE,
						  sizeof(interface_buffer_handle_t));
	assert(shmem_tx_postprocess_queue != NULL);

	for (i = 0; i < 2; ++i) {
		hw_queue[i].offset = (u32)(queue_data + i) - (u32)hw_queue;
		hw_queue[i].mask = QUEUE_SIZE - 1;
	}
	assert(xTaskCreate(esp_shmem_hw_queue_task , "hw_queue_task" ,
			   TASK_DEFAULT_STACK_SIZE , NULL , TASK_DEFAULT_PRIO, NULL) == pdTRUE);

	intr_matrix_set(0, ETS_FROM_CPU_INTR0_SOURCE +
			CONFIG_ESP_SHMEM_IRQ_TO_HOST_IDX, 6);
	esp_intr_alloc(ETS_FROM_CPU_INTR0_SOURCE +
		       CONFIG_ESP_SHMEM_IRQ_FROM_HOST_IDX,
		       ESP_INTR_FLAG_SHARED,
		       esp_shmem_isr, NULL, NULL);

	*(volatile void **)(0x600c0004) = hw_queue;

	return &if_handle_g;
}

static void esp_shmem_deinit(interface_handle_t *handle)
{
	ESP_LOGD(TAG, "%s", __func__);
}

if_ops_t if_ops = {
	.init = esp_shmem_init,
	.write = esp_shmem_write,
	.read = esp_shmem_read,
	.reset = esp_shmem_reset,
	.deinit = esp_shmem_deinit,
};

interface_context_t *interface_insert_driver(int (*event_handler)(uint8_t val))
{
	ESP_LOGD(TAG, "Using SHMEM interface");
	memset(&context, 0, sizeof(context));

	context.type = SHMEM;
	context.if_ops = &if_ops;
	context.event_handler = event_handler;

	return &context;
}

int interface_remove_driver()
{
	ESP_LOGD(TAG, "%s", __func__);
	memset(&context, 0, sizeof(context));
	return 0;
}
