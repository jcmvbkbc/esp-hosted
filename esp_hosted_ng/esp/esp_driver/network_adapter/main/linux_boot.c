/* Linux boot Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_system.h"
#include "esp_partition.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const void * IRAM_ATTR map_partition(const char *name)
{
	const void *ptr;
	spi_flash_mmap_handle_t handle;
	esp_partition_iterator_t it;
	const esp_partition_t *part;

	it = esp_partition_find(ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, name);
	part = esp_partition_get(it);
	if (esp_partition_mmap(part, 0, part->size, SPI_FLASH_MMAP_INST, &ptr, &handle) != ESP_OK)
		abort();
	return ptr;
}

static void IRAM_ATTR map_flash_and_go(void)
{
	const void *ptr0, *ptr;

	ptr0 = map_partition("linux");
	printf("ptr = %p\n", ptr0);

	ptr = map_partition("rootfs");
	printf("ptr = %p\n", ptr);

	extern int g_abort_on_ipc;
	g_abort_on_ipc = 1;

	asm volatile ("jx %0" :: "r"(ptr0));
}

static void linux_task(void *p)
{
	map_flash_and_go();
	esp_restart();
}

void linux_boot(void)
{
	xTaskCreatePinnedToCore(linux_task, "linux_task", 4096, NULL, 5, NULL, 1);
}

