#include "system/time_sync.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gptimer.h"
#include "string.h"
#include "esp_log.h"

#define TAG "TIME_SYNC"

static struct
{
	TaskHandle_t ticker_task;
	gptimer_handle_t ticker_timer;
	biodyn_timesync_err_t ext_err;
	char ext_err_msg[128];
	ts_ticker_t ticker;
	ts_ticker_t rtt;
} time_sync = {
	.ticker_task = NULL,
	.ticker_timer = NULL,
	.ext_err = BIODYN_TIMESYNC_OK,
	.ticker = 0,
	.rtt = 0,
};

// Tick task
static void tick(void *arg)
{
	for (;;)
	{
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Wait to be woken up
		ESP_LOGI(TAG, "Beeted");
		++time_sync.ticker;
	}
}

// ISR. keep lean and mean
static bool IRAM_ATTR timer_isr(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	vTaskNotifyGiveFromISR(time_sync.ticker_task, &xHigherPriorityTaskWoken);
	return xHigherPriorityTaskWoken == pdTRUE;
}
// Collect errors
static esp_err_t collect_err(biodyn_timesync_err_t err, const char *msg, esp_err_t code);

// Initializer
esp_err_t biodyn_time_sync_init()
{
	esp_err_t res = ESP_OK;
	time_sync.ext_err = BIODYN_TIMESYNC_OK;

	// Create the worker task
	res = xTaskCreate(tick, TAG, 4096, NULL, 5, &time_sync.ticker_task);
	if (res != pdPASS) {
		collect_err(BIODYN_TIMESYNC_COULDNT_CREATE_TASK, "Couldn't create task: code", res);
		return -1;
	}

	// Create hardware timer
	gptimer_config_t cfg = {
		.clk_src = GPTIMER_CLK_SRC_DEFAULT,
		.direction = GPTIMER_COUNT_UP,
		.resolution_hz = 1 * 1000 * 1000, // 1 MHz
	};
	res = gptimer_new_timer(&cfg, &time_sync.ticker_timer);
	if (res != ESP_OK)
		return collect_err(BIODYN_TIMESYNC_COULDNT_CREATE_TIMER, "Could not create gptimer: code", res);

	// Register ISR with timer
	gptimer_event_callbacks_t callbacks = {
		.on_alarm = timer_isr,
	};
	res = gptimer_register_event_callbacks(time_sync.ticker_timer, &callbacks, NULL);
	if (res != ESP_OK)
		return collect_err(BIODYN_TIMESYNC_COULDNT_ADD_ISR, "Could not register timer ISR: code", res);

	// Enable and start timer
	res = gptimer_enable(time_sync.ticker_timer);
	if (res != ESP_OK)
		return collect_err(BIODYN_TIMESYNC_COULDNT_ENABLE_TIMER, "Could not create gptimer: code", res);
	res = gptimer_start(time_sync.ticker_timer);
	if (res != ESP_OK)
		return collect_err(BIODYN_TIMESYNC_COULDNT_START_TIMER, "Could not start gptimer: code", res);

	return res;
}

ts_ticker_t biodyn_time_sync_get_ticker()
{
	return time_sync.ticker;
}

esp_err_t biodyn_time_sync_self_test()
{
	esp_err_t err = ESP_OK;
	
	uint64_t ticker;
	err = gptimer_get_raw_count(time_sync.ticker_timer, &ticker);
	if (err != ESP_OK)
		return collect_err(BIODYN_TIMESYNC_COUDLNT_READ_TIMER, "Couldn't read raw timer value: code ", err);
	ESP_LOGI(TAG, "Ticker info: raw was %lld, stored was %ld", ticker, time_sync.ticker);

	ts_ticker_t before = time_sync.ticker;
	vTaskDelay(pdMS_TO_TICKS(100));
	ts_ticker_t after = time_sync.ticker;
	if (before == after) {
		collect_err(BIODYN_TIMESYNC_TIMER_NOT_INCREASING, "Timesync ticker did not increase, got", after);
		return -1;
	}

	ESP_LOGI(TAG, "Self-test OK");

	return ESP_OK;
}

bool time_sync_has_error()
{
	return time_sync.ext_err != BIODYN_TIMESYNC_OK;
}

const char *time_sync_get_error()
{
	if (time_sync.ext_err != BIODYN_TIMESYNC_OK)
		return &time_sync.ext_err_msg[0];
	return "No errors";
}

static esp_err_t collect_err(biodyn_timesync_err_t err, const char *msg, esp_err_t code)
{
	time_sync.ext_err |= err;
	snprintf(time_sync.ext_err_msg, sizeof(time_sync.ext_err_msg), "%s %x", msg, code);
	ESP_LOGE(TAG, "%s %x", msg, code);
	return code;
}

void ble_time_sync_ticker_read(uint16_t *size, void *out)
{
	*size = sizeof(time_sync.ticker);
	memcpy(out, &time_sync.ticker, *size);
	ESP_LOGI(TAG, "Read ticker as %ld", time_sync.ticker);
}

void ble_time_sync_rtt_read(uint16_t *size, void *out)
{
	*size = sizeof(time_sync.rtt);
	memcpy(out, &time_sync.rtt, *size);
	ESP_LOGI(TAG, "Read RTT as %ld", time_sync.rtt);
}