#include "system/time_sync.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gptimer.h"
#include "esp_log.h"

#define TAG "TimeSync"

static struct
{
	TaskHandle_t ticker_task;
	gptimer_handle_t ticker_timer;
	biodyn_timesync_err_t ext_err;
	char ext_err_msg[128];
	uint32_t ticker;
} time_sync = {
	.ticker_task = NULL,
	.ticker_timer = NULL,
	.ext_err = BIODYN_TIMESYNC_OK,
	.ticker = 0,
};

// Tick task
static void tick(void *arg)
{
	for (;;)
	{
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Wait to be woken up
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
	if (res != ESP_OK)
		return collect_err(BIODYN_TIMESYNC_COULDNT_CREATE_TASK, "Could not create xTask", res);

	// Create hardware timer
	gptimer_config_t cfg = {
		.clk_src = GPTIMER_CLK_SRC_DEFAULT,
		.direction = GPTIMER_COUNT_UP,
		.resolution_hz = 1 * 1000 * 1000, // 1 MHz
	};
	res = gptimer_new_timer(&cfg, &time_sync.ticker_timer);
	if (res != ESP_OK)
		return collect_err(BIODYN_TIMESYNC_COULDNT_CREATE_TIMER, "Could not create gptimer", res);

	// Register ISR with timer
	gptimer_event_callbacks_t callbacks = {
		.on_alarm = timer_isr,
	};
	res = gptimer_register_event_callbacks(time_sync.ticker_timer, &callbacks, NULL);
	if (res != ESP_OK)
		return collect_err(BIODYN_TIMESYNC_COULDNT_ADD_ISR, "Could not register timer ISR", res);

	// Enable and start timer
	res = gptimer_enable(time_sync.ticker_timer);
	if (res != ESP_OK)
		return collect_err(BIODYN_TIMESYNC_COULDNT_ENABLE_TIMER, "Could not create gptimer", res);
	res = gptimer_start(time_sync.ticker_timer);
	if (res != ESP_OK)
		return collect_err(BIODYN_TIMESYNC_COULDNT_START_TIMER, "Could not start gptimer", res);

	return res;
}

uint32_t biodyn_time_sync_get_ticker()
{
	return 0;
}

esp_err_t biodyn_time_sync_self_test()
{
	return ESP_OK;
}

bool time_sync_has_error()
{
	return time_sync.ext_err == BIODYN_TIMESYNC_OK;
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
	snprintf(time_sync.ext_err_msg, sizeof(time_sync.ext_err_msg), "%s: code %x", msg, code);
	ESP_LOGE(TAG, "%s: code %x", msg, code);
	return code;
}