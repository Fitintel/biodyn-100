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
	ts_ticker_t rtt;
} time_sync = {
	.ticker_task = NULL,
	.ticker_timer = NULL,
	.ext_err = BIODYN_TIMESYNC_OK,
	.rtt = 0,
};

// Collect errors
static esp_err_t collect_err(biodyn_timesync_err_t err, const char *msg, esp_err_t code);

// Initializer
esp_err_t biodyn_time_sync_init()
{
	esp_err_t res = ESP_OK;
	time_sync.ext_err = BIODYN_TIMESYNC_OK;

	// Create hardware timer
	gptimer_config_t cfg = {
		.clk_src = GPTIMER_CLK_SRC_DEFAULT,
		.direction = GPTIMER_COUNT_UP,
		.resolution_hz = 1 * 1000 * 1000, // 1 MHz
	};
	res = gptimer_new_timer(&cfg, &time_sync.ticker_timer);
	if (res != ESP_OK)
		return collect_err(BIODYN_TIMESYNC_COULDNT_CREATE_TIMER, "Could not create gptimer: code", res);

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
	uint64_t ticker;
	gptimer_get_raw_count(time_sync.ticker_timer, &ticker);
	return ticker;
}

esp_err_t biodyn_time_sync_self_test()
{
	esp_err_t err = ESP_OK;

	uint64_t ticker;
	err = gptimer_get_raw_count(time_sync.ticker_timer, &ticker);
	if (err != ESP_OK)
		return collect_err(BIODYN_TIMESYNC_COUDLNT_READ_TIMER, "Couldn't read raw timer value: code ", err);
	ESP_LOGI(TAG, "Ticker info: raw was %lld", ticker);

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
	ts_ticker_t ticker = biodyn_time_sync_get_ticker();
	*size = sizeof(ticker);
	memcpy(out, &ticker, *size);
	ESP_LOGI(TAG, "Read ticker as %lld", ticker);
}

void ble_time_sync_rtt_read(uint16_t *size, void *out)
{
	*size = sizeof(time_sync.rtt);
	memcpy(out, &time_sync.rtt, *size);
	ESP_LOGI(TAG, "Read RTT as %lld", time_sync.rtt);
}

void ble_time_sync_ticker_write(uint16_t size, void *src)
{
	if (size < sizeof(ts_ticker_t))
	{
		collect_err(BIODYN_TIMESYNC_TOO_LITTLE_DATA_WRIT, "Not enough data, got %d", size);
		return;
	}

	uint64_t ticker = 0;
	memcpy(&ticker, src, sizeof(ts_ticker_t));
	ticker += time_sync.rtt >> 1;
	esp_err_t res = gptimer_set_raw_count(time_sync.ticker_timer, ticker);
	if (res != ESP_OK)
	{
		collect_err(BIODYN_TIMESYNC_COUDLNT_WRITE_TIMER, "Couldn't write ticker: code", res);
		return;
	}
	ESP_LOGI(TAG, "Wrote new ticker: %lld", ticker);
}

void ble_time_sync_rtt_write(uint16_t size, void *src)
{
	if (size < sizeof(ts_ticker_t))
	{
		collect_err(BIODYN_TIMESYNC_TOO_LITTLE_DATA_WRIT, "Not enough data, got %d", size);
		return;
	}

	memcpy(&time_sync.rtt, src, sizeof(ts_ticker_t));
	ESP_LOGI(TAG, "Wrote new RTT: %lld", time_sync.rtt);
}

double ts_ticker_t_to_ms(ts_ticker_t t)
{
	return (double)t / 1000.0;
}

ts_ticker_t ms_to_ts_ticker_t(double ms)
{
	return (ts_ticker_t)(ms * 1000.0);
}
