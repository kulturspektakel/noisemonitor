#include "log_uploader.h"
#include <dirent.h>
#include "constants.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "event_group.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "http_auth_headers.h"
#include "network_request.h"

#define MAX_ERRORS 3

// New per-minute log files are a single aggregated record (~150 bytes).
// Pre-aggregation files held up to 300 per-second records (~14.6 KB) and can't
// be buffered for upload under WiFi+BLE memory pressure. Anything at/above this
// is a legacy file — delete it instead of retrying the (failing) big alloc
// forever. Doubles as an upper bound on the upload malloc below.
#define LEGACY_LOG_SIZE_THRESHOLD 1024

int log_files_to_upload = 0;

static int update_log_count() {
  DIR* dir = opendir(LOG_DIR);

  if (dir == NULL) {
    ESP_LOGE(LOG_UPLOADER_TASK, "Failed to open directory");
    return 0;
  }

  struct dirent* entry;
  int count = 0;
  while ((entry = readdir(dir))) {
    if (entry->d_type == DT_REG) {
      count++;
    }
  }
  closedir(dir);
  return count;
}

typedef enum {
  FILE_HANDLED,
  FILE_SKIPPED,
  HTTP_ISSUE,
} log_uploader_event_t;

static log_uploader_event_t upload_file(char* filename) {
  ESP_LOGI(LOG_UPLOADER_TASK, "Uploading %s", filename);
  FILE* f = fopen(filename, "r");

  if (f == NULL) {
    ESP_LOGE(LOG_UPLOADER_TASK, "Failed to open %s for reading", filename);
    return FILE_SKIPPED;
  }

  esp_http_client_config_t config = {
      .host = API_HOST,
      .transport_type = HTTP_TRANSPORT_OVER_SSL,
      .path = "/api/noise/log",
      .method = HTTP_METHOD_POST,
  };

  fseek(f, 0, SEEK_END);
  int file_size = ftell(f);
  if (file_size < 1) {
    ESP_LOGE(LOG_UPLOADER_TASK, "File %s is corrupt. Deleting it.", filename);
    fclose(f);
    remove(filename);
    return FILE_HANDLED;
  }
  if (file_size >= LEGACY_LOG_SIZE_THRESHOLD) {
    ESP_LOGW(LOG_UPLOADER_TASK,
             "%s is %d bytes — pre-aggregation legacy file, deleting (not uploaded)",
             filename, file_size);
    fclose(f);
    return FILE_HANDLED;  // caller removes it and decrements the count
  }
  fseek(f, 0, SEEK_SET);
  char* buffer = pvPortMalloc(file_size);
  if (buffer == NULL) {
    ESP_LOGW(LOG_UPLOADER_TASK,
             "out of heap allocating %d bytes for %s; will retry later",
             file_size, filename);
    fclose(f);
    return HTTP_ISSUE;
  }
  size_t n = fread(buffer, 1, (size_t)file_size, f);
  fclose(f);
  if (n != (size_t)file_size) {
    ESP_LOGW(LOG_UPLOADER_TASK,
             "short read on %s (got %zu of %d); will retry later",
             filename, n, file_size);
    vPortFree(buffer);
    return HTTP_ISSUE;
  }

  esp_http_client_handle_t client = esp_http_client_init(&config);
  http_auth_headers(client);
  esp_http_client_set_post_field(client, buffer, file_size);
  esp_err_t err = esp_http_client_perform(client);
  vPortFree(buffer);
  buffer = NULL;

  if (err != ESP_OK) {
    ESP_LOGE(LOG_UPLOADER_TASK, "Request failed (Error: %s)", esp_err_to_name(err));
    esp_http_client_cleanup(client);
    return HTTP_ISSUE;
  }

  int status_code = esp_http_client_get_status_code(client);
  esp_http_client_cleanup(client);

  if (status_code == 201 || status_code == 409) {
    ESP_LOGI(
        LOG_UPLOADER_TASK,
        "Upload successful (HTTP %d), deleting log file %s",
        status_code,
        filename
    );
    return FILE_HANDLED;
  } else if (status_code == 400) {
    ESP_LOGE(LOG_UPLOADER_TASK, "Bad request (HTTP 400), deleting log file %s", filename);
    return FILE_HANDLED;
  } else {
    ESP_LOGE(LOG_UPLOADER_TASK, "Server error (HTTP %d), skipping file %s", status_code, filename);
    return FILE_SKIPPED;
  }
}

void maybe_create_log_dir() {
  DIR* dir = opendir(LOG_DIR);
  if (dir == NULL) {
    // create directory if it doesn't exist
    ESP_LOGI(LOG_UPLOADER_TASK, "Creating directory %s", LOG_DIR);
    if (mkdir(LOG_DIR, 0777) != 0) {
      ESP_LOGE(LOG_UPLOADER_TASK, "Failed to create directory");
      vTaskDelete(NULL);
      return;
    }
  }
  closedir(dir);
}

static void retry_upload(TimerHandle_t xTimer) {
  xTaskNotify(xTaskGetHandle(LOG_UPLOADER_TASK), 0, eNoAction);
}

void log_uploader(void* params) {
  TimerHandle_t retry_timer = NULL;
  maybe_create_log_dir();

  // initial value
  log_files_to_upload = update_log_count();
  if (log_files_to_upload > 0) {
    xTaskNotify(xTaskGetHandle(LOG_UPLOADER_TASK), 0, eNoAction);
  }
  ESP_LOGI(LOG_UPLOADER_TASK, "Found %d logs", log_files_to_upload);

  while (1) {
    uint32_t increment = 0;
    xTaskNotifyWait(0, ULONG_MAX, &increment, portMAX_DELAY);
    log_files_to_upload += increment;

    // Gate on MQTT being up, not just WiFi: at boot MQTT's TLS handshake
    // grabs a contiguous mbedtls IN+OUT buffer pair (~8 KB), and if we
    // start our own HTTPS handshake at the same instant the second
    // mbedtls_ssl_setup hits MBEDTLS_ERR_SSL_ALLOC_FAILED. Once MQTT is
    // connected it holds those buffers steadily; the heap shape is
    // predictable and our handshake finds the contiguous chunk it needs.
    bool ready = (xEventGroupGetBits(event_group) & (WIFI_CONNECTED | MQTT_CONNECTED))
                 == (WIFI_CONNECTED | MQTT_CONNECTED);
    if (log_files_to_upload == 0 || !ready) {
      continue;
    }

    int error_count = 0;
    DIR* dir = opendir(LOG_DIR);
    struct dirent* entry;
    while ((entry = readdir(dir))) {
      if (entry->d_type != DT_REG) {
        // skip non-regular files (e.g. directories)
        continue;
      }

      char filename[320];
      snprintf(filename, sizeof(filename), "%s/%s", LOG_DIR, entry->d_name);
      ESP_LOGI(LOG_UPLOADER_TASK, "Found log file %s", filename);
      xSemaphoreTake(network_request, portMAX_DELAY);
      log_uploader_event_t status = upload_file(filename);
      xSemaphoreGive(network_request);

      switch (status) {
        case FILE_SKIPPED:
          error_count++;
          break;
        case FILE_HANDLED:
          if (remove(filename) == 0) {
            log_files_to_upload--;
            if (log_files_to_upload < 0) {
              log_files_to_upload = 0;
            }
          }
          break;
        case HTTP_ISSUE:
          goto end;
          break;
      }

      if (error_count >= MAX_ERRORS) {
        ESP_LOGE(LOG_UPLOADER_TASK, "Stopping log uploader after %d error(s)", error_count);
        goto end;
      }
    }
  end:
    closedir(dir);

    if (log_files_to_upload > 0) {
      log_files_to_upload = update_log_count();
    }

    if (log_files_to_upload > 0) {
      // 30 s retry. The boot-time SSL handshake race (heap still
      // settling from MQTT init, mbedtls_ssl_setup hits ALLOC_FAILED)
      // resolves within seconds, so 5 min was overkill and made the
      // backlog drain at ~one-file-per-5-min worst case. With 30 s
      // a backlog of N files drains in about N×(upload_time+30 s)
      // assuming subsequent uploads succeed once heap is stable.
      if (retry_timer == NULL) {
        retry_timer =
            xTimerCreate("retry_timer", pdMS_TO_TICKS(30000), pdFALSE, NULL, retry_upload);
      }
      ESP_LOGI(
          LOG_UPLOADER_TASK,
          "Encountered %d errors while uploading. Scheduling retry in 30 s",
          error_count
      );
      xTimerReset(retry_timer, 0);
    }
  }
}
