#include "record_writer.h"
#include <dirent.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include "audio_dsp.h"
#include "constants.h"
#include "esp_littlefs.h"
#include "esp_log.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "logmessage.pb.h"
#include "pb_encode.h"
#include "power_management.h"

#define HEADROOM_BYTES (200 * 1024)

static const char* TAG = "record_writer";
static const char* ALPHABET = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";

// Static BSS staging for the one-record LogMessage. Tiny now that
// NoiseRecording is flattened (no records[] array); kept in BSS to stay out of
// the heap pool. Fields are rewritten in flush_to_file() on every write.
static LogMessage log_message_buf;
static bool write_pb_to_file(pb_ostream_t* stream, const uint8_t* buf, size_t count) {
  return fwrite(buf, 1, count, (FILE*)stream->state) == count;
}

static void maybe_create_log_dir(void) {
  DIR* dir = opendir(LOG_DIR);
  if (dir == NULL) {
    if (mkdir(LOG_DIR, 0777) != 0) {
      ESP_LOGE(TAG, "Failed to create directory %s", LOG_DIR);
    }
    return;
  }
  closedir(dir);
}

static char* find_oldest_file(const char* dir_path) {
  DIR* dir = opendir(dir_path);
  if (!dir) return NULL;

  char oldest[FILENAME_MAX] = {0};
  struct dirent* entry;
  while ((entry = readdir(dir))) {
    if (entry->d_type != DT_REG) continue;
    if (oldest[0] == '\0' || strcmp(entry->d_name, oldest) < 0) {
      strncpy(oldest, entry->d_name, sizeof(oldest) - 1);
    }
  }
  closedir(dir);

  return oldest[0] ? strdup(oldest) : NULL;
}

static void ensure_free_space(void) {
  while (true) {
    size_t total = 0, used = 0;
    if (esp_littlefs_info("littlefs", &total, &used) != ESP_OK) {
      ESP_LOGE(TAG, "esp_littlefs_info failed");
      return;
    }
    if (total - used >= HEADROOM_BYTES) return;

    char* oldest = find_oldest_file(LOG_DIR);
    if (oldest == NULL) {
      ESP_LOGE(TAG, "out of space and no files to delete");
      return;
    }

    char path[256];
    snprintf(path, sizeof(path), "%s/%s", LOG_DIR, oldest);
    ESP_LOGW(TAG, "low storage, deleting oldest file: %s", oldest);
    remove(path);
    free(oldest);
  }
}

static void generate_client_id(char* dst, size_t dst_size) {
  size_t n = dst_size - 1;
  size_t alphabet_len = strlen(ALPHABET);
  for (size_t i = 0; i < n; i++) {
    dst[i] = ALPHABET[esp_random() % alphabet_len];
  }
  dst[n] = '\0';
}

static void flush_to_file(time_t window_start) {
  strlcpy(log_message_buf.device_id, DEVICE_ID, sizeof(log_message_buf.device_id));
  generate_client_id(log_message_buf.client_id, sizeof(log_message_buf.client_id));
  // device_time is the measurement window's start, captured by the producer
  // (audio_dsp) — robust to any queue latency between produce and flush.
  log_message_buf.device_time = (int32_t)window_start;
  log_message_buf.device_time_is_utc = true;
  log_message_buf.has_battery_voltage = true;
  log_message_buf.battery_voltage = battery_voltage;
  log_message_buf.has_usb_voltage = true;
  log_message_buf.usb_voltage = usb_voltage;
  log_message_buf.has_noise_recording = true;
  log_message_buf.noise_recording.record_interval_seconds = RECORD_INTERVAL_SECONDS;
  record_apply_aggregates(&log_message_buf.noise_recording);

  char filename[256];
  snprintf(
      filename,
      sizeof(filename),
      "%s/%010ld_%s.log",
      LOG_DIR,
      (long)log_message_buf.device_time,
      log_message_buf.client_id
  );
  ESP_LOGI(TAG, "Writing log file %s", filename);

  ensure_free_space();

  FILE* file = fopen(filename, "w");
  if (file == NULL) {
    ESP_LOGE(TAG, "Failed to open %s for writing", filename);
    return;
  }

  pb_ostream_t stream = {
      .callback = write_pb_to_file,
      .state = file,
      .max_size = SIZE_MAX,
      .bytes_written = 0,
  };
  bool ok = pb_encode(&stream, LogMessage_fields, &log_message_buf);
  if (!ok) ESP_LOGE(TAG, "Failed to encode log: %s", stream.errmsg);
  fclose(file);

  // Nudge the uploader to pick up the new file. Look it up by name rather than
  // holding a handle; it may be absent or have exited, in which case
  // xTaskNotify(NULL) would assert.
  if (ok) {
    TaskHandle_t uploader = xTaskGetHandle(LOG_UPLOADER_TASK);
    if (uploader) xTaskNotify(uploader, 0, eIncrement);
  }
}

void record_writer(void* params) {
  maybe_create_log_dir();

  while (true) {
    record_t r;
    if (xQueueReceive(record_writer_queue, &r, portMAX_DELAY) != pdTRUE) continue;

    // One aggregate record per file, flushed immediately for power-loss
    // durability (see RECORD_INTERVAL_SECONDS in constants.h).
    record_to_pb(&r, &log_message_buf.noise_recording);
    flush_to_file(r.window_start);
  }
}
