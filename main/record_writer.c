#include "record_writer.h"
#include <dirent.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include "audio_dsp.h"
#include "calibration.h"
#include "constants.h"
#include "esp_littlefs.h"
#include "esp_log.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "logmessage.pb.h"
#include "nanopb/pb_encode.h"
#include "power_management.h"

#define HEADROOM_BYTES (200 * 1024)
// 60 records → ~1 minute per file → ~3.5 KB LogMessage staging buffer.
// Was 300 (5 min, ~17 KB); the larger struct couldn't be heap-allocated
// after WiFi + BLE + MQTT had fragmented the heap. The 200 KB partition
// headroom check still gates ring-buffer eviction the same way; we just
// rotate files 5× more often.
#define RECORDS_PER_FILE 60

static const char* TAG = "record_writer";
static const char* ALPHABET = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";

// Heap-allocated lazily on first record arrival (see record_writer task body).
// The struct is ~16 KB and was previously static .bss — moving it to heap
// frees that DRAM at boot so BLE controller init has room. Allocated once,
// never freed — same fragmentation profile as static (spec §7 intent).
static LogMessage* log_message_buf = NULL;
static int records_count = 0;

static bool write_pb_to_file(pb_ostream_t* stream, const uint8_t* buffer, size_t count) {
  FILE* file = (FILE*)stream->state;
  size_t written = fwrite(buffer, 1, count, file);
  return written == count;
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

static void copy_record(Record* dst, const record_t* src) {
  dst->seq_no = src->seq_no;
  memcpy(dst->bands, src->bands, sizeof(src->bands));
  dst->laeq_1s   = src->laeq_1s;
  dst->lceq_1s   = src->lceq_1s;
  dst->lafmax_1s = src->lafmax_1s;
  dst->lcfmax_1s = src->lcfmax_1s;
  dst->lcpeak_1s = src->lcpeak_1s;
}

static void flush_to_file(void) {
  strlcpy(log_message_buf->device_id, DEVICE_ID, sizeof(log_message_buf->device_id));
  generate_client_id(log_message_buf->client_id, sizeof(log_message_buf->client_id));
  log_message_buf->device_time = (int32_t)time(NULL);
  log_message_buf->device_time_is_utc = true;
  log_message_buf->has_battery_voltage = true;
  log_message_buf->battery_voltage = battery_voltage;
  log_message_buf->has_usb_voltage = true;
  log_message_buf->usb_voltage = usb_voltage;
  log_message_buf->has_noise_recording = true;
  log_message_buf->noise_recording.calibration_offset_db_x100 = calibration_offset_x100();
  log_message_buf->noise_recording.records_count = records_count;

  ensure_free_space();

  char filename[256];
  snprintf(
      filename,
      sizeof(filename),
      "%s/%010ld_%s.log",
      LOG_DIR,
      (long)log_message_buf->device_time,
      log_message_buf->client_id
  );
  ESP_LOGI(TAG, "Writing log file %s (%d records)", filename, records_count);

  FILE* file = fopen(filename, "w");
  if (file == NULL) {
    ESP_LOGE(TAG, "Failed to open %s for writing", filename);
    records_count = 0;
    return;
  }

  pb_ostream_t stream = {
      .callback = write_pb_to_file,
      .state = file,
      .max_size = SIZE_MAX,
      .bytes_written = 0,
  };

  if (!pb_encode(&stream, LogMessage_fields, log_message_buf)) {
    ESP_LOGE(TAG, "Failed to encode log: %s", stream.errmsg);
  }
  fclose(file);

  xTaskNotify(xTaskGetHandle(LOG_UPLOADER_TASK), 0, eIncrement);

  records_count = 0;
}

void record_writer(void* params) {
  maybe_create_log_dir();

  while (true) {
    record_t r;
    if (xQueueReceive(record_writer_queue, &r, portMAX_DELAY) != pdTRUE) continue;

    // First-record-arrived path: do the one-time heap alloc of the LogMessage
    // staging buffer. By now WiFi/BLE init are long done — this avoids tying
    // up ~16 KB of DRAM at boot when the controllers most need contiguous
    // memory.
    if (log_message_buf == NULL) {
      log_message_buf = calloc(1, sizeof(LogMessage));
      if (log_message_buf == NULL) {
        ESP_LOGE(TAG, "failed to allocate LogMessage buffer (%u bytes); dropping records",
                 (unsigned)sizeof(LogMessage));
        // Drain queue forever to keep DSP non-blocking sends successful.
        continue;
      }
      ESP_LOGI(TAG, "LogMessage staging buffer allocated (%u bytes)",
               (unsigned)sizeof(LogMessage));
    }

    copy_record(&log_message_buf->noise_recording.records[records_count], &r);
    records_count++;
    if (records_count >= RECORDS_PER_FILE) {
      flush_to_file();
    }
  }
}
