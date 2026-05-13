#include "heap_diag.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

void heap_diag(const char* label) {
  // IRAM-as-heap (MALLOC_CAP_IRAM_8BIT) is always 0 on ESP32-S3 without
  // ESP32-original-only configs we can't enable here, so we omit it.
  ESP_LOGI(
      "heap",
      "%s: INT free=%u largest=%u | DMA free=%u largest=%u",
      label,
      (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
      (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
      (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
      (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA)
  );
}
