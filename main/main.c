#include <dirent.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/sdmmc_host.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "bsp/esp-bsp.h"
#include "bsp/display.h"
#include "esp_lvgl_port_touch.h"
#include "esp_codec_dev.h"
#include "esp_audio_dec_default.h"
#include "esp_audio_simple_dec.h"
#include "esp_audio_simple_dec_default.h"
#include "lvgl.h"
#include "draw/lv_image_decoder_private.h"

// Core slideshow limits and timing.
#define MAX_IMAGES 256                         // Maximum number of image entries to index from SD root.
#define IMAGE_CHANGE_MS 5000                  // Auto-advance interval between slides (ms).

// LVGL filesystem integration and task sizing.
#define LVGL_FS_DRIVE_LETTER 'S'              // LVGL drive letter mapped to SD mount.
#define SLIDESHOW_TASK_STACK_SIZE 24576       // Stack size (bytes) for slideshow/LVGL orchestration task.

// Image format and SD diagnostics.
#define ENABLE_PNG_IN_SLIDESHOW 1             // 1: include PNG files when scanning supported image types.
#define SD_READ_TEST_MODE 0                   // 1: bypass slideshow UI and run SD read stress loop.
#define SD_READ_CHUNK_SIZE 4096               // Chunk size (bytes) used by SD read stress mode.
#define SD_MOUNT_RETRY_DELAY_MS 1000          // Delay between SD mount retries (ms).

// Preload policy (RAM/PSRAM vs file-backed decode).
#define PRELOAD_MAX_FILE_BYTES (180 * 1024)   // In hybrid mode, images above this threshold stay file-backed.
#define SLIDESHOW_VERBOSE_LOGS 0              // 1: emit per-file preload/memory diagnostics.
#define STRICT_RAM_PRELOAD_MODE 1             // 1: require all images to preload; fail startup otherwise.

// Display geometry used for fill-scale calculations.
#define DISPLAY_DIAMETER 466                  // Circular panel diameter in pixels.
#define DISPLAY_RADIUS (DISPLAY_DIAMETER / 2) // Derived radius in pixels.

// Audio preload, touch debounce, and MP3 decode buffers.
#define MAX_MP3_PRELOAD_BYTES (5 * 1024 * 1024) // Max MP3 size accepted for preload per image.
#define TOUCH_DEBOUNCE_MS 300                 // Minimum interval between accepted touch events.
#define MP3_DEC_IN_CHUNK 1024                 // MP3 input chunk fed into decoder per process step.
#define MP3_DEC_OUT_CHUNK 4096                // Initial PCM output buffer size for decoder output.
#define MP3_DEC_MAX_OUT_CHUNK (32 * 1024)     // Hard cap for decoder-requested output buffer growth.

// Audio task/runtime settings.
#define AUDIO_PLAYBACK_TASK_STACK_SIZE 16384  // Stack size (bytes) for asynchronous audio playback task.
#define AUDIO_OUTPUT_VOLUME_PERCENT 85        // Speaker output gain percentage sent to codec.

static const char *TAG = "SLIDESHOW";

typedef struct {
    char *file_name;
    const void *src;
    uint8_t *file_data;
    size_t file_size;
    lv_image_dsc_t image_dsc;
    uint16_t img_w;
    uint16_t img_h;
    uint32_t display_scale;
    lv_color_t backdrop_color;
    // Associated MP3 file (optional, same basename as image)
    uint8_t *mp3_data;
    size_t mp3_size;
} image_entry_t;

static lv_obj_t *s_screen_obj; // Root screen object used for backdrop color updates.
static lv_obj_t *s_image_objs[2]; // Double-buffered LVGL image widgets for flip-style swaps.
static uint8_t s_active_image_obj_idx = 0; // Index of currently visible image object.
static lv_obj_t *s_status_label; // Footer label showing index/name/decode status.
static image_entry_t s_images[MAX_IMAGES];
static size_t s_image_count;
static size_t s_current_index;
static sdmmc_card_t *s_sdcard; // Non-NULL while SD is mounted through VFS FAT.
static esp_codec_dev_handle_t s_speaker_codec_dev; // BSP speaker codec handle.
static bool s_audio_initialized = false;
static bool s_audio_decoders_registered = false;
static bool s_audio_playing = false;
static TickType_t s_last_touch_tick = 0;
static TaskHandle_t s_audio_task_handle = NULL; // Worker task that performs decode/write off UI callback.
static size_t s_pending_audio_index = SIZE_MAX; // Slide index queued for next touch-triggered playback.

/**
 * Mount the SD card using stable 1-bit SDMMC settings for this board.
 *
 * @return ESP_OK on success, otherwise an ESP-IDF error from mount call.
 */
static esp_err_t mount_sdcard_stable(void)
{
    const esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024,
    };

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.max_freq_khz = SDMMC_FREQ_PROBING;

    const sdmmc_slot_config_t slot_config = {
        .clk = BSP_SD_CLK,
        .cmd = BSP_SD_CMD,
        .d0 = BSP_SD_D0,
        .d1 = GPIO_NUM_NC,
        .d2 = GPIO_NUM_NC,
        .d3 = GPIO_NUM_NC,
        .d4 = GPIO_NUM_NC,
        .d5 = GPIO_NUM_NC,
        .d6 = GPIO_NUM_NC,
        .d7 = GPIO_NUM_NC,
        .cd = SDMMC_SLOT_NO_CD,
        .wp = SDMMC_SLOT_NO_WP,
        .width = 1,
        .flags = 0,
    };

    ESP_LOGI(TAG, "Mounting SD card in 1-bit mode at %d KHz", host.max_freq_khz);
    return esp_vfs_fat_sdmmc_mount(BSP_SD_MOUNT_POINT, &host, &slot_config, &mount_config, &s_sdcard);
}

/**
 * Retry SD mounting until it succeeds.
 *
 * @return Always ESP_OK after successful mount.
 */
static esp_err_t mount_sdcard_with_retries(void)
{
    esp_err_t mount_err = ESP_FAIL;
    // Keep retrying forever so a late-inserted SD card still recovers the app.
    while (mount_err != ESP_OK) {
        mount_err = mount_sdcard_stable();
        if (mount_err != ESP_OK) {
            ESP_LOGE(TAG, "SD mount failed (%s), retrying in %u ms",
                     esp_err_to_name(mount_err),
                     (unsigned)SD_MOUNT_RETRY_DELAY_MS);
            vTaskDelay(pdMS_TO_TICKS(SD_MOUNT_RETRY_DELAY_MS));
        }
    }
    return ESP_OK;
}

/**
 * Parse JPEG bytes and extract width/height from SOF markers.
 *
 * @param data JPEG file bytes.
 * @param size Number of bytes in data.
 * @param out_w Output image width in pixels.
 * @param out_h Output image height in pixels.
 * @return true if dimensions were found and valid; false otherwise.
 */
static bool jpeg_get_dimensions(const uint8_t *data, size_t size, uint16_t *out_w, uint16_t *out_h)
{
    if (data == NULL || size < 4 || out_w == NULL || out_h == NULL) {
        return false;
    }

    if (data[0] != 0xFF || data[1] != 0xD8) {
        return false;
    }

    size_t pos = 2;
    // Walk JPEG segments until we find a SOF marker that carries width/height.
    while (pos + 3 < size) {
        if (data[pos] != 0xFF) {
            pos++;
            continue;
        }

        while (pos < size && data[pos] == 0xFF) {
            pos++;
        }
        if (pos >= size) {
            return false;
        }

        uint8_t marker = data[pos++];
        if (marker == 0xD9 || marker == 0xDA) {
            break;
        }

        if (pos + 1 >= size) {
            return false;
        }

        uint16_t segment_len = ((uint16_t)data[pos] << 8) | data[pos + 1];
        if (segment_len < 2 || pos + segment_len > size) {
            return false;
        }

        bool is_sof = (marker >= 0xC0 && marker <= 0xC3) ||
                      (marker >= 0xC5 && marker <= 0xC7) ||
                      (marker >= 0xC9 && marker <= 0xCB) ||
                      (marker >= 0xCD && marker <= 0xCF);
        if (is_sof && segment_len >= 7) {
            *out_h = ((uint16_t)data[pos + 3] << 8) | data[pos + 4];
            *out_w = ((uint16_t)data[pos + 5] << 8) | data[pos + 6];
            return (*out_w > 0 && *out_h > 0);
        }

        pos += segment_len;
    }

    return false;
}

/**
 * Check whether a filename extension is one of the supported image types.
 *
 * @param name File name to inspect.
 * @return true if extension is supported by slideshow scan logic.
 */
static bool has_supported_ext(const char *name)
{
    const char *dot = strrchr(name, '.');
    if (dot == NULL) {
        return false;
    }

    return strcasecmp(dot, ".jpg") == 0 ||
           strcasecmp(dot, ".jpeg") == 0 ||
#if ENABLE_PNG_IN_SLIDESHOW
           strcasecmp(dot, ".png") == 0 ||
#endif
           strcasecmp(dot, ".bmp") == 0;
}

/**
 * Read PNG IHDR dimensions from in-memory bytes.
 *
 * @param data PNG file bytes.
 * @param size Number of bytes in data.
 * @param out_w Output image width in pixels.
 * @param out_h Output image height in pixels.
 * @return true if dimensions were parsed successfully; false otherwise.
 */
static bool png_get_dimensions(const uint8_t *data, size_t size, uint16_t *out_w, uint16_t *out_h)
{
    // PNG: 8-byte signature + IHDR: 4 len + 4 type + 4 width + 4 height
    if (size < 24 || out_w == NULL || out_h == NULL) return false;
    if (data[0] != 0x89 || data[1] != 'P' || data[2] != 'N' || data[3] != 'G') return false;
    uint32_t w = ((uint32_t)data[16] << 24) | ((uint32_t)data[17] << 16) | ((uint32_t)data[18] << 8) | data[19];
    uint32_t h = ((uint32_t)data[20] << 24) | ((uint32_t)data[21] << 16) | ((uint32_t)data[22] << 8) | data[23];
    if (w == 0 || h == 0 || w > 65535 || h > 65535) return false;
    *out_w = (uint16_t)w;
    *out_h = (uint16_t)h;
    return true;
}

/**
 * Read BMP header dimensions from in-memory bytes.
 *
 * @param data BMP file bytes.
 * @param size Number of bytes in data.
 * @param out_w Output image width in pixels.
 * @param out_h Output image height in pixels.
 * @return true if dimensions were parsed successfully; false otherwise.
 */
static bool bmp_get_dimensions(const uint8_t *data, size_t size, uint16_t *out_w, uint16_t *out_h)
{
    // BMP: 'BM' + 8 bytes + 4 header size + 4 width (LE) + 4 height (LE) starting at byte 18
    if (size < 26 || out_w == NULL || out_h == NULL) return false;
    if (data[0] != 'B' || data[1] != 'M') return false;
    int32_t w = (int32_t)(((uint32_t)data[21] << 24) | ((uint32_t)data[20] << 16) | ((uint32_t)data[19] << 8) | data[18]);
    int32_t h = (int32_t)(((uint32_t)data[25] << 24) | ((uint32_t)data[24] << 16) | ((uint32_t)data[23] << 8) | data[22]);
    if (h < 0) h = -h;
    if (w <= 0 || h <= 0 || w > 65535 || h > 65535) return false;
    *out_w = (uint16_t)w;
    *out_h = (uint16_t)h;
    return true;
}

/**
 * Build the expected MP3 filename for an image by replacing extension with .mp3.
 *
 * @param image_name Source image filename.
 * @param out_mp3_path Output buffer receiving derived MP3 filename.
 * @param out_size Size of output buffer in bytes.
 */
static void build_mp3_filename(const char *image_name, char *out_mp3_path, size_t out_size)
{
    if (image_name == NULL || out_mp3_path == NULL || out_size == 0) return;

    const char *dot = strrchr(image_name, '.');
    if (dot == NULL) {
        // No extension found; just append .mp3
        snprintf(out_mp3_path, out_size, "%s.mp3", image_name);
    } else {
        size_t base_len = dot - image_name;
        snprintf(out_mp3_path, out_size, "%.*s.mp3", (int)base_len, image_name);
    }
}

/**
 * Try to preload an MP3 that shares basename with the indexed image.
 *
 * @param img_idx Index in s_images to attach preloaded MP3 data to.
 */
static void try_load_mp3_for_image(size_t img_idx)
{
    if (img_idx >= s_image_count) return;

    char mp3_path[256];
    char full_path[384];
    build_mp3_filename(s_images[img_idx].file_name, mp3_path, sizeof(mp3_path));

    int written = snprintf(full_path, sizeof(full_path), "%s/%s", BSP_SD_MOUNT_POINT, mp3_path);
    if (written <= 0 || written >= (int)sizeof(full_path)) {
        return;
    }

    FILE *mp3_file = fopen(full_path, "rb");
    if (mp3_file == NULL) {
        // MP3 file not found; this is fine, not all images need sound
        return;
    }

    // Get file size
    fseek(mp3_file, 0, SEEK_END);
    long file_size = ftell(mp3_file);
    fseek(mp3_file, 0, SEEK_SET);

    if (file_size <= 0 || file_size > MAX_MP3_PRELOAD_BYTES) {
        ESP_LOGW(TAG, "MP3 file too large or invalid: %s (%ld bytes)", mp3_path, file_size);
        fclose(mp3_file);
        return;
    }

    // Preload MP3 into PSRAM so touch playback does not block on SD I/O.
    uint8_t *mp3_data = (uint8_t *)heap_caps_malloc((size_t)file_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (mp3_data == NULL) {
        ESP_LOGW(TAG, "Failed to allocate PSRAM for MP3: %s", mp3_path);
        fclose(mp3_file);
        return;
    }

    size_t read_bytes = fread(mp3_data, 1, (size_t)file_size, mp3_file);
    fclose(mp3_file);

    if (read_bytes != (size_t)file_size) {
        ESP_LOGW(TAG, "Failed to fully read MP3: %s (got %u/%ld bytes)", mp3_path, (unsigned)read_bytes, file_size);
        heap_caps_free(mp3_data);
        return;
    }

    s_images[img_idx].mp3_data = mp3_data;
    s_images[img_idx].mp3_size = (size_t)file_size;
    ESP_LOGI(TAG, "Loaded MP3 for image %u: %s (%u bytes)", (unsigned)img_idx, mp3_path, (unsigned)read_bytes);
}

/**
 * Compute LVGL transform scale so image corners reach the circular display edge.
 *
 * @param img_w Source image width in pixels.
 * @param img_h Source image height in pixels.
 * @return LVGL scale value (LV_SCALE_NONE is 1:1).
 */
static uint32_t compute_fill_scale(uint16_t img_w, uint16_t img_h)
{
    if (img_w == 0 || img_h == 0) return LV_SCALE_NONE;
    float half_diag = sqrtf((float)img_w * (float)img_w / 4.0f + (float)img_h * (float)img_h / 4.0f);
    float scale = (float)DISPLAY_RADIUS / half_diag;
    uint32_t lvgl_scale = (uint32_t)(scale * (float)LV_SCALE_NONE + 0.5f);
    if (lvgl_scale < 1) lvgl_scale = 1;
    return lvgl_scale;
}

/**
 * Decode a pixel from an LVGL draw buffer into 8-bit RGB channels.
 *
 * @param buf Draw buffer returned by image decoder.
 * @param x X coordinate within draw buffer.
 * @param y Y coordinate within draw buffer.
 * @param out_r Output red channel.
 * @param out_g Output green channel.
 * @param out_b Output blue channel.
 * @return true if conversion succeeded for current color format; false otherwise.
 */
static bool decode_rgb_from_draw_buf(const lv_draw_buf_t *buf, uint32_t x, uint32_t y,
                                     uint8_t *out_r, uint8_t *out_g, uint8_t *out_b)
{
    if (buf == NULL || out_r == NULL || out_g == NULL || out_b == NULL) {
        return false;
    }

    lv_color_format_t cf = (lv_color_format_t)buf->header.cf;
    const uint8_t *ptr = (const uint8_t *)lv_draw_buf_goto_xy(buf, x, y);
    if (ptr == NULL) {
        return false;
    }

    if (cf == LV_COLOR_FORMAT_RGB888) {
        // lv_color_t byte order is B, G, R
        *out_b = ptr[0];
        *out_g = ptr[1];
        *out_r = ptr[2];
        return true;
    }

    if (cf == LV_COLOR_FORMAT_ARGB8888 ||
        cf == LV_COLOR_FORMAT_ARGB8888_PREMULTIPLIED ||
        cf == LV_COLOR_FORMAT_XRGB8888) {
        const lv_color32_t *c32 = (const lv_color32_t *)ptr;
        *out_r = c32->red;
        *out_g = c32->green;
        *out_b = c32->blue;
        return true;
    }

    if (cf == LV_COLOR_FORMAT_RGB565 || cf == LV_COLOR_FORMAT_RGB565_SWAPPED) {
        uint16_t c = *((const uint16_t *)ptr);
        if (cf == LV_COLOR_FORMAT_RGB565_SWAPPED) {
            c = lv_color_swap_16(c);
        }
        uint8_t r5 = (uint8_t)((c >> 11) & 0x1F);
        uint8_t g6 = (uint8_t)((c >> 5) & 0x3F);
        uint8_t b5 = (uint8_t)(c & 0x1F);
        *out_r = (uint8_t)((r5 * 255) / 31);
        *out_g = (uint8_t)((g6 * 255) / 63);
        *out_b = (uint8_t)((b5 * 255) / 31);
        return true;
    }

    return false;
}

/**
 * Compute dominant edge color for an image source to use as circular-screen backdrop.
 *
 * @param src LVGL image source (RAM-backed descriptor or file-backed path).
 * @return Dominant edge color, or black on decode failure.
 */
static lv_color_t compute_edge_dominant_color(const void *src)
{
    enum { COLOR_BINS_PER_CHANNEL = 6, COLOR_BIN_COUNT = 216 };

    lv_image_decoder_dsc_t dsc;
    lv_memzero(&dsc, sizeof(dsc));
    if (lv_image_decoder_open(&dsc, src, NULL) != LV_RESULT_OK) {
        return lv_color_black();
    }

    uint32_t img_w = dsc.header.w;
    uint32_t img_h = dsc.header.h;
    if (img_w == 0 || img_h == 0) {
        lv_image_decoder_close(&dsc);
        return lv_color_black();
    }

    uint32_t min_side = (img_w < img_h) ? img_w : img_h;
    uint32_t edge_band = min_side / 14;
    if (edge_band < 2) edge_band = 2;
    uint32_t sample_step = min_side / 90;
    if (sample_step < 1) sample_step = 1;

    uint16_t count[COLOR_BIN_COUNT] = {0};
    uint32_t sum_r[COLOR_BIN_COUNT] = {0};
    uint32_t sum_g[COLOR_BIN_COUNT] = {0};
    uint32_t sum_b[COLOR_BIN_COUNT] = {0};

    // lodepng (PNG) fully decodes the image into dsc.decoded during open(), so
    // get_area() returns LV_RESULT_INVALID immediately and the loop never runs.
    // TJPGD (JPEG) decodes incrementally — dsc.decoded is NULL after open().
    // Handle both cases.
    if (dsc.decoded != NULL) {
        // Full image already decoded (e.g. PNG via lodepng) — iterate directly.
        const lv_draw_buf_t *buf = dsc.decoded;
        for (uint32_t gy = 0; gy < img_h; gy++) {
            bool edge_y = (gy < edge_band) || (gy + edge_band >= img_h);
            for (uint32_t gx = 0; gx < img_w; gx++) {
                bool edge_x = (gx < edge_band) || (gx + edge_band >= img_w);
                if (!(edge_x || edge_y)) continue;
                if ((gx % sample_step) != 0 || (gy % sample_step) != 0) continue;

                uint8_t r = 0, g = 0, b = 0;
                if (!decode_rgb_from_draw_buf(buf, gx, gy, &r, &g, &b)) continue;

                uint32_t rb = (r * COLOR_BINS_PER_CHANNEL) / 256;
                uint32_t gb = (g * COLOR_BINS_PER_CHANNEL) / 256;
                uint32_t bb = (b * COLOR_BINS_PER_CHANNEL) / 256;
                if (rb >= COLOR_BINS_PER_CHANNEL) rb = COLOR_BINS_PER_CHANNEL - 1;
                if (gb >= COLOR_BINS_PER_CHANNEL) gb = COLOR_BINS_PER_CHANNEL - 1;
                if (bb >= COLOR_BINS_PER_CHANNEL) bb = COLOR_BINS_PER_CHANNEL - 1;

                uint32_t idx = rb * COLOR_BINS_PER_CHANNEL * COLOR_BINS_PER_CHANNEL +
                               gb * COLOR_BINS_PER_CHANNEL + bb;
                if (count[idx] < 0xFFFF) count[idx]++;
                sum_r[idx] += r;
                sum_g[idx] += g;
                sum_b[idx] += b;
            }
        }
    } else {
        // Incremental decoder (e.g. JPEG via TJPGD) — use get_area() loop.
        lv_area_t full_area = {.x1 = 0, .y1 = 0, .x2 = (int32_t)img_w - 1, .y2 = (int32_t)img_h - 1};
        lv_area_t decoded_area = {.x1 = LV_COORD_MIN, .y1 = LV_COORD_MIN, .x2 = LV_COORD_MIN, .y2 = LV_COORD_MIN};

        while (lv_image_decoder_get_area(&dsc, &full_area, &decoded_area) == LV_RESULT_OK) {
            const lv_draw_buf_t *buf = dsc.decoded;
            if (buf == NULL) break;

            uint32_t buf_w = buf->header.w;
            uint32_t buf_h = buf->header.h;
            for (uint32_t ly = 0; ly < buf_h; ly++) {
                uint32_t gy = (uint32_t)decoded_area.y1 + ly;
                bool edge_y = (gy < edge_band) || (gy + edge_band >= img_h);

                for (uint32_t lx = 0; lx < buf_w; lx++) {
                    uint32_t gx = (uint32_t)decoded_area.x1 + lx;
                    bool edge_x = (gx < edge_band) || (gx + edge_band >= img_w);
                    if (!(edge_x || edge_y)) continue;
                    if ((gx % sample_step) != 0 || (gy % sample_step) != 0) continue;

                    uint8_t r = 0, g = 0, b = 0;
                    if (!decode_rgb_from_draw_buf(buf, lx, ly, &r, &g, &b)) continue;

                    uint32_t rb = (r * COLOR_BINS_PER_CHANNEL) / 256;
                    uint32_t gb = (g * COLOR_BINS_PER_CHANNEL) / 256;
                    uint32_t bb = (b * COLOR_BINS_PER_CHANNEL) / 256;
                    if (rb >= COLOR_BINS_PER_CHANNEL) rb = COLOR_BINS_PER_CHANNEL - 1;
                    if (gb >= COLOR_BINS_PER_CHANNEL) gb = COLOR_BINS_PER_CHANNEL - 1;
                    if (bb >= COLOR_BINS_PER_CHANNEL) bb = COLOR_BINS_PER_CHANNEL - 1;

                    uint32_t idx = rb * COLOR_BINS_PER_CHANNEL * COLOR_BINS_PER_CHANNEL +
                                   gb * COLOR_BINS_PER_CHANNEL + bb;
                    if (count[idx] < 0xFFFF) count[idx]++;
                    sum_r[idx] += r;
                    sum_g[idx] += g;
                    sum_b[idx] += b;
                }
            }
        }
    }

    lv_image_decoder_close(&dsc);

    uint32_t best_idx = 0;
    uint16_t best_count = 0;
    for (uint32_t i = 0; i < COLOR_BIN_COUNT; i++) {
        if (count[i] > best_count) {
            best_count = count[i];
            best_idx = i;
        }
    }

    if (best_count == 0) {
        return lv_color_black();
    }

    uint8_t r = (uint8_t)(sum_r[best_idx] / best_count);
    uint8_t g = (uint8_t)(sum_g[best_idx] / best_count);
    uint8_t b = (uint8_t)(sum_b[best_idx] / best_count);
    return lv_color_make(r, g, b);
}

/**
 * Case-insensitive extension check against a single expected extension.
 *
 * @param name Filename to inspect.
 * @param ext Extension to compare, including leading dot.
 * @return true when name ends with ext.
 */
static bool has_ext(const char *name, const char *ext)
{
    const char *dot = strrchr(name, '.');
    if (dot == NULL) {
        return false;
    }
    return strcasecmp(dot, ext) == 0;
}

/**
 * Comparator used to sort image entries alphabetically by filename.
 *
 * @param a Pointer to first image_entry_t.
 * @param b Pointer to second image_entry_t.
 * @return <0, 0, >0 following qsort comparator contract.
 */
static int file_name_cmp(const void *a, const void *b)
{
    const image_entry_t *lhs = (const image_entry_t *)a;
    const image_entry_t *rhs = (const image_entry_t *)b;
    return strcasecmp(lhs->file_name, rhs->file_name);
}

/**
 * Duplicate a candidate LVGL source path only if decoder can identify it.
 *
 * @param src Candidate LVGL image source path.
 * @return Newly allocated duplicate string on success, otherwise NULL.
 */
static char *dup_if_decodable(const char *src)
{
    lv_image_header_t header;
    if (lv_image_decoder_get_info(src, &header) != LV_RESULT_OK) {
        return NULL;
    }

    return strdup(src);
}

/**
 * Build and validate an LVGL source string for an image file.
 *
 * @param file_name Image file name found on SD root.
 * @return Heap-allocated LVGL source string, or NULL if not decodable.
 */
static char *resolve_image_src(const char *file_name)
{
    char src_path[384];
    // Prefer explicit mount path first, then try root-relative fallback for LVGL FS.
    int written = snprintf(src_path, sizeof(src_path), "%c:%s/%s",
                           LVGL_FS_DRIVE_LETTER,
                           BSP_SD_MOUNT_POINT,
                           file_name);
    if (written > 0 && written < (int)sizeof(src_path)) {
        char *resolved = dup_if_decodable(src_path);
        if (resolved != NULL) {
            return resolved;
        }
    }

    written = snprintf(src_path, sizeof(src_path), "%c:/%s",
                       LVGL_FS_DRIVE_LETTER,
                       file_name);
    if (written > 0 && written < (int)sizeof(src_path)) {
        return dup_if_decodable(src_path);
    }

    return NULL;
}

/**
 * Scan SD card root directory and populate slideshow image list.
 *
 * @return ESP_OK on success, otherwise an error if directory or allocation fails.
 */
static esp_err_t scan_sdcard_root_images(void)
{
    DIR *dir = opendir(BSP_SD_MOUNT_POINT);
    if (dir == NULL) {
        ESP_LOGE(TAG, "Failed to open %s", BSP_SD_MOUNT_POINT);
        return ESP_FAIL;
    }

    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        if (entry->d_name[0] == '.') {
            continue;
        }
        // Only keep image files the slideshow can decode.
        if (!has_supported_ext(entry->d_name)) {
            continue;
        }
        if (s_image_count >= MAX_IMAGES) {
            ESP_LOGW(TAG, "Reached MAX_IMAGES=%d, ignoring the rest", MAX_IMAGES);
            break;
        }

        s_images[s_image_count].file_name = strdup(entry->d_name);
        if (s_images[s_image_count].file_name == NULL) {
            closedir(dir);
            return ESP_ERR_NO_MEM;
        }

        s_images[s_image_count].src = NULL;
        s_images[s_image_count].file_data = NULL;
        s_images[s_image_count].file_size = 0;
        memset(&s_images[s_image_count].image_dsc, 0, sizeof(s_images[s_image_count].image_dsc));
        s_images[s_image_count].img_w = 0;
        s_images[s_image_count].img_h = 0;
        s_images[s_image_count].display_scale = LV_SCALE_NONE;
        s_images[s_image_count].backdrop_color = lv_color_black();

        s_image_count++;
    }

    closedir(dir);

    if (s_image_count > 1) {
        qsort(s_images, s_image_count, sizeof(s_images[0]), file_name_cmp);
    }

    ESP_LOGI(TAG, "Found %u image(s) in SD root", (unsigned)s_image_count);
    return ESP_OK;
}

/**
 * Read an entire file into RAM/PSRAM.
 *
 * @param file_name File name relative to SD mount point.
 * @param out_data Output pointer receiving allocated file buffer.
 * @param out_bytes Output byte count of loaded file.
 * @return ESP_OK on success, otherwise an ESP-IDF error code.
 */
static esp_err_t read_file_into_memory(const char *file_name, uint8_t **out_data, size_t *out_bytes)
{
    char path[384];
    int written = snprintf(path, sizeof(path), "%s/%s", BSP_SD_MOUNT_POINT, file_name);
    if (written <= 0 || written >= (int)sizeof(path)) {
        return ESP_ERR_INVALID_SIZE;
    }

    FILE *f = fopen(path, "rb");
    if (f == NULL) {
        return ESP_FAIL;
    }

    if (fseek(f, 0, SEEK_END) != 0) {
        fclose(f);
        return ESP_FAIL;
    }

    long file_size = ftell(f);
    if (file_size <= 0) {
        fclose(f);
        return ESP_FAIL;
    }

    if (fseek(f, 0, SEEK_SET) != 0) {
        fclose(f);
        return ESP_FAIL;
    }

    size_t alloc_size = (size_t)file_size;
#if SLIDESHOW_VERBOSE_LOGS
    ESP_LOGI(TAG,
             "Preload candidate %s (%u bytes). free 8bit=%u, largest 8bit=%u",
             file_name,
             (unsigned)alloc_size,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
#endif

    // Try PSRAM first to protect internal RAM for stacks and LVGL runtime allocations.
    uint8_t *data = (uint8_t *)heap_caps_malloc(alloc_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (data == NULL) {
        data = (uint8_t *)heap_caps_malloc(alloc_size, MALLOC_CAP_8BIT);
    }
    if (data == NULL) {
        fclose(f);
        ESP_LOGW(TAG,
                 "Allocation failed for %s (%u bytes). free 8bit=%u, largest 8bit=%u",
                 file_name,
                 (unsigned)alloc_size,
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT),
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
        return ESP_ERR_NO_MEM;
    }

    size_t bytes_read = fread(data, 1, (size_t)file_size, f);
    fclose(f);
    if (bytes_read != (size_t)file_size) {
        free(data);
        return ESP_FAIL;
    }

    *out_data = data;
    *out_bytes = bytes_read;
    return ESP_OK;
}

/**
 * Preload slideshow images (and optional per-image MP3 files) into memory.
 *
 * @return ESP_OK when preload policy is satisfied; error otherwise.
 */
static esp_err_t preload_images_into_memory(void)
{
    size_t loaded_ram = 0;
    size_t file_backed = 0;

    if (STRICT_RAM_PRELOAD_MODE) {
        ESP_LOGI(TAG, "Strict RAM preload mode active (no file-backed fallback)");
    } else {
        ESP_LOGI(TAG,
                 "Hybrid preload active (RAM threshold: %u bytes)",
                 (unsigned)PRELOAD_MAX_FILE_BYTES);
    }

    for (size_t i = 0; i < s_image_count; i++) {
        uint8_t *data = NULL;
        size_t size = 0;
        esp_err_t err = read_file_into_memory(s_images[i].file_name, &data, &size);

        // In hybrid mode, keep very large or failed files as LVGL file-backed sources.
        if (!STRICT_RAM_PRELOAD_MODE && (err == ESP_ERR_NO_MEM || (err == ESP_OK && size > PRELOAD_MAX_FILE_BYTES))) {
            if (err == ESP_OK) {
                free(data);
                data = NULL;
                size = 0;
            }

            char *path_src = resolve_image_src(s_images[i].file_name);
            if (path_src != NULL) {
                s_images[i].src = path_src;
                file_backed++;
                ESP_LOGW(TAG,
                         "Using file-backed decode for %s (over RAM budget or no mem)",
                         s_images[i].file_name);
                continue;
            }
        }

        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Preload failed for %s: %s", s_images[i].file_name, esp_err_to_name(err));
            if (STRICT_RAM_PRELOAD_MODE) {
                return err;
            }
            continue;
        }

        s_images[i].file_data = data;
        s_images[i].file_size = size;
        s_images[i].image_dsc.header.magic = LV_IMAGE_HEADER_MAGIC;
        s_images[i].image_dsc.header.cf = LV_COLOR_FORMAT_RAW;
        s_images[i].image_dsc.header.flags = 0;
        s_images[i].image_dsc.header.w = 0;
        s_images[i].image_dsc.header.h = 0;
        s_images[i].image_dsc.header.stride = 0;
        s_images[i].image_dsc.header.reserved_2 = 0;
        s_images[i].image_dsc.data_size = (uint32_t)size;
        s_images[i].image_dsc.data = data;
        s_images[i].image_dsc.reserved = NULL;
        s_images[i].image_dsc.reserved_2 = NULL;

        uint16_t width = 0;
        uint16_t height = 0;
        if (has_ext(s_images[i].file_name, ".jpg") || has_ext(s_images[i].file_name, ".jpeg")) {
            if (!jpeg_get_dimensions(data, size, &width, &height)) {
                ESP_LOGW(TAG, "Could not parse JPEG dimensions: %s", s_images[i].file_name);
                free(s_images[i].file_data);
                s_images[i].file_data = NULL;
                s_images[i].file_size = 0;
                memset(&s_images[i].image_dsc, 0, sizeof(s_images[i].image_dsc));
                continue;
            }

            s_images[i].image_dsc.header.w = width;
            s_images[i].image_dsc.header.h = height;
        }

        // Use in-memory descriptor as the active source for this slide.
        s_images[i].src = &s_images[i].image_dsc;
        loaded_ram++;

        if (width == 0 || height == 0) {
            if (has_ext(s_images[i].file_name, ".png")) {
                png_get_dimensions(data, size, &width, &height);
            } else if (has_ext(s_images[i].file_name, ".bmp")) {
                bmp_get_dimensions(data, size, &width, &height);
            }
        }
        s_images[i].img_w = width;
        s_images[i].img_h = height;
        s_images[i].display_scale = compute_fill_scale(width, height);
        // Compute a dominant edge color so round-screen corners blend with the image.
        if (bsp_display_lock(portMAX_DELAY)) {
            s_images[i].backdrop_color = compute_edge_dominant_color(s_images[i].src);
            bsp_display_unlock();
        } else {
            s_images[i].backdrop_color = lv_color_black();
        }

        // Try to load associated MP3 file
        try_load_mp3_for_image(i);

#if SLIDESHOW_VERBOSE_LOGS
        if (width > 0 && height > 0) {
            ESP_LOGI(TAG,
                     "Preloaded %s into RAM (%u bytes, %ux%u)",
                     s_images[i].file_name,
                     (unsigned)size,
                     (unsigned)width,
                     (unsigned)height);
        } else {
            ESP_LOGI(TAG,
                     "Preloaded %s into RAM (%u bytes)",
                     s_images[i].file_name,
                     (unsigned)size);
        }
#endif
    }

    if (loaded_ram == 0 && file_backed == 0) {
        return ESP_FAIL;
    }

    if (STRICT_RAM_PRELOAD_MODE && loaded_ram != s_image_count) {
        ESP_LOGE(TAG,
                 "Strict RAM preload incomplete: loaded %u/%u image(s)",
                 (unsigned)loaded_ram,
                 (unsigned)s_image_count);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG,
             "Preload summary: RAM-backed=%u, file-backed=%u, total=%u",
             (unsigned)loaded_ram,
             (unsigned)file_backed,
             (unsigned)s_image_count);
    return ESP_OK;
}

/**
 * Check whether every image source is RAM-backed.
 *
 * @return true if all images use in-memory descriptors, false otherwise.
 */
static bool all_images_are_ram_backed(void)
{
    if (s_image_count == 0) {
        return false;
    }

    for (size_t i = 0; i < s_image_count; i++) {
        if (s_images[i].src != &s_images[i].image_dsc) {
            return false;
        }
    }

    return true;
}

/**
 * Log startup heap capacity and fragmentation indicators for internal RAM and PSRAM.
 */
static void log_startup_memory_health(void)
{
    size_t int_total = heap_caps_get_total_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    size_t int_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    size_t int_largest = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

    size_t psram_total = heap_caps_get_total_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    size_t psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    size_t psram_largest = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    ESP_LOGI(TAG,
             "Heap health (internal): total=%u free=%u largest=%u",
             (unsigned)int_total,
             (unsigned)int_free,
             (unsigned)int_largest);
    ESP_LOGI(TAG,
             "Heap health (psram): total=%u free=%u largest=%u",
             (unsigned)psram_total,
             (unsigned)psram_free,
             (unsigned)psram_largest);
}

#if SD_READ_TEST_MODE
/**
 * Read a file completely for SD stress-test instrumentation.
 *
 * @param file_name File name relative to SD mount point.
 * @param out_bytes Output total bytes read.
 * @return ESP_OK on success, otherwise an ESP-IDF error code.
 */
static esp_err_t read_file_fully(const char *file_name, size_t *out_bytes)
{
    char path[384];
    int written = snprintf(path, sizeof(path), "%s/%s", BSP_SD_MOUNT_POINT, file_name);
    if (written <= 0 || written >= (int)sizeof(path)) {
        return ESP_ERR_INVALID_SIZE;
    }

    FILE *f = fopen(path, "rb");
    if (f == NULL) {
        return ESP_FAIL;
    }

    size_t total = 0;
    static uint8_t buffer[SD_READ_CHUNK_SIZE];
    while (!feof(f)) {
        size_t n = fread(buffer, 1, sizeof(buffer), f);
        if (n > 0) {
            total += n;
        }
        if (ferror(f)) {
            fclose(f);
            return ESP_FAIL;
        }
    }

    fclose(f);
    *out_bytes = total;
    return ESP_OK;
}

/**
 * Continuous SD throughput/stability test task (enabled by SD_READ_TEST_MODE).
 *
 * @param arg Unused FreeRTOS task argument.
 */
static void sd_read_test_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Starting SD read stress test mode");

    ESP_ERROR_CHECK(mount_sdcard_with_retries());
    ESP_ERROR_CHECK(scan_sdcard_root_images());

    if (s_image_count == 0) {
        ESP_LOGE(TAG, "No files found for read test");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    uint32_t pass = 0;
    while (1) {
        pass++;
        ESP_LOGI(TAG, "Read pass %u start", (unsigned)pass);
        for (size_t i = 0; i < s_image_count; i++) {
            size_t bytes = 0;
            esp_err_t err = read_file_fully(s_images[i].file_name, &bytes);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Read failed on pass %u file %u/%u: %s (%s)",
                         (unsigned)pass,
                         (unsigned)(i + 1),
                         (unsigned)s_image_count,
                         s_images[i].file_name,
                         esp_err_to_name(err));
            } else {
                ESP_LOGI(TAG, "Read ok pass %u file %u/%u: %s (%u bytes)",
                         (unsigned)pass,
                         (unsigned)(i + 1),
                         (unsigned)s_image_count,
                         s_images[i].file_name,
                         (unsigned)bytes);
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}
#endif

/**
 * Apply an image entry to a target LVGL image object, including transform metadata.
 *
 * @param target_obj LVGL image object to update.
 * @param img_idx Index of image entry in s_images.
 * @return true when image source exists and was assigned; false if slide is invalid/undecodable.
 */
static bool apply_image_to_obj(lv_obj_t *target_obj, size_t img_idx)
{
    if (target_obj == NULL || img_idx >= s_image_count) {
        return false;
    }

    const void *image_src = s_images[img_idx].src;
    if (image_src == NULL) {
        lv_image_set_src(target_obj, NULL);
        return false;
    }

    int32_t pivot_x = (s_images[img_idx].img_w > 0) ?
                      (s_images[img_idx].img_w / 2) :
                      (lv_obj_get_width(target_obj) / 2);
    int32_t pivot_y = (s_images[img_idx].img_h > 0) ?
                      (s_images[img_idx].img_h / 2) :
                      (lv_obj_get_height(target_obj) / 2);

    lv_image_set_src(target_obj, image_src);
    uint32_t scale = s_images[img_idx].display_scale;
    if (scale == 0 || scale == LV_SCALE_NONE) {
        // Keep identity transform for native-sized assets to minimize redraw cost.
        lv_obj_set_style_transform_scale(target_obj, LV_SCALE_NONE, LV_PART_MAIN);
    } else {
        lv_obj_set_style_transform_scale(target_obj, (int32_t)scale, LV_PART_MAIN);
            lv_obj_set_style_transform_pivot_x(target_obj, pivot_x, LV_PART_MAIN);
            lv_obj_set_style_transform_pivot_y(target_obj, pivot_y, LV_PART_MAIN);
        }
    return true;
}

/**
 * Update displayed image object and top status text for the requested index.
 *
 * @param index Target image index (wrapped modulo image count).
 */
static void update_image_and_status(size_t index)
{
    if (s_image_count == 0) {
        lv_label_set_text(s_status_label, "No images found in /sdcard");
        return;
    }

    // Wrap index so both manual and timed transitions remain bounds-safe.
    s_current_index = index % s_image_count;

    uint8_t next_obj_idx = (uint8_t)(1U - s_active_image_obj_idx);
    lv_obj_t *front_obj = s_image_objs[s_active_image_obj_idx];
    lv_obj_t *back_obj = s_image_objs[next_obj_idx];

    // Stage next slide into hidden object, then atomically swap visibility.
    bool image_ok = apply_image_to_obj(back_obj, s_current_index);
    if (image_ok) {
        lv_obj_set_style_bg_color(s_screen_obj, s_images[s_current_index].backdrop_color, 0);
        lv_obj_clear_flag(back_obj, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(front_obj, LV_OBJ_FLAG_HIDDEN);
        s_active_image_obj_idx = next_obj_idx;
    } else {
        lv_obj_set_style_bg_color(s_screen_obj, lv_color_black(), 0);
        lv_image_set_src(back_obj, NULL);
        lv_obj_clear_flag(back_obj, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(front_obj, LV_OBJ_FLAG_HIDDEN);
        s_active_image_obj_idx = next_obj_idx;
        ESP_LOGW(TAG, "Image decode failed: %s", s_images[s_current_index].file_name);
    }

    char status[128];
    snprintf(status, sizeof(status), "[%u/%u] %s%s",
             (unsigned)(s_current_index + 1),
             (unsigned)s_image_count,
             s_images[s_current_index].file_name,
             image_ok ? "" : " (decode failed)");
    lv_label_set_text(s_status_label, status);
}

/**
 * Apply slide change while panel brightness is temporarily blanked.
 *
 * On this AMOLED BSP, "backlight" helpers drive register 0x51 brightness,
 * so this hides the visible chunked flush without requiring full-frame DMA buffers.
 */
static void update_image_with_masked_brightness(lv_display_t *display, size_t index)
{
    const int normal_brightness = 100;

    esp_err_t err = bsp_display_brightness_set(0);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to blank display before swap: %s", esp_err_to_name(err));
    }

    bsp_display_lock(portMAX_DELAY);
    update_image_and_status(index);
    if (display != NULL) {
        // Block until all queued flush chunks complete while display is blanked.
        lv_refr_now(display);
    }
    bsp_display_unlock();

    err = bsp_display_brightness_set(normal_brightness);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to restore display brightness after swap: %s", esp_err_to_name(err));
    }
}

/**
 * Decode and play preloaded MP3 associated with the selected image.
 *
 * @param img_idx Image index in s_images.
 */
static void play_image_sound(size_t img_idx)
{
    if (img_idx >= s_image_count) return;

    // Audio is optional per slide; silently skip when no matching MP3 is preloaded.
    if (s_images[img_idx].mp3_data != NULL && s_images[img_idx].mp3_size > 0) {
        if (!s_audio_initialized || s_speaker_codec_dev == NULL) {
            ESP_LOGW(TAG, "Audio not initialized, cannot play MP3");
            return;
        }

        ESP_LOGI(TAG, "Playing MP3 for image: %s (%u bytes)", 
                 s_images[img_idx].file_name, 
                 (unsigned)s_images[img_idx].mp3_size);

        esp_audio_simple_dec_cfg_t dec_cfg = {
            .dec_type = ESP_AUDIO_SIMPLE_DEC_TYPE_MP3,
            .dec_cfg = NULL,
            .cfg_size = 0,
            .use_frame_dec = false,
        };

        esp_audio_simple_dec_handle_t decoder = NULL;
        esp_audio_err_t dec_ret = esp_audio_simple_dec_open(&dec_cfg, &decoder);
        if (dec_ret != ESP_AUDIO_ERR_OK || decoder == NULL) {
            ESP_LOGW(TAG, "Failed to open MP3 decoder: %d", (int)dec_ret);
            return;
        }

        size_t out_buf_size = MP3_DEC_OUT_CHUNK;
        uint8_t *out_buf = (uint8_t *)heap_caps_malloc(out_buf_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (out_buf == NULL) {
            out_buf = (uint8_t *)malloc(out_buf_size);
        }
        if (out_buf == NULL) {
            ESP_LOGW(TAG, "No memory for MP3 decode output buffer");
            esp_audio_simple_dec_close(decoder);
            return;
        }

        size_t total_pcm_written = 0;
        size_t offset = 0;
        bool codec_opened = false;

        // Stream the preloaded MP3 buffer through the decoder in bounded chunks.
        while (offset < s_images[img_idx].mp3_size) {
            size_t chunk = s_images[img_idx].mp3_size - offset;
            if (chunk > MP3_DEC_IN_CHUNK) chunk = MP3_DEC_IN_CHUNK;

            esp_audio_simple_dec_raw_t raw = {
                .buffer = s_images[img_idx].mp3_data + offset,
                .len = (uint32_t)chunk,
                .eos = (offset + chunk >= s_images[img_idx].mp3_size),
            };

            while (raw.len > 0) {
                esp_audio_simple_dec_out_t out = {
                    .buffer = out_buf,
                    .len = out_buf_size,
                };

                dec_ret = esp_audio_simple_dec_process(decoder, &raw, &out);
                if (dec_ret == ESP_AUDIO_ERR_BUFF_NOT_ENOUGH) {
                    // Decoder can request larger output for frame bursts; grow within a hard cap.
                    if (out.needed_size > MP3_DEC_MAX_OUT_CHUNK) {
                        ESP_LOGW(TAG, "MP3 output buffer request too large: %u", (unsigned)out.needed_size);
                        dec_ret = ESP_AUDIO_ERR_MEM_LACK;
                        break;
                    }

                    uint8_t *new_out = (uint8_t *)heap_caps_malloc(out.needed_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
                    if (new_out == NULL) {
                        new_out = (uint8_t *)malloc(out.needed_size);
                    }
                    if (new_out == NULL) {
                        ESP_LOGW(TAG, "Failed to grow MP3 output buffer to %u", (unsigned)out.needed_size);
                        dec_ret = ESP_AUDIO_ERR_MEM_LACK;
                        break;
                    }

                    free(out_buf);
                    out_buf = new_out;
                    out_buf_size = out.needed_size;
                    continue;
                }

                if (dec_ret != ESP_AUDIO_ERR_OK) {
                    ESP_LOGW(TAG, "MP3 decode failed: %d", (int)dec_ret);
                    break;
                }

                if (out.decoded_size > 0) {
                    if (!codec_opened) {
                        // Open speaker codec lazily once decoder reports the final PCM format.
                        esp_audio_simple_dec_info_t dec_info = {0};
                        dec_ret = esp_audio_simple_dec_get_info(decoder, &dec_info);
                        if (dec_ret != ESP_AUDIO_ERR_OK) {
                            ESP_LOGW(TAG, "Failed to get MP3 decode info: %d", (int)dec_ret);
                            break;
                        }

                        esp_codec_dev_sample_info_t fs = {
                            .sample_rate = dec_info.sample_rate,
                            .channel = dec_info.channel,
                            .bits_per_sample = dec_info.bits_per_sample,
                        };

                        esp_err_t err = esp_codec_dev_open(s_speaker_codec_dev, &fs);
                        if (err != ESP_OK) {
                            ESP_LOGW(TAG, "Failed to open codec device: %s", esp_err_to_name(err));
                            break;
                        }

                        esp_codec_dev_set_out_vol(s_speaker_codec_dev, AUDIO_OUTPUT_VOLUME_PERCENT);
                        codec_opened = true;
                        ESP_LOGI(TAG, "MP3 decode info: %u Hz, %u-bit, %u ch",
                                 (unsigned)dec_info.sample_rate,
                                 (unsigned)dec_info.bits_per_sample,
                                 (unsigned)dec_info.channel);
                    }

                    esp_err_t write_err = esp_codec_dev_write(s_speaker_codec_dev, out.buffer, out.decoded_size);
                    if (write_err != ESP_OK) {
                        ESP_LOGW(TAG, "PCM write failed: %s", esp_err_to_name(write_err));
                        dec_ret = ESP_AUDIO_ERR_FAIL;
                        break;
                    }
                    total_pcm_written += out.decoded_size;
                }

                if (raw.consumed == 0) {
                    // Decoder needs more data to produce output
                    break;
                }

                raw.buffer += raw.consumed;
                raw.len -= raw.consumed;
            }

            if (dec_ret != ESP_AUDIO_ERR_OK) {
                break;
            }

            offset += chunk;
        }

        if (codec_opened) {
            esp_codec_dev_close(s_speaker_codec_dev);
        }
        esp_audio_simple_dec_close(decoder);
        free(out_buf);

        if (total_pcm_written > 0) {
            ESP_LOGI(TAG, "MP3 playback finished, PCM bytes written: %u", (unsigned)total_pcm_written);
        } else {
            ESP_LOGW(TAG, "MP3 playback produced no PCM output");
        }
    }
}

/**
 * Background audio worker task that runs decoding outside LVGL event callbacks.
 *
 * @param arg Unused FreeRTOS task argument.
 */
static void audio_playback_task(void *arg)
{
    (void)arg;

    while (1) {
        // Wake only when touch callback enqueues a slide index.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        size_t img_idx = s_pending_audio_index;
        if (img_idx >= s_image_count) {
            s_audio_playing = false;
            continue;
        }

        play_image_sound(img_idx);
        s_audio_playing = false;
    }
}

/**
 * Initialize codec, decoder registrations, and background audio task.
 *
 * @return ESP_OK on success, otherwise an ESP-IDF error code.
 */
static esp_err_t audio_init(void)
{
    if (s_audio_initialized) {
        return ESP_OK;
    }

    esp_err_t err;

    // Initialize audio with default I2S config
    err = bsp_audio_init(NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize audio: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "Audio initialized");

    // Initialize speaker codec device
    s_speaker_codec_dev = bsp_audio_codec_speaker_init();
    if (s_speaker_codec_dev == NULL) {
        ESP_LOGE(TAG, "Failed to initialize speaker codec device");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Speaker codec device initialized");

    if (!s_audio_decoders_registered) {
        esp_audio_err_t dec_ret = esp_audio_dec_register_default();
        if (dec_ret != ESP_AUDIO_ERR_OK) {
            ESP_LOGE(TAG, "Failed to register audio decoders: %d", (int)dec_ret);
            return ESP_FAIL;
        }

        dec_ret = esp_audio_simple_dec_register_default();
        if (dec_ret != ESP_AUDIO_ERR_OK) {
            ESP_LOGE(TAG, "Failed to register simple decoders: %d", (int)dec_ret);
            return ESP_FAIL;
        }
        s_audio_decoders_registered = true;
    }

    s_audio_initialized = true;

    if (s_audio_task_handle == NULL) {
        BaseType_t ok = xTaskCreatePinnedToCore(audio_playback_task,
                                                "audio_playback",
                                                AUDIO_PLAYBACK_TASK_STACK_SIZE,
                                                NULL,
                                                tskIDLE_PRIORITY + 2,
                                                &s_audio_task_handle,
                                                0);
        if (ok != pdPASS) {
            s_audio_task_handle = NULL;
            ESP_LOGE(TAG, "Failed to start audio playback task");
            s_audio_initialized = false;
            return ESP_FAIL;
        }
    }

    return ESP_OK;
}

/**
 * LVGL click callback that debounces touch and queues audio playback.
 *
 * @param event LVGL event object for current interaction.
 */
static void on_image_touched(lv_event_t *event)
{
    if (event == NULL) return;

    lv_event_code_t code = lv_event_get_code(event);
    if (code == LV_EVENT_CLICKED) {
        TickType_t now = xTaskGetTickCount();
        // Debounce touch noise and rapid taps to avoid overlapping playbacks.
        if ((now - s_last_touch_tick) < pdMS_TO_TICKS(TOUCH_DEBOUNCE_MS)) {
            return;
        }
        s_last_touch_tick = now;

        if (!s_audio_initialized) {
            esp_err_t audio_err = audio_init();
            if (audio_err != ESP_OK) {
                ESP_LOGW(TAG, "Audio initialization failed, playback disabled");
                return;
            }
        }

        if (s_audio_playing) {
            ESP_LOGI(TAG, "Audio is already playing, ignoring touch");
            return;
        }

        if (s_audio_task_handle == NULL) {
            ESP_LOGW(TAG, "Audio playback task is not available");
            return;
        }

        ESP_LOGI(TAG, "Screen touched, playing image sound");
        // Hand off decode/write work to audio task so LVGL callback stays responsive.
        s_audio_playing = true;
        s_pending_audio_index = s_current_index;
        xTaskNotifyGive(s_audio_task_handle);
    }
}

/**
 * Main slideshow task: initialize display, preload assets, and run auto-advance loop.
 *
 * @param arg Unused FreeRTOS task argument.
 */
static void slideshow_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Starting SD card slideshow");

    bsp_display_cfg_t display_cfg = {
        .lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG(),
        .buffer_size = BSP_LCD_DRAW_BUFF_SIZE,
        .double_buffer = BSP_LCD_DRAW_BUFF_DOUBLE,
        .flags = {
            .buff_dma = true,
            .buff_spiram = false,
        },
    };
    display_cfg.lvgl_port_cfg.task_stack = 16384;

    lv_display_t *display = bsp_display_start_with_config(&display_cfg);
    if (display == NULL) {
        ESP_LOGE(TAG, "Display initialization failed");
        return;
    }

    lv_indev_t *touch_indev = bsp_display_get_input_dev();
#ifdef ESP_LVGL_PORT_TOUCH_COMPONENT
    // Touch is now ENABLED to support touch-to-play feature
    // Previously disabled for stability testing, but now working correctly
    if (touch_indev != NULL) {
        ESP_LOGI(TAG, "Touch input enabled for image-sound playback");
    }
#endif

    ESP_ERROR_CHECK(bsp_display_backlight_on());
    ESP_ERROR_CHECK(bsp_display_brightness_set(100));

    log_startup_memory_health();

    ESP_ERROR_CHECK(mount_sdcard_with_retries());
    ESP_ERROR_CHECK(scan_sdcard_root_images());
    ESP_LOGI(TAG, "Starting image preload before slideshow transitions");
    // Load all slide assets up front to avoid SD latency during transitions.
    ESP_ERROR_CHECK(preload_images_into_memory());

    if (all_images_are_ram_backed()) {
        // If everything is memory-backed, release SD resources after startup.
        if (s_sdcard != NULL) {
            esp_vfs_fat_sdcard_unmount(BSP_SD_MOUNT_POINT, s_sdcard);
            s_sdcard = NULL;
            ESP_LOGI(TAG, "Unmounted SD card (all images are RAM-backed)");
        }
    } else {
        ESP_LOGW(TAG, "Keeping SD mounted because at least one image is file-backed");
    }

    bsp_display_lock(portMAX_DELAY);

#if LVGL_VERSION_MAJOR >= 9
    s_screen_obj = lv_screen_active();
#else
    s_screen_obj = lv_scr_act();
#endif

    lv_obj_set_style_bg_color(s_screen_obj, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(s_screen_obj, LV_OPA_COVER, 0);
    lv_obj_add_flag(s_screen_obj, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(s_screen_obj, on_image_touched, LV_EVENT_CLICKED, NULL);

    s_image_objs[0] = lv_image_create(s_screen_obj);
    lv_image_set_antialias(s_image_objs[0], false);
    lv_obj_add_flag(s_image_objs[0], LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_center(s_image_objs[0]);

    s_image_objs[1] = lv_image_create(s_screen_obj);
    lv_image_set_antialias(s_image_objs[1], false);
    lv_obj_add_flag(s_image_objs[1], LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_add_flag(s_image_objs[1], LV_OBJ_FLAG_HIDDEN);
    lv_obj_center(s_image_objs[1]);
    s_active_image_obj_idx = 0;

    s_status_label = lv_label_create(s_screen_obj);
    lv_obj_set_width(s_status_label, lv_pct(96));
    lv_label_set_long_mode(s_status_label, LV_LABEL_LONG_CLIP);
    lv_obj_set_style_text_color(s_status_label, lv_color_hex(0xD0D0D0), 0);
    lv_obj_align(s_status_label, LV_ALIGN_TOP_MID, 0, 10);

    update_image_and_status(0);
    // Keep first frame path simple while screen is already locked here.
    bsp_display_unlock();

    uint32_t elapsed_ms = 0;
    while (1) {
        UBaseType_t stack_words_free = uxTaskGetStackHighWaterMark(NULL);
        if (stack_words_free < 512) {
            ESP_LOGW(TAG, "Low slideshow stack watermark: %u words", (unsigned)stack_words_free);
        }

        // Tick slideshow at 1 Hz and advance when configured interval is reached.
        if (s_image_count > 1) {
            elapsed_ms += 1000;
            if (elapsed_ms >= IMAGE_CHANGE_MS) {
                elapsed_ms = 0;
                update_image_with_masked_brightness(display, (s_current_index + 1) % s_image_count);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/**
 * ESP-IDF application entry point; starts slideshow task or SD test task.
 */
void app_main(void)
{
#if SD_READ_TEST_MODE
    BaseType_t ok = xTaskCreatePinnedToCore(sd_read_test_task,
                                            "sd_read_test",
                                            SLIDESHOW_TASK_STACK_SIZE,
                                            NULL,
                                            tskIDLE_PRIORITY + 1,
                                            NULL,
                                            1);
#else
    BaseType_t ok = xTaskCreatePinnedToCore(slideshow_task,
                                            "slideshow",
                                            SLIDESHOW_TASK_STACK_SIZE,
                                            NULL,
                                            tskIDLE_PRIORITY + 1,
                                            NULL,
                                            1);
#endif
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to start slideshow task");
    }
}
