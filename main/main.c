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

#define MAX_IMAGES 256
#define IMAGE_CHANGE_MS 5000
#define LVGL_FS_DRIVE_LETTER 'S'
#define SLIDESHOW_TASK_STACK_SIZE 24576
#define ENABLE_PNG_IN_SLIDESHOW 1
#define SD_READ_TEST_MODE 0
#define SD_READ_CHUNK_SIZE 4096
#define SD_MOUNT_RETRY_DELAY_MS 1000
#define PRELOAD_MAX_FILE_BYTES (180 * 1024)
#define SLIDESHOW_VERBOSE_LOGS 0
#define STRICT_RAM_PRELOAD_MODE 1
#define DISPLAY_DIAMETER 466
#define DISPLAY_RADIUS (DISPLAY_DIAMETER / 2)
#define MAX_MP3_PRELOAD_BYTES (5 * 1024 * 1024)
#define TOUCH_DEBOUNCE_MS 300
#define MP3_DEC_IN_CHUNK 1024
#define MP3_DEC_OUT_CHUNK 4096

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

static lv_obj_t *s_screen_obj;
static lv_obj_t *s_image_obj;
static lv_obj_t *s_status_label;
static image_entry_t s_images[MAX_IMAGES];
static size_t s_image_count;
static size_t s_current_index;
static sdmmc_card_t *s_sdcard;
static esp_codec_dev_handle_t s_speaker_codec_dev;
static bool s_audio_initialized = false;
static bool s_audio_decoders_registered = false;
static bool s_audio_playing = false;
static TickType_t s_last_touch_tick = 0;

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

static esp_err_t mount_sdcard_with_retries(void)
{
    esp_err_t mount_err = ESP_FAIL;
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

static bool jpeg_get_dimensions(const uint8_t *data, size_t size, uint16_t *out_w, uint16_t *out_h)
{
    if (data == NULL || size < 4 || out_w == NULL || out_h == NULL) {
        return false;
    }

    if (data[0] != 0xFF || data[1] != 0xD8) {
        return false;
    }

    size_t pos = 2;
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

// Build an MP3 filename from an image filename by replacing extension with .mp3
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

// Try to load MP3 file matching the image basename
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

    // Allocate and read MP3 data
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

// Returns LVGL scale (256 = 1:1) so image corners just touch the circular display edge.
// scale = DISPLAY_RADIUS / (half-diagonal of image)
static uint32_t compute_fill_scale(uint16_t img_w, uint16_t img_h)
{
    if (img_w == 0 || img_h == 0) return LV_SCALE_NONE;
    float half_diag = sqrtf((float)img_w * (float)img_w / 4.0f + (float)img_h * (float)img_h / 4.0f);
    float scale = (float)DISPLAY_RADIUS / half_diag;
    uint32_t lvgl_scale = (uint32_t)(scale * (float)LV_SCALE_NONE + 0.5f);
    if (lvgl_scale < 1) lvgl_scale = 1;
    return lvgl_scale;
}

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

static bool has_ext(const char *name, const char *ext)
{
    const char *dot = strrchr(name, '.');
    if (dot == NULL) {
        return false;
    }
    return strcasecmp(dot, ext) == 0;
}

static int file_name_cmp(const void *a, const void *b)
{
    const image_entry_t *lhs = (const image_entry_t *)a;
    const image_entry_t *rhs = (const image_entry_t *)b;
    return strcasecmp(lhs->file_name, rhs->file_name);
}

static char *dup_if_decodable(const char *src)
{
    lv_image_header_t header;
    if (lv_image_decoder_get_info(src, &header) != LV_RESULT_OK) {
        return NULL;
    }

    return strdup(src);
}

static char *resolve_image_src(const char *file_name)
{
    char src_path[384];
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

static void update_image_and_status(size_t index)
{
    if (s_image_count == 0) {
        lv_label_set_text(s_status_label, "No images found in /sdcard");
        return;
    }

    s_current_index = index % s_image_count;

    const void *image_src = s_images[s_current_index].src;
    bool image_ok = image_src != NULL;
    if (image_ok) {
        lv_obj_set_style_bg_color(s_screen_obj, s_images[s_current_index].backdrop_color, 0);

        int32_t pivot_x = (s_images[s_current_index].img_w > 0) ? (s_images[s_current_index].img_w / 2) : (lv_obj_get_width(s_image_obj) / 2);
        int32_t pivot_y = (s_images[s_current_index].img_h > 0) ? (s_images[s_current_index].img_h / 2) : (lv_obj_get_height(s_image_obj) / 2);

        lv_image_set_src(s_image_obj, image_src);
        uint32_t scale = s_images[s_current_index].display_scale;
        if (scale == 0) scale = LV_SCALE_NONE;
        lv_image_set_scale(s_image_obj, LV_SCALE_NONE);
        lv_obj_set_style_transform_scale(s_image_obj, (int32_t)scale, LV_PART_MAIN);
        lv_obj_set_style_transform_pivot_x(s_image_obj, pivot_x, LV_PART_MAIN);
        lv_obj_set_style_transform_pivot_y(s_image_obj, pivot_y, LV_PART_MAIN);
        lv_obj_center(s_image_obj);
    } else {
        lv_image_set_src(s_image_obj, NULL);
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

static void play_image_sound(size_t img_idx)
{
    if (img_idx >= s_image_count) return;

    if (s_images[img_idx].mp3_data != NULL && s_images[img_idx].mp3_size > 0) {
        if (!s_audio_initialized || s_speaker_codec_dev == NULL) {
            ESP_LOGW(TAG, "Audio not initialized, cannot play MP3");
            return;
        }

        if (s_audio_playing) {
            ESP_LOGI(TAG, "Audio is already playing, ignoring touch");
            return;
        }

        s_audio_playing = true;

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
            s_audio_playing = false;
            return;
        }

        uint8_t *out_buf = (uint8_t *)malloc(MP3_DEC_OUT_CHUNK);
        if (out_buf == NULL) {
            ESP_LOGW(TAG, "No memory for MP3 decode output buffer");
            esp_audio_simple_dec_close(decoder);
            s_audio_playing = false;
            return;
        }

        size_t total_pcm_written = 0;
        size_t offset = 0;
        bool codec_opened = false;

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
                    .len = MP3_DEC_OUT_CHUNK,
                };

                dec_ret = esp_audio_simple_dec_process(decoder, &raw, &out);
                if (dec_ret == ESP_AUDIO_ERR_BUFF_NOT_ENOUGH) {
                    uint8_t *new_out = (uint8_t *)realloc(out_buf, out.needed_size);
                    if (new_out == NULL) {
                        ESP_LOGW(TAG, "Failed to grow MP3 output buffer to %u", (unsigned)out.needed_size);
                        dec_ret = ESP_AUDIO_ERR_MEM_LACK;
                        break;
                    }
                    out_buf = new_out;
                    continue;
                }

                if (dec_ret != ESP_AUDIO_ERR_OK) {
                    ESP_LOGW(TAG, "MP3 decode failed: %d", (int)dec_ret);
                    break;
                }

                if (out.decoded_size > 0) {
                    if (!codec_opened) {
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

                        esp_codec_dev_set_out_vol(s_speaker_codec_dev, 50);
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

        s_audio_playing = false;
    }
}

// Initialize audio subsystem
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
    return ESP_OK;
}

// LVGL event callback for touch-triggered sound playback
static void on_image_touched(lv_event_t *event)
{
    if (event == NULL) return;

    lv_event_code_t code = lv_event_get_code(event);
    if (code == LV_EVENT_CLICKED) {
        TickType_t now = xTaskGetTickCount();
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
        ESP_LOGI(TAG, "Screen touched, playing image sound");
        play_image_sound(s_current_index);
    }
}

static void slideshow_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "Starting SD card slideshow");

    lv_display_t *display = bsp_display_start();
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
    ESP_ERROR_CHECK(preload_images_into_memory());

    if (all_images_are_ram_backed()) {
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

    s_image_obj = lv_image_create(s_screen_obj);
    lv_image_set_antialias(s_image_obj, true);
    lv_obj_add_flag(s_image_obj, LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_center(s_image_obj);

    s_status_label = lv_label_create(s_screen_obj);
    lv_obj_set_width(s_status_label, lv_pct(96));
    lv_label_set_long_mode(s_status_label, LV_LABEL_LONG_SCROLL_CIRCULAR);
    lv_obj_set_style_text_color(s_status_label, lv_color_hex(0xD0D0D0), 0);
    lv_obj_align(s_status_label, LV_ALIGN_TOP_MID, 0, 10);

    update_image_and_status(0);

    lv_refr_now(display);
    bsp_display_unlock();

    uint32_t elapsed_ms = 0;
    while (1) {
        UBaseType_t stack_words_free = uxTaskGetStackHighWaterMark(NULL);
        if (stack_words_free < 512) {
            ESP_LOGW(TAG, "Low slideshow stack watermark: %u words", (unsigned)stack_words_free);
        }

        if (s_image_count > 1) {
            elapsed_ms += 1000;
            if (elapsed_ms >= IMAGE_CHANGE_MS) {
                elapsed_ms = 0;
                bsp_display_lock(portMAX_DELAY);
                update_image_and_status((s_current_index + 1) % s_image_count);
                lv_refr_now(display);
                bsp_display_unlock();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

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
