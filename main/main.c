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
#include "lvgl.h"

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
} image_entry_t;

static lv_obj_t *s_image_obj;
static lv_obj_t *s_status_label;
static image_entry_t s_images[MAX_IMAGES];
static size_t s_image_count;
static size_t s_current_index;
static sdmmc_card_t *s_sdcard;

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
        lv_image_set_src(s_image_obj, image_src);
        uint32_t scale = s_images[s_current_index].display_scale;
        if (scale == 0) scale = LV_SCALE_NONE;
        lv_image_set_scale(s_image_obj, LV_SCALE_NONE);
        lv_obj_set_style_transform_scale(s_image_obj, (int32_t)scale, LV_PART_MAIN);
        int32_t pivot_x = (s_images[s_current_index].img_w > 0) ? (s_images[s_current_index].img_w / 2) : (lv_obj_get_width(s_image_obj) / 2);
        int32_t pivot_y = (s_images[s_current_index].img_h > 0) ? (s_images[s_current_index].img_h / 2) : (lv_obj_get_height(s_image_obj) / 2);
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
    if (touch_indev != NULL) {
        esp_err_t touch_remove_err = lvgl_port_remove_touch(touch_indev);
        if (touch_remove_err == ESP_OK) {
            ESP_LOGI(TAG, "Touch input disabled for slideshow stability testing");
        } else {
            ESP_LOGW(TAG, "Failed to disable touch input: %s", esp_err_to_name(touch_remove_err));
        }
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
    lv_obj_t *screen = lv_screen_active();
#else
    lv_obj_t *screen = lv_scr_act();
#endif

    lv_obj_set_style_bg_color(screen, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);

    s_status_label = lv_label_create(screen);
    lv_obj_set_width(s_status_label, lv_pct(96));
    lv_label_set_long_mode(s_status_label, LV_LABEL_LONG_SCROLL_CIRCULAR);
    lv_obj_set_style_text_color(s_status_label, lv_color_hex(0xD0D0D0), 0);
    lv_obj_align(s_status_label, LV_ALIGN_TOP_MID, 0, 10);

    s_image_obj = lv_image_create(screen);
    lv_image_set_antialias(s_image_obj, true);
    lv_obj_center(s_image_obj);

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
