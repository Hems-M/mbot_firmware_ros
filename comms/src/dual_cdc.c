#include "comms/dual_cdc.h"
#include "tusb.h"
#include "pico/mutex.h"
#include "pico/time.h"
#include "pico/bootrom.h"
#include <string.h>

// Software BOOTSEL: when the Raspberry Pi opens the debug console (CDC0,
// /dev/mbot_debug) at 1200 baud, reboot the RP2040 into its USB bootloader
// (BOOTSEL mode). The Pi can then flash new firmware over the existing USB
// cable, with no buttons and no GPIO wires between the Pi and the board.
// 1200 baud is the same "magic baud rate" that pico_stdio_usb and Arduino use.
// pico_stdio_usb's own version of this callback is compiled out because this
// project links tinyusb_device directly, so we provide it here.
#define MBOT_BOOTSEL_MAGIC_BAUD 1200

void tud_cdc_line_coding_cb(uint8_t itf, cdc_line_coding_t const* p_line_coding) {
    if (itf == 0 && p_line_coding->bit_rate == MBOT_BOOTSEL_MAGIC_BAUD) {
        // Full chip reset into the ROM bootloader. Motor PWM outputs return to
        // their reset state, so the motors stop. Both USB boot interfaces
        // (mass storage and PICOBOOT for picotool) stay enabled.
        reset_usb_boot(0, 0);
    }
}

// Mutex to protect CDC access
static mutex_t dual_cdc_mutex;

// Redirect the standard C library output functions to custom CDC implementation
int _write(int fd, const void *buf, size_t count) {
    // Use our custom function to write to CDC0
    dual_cdc_write_chars(0, buf, count);
    return count;
}

// Wait for a given number of milliseconds but still service USB
void mbot_wait_ms(uint32_t ms) {
    uint32_t start = time_us_32();
    while ((time_us_32() - start) < ms * 1000) {
        dual_cdc_task();
        sleep_ms(5);
    }
}

bool dual_cdc_init(void) {
    // Initialize the mutex
    mutex_init(&dual_cdc_mutex);
    
    // TinyUSB may already be initialized by stdio_init_all()
    tusb_init();
    
    return true;
}

bool dual_cdc_connected(void) {
    return tud_cdc_n_connected(1);
}

void dual_cdc_write_chars(int itf, const char *buf, int length) {
    // Validate interface number
    if (itf < 0 || itf > 1) return;
    
    mutex_enter_blocking(&dual_cdc_mutex);
    
    if (tud_cdc_n_connected(itf)) {
        for (int i = 0; i < length;) {
            int n = length - i;
            int avail = (int)tud_cdc_n_write_available(itf);
            if (n > avail) n = avail;
            
            if (n) {
                int n2 = (int)tud_cdc_n_write(itf, buf + i, (uint32_t)n);
                tud_task();
                tud_cdc_n_write_flush(itf);
                i += n2;
            } else {
                tud_task();
                tud_cdc_n_write_flush(itf);
            }
        }
    }
    
    mutex_exit(&dual_cdc_mutex);
}

int dual_cdc_read_chars(int itf, char *buf, int length) {
    // Validate interface number
    if (itf < 0 || itf > 1) return -1;
    
    mutex_enter_blocking(&dual_cdc_mutex);
    
    int rc = -1;
    if (tud_cdc_n_connected(itf) && tud_cdc_n_available(itf)) {
        int count = (int)tud_cdc_n_read(itf, buf, (uint32_t)length);
        rc = count ? count : -1;
    }
    
    mutex_exit(&dual_cdc_mutex);
    return rc;
}

int dual_cdc_available(int itf) {
    // Validate interface number
    if (itf < 0 || itf > 1) return 0;
    
    return tud_cdc_n_available(itf);
}

void dual_cdc_task(void) {
    // Process TinyUSB tasks with mutex to prevent concurrent access
    if (mutex_try_enter(&dual_cdc_mutex, NULL)) {
        tud_task();
        mutex_exit(&dual_cdc_mutex);
    }
} 