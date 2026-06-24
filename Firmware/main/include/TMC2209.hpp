#pragma once

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"
#include "esp_log.h"

// ---------------------------------------------------------------------------
// TMC2209 register addresses
// ---------------------------------------------------------------------------
#define TMC_REG_GCONF 0x00
#define TMC_REG_GSTAT 0x01
#define TMC_REG_IFCNT 0x02
#define TMC_REG_SLAVECONF 0x03
#define TMC_REG_IOIN 0x06
#define TMC_REG_IHOLD_IRUN 0x10
#define TMC_REG_TPOWERDOWN 0x11
#define TMC_REG_TSTEP 0x12
#define TMC_REG_TPWMTHRS 0x13
#define TMC_REG_TCOOLTHRS 0x14
#define TMC_REG_SGTHRS 0x40
#define TMC_REG_SG_RESULT 0x41
#define TMC_REG_COOLCONF 0x42
#define TMC_REG_MSCNT 0x6A
#define TMC_REG_CHOPCONF 0x6C
#define TMC_REG_DRV_STATUS 0x6F
#define TMC_REG_PWMCONF 0x70

// ---------------------------------------------------------------------------
// Recommended register defaults
// ---------------------------------------------------------------------------
//
// GCONF: pdn_disable=1 (UART takes over PDN), mstep_reg_select=1 (MRES from
//        CHOPCONF, not from MS1/MS2 pins), multistep_filt=1 (step filter on)
#define TMC_GCONF_DEFAULT 0x000001C0UL
//
// CHOPCONF base value (microstep bits cleared, set by set_microsteps()):
//   TOFF=3, HSTRT=5, HEND=0, TBL=2, vsense=0, intpol=1
//   = 0001 0000 0000 0011 0000 0000 0101 0011  (bits 27..24 = MRES = 0)
//   When MRES is OR'd in (e.g. 4<<24 for 16 µsteps): 0x14010053
#define TMC_CHOPCONF_BASE 0x10010053UL
//
// PWMCONF: pwm_autoscale=1, pwm_autograd=1, pwm_freq=1, PWM_GRAD=0x0D,
//          PWM_OFS=0x24, PWM_REG=4, PWM_LIM=12 (datasheet recommended)
#define TMC_PWMCONF_DEFAULT 0xC10D0024UL

static const char *TMC_TAG = "TMC2209";

// ---------------------------------------------------------------------------
// CRC8 for TMC UART (poly 0x07, init 0x00, LSB-first processing)
// inline so multiple TUs can include this header safely
// ---------------------------------------------------------------------------
inline uint8_t tmc_calc_crc(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++)
    {
        uint8_t byte = data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if ((crc >> 7) ^ (byte & 0x01))
                crc = (crc << 1) ^ 0x07;
            else
                crc = (crc << 1);
            byte >>= 1;
        }
    }
    return crc;
}

class TMC2209
{
public:
    // ----- construction ----------------------------------------------------
    //
    // tx_pin / rx_pin: if your hardware uses a true single-wire bus (TX and
    //   RX merged through a 1k resistor onto PDN_UART), pass the same GPIO
    //   for both and set single_wire=true. Otherwise pass separate pins.
    //
    // shared_uart: pass true if multiple TMC2209 instances share this UART
    //   peripheral. Only the FIRST instance on a given UART should call
    //   uart_driver_install (handled automatically here).
    //
    TMC2209(uart_port_t uart_num,
            gpio_num_t tx_pin,
            gpio_num_t rx_pin,
            gpio_num_t step_pin,
            gpio_num_t dir_pin,
            gpio_num_t en_pin,
            uint8_t addr,
            bool single_wire = true)
        : m_uart(uart_num),
          m_step(step_pin),
          m_dir(dir_pin),
          m_en(en_pin),
          m_addr(addr & 0x03),
          m_single_wire(single_wire)
    {
        init_uart(tx_pin, rx_pin);
        init_gpio();
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    // ----- bring-up --------------------------------------------------------
    //
    // Call once after the constructor. Returns false if UART communication
    // could not be verified (IFCNT didn't increment).
    bool init(uint16_t rms_current_mA = 800,
              uint8_t microsteps = 16,
              float hold_current_ratio = 0.5f);

    // ----- enable / disable ------------------------------------------------
    void enable() { gpio_set_level(m_en, 0); } // EN is active low
    void disable() { gpio_set_level(m_en, 1); }

    // ----- step / direction -----------------------------------------------
    void set_direction(int dir) { gpio_set_level(m_dir, dir ? 1 : 0); }
    void step_motor(int steps, uint32_t delay_us = 200);

    // ----- runtime config --------------------------------------------------
    //
    // Set RMS current in milliamps. Picks IRUN and vsense automatically
    // assuming R_sense = 0.11 ohm (BTT TMC2209 V1.2 modules).
    void set_current(uint16_t rms_current_mA, float hold_ratio = 0.5f);

    // Set microsteps. Valid values: 1, 2, 4, 8, 16, 32, 64, 128, 256.
    // The motor should be stopped when calling this.
    void set_microsteps(uint16_t microsteps);

    // StealthChop <-> SpreadCycle auto-switch threshold. 0 = stay in
    // StealthChop forever (recommended for robot arm).
    void set_pwm_threshold(uint32_t tpwmthrs) { write_reg(TMC_REG_TPWMTHRS, tpwmthrs); }

    // StallGuard configuration for sensorless homing.
    void set_stallguard(uint8_t threshold, uint32_t coolthrs = 0x000FFFFFUL)
    {
        write_reg(TMC_REG_TCOOLTHRS, coolthrs);
        write_reg(TMC_REG_SGTHRS, threshold);
    }

    // ----- diagnostics -----------------------------------------------------
    int64_t read_drv_status() { return read_reg(TMC_REG_DRV_STATUS); }
    int64_t read_gstat() { return read_reg(TMC_REG_GSTAT); }
    int64_t read_ifcnt() { return read_reg(TMC_REG_IFCNT); }
    int64_t read_sg_result() { return read_reg(TMC_REG_SG_RESULT); }
    int64_t read_mscnt() { return read_reg(TMC_REG_MSCNT); }

    // Pretty-print DRV_STATUS flags to console.
    void print_drv_status();

    uint8_t address() const { return m_addr; }

    // Pin accessors — needed by the step generator so it can toggle STEP
    // directly from the ISR without going through any class method.
    gpio_num_t step_pin() const { return m_step; }
    gpio_num_t dir_pin() const { return m_dir; }
    gpio_num_t en_pin() const { return m_en; }

private:
    // ----- low-level UART --------------------------------------------------
    void write_reg(uint8_t reg, uint32_t value);
    int64_t read_reg(uint8_t reg);

    // ----- setup helpers ---------------------------------------------------
    void init_uart(gpio_num_t tx, gpio_num_t rx);
    void init_gpio();

    // ----- members ---------------------------------------------------------
    uart_port_t m_uart;
    gpio_num_t m_step;
    gpio_num_t m_dir;
    gpio_num_t m_en;
    uint8_t m_addr;
    bool m_single_wire;

    // Track installed UART ports so multiple instances on the same UART
    // don't double-install the driver. One bit per UART port.
    static uint32_t s_uart_installed_mask;
};