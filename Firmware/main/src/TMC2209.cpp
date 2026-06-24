#include "TMC2209.hpp"

uint32_t TMC2209::s_uart_installed_mask = 0;

// ===========================================================================
// Low-level UART
// ===========================================================================

void TMC2209::write_reg(uint8_t reg, uint32_t value)
{
    uint8_t datagram[8];
    datagram[0] = 0x05;       // sync nibble
    datagram[1] = m_addr;     // slave address (0..3)
    datagram[2] = reg | 0x80; // write bit set in MSB
    datagram[3] = (value >> 24) & 0xFF;
    datagram[4] = (value >> 16) & 0xFF;
    datagram[5] = (value >> 8) & 0xFF;
    datagram[6] = (value) & 0xFF;
    datagram[7] = tmc_calc_crc(datagram, 7);

    uart_write_bytes(m_uart, (const char *)datagram, 8);
    uart_wait_tx_done(m_uart, pdMS_TO_TICKS(10));

    // On single-wire half-duplex, our own TX is echoed back on RX.
    // Flush it so the next read_reg() doesn't pick up stale bytes.
    if (m_single_wire)
    {
        uart_flush_input(m_uart);
    }

    // Small inter-frame gap so the driver can settle. 10us is plenty.
    esp_rom_delay_us(100);
}

int64_t TMC2209::read_reg(uint8_t reg)
{
    uint8_t request[4];
    request[0] = 0x05;
    request[1] = m_addr;
    request[2] = reg & 0x7F; // no write bit on reads
    request[3] = tmc_calc_crc(request, 3);

    // Flush any stale bytes from prior transactions
    uart_flush_input(m_uart);

    uart_write_bytes(m_uart, (const char *)request, 4);
    uart_wait_tx_done(m_uart, pdMS_TO_TICKS(10));

    // On single-wire, we'll see our own 4-byte request echoed back first.
    // Read and discard it.
    if (m_single_wire)
    {
        uint8_t echo[4];
        int echoed = uart_read_bytes(m_uart, echo, 4, pdMS_TO_TICKS(20));
        if (echoed < 4)
        {
            ESP_LOGW(TMC_TAG, "addr %u: read 0x%02X echo timeout (%d/4)",
                     m_addr, reg, echoed);
            return -1;
        }
    }

    // The reply is always 8 bytes
    uint8_t reply[8];
    int len = uart_read_bytes(m_uart, reply, 8, pdMS_TO_TICKS(50));
    if (len < 8)
    {
        ESP_LOGW(TMC_TAG, "addr %u: read 0x%02X reply timeout (%d/8)",
                 m_addr, reg, len);
        return -1;
    }

    uint8_t crc = tmc_calc_crc(reply, 7);
    if (crc != reply[7])
    {
        ESP_LOGW(TMC_TAG, "addr %u: read 0x%02X CRC error (exp 0x%02X got 0x%02X)",
                 m_addr, reg, crc, reply[7]);
        return -1;
    }

    // Bytes 3..6 are the 32-bit data, MSB first
    uint32_t value = ((uint32_t)reply[3] << 24) |
                     ((uint32_t)reply[4] << 16) |
                     ((uint32_t)reply[5] << 8) |
                     ((uint32_t)reply[6]);
    return (int64_t)value;
}

// ===========================================================================
// Setup
// ===========================================================================

void TMC2209::init_uart(gpio_num_t tx, gpio_num_t rx)
{
    uart_config_t cfg = {};
    cfg.baud_rate = 115200;
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;

    // Only install the driver once per UART port. Multiple TMC2209 instances
    // sharing a UART (address-multiplexed) all use the same driver.
    uint32_t bit = 1UL << (uint32_t)m_uart;
    if ((s_uart_installed_mask & bit) == 0)
    {
        uart_param_config(m_uart, &cfg);
        uart_set_pin(m_uart, tx, rx,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        uart_driver_install(m_uart, 256, 256, 0, NULL, 0);
        s_uart_installed_mask |= bit;
    }
}

void TMC2209::init_gpio()
{
    // Configure step/dir/en as outputs. If multiple instances share an EN
    // line, reconfiguring it is harmless.
    gpio_config_t io = {};
    io.pin_bit_mask = (1ULL << m_step) | (1ULL << m_dir) | (1ULL << m_en);
    io.mode = GPIO_MODE_OUTPUT;
    io.pull_up_en = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io);

    gpio_set_level(m_en, 1); // start disabled (active low)
    gpio_set_level(m_dir, 0);
    gpio_set_level(m_step, 0);
}

// ===========================================================================
// Bring-up
// ===========================================================================

bool TMC2209::init(uint16_t rms_current_mA, uint8_t microsteps, float hold_ratio)
{
    ESP_LOGI(TMC_TAG, "addr %u: initializing", m_addr);

    // 1. Snapshot IFCNT so we can verify writes land
    int64_t ifcnt_before = read_reg(TMC_REG_IFCNT);
    if (ifcnt_before < 0)
    {
        ESP_LOGE(TMC_TAG, "addr %u: cannot read IFCNT - UART dead?", m_addr);
        return false;
    }
    ESP_LOGI(TMC_TAG, "addr %u: IFCNT before = %lld", m_addr, ifcnt_before);

    // 2. Clear GSTAT (reset / drv_err / uv_cp flags) by writing 1s
    write_reg(TMC_REG_GSTAT, 0x07);

    // 3. GCONF: enable UART control + register-based microstep select
    write_reg(TMC_REG_GCONF, TMC_GCONF_DEFAULT);

    // 4. CHOPCONF + microsteps (set_microsteps writes CHOPCONF)
    set_microsteps(microsteps);

    // 5. Run/hold current
    set_current(rms_current_mA, hold_ratio);

    // 6. TPOWERDOWN: ~0.5s before transitioning to IHOLD
    write_reg(TMC_REG_TPOWERDOWN, 20);

    // 7. Stay in StealthChop (no auto-switch to SpreadCycle)
    write_reg(TMC_REG_TPWMTHRS, 0);

    // 8. PWMCONF with autotune
    write_reg(TMC_REG_PWMCONF, TMC_PWMCONF_DEFAULT);

    // 9. Verify IFCNT incremented by 7 (GSTAT + GCONF + CHOPCONF + IHOLD_IRUN
    //    + TPOWERDOWN + TPWMTHRS + PWMCONF)
    int64_t ifcnt_after = read_reg(TMC_REG_IFCNT);
    int expected_delta = 7;
    int actual_delta = (ifcnt_after >= 0)
                           ? ((int)(ifcnt_after - ifcnt_before) & 0xFF)
                           : -1;
    ESP_LOGI(TMC_TAG, "addr %u: IFCNT after = %lld (delta=%d, expected %d)",
             m_addr, ifcnt_after, actual_delta, expected_delta);
    if (actual_delta != expected_delta)
    {
        ESP_LOGE(TMC_TAG, "addr %u: IFCNT mismatch - writes not landing!",
                 m_addr);
        return false;
    }

    ESP_LOGI(TMC_TAG, "addr %u: init OK", m_addr);
    return true;
}

// ===========================================================================
// Runtime config
// ===========================================================================

void TMC2209::set_current(uint16_t rms_current_mA, float hold_ratio)
{
    // R_sense = 0.11 ohm (BTT TMC2209 V1.2 modules).
    // Full-scale RMS current depends on vsense:
    //   vsense=0 : V_fs = 0.325 V  ->  I_RMS_max ~= 1.92 A
    //   vsense=1 : V_fs = 0.180 V  ->  I_RMS_max ~= 1.06 A  (finer resolution)
    //
    // Formula (datasheet eq.):
    //   I_RMS = (CS + 1)/32 * V_fs / (R_sense + 0.02) / sqrt(2)
    //
    // We pick vsense=1 when the requested current allows IRUN to land in the
    // recommended 16..31 range; otherwise vsense=0.

    const float R_sense = 0.11f;
    const float sqrt2 = 1.41421356f;

    auto compute_cs_raw = [&](float V_fs) -> float
    {
        // Solve for CS:  CS = I*sqrt(2)*32*(R+0.02)/V_fs - 1
        float I = rms_current_mA / 1000.0f;
        return I * sqrt2 * 32.0f * (R_sense + 0.02f) / V_fs - 1.0f;
    };

    auto clamp_cs = [](float raw) -> int
    {
        int cs = (int)(raw + 0.5f);
        if (cs < 0)
            cs = 0;
        if (cs > 31)
            cs = 31;
        return cs;
    };

    // Prefer vsense=1 (high resolution) when the requested current is low
    // enough that IRUN doesn't saturate at 31. We check the *unclamped* CS:
    // if it exceeds 31, we need vsense=0 to reach the target.
    float cs_low_raw = compute_cs_raw(0.180f); // vsense=1 candidate
    bool vsense = (cs_low_raw <= 31.0f);

    int irun;
    if (vsense)
    {
        irun = clamp_cs(cs_low_raw);
    }
    else
    {
        irun = clamp_cs(compute_cs_raw(0.325f));
    }
    int ihold = (int)(irun * hold_ratio + 0.5f);
    if (ihold < 0)
        ihold = 0;
    if (ihold > 31)
        ihold = 31;

    // Update CHOPCONF.vsense (bit 17) without touching the other bits
    int64_t chop = read_reg(TMC_REG_CHOPCONF);
    if (chop < 0)
        chop = TMC_CHOPCONF_BASE;
    uint32_t chopconf = (uint32_t)chop;
    if (vsense)
        chopconf |= (1UL << 17);
    else
        chopconf &= ~(1UL << 17);
    write_reg(TMC_REG_CHOPCONF, chopconf);

    // Write IHOLD_IRUN
    const uint8_t iholddelay = 8; // ~256ms smooth ramp to hold current
    uint32_t v = ((uint32_t)iholddelay << 16) | ((uint32_t)irun << 8) | ((uint32_t)ihold);
    write_reg(TMC_REG_IHOLD_IRUN, v);

    ESP_LOGI(TMC_TAG,
             "addr %u: current set to %u mA RMS (vsense=%d, IRUN=%d, IHOLD=%d)",
             m_addr, rms_current_mA, vsense ? 1 : 0, irun, ihold);
}

void TMC2209::set_microsteps(uint16_t microsteps)
{
    // Map microsteps -> MRES code (bits 27..24 of CHOPCONF)
    uint8_t mres;
    switch (microsteps)
    {
    case 256:
        mres = 0;
        break;
    case 128:
        mres = 1;
        break;
    case 64:
        mres = 2;
        break;
    case 32:
        mres = 3;
        break;
    case 16:
        mres = 4;
        break;
    case 8:
        mres = 5;
        break;
    case 4:
        mres = 6;
        break;
    case 2:
        mres = 7;
        break;
    case 1:
        mres = 8;
        break;
    default:
        ESP_LOGW(TMC_TAG, "addr %u: invalid microsteps %u, using 16",
                 m_addr, microsteps);
        mres = 4;
        break;
    }

    int64_t current = read_reg(TMC_REG_CHOPCONF);
    uint32_t chopconf = (current >= 0) ? (uint32_t)current : TMC_CHOPCONF_BASE;

    chopconf &= ~(0xFUL << 24);         // clear MRES
    chopconf |= ((uint32_t)mres << 24); // set new MRES
    write_reg(TMC_REG_CHOPCONF, chopconf);

    ESP_LOGI(TMC_TAG, "addr %u: microsteps=%u (MRES=%u, CHOPCONF=0x%08lX)",
             m_addr, microsteps, mres, (unsigned long)chopconf);
}

// ===========================================================================
// Step generation (blocking — for testing; replace with timer-driven later)
// ===========================================================================

void TMC2209::step_motor(int steps, uint32_t delay_us)
{
    for (int i = 0; i < steps; i++)
    {
        gpio_set_level(m_step, 1);
        esp_rom_delay_us(delay_us);
        gpio_set_level(m_step, 0);
        esp_rom_delay_us(delay_us);
    }
}

// ===========================================================================
// Diagnostics
// ===========================================================================

void TMC2209::print_drv_status()
{
    int64_t s = read_drv_status();
    if (s < 0)
    {
        printf("[TMC %u] DRV_STATUS read failed\n", m_addr);
        return;
    }
    uint32_t v = (uint32_t)s;
    printf("[TMC %u] DRV_STATUS = 0x%08lX\n", m_addr, (unsigned long)v);

    if (v & (1UL << 0))
        printf("  otpw     - overtemp pre-warning (120C)\n");
    if (v & (1UL << 1))
        printf("  ot       - OVERTEMP SHUTDOWN (150C)\n");
    if (v & (1UL << 2))
        printf("  s2ga     - short to GND on coil A\n");
    if (v & (1UL << 3))
        printf("  s2gb     - short to GND on coil B\n");
    if (v & (1UL << 4))
        printf("  s2vsa    - short to supply on coil A\n");
    if (v & (1UL << 5))
        printf("  s2vsb    - short to supply on coil B\n");
    if (v & (1UL << 6))
        printf("  ola      - open load A (motor disconnected?)\n");
    if (v & (1UL << 7))
        printf("  olb      - open load B\n");
    if (v & (1UL << 8))
        printf("  t120     - 120C reached\n");
    if (v & (1UL << 9))
        printf("  t143     - 143C reached\n");
    if (v & (1UL << 10))
        printf("  t150     - 150C reached\n");
    if (v & (1UL << 11))
        printf("  t157     - 157C reached\n");

    uint8_t cs_actual = (v >> 16) & 0x1F;
    bool stealth = (v >> 30) & 1;
    bool stst = (v >> 31) & 1;
    printf("  CS_ACTUAL = %u  stealth=%d  stst=%d\n", cs_actual, stealth, stst);
}