#include "MotorController.hpp"

#include <string.h>
#include "esp_log.h"
#include "esp_rom_sys.h"

static const char *TAG = "motion";

// ---------------------------------------------------------------------------
// Shared step generator
// ---------------------------------------------------------------------------
//
// ESP32-S3 has only 4 hardware GPTimers, so we cannot use one timer per
// axis. Instead, a SINGLE timer fires at TICK_HZ and the ISR decides on
// each tick which axes need a step pulse.
//
// Each axis accumulates `velocity_sps` into `step_accumulator` every tick.
// When the accumulator >= TICK_HZ, we emit one step and subtract TICK_HZ.
// This is the standard "Bresenham stepping" pattern used by Marlin/Klipper.
//
// TICK_HZ must be >= max desired step rate per axis. 20 kHz gives:
//   - 50 µs between ticks, plenty of headroom in the ISR
//   - Max output 20,000 steps/sec/axis — way above any realistic robot arm move
//
constexpr uint32_t TICK_HZ = 20000;                    // 20 kHz step generator
constexpr uint64_t TICK_PERIOD_US = 1000000 / TICK_HZ; // = 50 µs

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------

AxisState axes[NUM_AXES] = {};
gpio_num_t step_pins[NUM_AXES] = {};
gptimer_handle_t step_timers[NUM_AXES] = {}; // unused; kept for header ABI
TMC2209 *drivers[NUM_AXES] = {};

QueueHandle_t motion_queue = nullptr;
TaskHandle_t motion_task_handle = nullptr;

// Step generator state — accumulator-based fractional step rate
static volatile int32_t step_accumulator[NUM_AXES] = {};

// The single shared timer
static gptimer_handle_t step_tick_timer = nullptr;
static volatile uint8_t active_axis_mask = 0; // bit i set if axis i is running

// ---------------------------------------------------------------------------
// Forward declarations — IRAM_ATTR on both decl and def
// ---------------------------------------------------------------------------

static void motion_task(void *arg);
static void handle_command(const MotionCmd &cmd);
static void start_axis(int axis);
static void stop_axis(int axis);
static void init_step_generator();

// ===========================================================================
// Public API
// ===========================================================================

void init_motor_controller(TMC2209 *driver_instances[NUM_AXES])
{
    for (int i = 0; i < NUM_AXES; i++)
    {
        drivers[i] = driver_instances[i];
        step_pins[i] = drivers[i]->step_pin();
        axes[i] = {};
        step_accumulator[i] = 0;
    }

    // Shared EN pin — held HIGH (disarmed) at boot
    gpio_config_t io = {};
    io.pin_bit_mask = (1ULL << ARM_ENABLE_PIN);
    io.mode = GPIO_MODE_OUTPUT;
    io.pull_up_en = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io);
    gpio_set_level(ARM_ENABLE_PIN, 1);

    // ONE shared step generator timer (not 6)
    init_step_generator();

    motion_queue = xQueueCreate(32, sizeof(MotionCmd));
    configASSERT(motion_queue != nullptr);

    BaseType_t ok = xTaskCreatePinnedToCore(
        motion_task, "motion", 4096, nullptr,
        configMAX_PRIORITIES - 2,
        &motion_task_handle,
        1);
    configASSERT(ok == pdPASS);

    ESP_LOGI(TAG, "motor controller ready, %d axes, %u Hz tick, disarmed",
             NUM_AXES, (unsigned)TICK_HZ);
}

bool enqueue_command(const MotionCmd &cmd)
{
    if (motion_queue == nullptr)
        return false;
    return xQueueSend(motion_queue, &cmd, pdMS_TO_TICKS(10)) == pdTRUE;
}

// ===========================================================================
// Motion task
// ===========================================================================

static void motion_task(void *)
{
    MotionCmd cmd;
    while (true)
    {
        if (xQueueReceive(motion_queue, &cmd, portMAX_DELAY) == pdTRUE)
        {
            handle_command(cmd);
        }
    }
}

// ---------------------------------------------------------------------------
// Shared ISR — runs at TICK_HZ. Iterates active axes and emits steps.
// ---------------------------------------------------------------------------
static bool IRAM_ATTR step_tick_isr(gptimer_handle_t /*timer*/,
                                    const gptimer_alarm_event_data_t * /*edata*/,
                                    void * /*user_data*/)
{
    uint8_t mask = active_axis_mask;
    if (mask == 0)
        return false; // no work

    BaseType_t woken = pdFALSE;

    for (int axis = 0; axis < NUM_AXES; axis++)
    {
        if ((mask & (1u << axis)) == 0)
            continue;

        AxisState &a = axes[axis];

        // Asked to stop?
        if (!a.moving)
        {
            active_axis_mask &= ~(1u << axis);
            continue;
        }

        // MOVE_TO target reached?
        if (!a.jog_mode)
        {
            int32_t remaining = a.target_steps - a.current_steps;
            bool done = (a.direction > 0 && remaining <= 0) ||
                        (a.direction < 0 && remaining >= 0);
            if (done)
            {
                a.moving = false;
                active_axis_mask &= ~(1u << axis);

                MotionCmd done_cmd = {};
                done_cmd.type = CmdType::MOVE_DONE;
                done_cmd.axis = (uint8_t)axis;
                xQueueSendFromISR(motion_queue, &done_cmd, &woken);
                continue;
            }
        }

        // Bresenham step accumulator
        step_accumulator[axis] += (int32_t)a.velocity_sps;
        if (step_accumulator[axis] >= (int32_t)TICK_HZ)
        {
            step_accumulator[axis] -= (int32_t)TICK_HZ;

            gpio_set_level(step_pins[axis], 1);
            esp_rom_delay_us(1);
            gpio_set_level(step_pins[axis], 0);

            a.current_steps += a.direction;
        }
    }

    return woken == pdTRUE;
}

// ===========================================================================
// Command handler
// ===========================================================================

static void handle_command(const MotionCmd &cmd)
{
    switch (cmd.type)
    {
    case CmdType::ARM:
        gpio_set_level(ARM_ENABLE_PIN, 0);
        ESP_LOGI(TAG, "armed");
        return;

    case CmdType::DISARM:
        for (int i = 0; i < NUM_AXES; i++)
        {
            axes[i].moving = false;
            stop_axis(i);
        }
        gpio_set_level(ARM_ENABLE_PIN, 1);
        ESP_LOGI(TAG, "disarmed");
        return;

    case CmdType::STOP_ALL:
        for (int i = 0; i < NUM_AXES; i++)
        {
            axes[i].moving = false;
            axes[i].jog_mode = false;
            stop_axis(i);
        }
        ESP_LOGI(TAG, "stop all");
        return;

    default:
        break;
    }

    if (cmd.axis >= NUM_AXES)
    {
        ESP_LOGW(TAG, "invalid axis %u", cmd.axis);
        return;
    }
    AxisState &state = axes[cmd.axis];

    switch (cmd.type)
    {
    case CmdType::JOG:
    {
        if (cmd.velocity_sps == 0 || cmd.direction == 0)
        {
            ESP_LOGW(TAG, "JOG axis %u: bad params", cmd.axis);
            return;
        }
        int8_t dir = (cmd.direction > 0) ? 1 : -1;
        uint32_t vel = cmd.velocity_sps;
        if (vel > TICK_HZ)
            vel = TICK_HZ; // can't step faster than tick rate

        drivers[cmd.axis]->set_direction(dir > 0 ? 1 : 0);
        state.direction = dir;
        state.velocity_sps = vel;
        state.jog_mode = true;
        state.moving = true;
        step_accumulator[cmd.axis] = 0;

        start_axis(cmd.axis);
        ESP_LOGI(TAG, "JOG axis %u dir=%d vel=%u sps",
                 cmd.axis, dir, (unsigned)vel);
        break;
    }

    case CmdType::MOVE_TO:
    {
        int32_t delta = cmd.target_steps - state.current_steps;
        if (delta == 0)
        {
            ESP_LOGI(TAG, "MOVE_TO axis %u: already at target", cmd.axis);
            return;
        }
        int8_t dir = (delta > 0) ? 1 : -1;
        uint32_t vel = (cmd.velocity_sps > 0) ? cmd.velocity_sps : 800;
        if (vel > TICK_HZ)
            vel = TICK_HZ;

        drivers[cmd.axis]->set_direction(dir > 0 ? 1 : 0);
        state.target_steps = cmd.target_steps;
        state.velocity_sps = vel;
        state.direction = dir;
        state.jog_mode = false;
        state.moving = true;
        step_accumulator[cmd.axis] = 0;

        start_axis(cmd.axis);
        ESP_LOGI(TAG, "MOVE_TO axis %u target=%ld vel=%u sps",
                 cmd.axis, (long)cmd.target_steps, (unsigned)vel);
        break;
    }

    case CmdType::STOP:
        state.moving = false;
        state.jog_mode = false;
        stop_axis(cmd.axis);
        ESP_LOGI(TAG, "STOP axis %u at step %ld",
                 cmd.axis, (long)state.current_steps);
        break;

    case CmdType::MOVE_DONE:
        ESP_LOGI(TAG, "MOVE_DONE axis %u at step %ld",
                 cmd.axis, (long)state.current_steps);
        break;

    default:
        break;
    }
}

// ===========================================================================
// Step generator — ONE shared timer
// ===========================================================================

static void init_step_generator()
{
    gptimer_config_t cfg = {};
    cfg.clk_src = GPTIMER_CLK_SRC_DEFAULT;
    cfg.direction = GPTIMER_COUNT_UP;
    cfg.resolution_hz = 1000000; // 1 µs resolution

    ESP_ERROR_CHECK(gptimer_new_timer(&cfg, &step_tick_timer));

    gptimer_event_callbacks_t cbs = {};
    cbs.on_alarm = step_tick_isr;
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(
        step_tick_timer, &cbs, nullptr));

    ESP_ERROR_CHECK(gptimer_enable(step_tick_timer));

    gptimer_alarm_config_t alarm = {};
    alarm.alarm_count = TICK_PERIOD_US;
    alarm.reload_count = 0;
    alarm.flags.auto_reload_on_alarm = true;
    ESP_ERROR_CHECK(gptimer_set_alarm_action(step_tick_timer, &alarm));

    // Timer runs continuously; axes opt-in via active_axis_mask
    ESP_ERROR_CHECK(gptimer_start(step_tick_timer));
}

static void start_axis(int axis)
{
    active_axis_mask |= (1u << axis);
}

static void stop_axis(int axis)
{
    active_axis_mask &= ~(1u << axis);
}