#pragma once

#include <stdint.h>
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "TMC2209.hpp"

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

constexpr int NUM_AXES = 6;

// Shared EN pin — all 6 drivers share this one line. Pulling it LOW arms
// the entire arm; HIGH frees all motors.
constexpr gpio_num_t ARM_ENABLE_PIN = GPIO_NUM_13; // <-- adjust to your PCB

// ---------------------------------------------------------------------------
// Command types
// ---------------------------------------------------------------------------

enum class CmdType : uint8_t
{
    JOG,       // run at a fixed velocity in a direction until STOP
    MOVE_TO,   // move to an absolute step target, then stop
    STOP,      // halt a single axis immediately
    STOP_ALL,  // halt every axis
    MOVE_DONE, // posted by ISR when MOVE_TO target reached
    ARM,       // pull EN low — motors hold position
    DISARM,    // pull EN high — motors freewheel
};

struct MotionCmd
{
    CmdType type;
    uint8_t axis;          // 0..NUM_AXES-1 (unused for STOP_ALL/ARM/DISARM)
    int8_t direction;      // +1 or -1 (used by JOG)
    int32_t target_steps;  // absolute target (used by MOVE_TO)
    uint32_t velocity_sps; // steps per second (JOG / MOVE_TO)
};

// ---------------------------------------------------------------------------
// Per-axis runtime state
// ---------------------------------------------------------------------------
//
// Fields touched by the ISR are marked volatile so the compiler doesn't cache
// them in registers across function calls. Reads/writes of 32-bit aligned
// scalars are atomic on ESP32-S3 — fine for single-producer/single-consumer
// patterns like (ISR writes current_steps, motion task reads it).
//
struct AxisState
{
    volatile int32_t current_steps; // ISR writes, anyone may read
    volatile int32_t target_steps;  // motion task writes, ISR reads
    volatile uint32_t velocity_sps;
    volatile int8_t direction; // +1 forward, -1 reverse, 0 idle
    volatile bool moving;
    volatile bool jog_mode;
};

// ---------------------------------------------------------------------------
// Globals exposed to the ISR / webserver
// ---------------------------------------------------------------------------

extern AxisState axes[NUM_AXES];
extern gpio_num_t step_pins[NUM_AXES];
extern gptimer_handle_t step_timers[NUM_AXES];
extern TMC2209 *drivers[NUM_AXES];

extern QueueHandle_t motion_queue;
extern TaskHandle_t motion_task_handle;

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

// One-time setup. Pass in the constructed TMC2209 instances (one per axis).
// init_motor_controller() will:
//   - cache the STEP pin GPIOs for fast ISR access
//   - configure the shared EN pin as an output (held HIGH — disarmed)
//   - create the GPTimer for each axis
//   - create the command queue
//   - spawn the motion task pinned to core 1
//
// After this returns, the system is disarmed. Post an ARM command to enable
// motors before sending any motion commands.
void init_motor_controller(TMC2209 *driver_instances[NUM_AXES]);

// Helper for HTTP handlers: build a command and queue it. Returns false if
// the queue is full (very rare; queue depth is 32).
bool enqueue_command(const MotionCmd &cmd);