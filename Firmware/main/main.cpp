// =============================================================================
//  6-axis Robot Arm Controller — application entry point
// =============================================================================
//
//  Boot order:
//    1. NVS flash init (required by WiFi)
//    2. Construct one TMC2209 per axis. The constructor configures the UART
//       peripheral and step/dir/en GPIOs but does NOT talk to the driver yet.
//    3. Call init() on each driver to push the register config over UART.
//    4. Hand the drivers to the motion controller, which spawns the motion
//       task and configures the per-axis GPTimers.
//    5. Bring up WiFi. The IP-got handler starts the webserver once an IP
//       address is assigned.
//
//  Important: the arm boots DISARMED. EN is held HIGH so all motors freewheel
//  until you POST to /arm.
// =============================================================================

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "TMC2209.hpp"
#include "MotorController.hpp"
#include "Server.hpp"

static const char *TAG = "main";

// =============================================================================
//  Pin configuration — taken from the KiCad schematic for the ESP32-S3-DEVKITC
//  carrier PCB. If you ever change wiring, update HERE and only here.
//
//  Pins to avoid on ESP32-S3:
//    GPIO 0       — boot strapping
//    GPIO 19, 20  — USB D-/D+
//    GPIO 26..32  — connected to onboard flash/PSRAM
//    GPIO 33, 34  — don't exist
//    GPIO 45, 46  — strapping pins (boot behaviour)
// =============================================================================

// ---- Shared EN line ----
//
// All six TMC2209 modules tie EN together. The motion controller pulls this
// LOW to arm the arm, HIGH to disarm.
constexpr gpio_num_t PIN_ARM_EN = GPIO_NUM_13;

// ---- UART bus 1 — drivers J1, J2, J3 (addresses 0, 1, 2) ----
constexpr uart_port_t UART_BUS_1 = UART_NUM_1;
constexpr gpio_num_t PIN_UART1_TX = GPIO_NUM_17; // TX1 on schematic
constexpr gpio_num_t PIN_UART1_RX = GPIO_NUM_18; // RX1 on schematic

// ---- UART bus 2 — drivers J4, J5, J6 (addresses 0, 1, 2) ----
constexpr uart_port_t UART_BUS_2 = UART_NUM_2;
constexpr gpio_num_t PIN_UART2_TX = GPIO_NUM_15; // TX2 on schematic
constexpr gpio_num_t PIN_UART2_RX = GPIO_NUM_16; // RX2 on schematic

// ---- STEP / DIR pins, one pair per axis ----
constexpr gpio_num_t PIN_STEP_J1 = GPIO_NUM_1;
constexpr gpio_num_t PIN_DIR_J1 = GPIO_NUM_7;

constexpr gpio_num_t PIN_STEP_J2 = GPIO_NUM_2;
constexpr gpio_num_t PIN_DIR_J2 = GPIO_NUM_8;

constexpr gpio_num_t PIN_STEP_J3 = GPIO_NUM_3;
constexpr gpio_num_t PIN_DIR_J3 = GPIO_NUM_9;

constexpr gpio_num_t PIN_STEP_J4 = GPIO_NUM_4;
constexpr gpio_num_t PIN_DIR_J4 = GPIO_NUM_10;

constexpr gpio_num_t PIN_STEP_J5 = GPIO_NUM_5;
constexpr gpio_num_t PIN_DIR_J5 = GPIO_NUM_11;

constexpr gpio_num_t PIN_STEP_J6 = GPIO_NUM_6;
constexpr gpio_num_t PIN_DIR_J6 = GPIO_NUM_12;

// =============================================================================
//  Per-joint driver parameters
// =============================================================================
//
//  Currents follow the BOM:
//    J1 (base)     17HS19-2004S1, 2A rated -> 1400 mA RMS
//    J2 (shoulder) 17HS19-2004S1, 2A rated -> 1800 mA RMS  (highest load)
//    J3 (elbow)    17HS19-2004S1, 2A rated -> 1600 mA RMS
//    J4..J6 wrist  17HS13-0404S,  0.4A     ->  320 mA RMS
//
//  Microsteps: 16 everywhere to start. With CHOPCONF.intpol=1 the driver
//  internally interpolates to 256 so motion is smooth without overloading
//  the ESP32 with step rates.

struct JointConfig
{
    uart_port_t uart;
    gpio_num_t uart_tx;
    gpio_num_t uart_rx;
    gpio_num_t step;
    gpio_num_t dir;
    uint8_t uart_addr;
    uint16_t current_ma;
    uint16_t microsteps;
};

static const JointConfig joint_cfg[NUM_AXES] = {
    // J1 base
    {UART_BUS_1, PIN_UART1_TX, PIN_UART1_RX, PIN_STEP_J1, PIN_DIR_J1, 0, 1400, 16},
    // J2 shoulder
    {UART_BUS_1, PIN_UART1_TX, PIN_UART1_RX, PIN_STEP_J2, PIN_DIR_J2, 1, 1800, 16},
    // J3 elbow
    {UART_BUS_1, PIN_UART1_TX, PIN_UART1_RX, PIN_STEP_J3, PIN_DIR_J3, 2, 1600, 16},
    // J4 wrist rotation
    {UART_BUS_2, PIN_UART2_TX, PIN_UART2_RX, PIN_STEP_J4, PIN_DIR_J4, 0, 320, 16},
    // J5 wrist pitch
    {UART_BUS_2, PIN_UART2_TX, PIN_UART2_RX, PIN_STEP_J5, PIN_DIR_J5, 1, 320, 16},
    // J6 wrist roll
    {UART_BUS_2, PIN_UART2_TX, PIN_UART2_RX, PIN_STEP_J6, PIN_DIR_J6, 2, 320, 16},
};

// Drivers live in static storage so their addresses are stable for the
// lifetime of the program.
static TMC2209 *joints[NUM_AXES] = {};

// =============================================================================
//  Entry point
// =============================================================================

extern "C" void app_main()
{
    ESP_LOGI(TAG, "=== Robot Arm boot ===");

    // ---- 1. NVS flash (WiFi needs this) ----
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    // ---- 2. Construct drivers ----
    //
    // We allocate on the heap so the objects survive after app_main returns.
    // The constructor sets up UART + GPIO but does no TMC register writes yet.
    for (int i = 0; i < NUM_AXES; i++)
    {
        const JointConfig &c = joint_cfg[i];
        joints[i] = new TMC2209(
            c.uart, c.uart_tx, c.uart_rx,
            c.step, c.dir, PIN_ARM_EN,
            c.uart_addr,
            /* single_wire */ true);
        ESP_LOGI(TAG, "J%d driver constructed (UART%d addr %u step=%d dir=%d)",
                 i + 1, (int)c.uart, c.uart_addr, (int)c.step, (int)c.dir);
    }

    // ---- 3. Push register config to each driver over UART ----
    //
    // init() verifies that IFCNT incremented correctly — if the UART wiring
    // is broken, this is where you find out.
    // for (int i = 0; i < NUM_AXES; i++)
    // {
    //     const JointConfig &c = joint_cfg[i];
    //     if (!joints[i]->init(c.current_ma, c.microsteps))
    //     {
    //         ESP_LOGE(TAG, "J%d driver init FAILED — check UART wiring", i + 1);
    //     }
    // }

    // ---- 4. Motion controller ----
    //
    // Caches step pins, configures GPTimers, creates motion queue, spawns
    // motion task on core 1. Arm stays disarmed (EN HIGH).
    init_motor_controller(joints);

    // ---- 5. WiFi + webserver ----
    //
    // connect_wifi blocks until either connected or MAX_FAILURES reached.
    // start_webserver is invoked from the IP-got event handler once an IP is
    // assigned, so by the time connect_wifi returns SUCESS, the HTTP API is
    // already live.
    if (connect_wifi() != WIFI_SUCESS)
    {
        ESP_LOGE(TAG, "WiFi connection failed — server not started");
        // We don't return; the motion task is still running and the arm
        // can in principle be driven from a serial console if you add one.
    }
    else
    {
        ESP_LOGI(TAG, "=== Boot complete, web UI available ===");
    }

    // app_main can return — FreeRTOS keeps the motion task and HTTP server
    // running. The idle task handles whatever cleanup is needed.
}