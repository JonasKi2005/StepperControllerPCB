#include "Server.hpp"
#include "MotorController.hpp"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_netif.h"

static const char *TAG = "Server";

static int s_retry_num = 0;
static EventGroupHandle_t wifi_event_group = nullptr;
static httpd_handle_t s_server = nullptr;

// ---------------------------------------------------------------------------
// Query-string helpers
// ---------------------------------------------------------------------------

static int get_query_int(httpd_req_t *req, const char *key, int default_val)
{
    char query[128] = {};
    char value[32] = {};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK)
    {
        if (httpd_query_key_value(query, key, value, sizeof(value)) == ESP_OK)
        {
            return atoi(value);
        }
    }
    return default_val;
}

// Validate axis index in query string. Returns -1 if missing/out-of-range,
// and sends an HTTP 400 response in that case.
static int parse_axis(httpd_req_t *req)
{
    int axis = get_query_int(req, "axis", -1);
    if (axis < 0 || axis >= NUM_AXES)
    {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_sendstr(req, "missing or invalid 'axis'");
        return -1;
    }
    return axis;
}

static esp_err_t send_text(httpd_req_t *req, const char *msg)
{
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_sendstr(req, msg);
}

static esp_err_t send_ok(httpd_req_t *req)
{
    return send_text(req, "ok");
}

// ===========================================================================
// HTML UI — served at /
// ===========================================================================

static const char INDEX_HTML[] = R"rawliteral(<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>6-Axis Arm Control</title>
<style>
  body { font-family: system-ui, sans-serif; background:#1a1a2e; color:#eee;
         margin:0; padding:20px; }
  h1 { color:#e94560; margin:0 0 16px; font-size:1.4em; }
  .row { display:flex; gap:12px; flex-wrap:wrap; margin:8px 0;
         align-items:center; }
  .card { background:#16213e; padding:14px 16px; border-radius:10px;
          margin:10px 0; }
  .axis { display:grid; grid-template-columns:60px 1fr 1fr 1fr;
          gap:8px; align-items:center; padding:8px 0;
          border-bottom:1px solid #243056; }
  .axis:last-child { border:0; }
  .label { font-weight:600; color:#4ecca3; }
  .pos { font-family:monospace; color:#aaa; font-size:0.95em; }
  button { background:#3282b8; color:#fff; border:0; padding:8px 14px;
           border-radius:6px; font-size:0.95em; cursor:pointer;
           transition:opacity .15s; }
  button:hover { opacity:0.8; }
  button:active { transform:translateY(1px); }
  .fwd  { background:#4ecca3; color:#0a2920; }
  .rev  { background:#3282b8; }
  .stop { background:#e94560; }
  .big  { padding:12px 20px; font-size:1.05em; }
  input[type=number], input[type=range] {
    background:#0f1a2e; color:#eee; border:1px solid #2d3e6e;
    border-radius:4px; padding:6px 8px; }
  input[type=number] { width:90px; }
  label { color:#aaa; font-size:0.9em; margin-right:4px; }
  #log { background:#0a0f1c; padding:10px 12px; border-radius:6px;
         font-family:monospace; font-size:0.85em; color:#8fa;
         min-height:48px; white-space:pre-wrap; }
  .status-bar { display:flex; gap:16px; align-items:center;
                font-size:1.1em; }
  .armed { color:#4ecca3; }
  .disarmed { color:#e94560; }
</style>
</head>
<body>
  <h1>6-Axis Arm Control</h1>

  <div class="card">
    <div class="status-bar">
      <span>Arm:</span>
      <span id="armState" class="disarmed">disarmed</span>
      <button class="fwd big" onclick="post('/arm')">Arm</button>
      <button class="stop big" onclick="post('/disarm')">Disarm</button>
      <button class="stop big" onclick="post('/stop_all')">STOP ALL</button>
    </div>
  </div>

  <div class="card">
    <div class="row">
      <label>Jog speed (sps):</label>
      <input type="number" id="jogVel" value="800" min="50" max="5000">
      <label>Move speed (sps):</label>
      <input type="number" id="moveVel" value="1600" min="50" max="5000">
    </div>
  </div>

  <div class="card" id="axes"></div>

  <div class="card">
    <div class="row">
      <label>Axis:</label>
      <input type="number" id="cfgAxis" value="0" min="0" max="5">
      <label>Current (mA):</label>
      <input type="number" id="cfgMa" value="800" min="100" max="2000" step="50">
      <button onclick="setCurrent()">Set current</button>
      <label>Microsteps:</label>
      <select id="cfgUs" style="background:#0f1a2e;color:#eee;border:1px solid #2d3e6e;border-radius:4px;padding:6px">
        <option>1</option><option>2</option><option>4</option><option>8</option>
        <option selected>16</option><option>32</option><option>64</option>
        <option>128</option><option>256</option>
      </select>
      <button onclick="setMicrosteps()">Set microsteps</button>
    </div>
  </div>

  <div class="card">
    <div id="log">ready</div>
  </div>

<script>
const NUM_AXES = 6;
const axesDiv = document.getElementById('axes');
for (let i = 0; i < NUM_AXES; i++) {
  axesDiv.insertAdjacentHTML('beforeend', `
    <div class="axis">
      <span class="label">J${i+1}</span>
      <span class="pos" id="pos${i}">step 0</span>
      <div class="row">
        <button class="rev"  onmousedown="jogStart(${i},-1)" onmouseup="jogStop(${i})"
                            ontouchstart="jogStart(${i},-1)" ontouchend="jogStop(${i})">◀ Jog -</button>
        <button class="fwd"  onmousedown="jogStart(${i},1)"  onmouseup="jogStop(${i})"
                            ontouchstart="jogStart(${i},1)"  ontouchend="jogStop(${i})">Jog + ▶</button>
      </div>
      <div class="row">
        <input type="number" id="tgt${i}" value="0" style="width:100px">
        <button onclick="moveTo(${i})">Move to</button>
      </div>
    </div>`);
}

async function post(path) {
  try {
    const res = await fetch(path, { method: 'POST' });
    const text = await res.text();
    log(path + ' → ' + text);
  } catch (e) { log('error: ' + e.message); }
}
async function postQ(path, params) {
  const q = new URLSearchParams(params).toString();
  return post(path + '?' + q);
}
async function jogStart(axis, dir) {
  const v = document.getElementById('jogVel').value;
  postQ('/jog', { axis, dir, v });
}
async function jogStop(axis) {
  postQ('/stop', { axis });
}
async function moveTo(axis) {
  const target = document.getElementById('tgt'+axis).value;
  const v = document.getElementById('moveVel').value;
  postQ('/move_to', { axis, target, v });
}
async function setCurrent() {
  const axis = document.getElementById('cfgAxis').value;
  const ma   = document.getElementById('cfgMa').value;
  postQ('/current', { axis, ma });
}
async function setMicrosteps() {
  const axis = document.getElementById('cfgAxis').value;
  const us   = document.getElementById('cfgUs').value;
  postQ('/microsteps', { axis, us });
}
function log(msg) {
  const el = document.getElementById('log');
  el.textContent = new Date().toLocaleTimeString() + '  ' + msg;
}

async function refresh() {
  try {
    const res = await fetch('/status');
    const j = await res.json();
    document.getElementById('armState').textContent = j.armed ? 'armed' : 'disarmed';
    document.getElementById('armState').className   = j.armed ? 'armed' : 'disarmed';
    for (let i = 0; i < NUM_AXES; i++) {
      document.getElementById('pos'+i).textContent =
        'step ' + j.steps[i] + (j.moving[i] ? ' (moving)' : '');
    }
  } catch (e) { /* network blip; ignore */ }
}
setInterval(refresh, 250);
refresh();
</script>
</body>
</html>)rawliteral";

// ===========================================================================
// HTTP handlers
// ===========================================================================

static esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

// ---- power: /arm, /disarm, /stop_all ----

static esp_err_t arm_handler(httpd_req_t *req)
{
    MotionCmd c = {};
    c.type = CmdType::ARM;
    enqueue_command(c);
    return send_ok(req);
}

static esp_err_t disarm_handler(httpd_req_t *req)
{
    MotionCmd c = {};
    c.type = CmdType::DISARM;
    enqueue_command(c);
    return send_ok(req);
}

static esp_err_t stop_all_handler(httpd_req_t *req)
{
    MotionCmd c = {};
    c.type = CmdType::STOP_ALL;
    enqueue_command(c);
    return send_ok(req);
}

// ---- motion: /jog, /stop, /move_to ----

static esp_err_t jog_handler(httpd_req_t *req)
{
    int axis = parse_axis(req);
    if (axis < 0)
        return ESP_OK;

    int dir = get_query_int(req, "dir", 0);
    int vel = get_query_int(req, "v", 0);
    if (dir == 0 || vel <= 0)
    {
        httpd_resp_set_status(req, "400 Bad Request");
        return send_text(req, "need dir (-1|+1) and v (sps>0)");
    }

    MotionCmd c = {};
    c.type = CmdType::JOG;
    c.axis = (uint8_t)axis;
    c.direction = (int8_t)(dir > 0 ? 1 : -1);
    c.velocity_sps = (uint32_t)vel;
    enqueue_command(c);
    return send_ok(req);
}

static esp_err_t stop_handler(httpd_req_t *req)
{
    int axis = parse_axis(req);
    if (axis < 0)
        return ESP_OK;

    MotionCmd c = {};
    c.type = CmdType::STOP;
    c.axis = (uint8_t)axis;
    enqueue_command(c);
    return send_ok(req);
}

static esp_err_t move_to_handler(httpd_req_t *req)
{
    int axis = parse_axis(req);
    if (axis < 0)
        return ESP_OK;

    int target = get_query_int(req, "target", INT32_MIN);
    int vel = get_query_int(req, "v", 800);
    if (target == INT32_MIN)
    {
        httpd_resp_set_status(req, "400 Bad Request");
        return send_text(req, "need target (step count)");
    }

    MotionCmd c = {};
    c.type = CmdType::MOVE_TO;
    c.axis = (uint8_t)axis;
    c.target_steps = (int32_t)target;
    c.velocity_sps = (uint32_t)(vel > 0 ? vel : 800);
    enqueue_command(c);
    return send_ok(req);
}

// ---- driver config: /current, /microsteps ----
//
// These call the TMC2209 class directly rather than going through the queue.
// They're cheap (one UART write each) and don't affect step generation, so
// it's safe to do from the HTTP task. If you ever see lock contention with
// the motion task, route these through the queue too.

static esp_err_t current_handler(httpd_req_t *req)
{
    int axis = parse_axis(req);
    if (axis < 0)
        return ESP_OK;

    int ma = get_query_int(req, "ma", -1);
    if (ma < 50 || ma > 2000)
    {
        httpd_resp_set_status(req, "400 Bad Request");
        return send_text(req, "ma must be 50..2000");
    }
    drivers[axis]->set_current((uint16_t)ma);

    char msg[64];
    snprintf(msg, sizeof(msg), "axis %d current=%d mA", axis, ma);
    return send_text(req, msg);
}

static esp_err_t microsteps_handler(httpd_req_t *req)
{
    int axis = parse_axis(req);
    if (axis < 0)
        return ESP_OK;

    int us = get_query_int(req, "us", -1);
    // Valid values: 1,2,4,8,16,32,64,128,256
    bool valid = (us == 1 || us == 2 || us == 4 || us == 8 || us == 16 ||
                  us == 32 || us == 64 || us == 128 || us == 256);
    if (!valid)
    {
        httpd_resp_set_status(req, "400 Bad Request");
        return send_text(req, "us must be one of 1,2,4,8,16,32,64,128,256");
    }
    drivers[axis]->set_microsteps((uint16_t)us);

    char msg[64];
    snprintf(msg, sizeof(msg), "axis %d microsteps=%d", axis, us);
    return send_text(req, msg);
}

// ---- /status: JSON snapshot for UI polling ----

static esp_err_t status_handler(httpd_req_t *req)
{
    // Read current EN pin level to report armed/disarmed.
    // (We could keep a software flag, but reading the pin is honest.)
    int en_level = gpio_get_level(ARM_ENABLE_PIN);
    bool armed = (en_level == 0);

    char buf[512];
    int n = snprintf(buf, sizeof(buf),
                     "{\"armed\":%s,"
                     "\"steps\":[%ld,%ld,%ld,%ld,%ld,%ld],"
                     "\"moving\":[%d,%d,%d,%d,%d,%d]}",
                     armed ? "true" : "false",
                     (long)axes[0].current_steps, (long)axes[1].current_steps,
                     (long)axes[2].current_steps, (long)axes[3].current_steps,
                     (long)axes[4].current_steps, (long)axes[5].current_steps,
                     axes[0].moving ? 1 : 0, axes[1].moving ? 1 : 0,
                     axes[2].moving ? 1 : 0, axes[3].moving ? 1 : 0,
                     axes[4].moving ? 1 : 0, axes[5].moving ? 1 : 0);

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, buf, n);
}

// ===========================================================================
// Webserver setup
// ===========================================================================

httpd_handle_t start_webserver()
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 16;
    config.lru_purge_enable = true;

    if (httpd_start(&s_server, &config) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start http server");
        return nullptr;
    }

    // Table of all endpoints. URIs must start with '/'.
    const httpd_uri_t uris[] = {
        {"/", HTTP_GET, index_handler, nullptr},

        // Power
        {"/arm", HTTP_POST, arm_handler, nullptr},
        {"/disarm", HTTP_POST, disarm_handler, nullptr},
        {"/stop_all", HTTP_POST, stop_all_handler, nullptr},

        // Motion
        {"/jog", HTTP_POST, jog_handler, nullptr},
        {"/stop", HTTP_POST, stop_handler, nullptr},
        {"/move_to", HTTP_POST, move_to_handler, nullptr},

        // Driver config
        {"/current", HTTP_POST, current_handler, nullptr},
        {"/microsteps", HTTP_POST, microsteps_handler, nullptr},

        // Status
        {"/status", HTTP_GET, status_handler, nullptr},
    };

    const size_t n = sizeof(uris) / sizeof(uris[0]);
    for (size_t i = 0; i < n; i++)
    {
        httpd_register_uri_handler(s_server, &uris[i]);
    }

    ESP_LOGI(TAG, "webserver started, %u endpoints", (unsigned)n);
    return s_server;
}

// ===========================================================================
// WiFi
// ===========================================================================

void wifi_event_handler(void *arg, esp_event_base_t event_base,
                        int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        ESP_LOGI(TAG, "Connecting to AP...");
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < MAX_FAILURES)
        {
            ESP_LOGI(TAG, "Reconnecting to AP (retry %d)...", s_retry_num);
            esp_wifi_connect();
            s_retry_num++;
        }
        else
        {
            xEventGroupSetBits(wifi_event_group, WIFI_FAILURE);
        }
    }
}

void ip_event_handler(void *arg, esp_event_base_t event_base,
                      int32_t event_id, void *event_data)
{
    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "STA got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        start_webserver();
        xEventGroupSetBits(wifi_event_group, WIFI_SUCESS);
    }
}

esp_err_t connect_wifi()
{
    int status = WIFI_FAILURE;

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_event_group = xEventGroupCreate();

    esp_event_handler_instance_t wifi_handler_event_instance;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler,
        nullptr, &wifi_handler_event_instance));

    esp_event_handler_instance_t got_ip_event_instance;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_handler,
        nullptr, &got_ip_event_instance));

    wifi_config_t wifi_config = {};
    strcpy((char *)wifi_config.sta.ssid, WIFI_SSID);
    strcpy((char *)wifi_config.sta.password, WIFI_PASS);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "STA initialization complete");

    EventBits_t bits = xEventGroupWaitBits(
        wifi_event_group, WIFI_SUCESS | WIFI_FAILURE,
        pdFALSE, pdFALSE, portMAX_DELAY);

    if (bits & WIFI_SUCESS)
    {
        ESP_LOGI(TAG, "Connected to AP");
        status = WIFI_SUCESS;
    }
    else
    {
        ESP_LOGE(TAG, "Failed to connect to AP");
        status = WIFI_FAILURE;
    }

    // NOTE: we intentionally do NOT unregister the WiFi/IP handlers here.
    // We want reconnect attempts to keep working if the AP drops later.

    return (esp_err_t)status;
}