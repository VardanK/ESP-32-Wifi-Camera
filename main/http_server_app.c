#include "http_server_app.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <ctype.h>
#ifdef __has_include
#if __has_include(<sys/statvfs.h>)
#include <sys/statvfs.h>
#define HTTP_HAS_STATVFS 1
#else
#define HTTP_HAS_STATVFS 0
#endif
#else
#define HTTP_HAS_STATVFS 0
#endif

#include "esp_err.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"

#include "camera_manager.h"
#include "wifi_manager.h"
#include "sensor.h"

#define HTTP_SERVER_TAG "http_server"
#define HTTP_RESPONSE_BUF_SIZE 4096
#define HTTP_SCAN_BUFFER_SIZE 2048
#define RESTART_DELAY_US (5 * 1000 * 1000)

static httpd_handle_t s_http_handle = NULL;
static bool s_config_session_active = false;
static bool s_restart_timer_active = false;
static int64_t s_restart_deadline_us = 0;
static esp_timer_handle_t s_restart_timer_handle = NULL;
static esp_timer_handle_t s_factory_reset_timer_handle = NULL;
static bool s_factory_reset_pending = false;
static int64_t s_factory_reset_deadline_us = 0;

static const char *FACTORY_RESET_CONFIRM_TOKEN = "factory_reset";

static esp_err_t handle_root(httpd_req_t *req);
static esp_err_t handle_scan(httpd_req_t *req);
static esp_err_t handle_config(httpd_req_t *req);
static esp_err_t handle_status(httpd_req_t *req);
static esp_err_t handle_device_status(httpd_req_t *req);
static esp_err_t handle_capture(httpd_req_t *req);
static esp_err_t handle_capture_with_controls(httpd_req_t *req);
static esp_err_t handle_camera_control(httpd_req_t *req);
static esp_err_t handle_factory_reset(httpd_req_t *req);
static void url_decode(char *str);
static void ensure_restart_timer_created(void);
static void schedule_restart_countdown(void);
static void cancel_restart_countdown(void);
static int escape_json_string(const char *input, char *output, size_t output_len);
static void factory_reset_timer_callback(void *arg);
static void ensure_factory_reset_timer_created(void);
static void schedule_factory_reset_reboot(void);
static framesize_t camera_framesize_from_label(const char *label, bool *out_supported);
static bool parse_boolean_arg(const char *value_str, bool *out_value);
static esp_err_t handle_capture_internal(httpd_req_t *req, bool use_query_controls);

static const httpd_uri_t root_uri = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = handle_root,
    .user_ctx = NULL
};

static const httpd_uri_t scan_uri = {
    .uri = "/api/scan",
    .method = HTTP_GET,
    .handler = handle_scan,
    .user_ctx = NULL
};

static const httpd_uri_t config_uri = {
    .uri = "/api/config",
    .method = HTTP_POST,
    .handler = handle_config,
    .user_ctx = NULL
};

static const httpd_uri_t status_uri = {
    .uri = "/api/status",
    .method = HTTP_GET,
    .handler = handle_status,
    .user_ctx = NULL
};

static const httpd_uri_t device_status_uri = {
    .uri = "/api/device/status",
    .method = HTTP_GET,
    .handler = handle_device_status,
    .user_ctx = NULL
};

static const httpd_uri_t capture_uri = {
    .uri = "/api/capture",
    .method = HTTP_GET,
    .handler = handle_capture,
    .user_ctx = NULL
};

static const httpd_uri_t capture_with_controls_uri = {
    .uri = "/api/capture/config",
    .method = HTTP_GET,
    .handler = handle_capture_with_controls,
    .user_ctx = NULL
};

static const httpd_uri_t control_uri = {
    .uri = "/api/control",
    .method = HTTP_GET,
    .handler = handle_camera_control,
    .user_ctx = NULL
};

static const httpd_uri_t factory_reset_uri = {
    .uri = "/api/factory_reset",
    .method = HTTP_POST,
    .handler = handle_factory_reset,
    .user_ctx = NULL
};

static const char PROVISION_HTML[] =
    "<!DOCTYPE html>"
    "<html lang=\"en\">"
    "<head>"
    "<meta charset=\"UTF-8\" />"
    "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\" />"
    "<title>WiFi Camera Setup</title>"
    "<style>"
    "body{font-family:Arial,Helvetica,sans-serif;margin:40px;background:#f5f5f5;color:#333;}"
    ".container{max-width:480px;margin:0 auto;background:#fff;padding:24px;border-radius:12px;box-shadow:0 2px 16px rgba(0,0,0,0.1);}"
    "h1{font-size:1.5rem;margin-bottom:1rem;text-align:center;}"
    "label{display:block;margin:0.75rem 0 0.25rem;font-weight:bold;}"
    "select,input{width:100%;padding:0.65rem;margin-bottom:0.25rem;border:1px solid #ccc;border-radius:6px;box-sizing:border-box;}"
    ".password-wrapper{position:relative;}"
    ".password-wrapper input{padding-right:2.5rem;}"
    ".toggle-password{position:absolute;top:50%;right:0.75rem;transform:translateY(-50%);background:transparent;border:none;padding:0;cursor:pointer;display:flex;align-items:center;justify-content:center;color:#555;width:auto;height:100%;}"
    ".toggle-password:focus{outline:none;}"
    ".toggle-password svg{width:20px;height:20px;}"
    "#submit{width:100%;padding:0.75rem;border:none;border-radius:6px;background:#0070f3;color:#fff;font-size:1rem;cursor:pointer;}"
    "#submit:disabled{background:#888;cursor:not-allowed;}"
    ".status{margin-top:1rem;text-align:center;font-weight:bold;min-height:2.5rem;display:flex;align-items:center;justify-content:center;}"
    "</style>"
    "</head>"
    "<body>"
    "<div class=\"container\">"
    "<h1>WiFi Camera Setup</h1>"
    "<label for=\"ssid\">Available Networks</label>"
    "<select id=\"ssid\"></select>"
    "<label for=\"password\">WiFi Password</label>"
    "<div class=\"password-wrapper\">"
    "<input id=\"password\" type=\"password\" placeholder=\"Enter WiFi password\" autocomplete=\"current-password\" />"
    "<button type=\"button\" id=\"toggle-password\" class=\"toggle-password\" aria-label=\"Toggle password visibility\"></button>"
    "</div>"
    "<div class=\"status\" id=\"status\">Scanning for networks...</div>"
    "<button id=\"submit\" disabled>Save & Connect</button>"
    "</div>"
    "<script>"
    "const ssidSelect=document.getElementById('ssid');"
    "const passwordInput=document.getElementById('password');"
    "const togglePasswordBtn=document.getElementById('toggle-password');"
    "const statusLabel=document.getElementById('status');"
    "const submitBtn=document.getElementById('submit');"
    "let lastSelectedSsid='';"
    "let lastPasswordValue='';"
    "let passwordVisible=false;"
    "let scanTimer=null;"
    "let statusTimer=null;"
    "let connectionComplete=false;"
    "const ICON_EYE='<svg viewBox=\\\"0 0 24 24\\\" fill=\\\"none\\\" stroke=\\\"currentColor\\\" stroke-width=\\\"2\\\" stroke-linecap=\\\"round\\\" stroke-linejoin=\\\"round\\\"><path d=\\\"M1 12s4-7 11-7 11 7 11 7-4 7-11 7-11-7-11-7z\\\"></path><circle cx=\\\"12\\\" cy=\\\"12\\\" r=\\\"3\\\"></circle></svg>';"
    "const ICON_EYE_OFF='<svg viewBox=\\\"0 0 24 24\\\" fill=\\\"none\\\" stroke=\\\"currentColor\\\" stroke-width=\\\"2\\\" stroke-linecap=\\\"round\\\" stroke-linejoin=\\\"round\\\"><path d=\\\"M17.94 17.94C16.09 19.22 14.11 20 12 20 5 20 1 12 1 12a21.3 21.3 0 0 1 5.17-6.3m4.11-2.14C11.03 3.18 11.52 3 12 3c7 0 11 9 11 9a21.2 21.2 0 0 1-3.8 5.25\\\"></path><path d=\\\"M9.9 9.9a3 3 0 0 0 4.24 4.24\\\"></path><line x1=\\\"1\\\" y1=\\\"1\\\" x2=\\\"23\\\" y2=\\\"23\\\"></line></svg>';"
    "togglePasswordBtn.innerHTML=ICON_EYE;"
    "togglePasswordBtn.addEventListener('click',()=>{"
        "passwordVisible=!passwordVisible;"
        "passwordInput.type=passwordVisible?'text':'password';"
        "togglePasswordBtn.innerHTML=passwordVisible?ICON_EYE_OFF:ICON_EYE;"
    "});"
    "passwordInput.addEventListener('input',()=>{"
        "lastPasswordValue=passwordInput.value;"
    "});"
    "ssidSelect.addEventListener('change',()=>{"
        "lastSelectedSsid=ssidSelect.value;"
    "});"
    "function setStatus(message){"
        "statusLabel.textContent=message;"
    "}"
    "async function refreshNetworks(){"
        "if(connectionComplete){return;}"
        "const previousSelection=lastSelectedSsid||ssidSelect.value;"
        "try{"
            "const res=await fetch('/api/scan');"
            "const data=await res.json();"
            "ssidSelect.innerHTML='';"
            "let selectionRestored=false;"
            "data.networks.forEach(net=>{"
                "const option=document.createElement('option');"
                "option.value=net.ssid;"
                "option.textContent=`${net.ssid} (RSSI: ${net.rssi}dBm)`;"
                "if(net.ssid===previousSelection){"
                    "option.selected=true;"
                    "selectionRestored=true;"
                "}"
                "ssidSelect.appendChild(option);"
            "});"
            "if(!selectionRestored&&previousSelection){"
                "const option=document.createElement('option');"
                "option.value=previousSelection;"
                "option.textContent=`${previousSelection} (previous selection)`;"
                "option.selected=true;"
                "option.dataset.stale='true';"
                "ssidSelect.insertBefore(option,ssidSelect.firstChild);"
            "}"
            "if(ssidSelect.options.length===0){"
                "const option=document.createElement('option');"
                "option.textContent='No networks found';"
                "option.disabled=true;"
                "option.selected=true;"
                "ssidSelect.appendChild(option);"
                "submitBtn.disabled=true;"
                "setStatus('No WiFi networks found. Retry in a moment.');"
            "}else if(!connectionComplete){"
                "submitBtn.disabled=false;"
                "if(statusLabel.textContent==='Scanning for networks...'){"
                    "setStatus('Select your network and enter the password.');"
                "}"
            "}"
        "}catch(err){"
            "console.error(err);"
            "if(ssidSelect.options.length===0){"
                "const option=document.createElement('option');"
                "option.textContent='Unable to scan';"
                "option.disabled=true;"
                "option.selected=true;"
                "ssidSelect.appendChild(option);"
            "}"
            "setStatus('Failed to scan networks.');"
        "}"
        "if(ssidSelect.value){"
            "lastSelectedSsid=ssidSelect.value;"
        "}else if(previousSelection){"
            "lastSelectedSsid=previousSelection;"
        "}"
        "passwordInput.value=lastPasswordValue;"
    "}"
    "async function pollStatus(){"
        "try{"
            "const res=await fetch('/api/status');"
            "const data=await res.json();"
            "if(data.connected){"
                "connectionComplete=true;"
                "if(scanTimer){clearInterval(scanTimer);scanTimer=null;}"
                "submitBtn.disabled=true;"
                "ssidSelect.disabled=true;"
                "passwordInput.disabled=true;"
                "togglePasswordBtn.disabled=true;"
                "const seconds=Math.max(0,Math.ceil((data.restart_in_ms||0)/1000));"
                "if(seconds>0){"
                    "setStatus(`Connected to ${data.ssid||'network'}. Restarting in ${seconds}s...`);"
                "}else if(data.restart_pending){"
                    "setStatus('Restarting...');"
                "}else{"
                    "setStatus('Connected successfully.');"
                "}"
            "}else{"
                "if(data.sta_connecting){"
                    "let message='Attempting to establish WiFi connection...';"
                    "if(data.sta_reason_message){message+=` ${data.sta_reason_message}`;}"
                    "setStatus(message);"
                "}else if(data.sta_reason_message){"
                    "setStatus(data.sta_reason_message);"
                "}else if(!connectionComplete){"
                    "setStatus('Provisioning mode active. Configure WiFi credentials.');"
                "}"
            "}"
        "}catch(err){"
            "console.error(err);"
        "}"
    "}"
    "function startScanLoop(){"
        "refreshNetworks();"
        "if(scanTimer){clearInterval(scanTimer);}"
        "scanTimer=setInterval(()=>{if(!connectionComplete){refreshNetworks();}},10000);"
    "}"
    "function startStatusPolling(){"
        "if(statusTimer){return;}"
        "pollStatus();"
        "statusTimer=setInterval(pollStatus,1000);"
    "}"
    "submitBtn.addEventListener('click',async()=>{"
        "const selected=ssidSelect.value;"
        "const password=passwordInput.value;"
        "if(!selected||submitBtn.disabled){return;}"
        "lastSelectedSsid=selected;"
        "lastPasswordValue=password;"
        "submitBtn.disabled=true;"
        "setStatus('Saving credentials...');"
        "const params=new URLSearchParams({ssid:selected,password});"
        "try{"
            "const res=await fetch('/api/config',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:params.toString()});"
            "if(res.ok){"
                "setStatus('Credentials saved. Connecting to WiFi...');"
                "startStatusPolling();"
            "}else{"
                "submitBtn.disabled=false;"
                "setStatus('Failed to save credentials.');"
            "}"
        "}catch(err){"
            "console.error(err);"
            "submitBtn.disabled=false;"
            "setStatus('Error while sending credentials.');"
        "}"
        "passwordInput.value=lastPasswordValue;"
    "});"
    "startScanLoop();"
    "</script>"
    "</body>"
    "</html>";

static const char DASHBOARD_HTML[] =
    "<!DOCTYPE html>"
    "<html lang=\"en\">"
    "<head>"
    "<meta charset=\"UTF-8\" />"
    "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\" />"
    "<title>WiFi Camera Dashboard</title>"
    "<style>"
    "body{font-family:Arial,Helvetica,sans-serif;margin:24px;background:#f5f5f5;color:#333;}"
    "h1{text-align:center;margin-bottom:1.5rem;}"
    ".layout{max-width:960px;margin:0 auto;display:flex;flex-direction:column;gap:1.5rem;}"
    ".card{background:#fff;border-radius:12px;padding:20px;box-shadow:0 2px 16px rgba(0,0,0,0.08);}"
    ".card h2{margin-top:0;font-size:1.25rem;margin-bottom:1rem;}"
    ".info-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(180px,1fr));gap:12px;}"
    ".info-item{padding:12px;border:1px solid #eee;border-radius:8px;background:#fafafa;}"
    ".controls{display:grid;grid-template-columns:repeat(auto-fit,minmax(200px,1fr));gap:16px;}"
    ".control{display:flex;flex-direction:column;gap:6px;}"
    "label{font-weight:600;}"
    "select,input[type=range]{padding:0.5rem;border-radius:6px;border:1px solid #ccc;}"
    ".toggle{display:flex;align-items:center;gap:8px;font-weight:600;}"
    ".toggle.disabled{opacity:0.5;}"
    ".control.disabled{opacity:0.5;}"
    ".actions{display:flex;flex-wrap:wrap;gap:12px;align-items:center;}"
    "#capture{padding:0.75rem 1.5rem;border:none;border-radius:8px;background:#0b74ff;color:#fff;font-size:1rem;cursor:pointer;}"
    "#capture:disabled{background:#888;cursor:not-allowed;}"
    "#snapshot{max-width:100%;border-radius:12px;margin-top:1rem;border:1px solid #ddd;}"
    ".value-pill{display:inline-block;padding:0.25rem 0.5rem;border-radius:12px;background:#eef;margin-left:auto;font-size:0.85rem;}"
    "#factory-reset{padding:0.75rem 1.5rem;border:none;border-radius:8px;background:#d32f2f;color:#fff;font-size:1rem;cursor:pointer;}"
    "#factory-reset:disabled{opacity:0.7;cursor:not-allowed;}"
    "</style>"
    "</head>"
    "<body>"
    "<div class=\"layout\">"
    "<h1>WiFi Camera Dashboard</h1>"
    "<section class=\"card\">"
    "<h2>Capture</h2>"
    "<img id=\"snapshot\" alt=\"Camera snapshot\" />"
    "<div class=\"actions\">"
    "<button id=\"capture\">Capture Photo</button>"
    "</div>"
    "</section>"
    "<section class=\"card\">"
    "<h2>Diagnostics</h2>"
    "<div id=\"diagnostics\"><p id=\"diag-message\">Collecting status...</p></div>"
    "</section>"
    "<section class=\"card\">"
    "<h2>Camera Controls</h2>"
    "<div class=\"controls\">"
    "<div class=\"control\">"
    "<label for=\"framesize\">Resolution</label>"
    "<select id=\"framesize\" data-control=\"framesize\">"
    "<option value=\"6\">QVGA (320x240)</option>"
    "<option value=\"10\">VGA (640x480)</option>"
    "<option value=\"11\">SVGA (800x600)</option>"
    "<option value=\"12\">XGA (1024x768)</option>"
    "<option value=\"14\">SXGA (1280x1024)</option>"
    "<option value=\"15\">UXGA (1600x1200)</option>"
    "</select>"
    "</div>"
    "<div class=\"control\">"
    "<label for=\"quality\">JPEG Quality</label>"
    "<div class=\"actions\">"
    "<input type=\"range\" id=\"quality\" data-control=\"quality\" min=\"10\" max=\"63\" value=\"12\" />"
    "<span class=\"value-pill\" id=\"quality-value\">12</span>"
    "</div>"
    "</div>"
    "<div class=\"control\">"
    "<label for=\"brightness\">Brightness</label>"
    "<div class=\"actions\">"
    "<input type=\"range\" id=\"brightness\" data-control=\"brightness\" min=\"-2\" max=\"2\" value=\"0\" />"
    "<span class=\"value-pill\" id=\"brightness-value\">0</span>"
    "</div>"
    "</div>"
    "<div class=\"control\">"
    "<label for=\"contrast\">Contrast</label>"
    "<div class=\"actions\">"
    "<input type=\"range\" id=\"contrast\" data-control=\"contrast\" min=\"-2\" max=\"2\" value=\"0\" />"
    "<span class=\"value-pill\" id=\"contrast-value\">0</span>"
    "</div>"
    "</div>"
    "<div class=\"control\">"
    "<label for=\"saturation\">Saturation</label>"
    "<div class=\"actions\">"
    "<input type=\"range\" id=\"saturation\" data-control=\"saturation\" min=\"-2\" max=\"2\" value=\"0\" />"
    "<span class=\"value-pill\" id=\"saturation-value\">0</span>"
    "</div>"
    "</div>"
    "<div class=\"control\">"
    "<label for=\"sharpness\">Sharpness</label>"
    "<div class=\"actions\">"
    "<input type=\"range\" id=\"sharpness\" data-control=\"sharpness\" min=\"-2\" max=\"2\" value=\"0\" />"
    "<span class=\"value-pill\" id=\"sharpness-value\">0</span>"
    "</div>"
    "</div>"
    "<div class=\"control\">"
    "<label for=\"special_effect\">Special Effect</label>"
    "<select id=\"special_effect\" data-control=\"special_effect\">"
    "<option value=\"0\" selected>None</option>"
    "<option value=\"1\">Negative</option>"
    "<option value=\"2\">Grayscale</option>"
    "<option value=\"3\">Red Tint</option>"
    "<option value=\"4\">Green Tint</option>"
    "<option value=\"5\">Blue Tint</option>"
    "<option value=\"6\">Sepia</option>"
    "</select>"
    "</div>"
    "<div class=\"control\">"
    "<label for=\"flash-brightness\">Flash Brightness</label>"
    "<div class=\"actions\">"
    "<input type=\"range\" id=\"flash-brightness\" data-control=\"flash_brightness\" min=\"0\" max=\"100\" value=\"100\" />"
    "<span class=\"value-pill\" id=\"flash-brightness-value\">100</span>"
    "</div>"
    "</div>"
    "<label class=\"toggle\"><input type=\"checkbox\" id=\"awb\" data-control=\"awb\" checked /> Auto White Balance</label>"
    "<label class=\"toggle\"><input type=\"checkbox\" id=\"aec\" data-control=\"aec\" checked /> Auto Exposure</label>"
    "<label class=\"toggle\"><input type=\"checkbox\" id=\"agc\" data-control=\"agc\" checked /> Auto Gain Control</label>"
    "<label class=\"toggle\"><input type=\"checkbox\" id=\"flash\" data-control=\"flash\" /> Flashlight</label>"
    "<label class=\"toggle\"><input type=\"checkbox\" id=\"hmirror\" data-control=\"hmirror\" /> Horizontal Mirror</label>"
    "<label class=\"toggle\"><input type=\"checkbox\" id=\"vflip\" data-control=\"vflip\" /> Vertical Flip</label>"
    "</div>"
    "</section>"
    "<section class=\"card\">"
    "<h2>Maintenance</h2>"
    "<p>Factory reset clears stored WiFi credentials and returns the device to provisioning mode.</p>"
    "<button id=\"factory-reset\">Factory Reset</button>"
    "</section>"
    "<section class=\"card\">"
    "<h2>Connection</h2>"
    "<div class=\"info-grid\">"
    "<div class=\"info-item\"><div><strong>SSID</strong></div><div id=\"info-ssid\">-</div></div>"
    "<div class=\"info-item\"><div><strong>RSSI</strong></div><div><span id=\"info-rssi\">-</span> dBm</div></div>"
    "<div class=\"info-item\"><div><strong>IP Address</strong></div><div id=\"info-ip\">-</div></div>"
    "</div>"
    "</section>"
    "</div>"
    "<script>"
    "const infoSsid=document.getElementById('info-ssid');"
    "const infoRssi=document.getElementById('info-rssi');"
    "const infoIp=document.getElementById('info-ip');"
    "const diagMessage=document.getElementById('diag-message');"
    "const captureBtn=document.getElementById('capture');"
    "const snapshotImg=document.getElementById('snapshot');"
    "const framesizeSelect=document.getElementById('framesize');"
    "const flashToggle=document.getElementById('flash');"
    "const flashBrightness=document.getElementById('flash-brightness');"
    "const flashBrightnessValue=document.getElementById('flash-brightness-value');"
    "const controlElements=document.querySelectorAll('[data-control]');"
    "const factoryResetBtn=document.getElementById('factory-reset');"
    "const factoryResetDefaultText=factoryResetBtn?factoryResetBtn.textContent:'';"
    "let snapshotInitialized=false;"
    "let captureInFlight=false;"
    "let activeObjectUrl=null;"
    "let autoCaptureTriggered=false;"
    "function updateValueDisplay(element){"
        "const badge=document.getElementById(`${element.id}-value`);"
        "if(badge){badge.textContent=element.type==='checkbox'?(element.checked?'On':'Off'):element.value;}"
    "}"
    "async function applyControl(element){"
        "const varName=element.dataset.control;"
        "const value=element.type==='checkbox'?(element.checked?1:0):element.value;"
        "const url=`/api/control?var=${encodeURIComponent(varName)}&val=${encodeURIComponent(value)}`;"
        "try{"
            "const res=await fetch(url);"
            "if(!res.ok){console.error('Control update failed',varName,res.status);}" 
            "await loadStatus();"
        "}catch(err){console.error(err);}" 
    "}"
    "controlElements.forEach(el=>{"
        "updateValueDisplay(el);"
        "el.addEventListener(el.type==='range'?'input':'change',()=>{updateValueDisplay(el);});"
        "el.addEventListener('change',()=>{applyControl(el);});"
    "});"
    "if(factoryResetBtn){"
        "factoryResetBtn.addEventListener('click',async()=>{"
            "const confirmReset=confirm('Factory reset will erase saved WiFi settings and restart the device. Continue?');"
            "if(!confirmReset){return;}"
            "factoryResetBtn.disabled=true;"
            "factoryResetBtn.textContent='Resetting...';"
            "try{"
                "const res=await fetch('/api/factory_reset?confirm=factory_reset',{method:'POST'});"
                "if(!res.ok){throw new Error('reset_failed');}"
            "}catch(err){"
                "console.error(err);"
                "factoryResetBtn.disabled=false;"
                "factoryResetBtn.textContent=factoryResetDefaultText||'Factory Reset';"
            "}"
        "});"
    "}"
    "async function loadStatus(){"
        "try{"
            "const res=await fetch('/api/status');"
            "const data=await res.json();"
            "if(data.connected){"
                "infoSsid.textContent=data.ssid||'-';"
                "infoRssi.textContent=data.rssi!=null?data.rssi:'-';"
                "infoIp.textContent=data.ip||'-';"
            "}else{"
                "infoSsid.textContent='Not connected';"
                "infoRssi.textContent='-';"
                "infoIp.textContent='-';"
            "}"
            "if(factoryResetBtn){"
                "if(data.factory_reset_pending||data.restart_pending){"
                    "factoryResetBtn.disabled=true;"
                    "factoryResetBtn.textContent=data.factory_reset_pending?'Resetting...':'Restarting...';"
                "}else{"
                    "factoryResetBtn.disabled=false;"
                    "factoryResetBtn.textContent=factoryResetDefaultText||'Factory Reset';"
                "}"
            "}"
            "if(diagMessage){"
                "const messages=[];"
                "if(!data.connected){"
                    "if(data.sta_connecting){messages.push('Attempting to connect to WiFi...');}"
                    "if(data.sta_reason_message){messages.push(data.sta_reason_message);}"
                "}else{"
                    "messages.push('WiFi connected.');"
                "}"
                "const cameraMessageText=typeof data.camera_message==='string'?data.camera_message:'';"
                "if(cameraMessageText){messages.push(cameraMessageText);}" 
                "const cameraMessageMentionsPsram=cameraMessageText.toLowerCase().includes('psram');"
                "if(!cameraMessageMentionsPsram){"
                    "if(data.camera_psram_detected===false){messages.push('PSRAM not detected; high resolutions above VGA are disabled.');}"
                    "else if(data.camera_psram_detected===true&&data.camera_low_mem){messages.push('PSRAM detected but running in reduced-memory mode.');}"
                    "else if(data.camera_psram_detected===true){messages.push('PSRAM detected.');}"
                    "else if(data.camera_low_mem){messages.push('Running in reduced-memory mode.');}"
                "}"
                "if(data.camera_frame_width&&data.camera_frame_height){messages.push(`Frame ${data.camera_frame_width}x${data.camera_frame_height}`);}" 
                "if(data.camera_flash_supported===false){messages.push('Flashlight not supported on this device.');}" 
                "else if(data.camera_flash_enabled){"
                    "if(typeof data.camera_flash_brightness==='number'&&data.camera_flash_brightness>=0){"
                        "messages.push(`Flashlight enabled (${data.camera_flash_brightness}% brightness).`);"
                    "}else{messages.push('Flashlight enabled (fixed brightness).');}"
                "}" 
                "else if(typeof data.camera_flash_brightness!=='number'||data.camera_flash_brightness<0){messages.push('Flashlight brightness control unavailable on this device.');}"
                "if(messages.length===0){messages.push('Diagnostics idle.');}" 
                "diagMessage.textContent=messages.join(' ');"
            "}"
            "if(framesizeSelect){"
                "const desired=typeof data.camera_framesize==='number'?String(data.camera_framesize):'';"
                "if(desired&&framesizeSelect.value!==desired){framesizeSelect.value=desired;}"
                "const lowMem=!!data.camera_low_mem;"
                "Array.from(framesizeSelect.options).forEach(opt=>{"
                    "const numeric=parseInt(opt.value,10);"
                    "if(Number.isNaN(numeric)){return;}"
                    "opt.disabled=lowMem&&numeric>10;"
                "});"
            "}"
            "if(flashToggle){"
                "const supported=data.camera_flash_supported!==false;"
                "flashToggle.disabled=!supported;"
                "flashToggle.checked=supported&&!!data.camera_flash_enabled;"
                "if(!supported){flashToggle.checked=false;}"
                "const flashLabel=flashToggle.closest('.toggle');"
                "if(flashLabel){flashLabel.classList.toggle('disabled',!supported);}"
            "}"
            "if(flashBrightness){"
                "const brightnessSupported=(data.camera_flash_supported!==false)&&typeof data.camera_flash_brightness==='number'&&data.camera_flash_brightness>=0;"
                "const brightness=brightnessSupported?data.camera_flash_brightness:0;"
                "flashBrightness.disabled=!brightnessSupported;"
                "flashBrightness.value=brightness;"
                "updateValueDisplay(flashBrightness);"
                "if(flashBrightnessValue){flashBrightnessValue.textContent=brightnessSupported?flashBrightness.value:'--';}"
                "const flashBrightnessControl=flashBrightness.closest('.control');"
                "if(flashBrightnessControl){flashBrightnessControl.classList.toggle('disabled',!brightnessSupported);}" 
            "}"
            "if(snapshotImg&&data.camera_frame_width&&data.camera_frame_height){"
                "snapshotImg.alt=`Camera snapshot (${data.camera_frame_width}x${data.camera_frame_height})`;"
            "}"
            "if(!snapshotInitialized&&data.camera_ready&&!captureInFlight&&!autoCaptureTriggered){"
                "autoCaptureTriggered=true;"
                "performCapture(true);"
            "}"
        "}catch(err){"
            "console.error(err);"
            "if(diagMessage){diagMessage.textContent='Status refresh failed. Ensure the device remains connected.';}"
        "}"
    "}"
    "async function performCapture(auto){"
        "if(captureInFlight){return;}"
        "captureInFlight=true;"
        "const previousDisabled=captureBtn.disabled;"
        "const previousText=captureBtn.textContent;"
        "if(!auto){captureBtn.disabled=true;captureBtn.textContent='Capturing...';}"
        "try{"
            "const response=await fetch(`/api/capture?ts=${Date.now()}`);"
            "if(!response.ok){"
                "const errorText=await response.text();"
                "throw new Error(errorText||'capture_failed');"
            "}"
            "const blob=await response.blob();"
            "if(activeObjectUrl){URL.revokeObjectURL(activeObjectUrl);}"
            "activeObjectUrl=URL.createObjectURL(blob);"
            "snapshotImg.src=activeObjectUrl;"
            "snapshotImg.alt=`Camera snapshot (${new Date().toLocaleTimeString()})`;"
            "snapshotInitialized=true;"
            "loadStatus();"
        "}catch(err){"
            "console.error(err);"
            "let message=err.message||'Check the diagnostics below.';"
            "try{const parsed=JSON.parse(err.message);if(parsed&&parsed.message){message=parsed.message;}}catch(parseErr){}"
            "if(diagMessage){diagMessage.textContent='Capture failed. '+message;}"
            "if(auto){autoCaptureTriggered=false;}"
            "loadStatus();"
        "}finally{"
            "if(auto){captureBtn.disabled=previousDisabled;captureBtn.textContent=previousText;}else{captureBtn.disabled=false;captureBtn.textContent=previousText;}"
            "captureInFlight=false;"
        "}"
    "}"
    "captureBtn.addEventListener('click',()=>{performCapture(false);});"
    "loadStatus();"
    "setInterval(loadStatus,5000);"
    "</script>"
    "</body>"
    "</html>";

static void restart_timer_callback(void *arg) {
    ESP_LOGI(HTTP_SERVER_TAG, "Restart timer elapsed. Rebooting device.");
    esp_restart();
}

static void ensure_restart_timer_created(void) {
    if (s_restart_timer_handle) {
        return;
    }
    const esp_timer_create_args_t timer_args = {
        .callback = restart_timer_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "provision_restart"
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &s_restart_timer_handle));
}

static void schedule_restart_countdown(void) {
    ensure_restart_timer_created();
    if (!s_restart_timer_handle) {
        return;
    }
    esp_err_t err = wifi_manager_disable_softap();
    if (err != ESP_OK) {
        ESP_LOGW(HTTP_SERVER_TAG, "Failed to disable SoftAP before restart: %s", esp_err_to_name(err));
    }
    s_restart_deadline_us = esp_timer_get_time() + RESTART_DELAY_US;
    ESP_ERROR_CHECK(esp_timer_stop(s_restart_timer_handle));
    ESP_ERROR_CHECK(esp_timer_start_once(s_restart_timer_handle, RESTART_DELAY_US));
    s_restart_timer_active = true;
    ESP_LOGI(HTTP_SERVER_TAG, "Scheduled device restart in %d seconds", RESTART_DELAY_US / 1000000);
}

static void cancel_restart_countdown(void) {
    if (s_restart_timer_handle && s_restart_timer_active) {
        ESP_ERROR_CHECK(esp_timer_stop(s_restart_timer_handle));
    }
    s_restart_timer_active = false;
    s_restart_deadline_us = 0;
}

static int escape_json_string(const char *input, char *output, size_t output_len) {
    if (!input || !output || output_len == 0) {
        return 0;
    }
    size_t written = 0;
    for (size_t i = 0; input[i] != '\0' && written + 1 < output_len; ++i) {
        unsigned char c = input[i];
        if (c == '"' || c == '\\') {
            if (written + 2 >= output_len) {
                break;
            }
            output[written++] = '\\';
            output[written++] = (char)c;
        } else if (c < 0x20) {
            if (written + 6 >= output_len) {
                break;
            }
            int len = snprintf(output + written, output_len - written, "\\u%04x", c);
            if (len < 0 || (size_t)len >= output_len - written) {
                break;
            }
            written += (size_t)len;
        } else {
            output[written++] = (char)c;
        }
    }
    if (written < output_len) {
        output[written] = '\0';
    } else {
        output[output_len - 1] = '\0';
    }
    return (int)written;
}

static void factory_reset_timer_callback(void *arg) {
    ESP_LOGI(HTTP_SERVER_TAG, "Factory reset timer triggered");
    esp_err_t err = wifi_manager_factory_reset();
    if (err != ESP_OK) {
        ESP_LOGE(HTTP_SERVER_TAG, "Factory reset failed: %s", esp_err_to_name(err));
    }
    esp_restart();
}

static void ensure_factory_reset_timer_created(void) {
    if (s_factory_reset_timer_handle) {
        return;
    }
    const esp_timer_create_args_t timer_args = {
        .callback = factory_reset_timer_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "factory_reset"
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &s_factory_reset_timer_handle));
}

static void schedule_factory_reset_reboot(void) {
    ensure_factory_reset_timer_created();
    if (!s_factory_reset_timer_handle) {
        return;
    }
    esp_timer_stop(s_factory_reset_timer_handle);
    s_factory_reset_deadline_us = esp_timer_get_time() + 1000000;
    ESP_ERROR_CHECK(esp_timer_start_once(s_factory_reset_timer_handle, 1000000)); // 1 second
    s_factory_reset_pending = true;
    ESP_LOGI(HTTP_SERVER_TAG, "Factory reset scheduled in 1 second");
}

typedef struct {
    const char *label;
    framesize_t framesize;
} framesize_map_entry_t;

static const framesize_map_entry_t s_framesize_map[] = {
    {"96X96", FRAMESIZE_96X96},
    {"QQVGA", FRAMESIZE_QQVGA},
    {"128X128", FRAMESIZE_128X128},
    {"QCIF", FRAMESIZE_QCIF},
    {"HQVGA", FRAMESIZE_HQVGA},
    {"240X240", FRAMESIZE_240X240},
    {"QVGA", FRAMESIZE_QVGA},
    {"320X320", FRAMESIZE_320X320},
    {"CIF", FRAMESIZE_CIF},
    {"HVGA", FRAMESIZE_HVGA},
    {"VGA", FRAMESIZE_VGA},
    {"SVGA", FRAMESIZE_SVGA},
    {"XGA", FRAMESIZE_XGA},
    {"HD", FRAMESIZE_HD},
    {"SXGA", FRAMESIZE_SXGA},
    {"UXGA", FRAMESIZE_UXGA},
    {"FHD", FRAMESIZE_FHD},
    {"P_HD", FRAMESIZE_P_HD},
    {"P_3MP", FRAMESIZE_P_3MP},
    {"QXGA", FRAMESIZE_QXGA},
    {"QHD", FRAMESIZE_QHD},
    {"WQXGA", FRAMESIZE_WQXGA},
    {"P_FHD", FRAMESIZE_P_FHD},
    {"QSXGA", FRAMESIZE_QSXGA},
    {"5MP", FRAMESIZE_5MP},
};

static framesize_t camera_framesize_from_label(const char *label, bool *out_supported) {
    if (out_supported) {
        *out_supported = false;
    }
    if (!label) {
        return FRAMESIZE_INVALID;
    }

    char upper[32];
    size_t len = strlen(label);
    if (len >= sizeof(upper)) {
        len = sizeof(upper) - 1;
    }
    for (size_t i = 0; i < len; ++i) {
        upper[i] = (char)toupper((unsigned char)label[i]);
    }
    upper[len] = '\0';

    for (size_t i = 0; i < sizeof(s_framesize_map) / sizeof(s_framesize_map[0]); ++i) {
        if (strcmp(upper, s_framesize_map[i].label) == 0) {
            if (out_supported) {
                *out_supported = true;
            }
            return s_framesize_map[i].framesize;
        }
    }
    return FRAMESIZE_INVALID;
}

static bool parse_boolean_arg(const char *value_str, bool *out_value) {
    if (!value_str || !out_value) {
        return false;
    }
    if (value_str[0] == '0' && value_str[1] == '\0') {
        *out_value = false;
        return true;
    }
    if (value_str[0] == '1' && value_str[1] == '\0') {
        *out_value = true;
        return true;
    }
    return false;
}

static void append_filesystem_stats_json(char **buffer_ptr, size_t *remaining) {
    if (!buffer_ptr || !*buffer_ptr || !remaining || *remaining == 0) {
        return;
    }

    long long total = -1;
    long long free_space = -1;
#if HTTP_HAS_STATVFS
    struct statvfs vfs_info;
    if (statvfs("/", &vfs_info) == 0) {
        total = (long long)vfs_info.f_frsize * (long long)vfs_info.f_blocks;
        free_space = (long long)vfs_info.f_frsize * (long long)vfs_info.f_bavail;
    }
#endif

    int written = snprintf(*buffer_ptr, *remaining, "\"storage_total\":%lld,\"storage_free\":%lld",
                           total, free_space);
    if (written > 0 && (size_t)written < *remaining) {
        *buffer_ptr += written;
        *remaining -= (size_t)written;
    } else {
        *remaining = 0;
    }
}

static esp_err_t handle_root(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    if (wifi_manager_is_connected()) {
        return httpd_resp_send(req, DASHBOARD_HTML, HTTPD_RESP_USE_STRLEN);
    }
    return httpd_resp_send(req, PROVISION_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t handle_scan(httpd_req_t *req) {
    static char response[HTTP_SCAN_BUFFER_SIZE];
    esp_err_t err = wifi_manager_scan_networks(response, sizeof(response));
    if (err != ESP_OK) {
        ESP_LOGE(HTTP_SERVER_TAG, "Network scan failed: %s", esp_err_to_name(err));
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_send(req, "{\"error\":\"scan_failed\"}", HTTPD_RESP_USE_STRLEN);
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t handle_config(httpd_req_t *req) {
    char content[HTTP_RESPONSE_BUF_SIZE];
    size_t received = 0;

    while (received < req->content_len && received < sizeof(content) - 1) {
        size_t remaining = req->content_len - received;
        size_t space = sizeof(content) - 1 - received;
        size_t chunk = remaining < space ? remaining : space;
        int ret = httpd_req_recv(req, content + received, chunk);
        if (ret <= 0) {
            break;
        }
        received += (size_t)ret;
    }

    if (received == 0) {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_send(req, "{\"error\":\"empty_body\"}", HTTPD_RESP_USE_STRLEN);
    }

    if (received >= sizeof(content)) {
        received = sizeof(content) - 1;
    }
    content[received] = '\0';

    wifi_credentials_t credentials = {0};
    char value[WIFI_MANAGER_MAX_PASSWORD_LEN];

    if (httpd_query_key_value(content, "ssid", credentials.ssid, sizeof(credentials.ssid)) != ESP_OK) {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_send(req, "{\"error\":\"missing_ssid\"}", HTTPD_RESP_USE_STRLEN);
    }
    if (httpd_query_key_value(content, "password", value, sizeof(value)) == ESP_OK) {
        strncpy(credentials.password, value, sizeof(credentials.password));
    }

    url_decode(credentials.ssid);
    url_decode(credentials.password);

    if (credentials.ssid[0] == '\0') {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_send(req, "{\"error\":\"invalid_ssid\"}", HTTPD_RESP_USE_STRLEN);
    }

    ESP_LOGI(HTTP_SERVER_TAG, "Received new credentials for SSID: %s", credentials.ssid);

    esp_err_t err = wifi_manager_save_credentials(&credentials);
    if (err != ESP_OK) {
        ESP_LOGE(HTTP_SERVER_TAG, "Failed to save credentials: %s", esp_err_to_name(err));
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_send(req, "{\"error\":\"save_failed\"}", HTTPD_RESP_USE_STRLEN);
    }

    ESP_LOGI(HTTP_SERVER_TAG, "Credentials saved. Connecting to WiFi...");

    cancel_restart_countdown();
    s_config_session_active = true;

    err = wifi_manager_connect_station(&credentials);
    if (err != ESP_OK) {
        ESP_LOGE(HTTP_SERVER_TAG, "Failed to start STA mode (%s)", esp_err_to_name(err));
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_send(req, "{\"error\":\"connect_failed\"}", HTTPD_RESP_USE_STRLEN);
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, "{\"status\":\"ok\"}", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t handle_status(httpd_req_t *req) {
    bool connected = wifi_manager_is_connected();
    bool provisioning = wifi_manager_is_provisioning();
    bool factory_pending = s_factory_reset_pending;

    bool camera_initialized = camera_manager_is_initialized();
    bool camera_ready = camera_manager_is_ready();
    esp_err_t camera_error = camera_manager_last_error();
    const char *camera_message = camera_manager_status_message();

    char camera_message_escaped[256];
    escape_json_string(camera_message ? camera_message : "", camera_message_escaped, sizeof(camera_message_escaped));

    wifi_sta_status_t sta_status = {0};
    wifi_manager_get_sta_status(&sta_status);
    const char *sta_reason_text = wifi_manager_reason_to_string(sta_status.last_disconnect_reason);
    char sta_reason_escaped[256];
    escape_json_string(sta_reason_text ? sta_reason_text : "", sta_reason_escaped, sizeof(sta_reason_escaped));

    if (provisioning) {
        s_config_session_active = false;
        cancel_restart_countdown();
    }

    char ssid[WIFI_MANAGER_MAX_SSID_LEN + 1] = {0};
    int rssi = 0;
    char ip_str[16] = "0.0.0.0";

    if (connected) {
        wifi_ap_record_t ap_info = {0};
        if (wifi_manager_get_connected_ap(&ap_info) == ESP_OK) {
            strncpy(ssid, (const char *)ap_info.ssid, sizeof(ssid) - 1);
            rssi = ap_info.rssi;
        }
        esp_netif_ip_info_t ip_info = {0};
        if (wifi_manager_get_ip_info(&ip_info) == ESP_OK) {
            esp_ip4addr_ntoa(&ip_info.ip, ip_str, sizeof(ip_str));
        }
    }

    if (connected && s_config_session_active && !s_restart_timer_active) {
        schedule_restart_countdown();
    }

    int restart_in_ms = 0;
    bool restart_pending = false;
    int64_t now = esp_timer_get_time();
    if (s_factory_reset_pending) {
        int64_t remaining = s_factory_reset_deadline_us - now;
        if (remaining < 0) {
            remaining = 0;
        }
        restart_in_ms = (int)(remaining / 1000);
        restart_pending = true;
    }
    if (s_restart_timer_active) {
        int64_t remaining = s_restart_deadline_us - now;
        if (remaining < 0) {
            remaining = 0;
        }
        int restart_ms = (int)(remaining / 1000);
        if (!restart_pending || restart_ms < restart_in_ms) {
            restart_in_ms = restart_ms;
        }
        restart_pending = true;
    }

    char escaped_ssid[(WIFI_MANAGER_MAX_SSID_LEN * 4) + 4];
    escape_json_string(ssid, escaped_ssid, sizeof(escaped_ssid));

    const char *mode = provisioning ? "provisioning" : (connected ? "station" : "connecting");
    bool camera_low_mem = camera_manager_is_low_mem_mode();
    bool camera_psram_detected = camera_manager_psram_detected();
    framesize_t camera_framesize = camera_manager_current_framesize();
    uint16_t camera_frame_width = 0;
    uint16_t camera_frame_height = 0;
    camera_manager_get_frame_dimensions(&camera_frame_width, &camera_frame_height);
    bool camera_flash_supported = camera_manager_is_flash_supported();
    bool camera_flash_enabled = camera_manager_is_flash_enabled();
    int camera_flash_brightness = camera_manager_flash_brightness();

    char response[1200];
    snprintf(
        response,
        sizeof(response),
        "{\"connected\":%s,\"mode\":\"%s\",\"ssid\":\"%s\",\"rssi\":%d,\"ip\":\"%s\",\"restart_in_ms\":%d,\"restart_pending\":%s,\"factory_reset_pending\":%s,\"camera_ready\":%s,\"camera_initialized\":%s,\"camera_error_code\":%d,\"camera_message\":\"%s\",\"camera_low_mem\":%s,\"camera_psram_detected\":%s,\"camera_framesize\":%d,\"camera_frame_width\":%u,\"camera_frame_height\":%u,\"camera_flash_supported\":%s,\"camera_flash_enabled\":%s,\"camera_flash_brightness\":%d,\"sta_connecting\":%s,\"sta_retry_count\":%d,\"sta_reason\":%d,\"sta_reason_message\":\"%s\"}",
        connected ? "true" : "false",
        mode,
        escaped_ssid,
        connected ? rssi : 0,
        connected ? ip_str : "",
        restart_in_ms,
        restart_pending ? "true" : "false",
        factory_pending ? "true" : "false",
        camera_ready ? "true" : "false",
        camera_initialized ? "true" : "false",
        (int)camera_error,
        camera_message_escaped,
        camera_low_mem ? "true" : "false",
        camera_psram_detected ? "true" : "false",
        (int)camera_framesize,
        (unsigned int)camera_frame_width,
        (unsigned int)camera_frame_height,
        camera_flash_supported ? "true" : "false",
        camera_flash_enabled ? "true" : "false",
        camera_flash_brightness,
        sta_status.connecting ? "true" : "false",
        sta_status.retry_count,
        sta_status.last_disconnect_reason,
        sta_reason_escaped
    );

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t handle_device_status(httpd_req_t *req) {
    bool connected = wifi_manager_is_connected();
    wifi_ap_record_t ap_info = {0};
    char ssid[WIFI_MANAGER_MAX_SSID_LEN + 1] = {0};
    int rssi = 0;
    if (connected && wifi_manager_get_connected_ap(&ap_info) == ESP_OK) {
        strncpy(ssid, (const char *)ap_info.ssid, sizeof(ssid) - 1);
        rssi = ap_info.rssi;
    }

    char escaped_ssid[(WIFI_MANAGER_MAX_SSID_LEN * 4) + 4];
    escape_json_string(ssid, escaped_ssid, sizeof(escaped_ssid));

    uint64_t uptime_seconds = esp_timer_get_time() / 1000000ULL;

    char response[256];
    char *cursor = response;
    size_t remaining = sizeof(response);

    int written = snprintf(cursor, remaining,
                           "{\"ssid\":\"%s\",\"connected\":%s,\"rssi\":%d,\"uptime_seconds\":%llu,",
                           escaped_ssid,
                           connected ? "true" : "false",
                           connected ? rssi : 0,
                           (unsigned long long)uptime_seconds);
    if (written < 0 || (size_t)written >= remaining) {
        written = (int)remaining;
    }
    cursor += written;
    remaining -= (size_t)written;

    append_filesystem_stats_json(&cursor, &remaining);
    if (remaining > 0) {
        *cursor++ = '}';
        *cursor = '\0';
    } else {
        response[sizeof(response) - 2] = '}';
        response[sizeof(response) - 1] = '\0';
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t handle_capture_internal(httpd_req_t *req, bool use_query_controls) {
    char *query_buffer = NULL;
    bool controls_applied = false;
    esp_err_t result = ESP_OK;

    sensor_t *sensor = esp_camera_sensor_get();
    if (!sensor) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, "{\"error\":\"sensor_unavailable\"}", HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }

    camera_status_t prev_status = sensor->status;
    bool framesize_changed = false;
    bool quality_changed = false;
    bool brightness_changed = false;
    bool contrast_changed = false;
    bool saturation_changed = false;
    bool sharpness_changed = false;
    bool special_effect_changed = false;
    bool awb_changed = false;
    bool aec_changed = false;
    bool agc_changed = false;
    bool hmirror_changed = false;
    bool vflip_changed = false;
    bool flash_changed = false;
    bool flash_brightness_changed = false;

    bool prev_flash_enabled = camera_manager_is_flash_enabled();
    int prev_flash_brightness = camera_manager_flash_brightness();

    if (use_query_controls) {
        int query_len = httpd_req_get_url_query_len(req);
        if (query_len > 0) {
            query_buffer = malloc((size_t)query_len + 1);
            if (!query_buffer) {
                httpd_resp_set_status(req, "500 Internal Server Error");
                httpd_resp_set_type(req, "application/json");
                httpd_resp_send(req, "{\"error\":\"no_memory\"}", HTTPD_RESP_USE_STRLEN);
                return ESP_OK;
            }
            if (httpd_req_get_url_query_str(req, query_buffer, (size_t)query_len + 1) != ESP_OK) {
                free(query_buffer);
                httpd_resp_set_status(req, "400 Bad Request");
                httpd_resp_set_type(req, "application/json");
                httpd_resp_send(req, "{\"error\":\"invalid_query\"}", HTTPD_RESP_USE_STRLEN);
                return ESP_OK;
            }

            char *context = NULL;
            char *token = strtok_r(query_buffer, "&", &context);
            while (token) {
                char *eq = strchr(token, '=');
                char *key = token;
                char *value = eq ? (eq + 1) : (char *)"";
                if (eq) {
                    *eq = '\0';
                }
                url_decode(key);
                url_decode(value);
                for (char *p = key; *p; ++p) {
                    *p = (char)tolower((unsigned char)*p);
                }

                esp_err_t control_err = ESP_OK;
                bool handled = true;

                if (strcmp(key, "resolution") == 0 || strcmp(key, "framesize") == 0) {
                    bool supported = false;
                    framesize_t target = camera_framesize_from_label(value, &supported);
                    if (!supported) {
                        control_err = ESP_ERR_INVALID_ARG;
                    } else if (target != sensor->status.framesize) {
                        control_err = camera_manager_control("framesize", (int)target);
                        if (control_err == ESP_OK) {
                            framesize_changed = true;
                            controls_applied = true;
                        }
                    }
                } else if (strcmp(key, "quality") == 0) {
                    char *end = NULL;
                    long parsed = strtol(value, &end, 10);
                    if (!value[0] || end == NULL || *end != '\0' || parsed < 10 || parsed > 63) {
                        control_err = ESP_ERR_INVALID_ARG;
                    } else if ((int)parsed != sensor->status.quality) {
                        control_err = camera_manager_control("quality", (int)parsed);
                        if (control_err == ESP_OK) {
                            quality_changed = true;
                            controls_applied = true;
                        }
                    }
                } else if (strcmp(key, "brightness") == 0) {
                    char *end = NULL;
                    long parsed = strtol(value, &end, 10);
                    if (!value[0] || end == NULL || *end != '\0' || parsed < -2 || parsed > 2) {
                        control_err = ESP_ERR_INVALID_ARG;
                    } else if ((int)parsed != sensor->status.brightness) {
                        control_err = camera_manager_control("brightness", (int)parsed);
                        if (control_err == ESP_OK) {
                            brightness_changed = true;
                            controls_applied = true;
                        }
                    }
                } else if (strcmp(key, "contrast") == 0) {
                    char *end = NULL;
                    long parsed = strtol(value, &end, 10);
                    if (!value[0] || end == NULL || *end != '\0' || parsed < -2 || parsed > 2) {
                        control_err = ESP_ERR_INVALID_ARG;
                    } else if ((int)parsed != sensor->status.contrast) {
                        control_err = camera_manager_control("contrast", (int)parsed);
                        if (control_err == ESP_OK) {
                            contrast_changed = true;
                            controls_applied = true;
                        }
                    }
                } else if (strcmp(key, "saturation") == 0) {
                    char *end = NULL;
                    long parsed = strtol(value, &end, 10);
                    if (!value[0] || end == NULL || *end != '\0' || parsed < -2 || parsed > 2) {
                        control_err = ESP_ERR_INVALID_ARG;
                    } else if ((int)parsed != sensor->status.saturation) {
                        control_err = camera_manager_control("saturation", (int)parsed);
                        if (control_err == ESP_OK) {
                            saturation_changed = true;
                            controls_applied = true;
                        }
                    }
                } else if (strcmp(key, "sharpness") == 0) {
                    char *end = NULL;
                    long parsed = strtol(value, &end, 10);
                    if (!value[0] || end == NULL || *end != '\0' || parsed < -2 || parsed > 2) {
                        control_err = ESP_ERR_INVALID_ARG;
                    } else if ((int)parsed != sensor->status.sharpness) {
                        control_err = camera_manager_control("sharpness", (int)parsed);
                        if (control_err == ESP_OK) {
                            sharpness_changed = true;
                            controls_applied = true;
                        }
                    }
                } else if (strcmp(key, "special_effect") == 0) {
                    char *end = NULL;
                    long parsed = strtol(value, &end, 10);
                    if (!value[0] || end == NULL || *end != '\0' || parsed < 0 || parsed > 6) {
                        control_err = ESP_ERR_INVALID_ARG;
                    } else if ((int)parsed != sensor->status.special_effect) {
                        control_err = camera_manager_control("special_effect", (int)parsed);
                        if (control_err == ESP_OK) {
                            special_effect_changed = true;
                            controls_applied = true;
                        }
                    }
                } else if (strcmp(key, "awb") == 0) {
                    bool parsed_bool;
                    if (!parse_boolean_arg(value, &parsed_bool)) {
                        control_err = ESP_ERR_INVALID_ARG;
                    } else if ((parsed_bool ? 1 : 0) != sensor->status.awb) {
                        control_err = camera_manager_control("awb", parsed_bool ? 1 : 0);
                        if (control_err == ESP_OK) {
                            awb_changed = true;
                            controls_applied = true;
                        }
                    }
                } else if (strcmp(key, "aec") == 0) {
                    bool parsed_bool;
                    if (!parse_boolean_arg(value, &parsed_bool)) {
                        control_err = ESP_ERR_INVALID_ARG;
                    } else if ((parsed_bool ? 1 : 0) != sensor->status.aec) {
                        control_err = camera_manager_control("aec", parsed_bool ? 1 : 0);
                        if (control_err == ESP_OK) {
                            aec_changed = true;
                            controls_applied = true;
                        }
                    }
                } else if (strcmp(key, "agc") == 0) {
                    bool parsed_bool;
                    if (!parse_boolean_arg(value, &parsed_bool)) {
                        control_err = ESP_ERR_INVALID_ARG;
                    } else if ((parsed_bool ? 1 : 0) != sensor->status.agc) {
                        control_err = camera_manager_control("agc", parsed_bool ? 1 : 0);
                        if (control_err == ESP_OK) {
                            agc_changed = true;
                            controls_applied = true;
                        }
                    }
                } else if (strcmp(key, "hmirror") == 0) {
                    bool parsed_bool;
                    if (!parse_boolean_arg(value, &parsed_bool)) {
                        control_err = ESP_ERR_INVALID_ARG;
                    } else if ((parsed_bool ? 1 : 0) != sensor->status.hmirror) {
                        control_err = camera_manager_control("hmirror", parsed_bool ? 1 : 0);
                        if (control_err == ESP_OK) {
                            hmirror_changed = true;
                            controls_applied = true;
                        }
                    }
                } else if (strcmp(key, "vflip") == 0) {
                    bool parsed_bool;
                    if (!parse_boolean_arg(value, &parsed_bool)) {
                        control_err = ESP_ERR_INVALID_ARG;
                    } else if ((parsed_bool ? 1 : 0) != sensor->status.vflip) {
                        control_err = camera_manager_control("vflip", parsed_bool ? 1 : 0);
                        if (control_err == ESP_OK) {
                            vflip_changed = true;
                            controls_applied = true;
                        }
                    }
                } else if (strcmp(key, "flash") == 0) {
                    bool parsed_bool;
                    if (!parse_boolean_arg(value, &parsed_bool)) {
                        control_err = ESP_ERR_INVALID_ARG;
                    } else {
                        esp_err_t flash_err = camera_manager_set_flash_enabled(parsed_bool);
                        if (flash_err == ESP_ERR_NOT_SUPPORTED) {
                            control_err = ESP_ERR_NOT_SUPPORTED;
                        } else if (flash_err == ESP_OK) {
                            flash_changed = true;
                            controls_applied = true;
                        } else {
                            control_err = flash_err;
                        }
                    }
                } else if (strcmp(key, "flash_brightness") == 0) {
                    char *end = NULL;
                    long parsed = strtol(value, &end, 10);
                    if (!value[0] || end == NULL || *end != '\0' || parsed < 0 || parsed > 100) {
                        control_err = ESP_ERR_INVALID_ARG;
                    } else {
                        esp_err_t flash_err = camera_manager_set_flash_brightness((int)parsed);
                        if (flash_err == ESP_ERR_NOT_SUPPORTED) {
                            control_err = ESP_ERR_NOT_SUPPORTED;
                        } else if (flash_err == ESP_OK) {
                            flash_brightness_changed = true;
                            controls_applied = true;
                        } else {
                            control_err = flash_err;
                        }
                    }
                } else if (strcmp(key, "ts") == 0) {
                    handled = true;
                } else {
                    handled = false;
                }

                if (!handled || control_err != ESP_OK) {
                    if (flash_brightness_changed && prev_flash_brightness >= 0) {
                        camera_manager_set_flash_brightness(prev_flash_brightness);
                    }
                    if (flash_changed) {
                        camera_manager_set_flash_enabled(prev_flash_enabled);
                    }
                    if (vflip_changed) {
                        camera_manager_control("vflip", prev_status.vflip);
                    }
                    if (hmirror_changed) {
                        camera_manager_control("hmirror", prev_status.hmirror);
                    }
                    if (agc_changed) {
                        camera_manager_control("agc", prev_status.agc);
                    }
                    if (aec_changed) {
                        camera_manager_control("aec", prev_status.aec);
                    }
                    if (awb_changed) {
                        camera_manager_control("awb", prev_status.awb);
                    }
                    if (special_effect_changed) {
                        camera_manager_control("special_effect", prev_status.special_effect);
                    }
                    if (sharpness_changed) {
                        camera_manager_control("sharpness", prev_status.sharpness);
                    }
                    if (saturation_changed) {
                        camera_manager_control("saturation", prev_status.saturation);
                    }
                    if (contrast_changed) {
                        camera_manager_control("contrast", prev_status.contrast);
                    }
                    if (brightness_changed) {
                        camera_manager_control("brightness", prev_status.brightness);
                    }
                    if (quality_changed) {
                        camera_manager_control("quality", prev_status.quality);
                    }
                    if (framesize_changed) {
                        camera_manager_control("framesize", prev_status.framesize);
                    }

                    char error_payload[128];
                    const char *error_key = (key && *key) ? key : "parameter";
                    const char *error_reason = (control_err == ESP_ERR_NOT_SUPPORTED) ? "not_supported" : "invalid";
                    snprintf(error_payload, sizeof(error_payload), "{\"error\":\"%s_%s\"}", error_reason, error_key);
                    httpd_resp_set_status(req, (control_err == ESP_ERR_NOT_SUPPORTED) ? "501 Not Implemented" : "400 Bad Request");
                    httpd_resp_set_type(req, "application/json");
                    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
                    httpd_resp_send(req, error_payload, HTTPD_RESP_USE_STRLEN);

                    free(query_buffer);
                    return ESP_OK;
                }

                token = strtok_r(NULL, "&", &context);
            }

            free(query_buffer);
            query_buffer = NULL;
        }
    }

    result = camera_manager_capture(req);

    if (controls_applied) {
        if (flash_brightness_changed && prev_flash_brightness >= 0) {
            camera_manager_set_flash_brightness(prev_flash_brightness);
        }
        if (flash_changed) {
            camera_manager_set_flash_enabled(prev_flash_enabled);
        }
        if (vflip_changed) {
            camera_manager_control("vflip", prev_status.vflip);
        }
        if (hmirror_changed) {
            camera_manager_control("hmirror", prev_status.hmirror);
        }
        if (agc_changed) {
            camera_manager_control("agc", prev_status.agc);
        }
        if (aec_changed) {
            camera_manager_control("aec", prev_status.aec);
        }
        if (awb_changed) {
            camera_manager_control("awb", prev_status.awb);
        }
        if (special_effect_changed) {
            camera_manager_control("special_effect", prev_status.special_effect);
        }
        if (sharpness_changed) {
            camera_manager_control("sharpness", prev_status.sharpness);
        }
        if (saturation_changed) {
            camera_manager_control("saturation", prev_status.saturation);
        }
        if (contrast_changed) {
            camera_manager_control("contrast", prev_status.contrast);
        }
        if (brightness_changed) {
            camera_manager_control("brightness", prev_status.brightness);
        }
        if (quality_changed) {
            camera_manager_control("quality", prev_status.quality);
        }
        if (framesize_changed) {
            camera_manager_control("framesize", prev_status.framesize);
        }
    }

    return result;
}

static esp_err_t handle_capture(httpd_req_t *req) {
    return handle_capture_internal(req, false);
}

static esp_err_t handle_capture_with_controls(httpd_req_t *req) {
    return handle_capture_internal(req, true);
}

static esp_err_t handle_camera_control(httpd_req_t *req) {
    int query_len = httpd_req_get_url_query_len(req);
    if (query_len <= 0) {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_send(req, "{\"error\":\"missing_query\"}", HTTPD_RESP_USE_STRLEN);
    }

    char *query = malloc((size_t)query_len + 1);
    if (!query) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_send(req, "{\"error\":\"no_memory\"}", HTTPD_RESP_USE_STRLEN);
    }

    esp_err_t err = httpd_req_get_url_query_str(req, query, (size_t)query_len + 1);
    if (err != ESP_OK) {
        free(query);
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_send(req, "{\"error\":\"invalid_query\"}", HTTPD_RESP_USE_STRLEN);
    }

    char var[32] = {0};
    char val_str[32] = {0};
    if (httpd_query_key_value(query, "var", var, sizeof(var)) != ESP_OK ||
        httpd_query_key_value(query, "val", val_str, sizeof(val_str)) != ESP_OK) {
        free(query);
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_send(req, "{\"error\":\"missing_params\"}", HTTPD_RESP_USE_STRLEN);
    }
    free(query);

    int value = atoi(val_str);
    err = camera_manager_control(var, value);
    if (err != ESP_OK) {
        ESP_LOGW(HTTP_SERVER_TAG, "Failed to apply camera control %s=%d (%s)", var, value, esp_err_to_name(err));
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_send(req, "{\"error\":\"invalid_control\"}", HTTPD_RESP_USE_STRLEN);
    }

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, "{\"status\":\"ok\"}", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t handle_factory_reset(httpd_req_t *req) {
    bool confirmed = false;
    int query_len = httpd_req_get_url_query_len(req);
    if (query_len > 0) {
        char *query = malloc((size_t)query_len + 1);
        if (!query) {
            httpd_resp_set_status(req, "500 Internal Server Error");
            httpd_resp_set_type(req, "application/json");
            httpd_resp_set_hdr(req, "Cache-Control", "no-store");
            return httpd_resp_send(req, "{\"error\":\"no_memory\"}", HTTPD_RESP_USE_STRLEN);
        }

        esp_err_t query_err = httpd_req_get_url_query_str(req, query, (size_t)query_len + 1);
        if (query_err != ESP_OK) {
            free(query);
            httpd_resp_set_status(req, "400 Bad Request");
            httpd_resp_set_type(req, "application/json");
            httpd_resp_set_hdr(req, "Cache-Control", "no-store");
            return httpd_resp_send(req, "{\"error\":\"invalid_query\"}", HTTPD_RESP_USE_STRLEN);
        }

        char confirm_value[32] = {0};
        if (httpd_query_key_value(query, "confirm", confirm_value, sizeof(confirm_value)) == ESP_OK &&
            strcmp(confirm_value, FACTORY_RESET_CONFIRM_TOKEN) == 0) {
            confirmed = true;
        }
        free(query);
    }

    if (!confirmed) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Cache-Control", "no-store");
        return httpd_resp_send(req, "{\"error\":\"confirm_token_required\"}", HTTPD_RESP_USE_STRLEN);
    }

    ESP_LOGI(HTTP_SERVER_TAG, "Factory reset requested via HTTP");
    cancel_restart_countdown();
    s_config_session_active = false;
    esp_err_t softap_err = wifi_manager_disable_softap();
    if (softap_err != ESP_OK) {
        ESP_LOGW(HTTP_SERVER_TAG, "Unable to disable SoftAP during factory reset: %s", esp_err_to_name(softap_err));
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t res = httpd_resp_send(req, "{\"status\":\"resetting\"}", HTTPD_RESP_USE_STRLEN);
    if (res == ESP_OK && !s_factory_reset_pending) {
        schedule_factory_reset_reboot();
    }
    return res;
}

static void url_decode(char *str) {
    char *src = str;
    char *dst = str;
    while (*src) {
        if (*src == '%') {
            if (src[1] && src[2]) {
                char hex[3] = {src[1], src[2], '\0'};
                *dst++ = (char)strtol(hex, NULL, 16);
                src += 3;
            } else {
                src++;
            }
        } else if (*src == '+') {
            *dst++ = ' ';
            src++;
        } else {
            *dst++ = *src++;
        }
    }
    *dst = '\0';
}

esp_err_t http_server_app_start(void) {
    if (s_http_handle) {
        return ESP_OK;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 10240;
    config.server_port = 80;
    config.uri_match_fn = httpd_uri_match_wildcard;
    config.recv_wait_timeout = 15;
    config.send_wait_timeout = 15;

    esp_err_t err = httpd_start(&s_http_handle, &config);
    if (err != ESP_OK) {
        ESP_LOGE(HTTP_SERVER_TAG, "Failed to start server: %s", esp_err_to_name(err));
        return err;
    }

    httpd_register_uri_handler(s_http_handle, &root_uri);
    httpd_register_uri_handler(s_http_handle, &scan_uri);
    httpd_register_uri_handler(s_http_handle, &config_uri);
    httpd_register_uri_handler(s_http_handle, &status_uri);
    httpd_register_uri_handler(s_http_handle, &device_status_uri);
    httpd_register_uri_handler(s_http_handle, &capture_uri);
    httpd_register_uri_handler(s_http_handle, &capture_with_controls_uri);
    httpd_register_uri_handler(s_http_handle, &control_uri);
    httpd_register_uri_handler(s_http_handle, &factory_reset_uri);

    ensure_restart_timer_created();

    ESP_LOGI(HTTP_SERVER_TAG, "HTTP server started on port %d", config.server_port);
    return ESP_OK;
}

void http_server_app_stop(void) {
    if (!s_http_handle) {
        return;
    }
    httpd_stop(s_http_handle);
    s_http_handle = NULL;
    cancel_restart_countdown();
    if (s_factory_reset_timer_handle) {
        esp_timer_stop(s_factory_reset_timer_handle);
    }
    s_factory_reset_pending = false;
    s_factory_reset_deadline_us = 0;
}
