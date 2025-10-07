/*
 * ESP32 Home Security System with FreeRTOS
 * Version: 1.0.0
 * Author: Julian Montoya
 * License: MIT
 * 
 * Features:
 * - Real-time door monitoring with FreeRTOS
 * - Multi-task concurrent execution on dual cores
 * - Google Home integration for whole-house alerts
 * - Time-based automatic arming (11 PM - 5 AM)
 * - Voice command deactivation
 * - Robust error handling and watchdog monitoring
 */

#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <time.h>
#include "esp_system.h"

// FreeRTOS Core Components
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "freertos/timers.h"

// Include configuration files
#include "secrets.h"  // Contains WiFi credentials and API keys

// Validate that secrets.h has been configured
#ifndef WIFI_SSID
  #error "Please create secrets.h from secrets.h.example and add your WiFi credentials"
#endif

// ============================================
// CONFIGURATION
// ============================================

// WiFi Credentials from secrets.h
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;

// Google Home Integration from secrets.h
const char* googleWebhookURL = "https://maker.ifttt.com/trigger";
const char* googleApiKey = IFTTT_KEY;

// Pin Definitions - Can be overridden in secrets.h
#ifdef CUSTOM_DOOR_PIN
  #define DOOR_SENSOR_PIN CUSTOM_DOOR_PIN
#else
  #define DOOR_SENSOR_PIN 4
#endif

#ifdef CUSTOM_BUZZER_PIN
  #define BUZZER_PIN CUSTOM_BUZZER_PIN
#else
  #define BUZZER_PIN 5
#endif

#ifdef CUSTOM_RESET_PIN
  #define ALARM_RESET_PIN CUSTOM_RESET_PIN
#else
  #define ALARM_RESET_PIN 15
#endif

#ifdef CUSTOM_STATUS_LED
  #define STATUS_LED_PIN CUSTOM_STATUS_LED
#else
  #define STATUS_LED_PIN 2
#endif

// Time Configuration
#ifndef TIMEZONE_OFFSET
  #define GMT_OFFSET_SEC -18000  // Default EST = -5 hours
#else
  #define GMT_OFFSET_SEC TIMEZONE_OFFSET
#endif
#define DAYLIGHT_OFFSET_SEC 0
#define NTP_SERVER "pool.ntp.org"

// Alarm Configuration
#ifndef CUSTOM_START_HOUR
  #define ALARM_START_HOUR 23    // Default 11 PM
#else
  #define ALARM_START_HOUR CUSTOM_START_HOUR
#endif

#ifndef CUSTOM_END_HOUR
  #define ALARM_END_HOUR 5       // Default 5 AM
#else
  #define ALARM_END_HOUR CUSTOM_END_HOUR
#endif

// Alarm Settings
#define BUZZER_FREQUENCY 2000   // Hz
#define BUZZER_DURATION 500     // ms

#ifdef TEST_ALARM_TIMEOUT
  #define ALARM_TIMEOUT_MS TEST_ALARM_TIMEOUT
#else
  #define ALARM_TIMEOUT_MS 300000 // 5 minutes default
#endif

// FreeRTOS Configuration
#define STACK_SIZE_SMALL 2048
#define STACK_SIZE_MEDIUM 4096
#define STACK_SIZE_LARGE 8192

// Task Priorities (higher number = higher priority)
#define PRIORITY_ALARM_CONTROL 4
#define PRIORITY_DOOR_MONITOR 3
#define PRIORITY_GOOGLE_HOME 2
#define PRIORITY_TIME_KEEPER 1
#define PRIORITY_STATUS 1
#define PRIORITY_WATCHDOG 1

// Event Group Bits
#define EVENT_WIFI_CONNECTED    BIT0
#define EVENT_TIME_SYNCED       BIT1
#define EVENT_ALARM_TRIGGERED   BIT2
#define EVENT_ALARM_RESET       BIT3
#define EVENT_GOOGLE_ACTIVATED  BIT4
#define EVENT_SYSTEM_READY      (EVENT_WIFI_CONNECTED | EVENT_TIME_SYNCED)

// ============================================
// DATA STRUCTURES
// ============================================

// System States
enum SystemState {
  STATE_IDLE,
  STATE_ARMED,
  STATE_TRIGGERED,
  STATE_ALARM_ACTIVE,
  STATE_ERROR
};

// Alarm Event Structure
struct AlarmEvent {
  uint32_t timestamp;
  uint8_t eventType;  // 0: door open, 1: door close, 2: manual reset
  uint8_t sensorPin;
  float confidence;
};

// System Statistics
struct SystemStats {
  uint32_t totalAlarms;
  uint32_t falseAlarms;
  uint32_t uptimeSeconds;
  uint32_t lastAlarmTime;
  uint16_t freeHeapSize;
  uint8_t wifiReconnects;
};

// ============================================
// GLOBAL VARIABLES
// ============================================

// FreeRTOS Handles
TaskHandle_t doorMonitorTaskHandle = NULL;
TaskHandle_t timeKeeperTaskHandle = NULL;
TaskHandle_t alarmControlTaskHandle = NULL;
TaskHandle_t googleHomeTaskHandle = NULL;
TaskHandle_t statusTaskHandle = NULL;
TaskHandle_t watchdogTaskHandle = NULL;

// Synchronization Primitives
SemaphoreHandle_t stateMutex;
SemaphoreHandle_t wifiSemaphore;
QueueHandle_t alarmEventQueue;
EventGroupHandle_t systemEventGroup;
TimerHandle_t alarmTimeoutTimer;

// State Variables
volatile SystemState currentState = STATE_IDLE;
volatile bool alarmEnabled = true;
volatile bool googleHomeActivated = false;
SystemStats systemStats = {0};

// ============================================
// TASK IMPLEMENTATIONS
// ============================================

// Task 1: Door Sensor Monitoring (Core 1 - Real-time)
void doorMonitorTask(void *pvParameters) {
  const char* taskName = pcTaskGetName(NULL);
  pinMode(DOOR_SENSOR_PIN, INPUT_PULLUP);
  
  bool lastDoorState = HIGH;
  AlarmEvent event;
  uint32_t notificationValue;
  
  #ifdef DEBUG_MODE
    Serial.printf("[%s] Task started on Core %d\n", taskName, xPortGetCoreID());
  #endif
  
  // Wait for system ready
  xEventGroupWaitBits(systemEventGroup, EVENT_SYSTEM_READY, pdFALSE, pdTRUE, portMAX_DELAY);
  
  while(1) {
    // Check for task notifications
    if(xTaskNotifyWait(0x00, ULONG_MAX, &notificationValue, 0) == pdTRUE) {
      #ifdef DEBUG_MODE
        Serial.printf("[%s] Notification received: 0x%X\n", taskName, notificationValue);
      #endif
    }
    
    bool doorState = digitalRead(DOOR_SENSOR_PIN);
    
    // Door opened detection
    if(doorState == LOW && lastDoorState == HIGH) {
      Serial.println("[DOOR] Door OPENED!");
      
      if(xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        bool shouldTrigger = (currentState == STATE_ARMED) && alarmEnabled && isWithinAlarmHours();
        xSemaphoreGive(stateMutex);
        
        if(shouldTrigger) {
          event.timestamp = millis();
          event.eventType = 0; // Door open
          event.sensorPin = DOOR_SENSOR_PIN;
          event.confidence = 1.0;
          
          if(xQueueSend(alarmEventQueue, &event, pdMS_TO_TICKS(100)) == pdPASS) {
            Serial.println("[DOOR] ⚠️  INTRUSION DETECTED! Alarm event queued");
            xEventGroupSetBits(systemEventGroup, EVENT_ALARM_TRIGGERED);
            xTaskNotify(alarmControlTaskHandle, 0x01, eSetBits);
          }
        } else {
          Serial.println("[DOOR] Door opened - System not armed");
        }
      }
    }
    
    // Door closed detection
    if(doorState == HIGH && lastDoorState == LOW) {
      Serial.println("[DOOR] Door closed");
      event.timestamp = millis();
      event.eventType = 1; // Door close
      event.sensorPin = DOOR_SENSOR_PIN;
      event.confidence = 1.0;
      xQueueSend(alarmEventQueue, &event, 0);
    }
    
    lastDoorState = doorState;
    vTaskDelay(pdMS_TO_TICKS(50));
    systemStats.freeHeapSize = esp_get_free_heap_size();
  }
}

// Task 2: Time Keeper (Core 0 - Low priority)
void timeKeeperTask(void *pvParameters) {
  const char* taskName = pcTaskGetName(NULL);
  struct tm timeinfo;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(60000); // Every minute
  
  #ifdef DEBUG_MODE
    Serial.printf("[%s] Task started on Core %d\n", taskName, xPortGetCoreID());
  #endif
  
  xEventGroupWaitBits(systemEventGroup, EVENT_TIME_SYNCED, pdFALSE, pdTRUE, portMAX_DELAY);
  
  while(1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    
    if(getLocalTime(&timeinfo)) {
      if(xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        SystemState oldState = currentState;
        
        if(isWithinAlarmHours()) {
          if(currentState == STATE_IDLE) {
            currentState = STATE_ARMED;
            Serial.printf("[TIME] System ARMED at %02d:%02d:%02d\n", 
                         timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
          }
        } else {
          if(currentState == STATE_ARMED || currentState == STATE_ALARM_ACTIVE) {
            currentState = STATE_IDLE;
            Serial.printf("[TIME] System DISARMED at %02d:%02d:%02d\n", 
                         timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
            if(oldState == STATE_ALARM_ACTIVE) {
              xEventGroupSetBits(systemEventGroup, EVENT_ALARM_RESET);
            }
          }
        }
        xSemaphoreGive(stateMutex);
      }
      systemStats.uptimeSeconds = millis() / 1000;
    }
  }
}

// Task 3: Alarm Control (Core 1 - Highest priority)
void alarmControlTask(void *pvParameters) {
  const char* taskName = pcTaskGetName(NULL);
  AlarmEvent event;
  uint32_t notificationValue;
  bool buzzerState = false;
  
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(ALARM_RESET_PIN, INPUT_PULLUP);
  
  #ifdef DEBUG_MODE
    Serial.printf("[%s] Task started on Core %d\n", taskName, xPortGetCoreID());
  #endif
  
  // Create software timer for alarm timeout
  alarmTimeoutTimer = xTimerCreate(
    "AlarmTimeout",
    pdMS_TO_TICKS(ALARM_TIMEOUT_MS),
    pdFALSE,
    (void*)0,
    alarmTimeoutCallback
  );
  
  while(1) {
    // Check for notifications or queue events
    if(xTaskNotifyWait(0x00, ULONG_MAX, &notificationValue, pdMS_TO_TICKS(BUZZER_DURATION))) {
      #ifdef DEBUG_MODE
        Serial.printf("[%s] Notification: 0x%X\n", taskName, notificationValue);
      #endif
    }
    
    // Process alarm events
    while(xQueueReceive(alarmEventQueue, &event, 0) == pdPASS) {
      if(event.eventType == 0) { // Door opened
        if(xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
          if(currentState == STATE_ARMED || currentState == STATE_TRIGGERED) {
            currentState = STATE_ALARM_ACTIVE;
            systemStats.totalAlarms++;
            systemStats.lastAlarmTime = millis();
            xTimerStart(alarmTimeoutTimer, 0);
            xTaskNotify(googleHomeTaskHandle, 0x01, eSetBits);
            
            Serial.println("\n🚨🚨🚨 ALARM ACTIVATED 🚨🚨🚨");
            Serial.printf("[ALARM] Total alarms: %lu\n", systemStats.totalAlarms);
          }
          xSemaphoreGive(stateMutex);
        }
      }
    }
    
    // Check manual reset button
    static uint32_t lastButtonPress = 0;
    if(digitalRead(ALARM_RESET_PIN) == LOW && (millis() - lastButtonPress > 1000)) {
      lastButtonPress = millis();
      xEventGroupSetBits(systemEventGroup, EVENT_ALARM_RESET);
      Serial.println("[ALARM] Manual reset pressed");
    }
    
    // Check for reset event
    if(xEventGroupGetBits(systemEventGroup) & EVENT_ALARM_RESET) {
      xEventGroupClearBits(systemEventGroup, EVENT_ALARM_RESET);
      if(xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if(currentState == STATE_ALARM_ACTIVE) {
          currentState = STATE_ARMED;
          xTimerStop(alarmTimeoutTimer, 0);
          Serial.println("[ALARM] Alarm RESET");
        }
        xSemaphoreGive(stateMutex);
      }
    }
    
    // Handle buzzer
    #ifndef SILENT_MODE
    if(xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      bool shouldSound = (currentState == STATE_ALARM_ACTIVE);
      xSemaphoreGive(stateMutex);
      
      if(shouldSound) {
        buzzerState = !buzzerState;
        if(buzzerState) {
          tone(BUZZER_PIN, BUZZER_FREQUENCY);
        } else {
          tone(BUZZER_PIN, BUZZER_FREQUENCY * 1.5);
        }
      } else {
        noTone(BUZZER_PIN);
        buzzerState = false;
      }
    }
    #endif
  }
}

// Task 4: Google Home Integration (Core 0 - Network)
void googleHomeTask(void *pvParameters) {
  const char* taskName = pcTaskGetName(NULL);
  HTTPClient http;
  WiFiClientSecure client;
  client.setInsecure();
  
  uint32_t notificationValue;
  bool lastAlarmState = false;
  
  #ifdef DEBUG_MODE
    Serial.printf("[%s] Task started on Core %d\n", taskName, xPortGetCoreID());
  #endif
  
  xEventGroupWaitBits(systemEventGroup, EVENT_WIFI_CONNECTED, pdFALSE, pdTRUE, portMAX_DELAY);
  
  while(1) {
    // Wait for notification
    if(xTaskNotifyWait(0x00, ULONG_MAX, &notificationValue, pdMS_TO_TICKS(1000))) {
      Serial.println("[GOOGLE] Activation triggered");
    }
    
    bool shouldActivate = false;
    if(xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      shouldActivate = (currentState == STATE_ALARM_ACTIVE);
      xSemaphoreGive(stateMutex);
    }
    
    // State change detection
    if(shouldActivate != lastAlarmState) {
      if(shouldActivate && !googleHomeActivated) {
        Serial.println("[GOOGLE] 🔴🔵 Activating police siren mode!");
        
        if(xSemaphoreTake(wifiSemaphore, pdMS_TO_TICKS(5000)) == pdTRUE) {
          triggerGoogleHome(true);
          googleHomeActivated = true;
          xEventGroupSetBits(systemEventGroup, EVENT_GOOGLE_ACTIVATED);
          xSemaphoreGive(wifiSemaphore);
        }
      } else if(!shouldActivate && googleHomeActivated) {
        Serial.println("[GOOGLE] ✅ Deactivating police siren mode");
        
        if(xSemaphoreTake(wifiSemaphore, pdMS_TO_TICKS(5000)) == pdTRUE) {
          triggerGoogleHome(false);
          googleHomeActivated = false;
          xEventGroupClearBits(systemEventGroup, EVENT_GOOGLE_ACTIVATED);
          xSemaphoreGive(wifiSemaphore);
        }
      }
      lastAlarmState = shouldActivate;
    }
  }
}

// Task 5: Status Monitor (Core 0)
void statusTask(void *pvParameters) {
  const char* taskName = pcTaskGetName(NULL);
  pinMode(STATUS_LED_PIN, OUTPUT);
  
  uint32_t lastPrint = 0;
  uint8_t ledPattern = 0;
  
  #ifdef DEBUG_MODE
    Serial.printf("[%s] Task started on Core %d\n", taskName, xPortGetCoreID());
  #endif
  
  while(1) {
    // Get current state
    SystemState state;
    if(xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      state = currentState;
      xSemaphoreGive(stateMutex);
    } else {
      state = STATE_ERROR;
    }
    
    // LED patterns
    switch(state) {
      case STATE_IDLE:
        digitalWrite(STATUS_LED_PIN, LOW);
        break;
      case STATE_ARMED:
        digitalWrite(STATUS_LED_PIN, (millis() / 1000) % 2);
        break;
      case STATE_TRIGGERED:
        digitalWrite(STATUS_LED_PIN, (millis() / 500) % 2);
        break;
      case STATE_ALARM_ACTIVE:
        digitalWrite(STATUS_LED_PIN, (millis() / 200) % 2);
        break;
      case STATE_ERROR:
        ledPattern = (millis() / 100) % 10;
        digitalWrite(STATUS_LED_PIN, (ledPattern == 0 || ledPattern == 2));
        break;
    }
    
    // Print status every 10 seconds
    if(millis() - lastPrint > 10000) {
      struct tm timeinfo;
      if(getLocalTime(&timeinfo)) {
        Serial.println("\n╔════════════════ SYSTEM STATUS ════════════════╗");
        Serial.printf("║ Time: %02d:%02d:%02d | Date: %02d/%02d/%04d        ║\n", 
                     timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec,
                     timeinfo.tm_mday, timeinfo.tm_mon + 1, timeinfo.tm_year + 1900);
        
        const char* stateStr[] = {"IDLE", "ARMED", "TRIGGERED", "ACTIVE", "ERROR"};
        Serial.printf("║ State: %-8s | Hours: %02d:00-%02d:00      ║\n", 
                     stateStr[state], ALARM_START_HOUR, ALARM_END_HOUR);
        Serial.printf("║ Uptime: %lu sec | Alarms: %lu              ║\n", 
                     systemStats.uptimeSeconds, systemStats.totalAlarms);
        Serial.printf("║ Free Heap: %d bytes | WiFi: %-11s║\n", 
                     systemStats.freeHeapSize,
                     WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected");
        Serial.println("╚════════════════════════════════════════════════╝");
      }
      lastPrint = millis();
    }
    
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// Task 6: Watchdog Monitor (Core 1)
void watchdogTask(void *pvParameters) {
  const char* taskName = pcTaskGetName(NULL);
  
  #ifdef DEBUG_MODE
    Serial.printf("[%s] Watchdog started on Core %d\n", taskName, xPortGetCoreID());
  #endif
  
  while(1) {
    // Monitor critical tasks
    TaskHandle_t criticalTasks[] = {
      doorMonitorTaskHandle,
      alarmControlTaskHandle,
      googleHomeTaskHandle
    };
    
    for(int i = 0; i < 3; i++) {
      if(criticalTasks[i] != NULL) {
        if(eTaskGetState(criticalTasks[i]) == eSuspended) {
          Serial.printf("[WATCHDOG] WARNING: Critical task %d suspended!\n", i);
        }
      }
    }
    
    // Monitor heap
    if(systemStats.freeHeapSize < 10000) {
      Serial.printf("[WATCHDOG] WARNING: Low heap: %d bytes\n", 
                   systemStats.freeHeapSize);
    }
    
    // Check WiFi connection
    if(WiFi.status() != WL_CONNECTED) {
      Serial.println("[WATCHDOG] WiFi disconnected - reconnecting...");
      WiFi.reconnect();
      systemStats.wifiReconnects++;
    }
    
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

// ============================================
// HELPER FUNCTIONS
// ============================================

void setupWiFi() {
  Serial.print("[WIFI] Connecting to ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while(WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if(WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[WIFI] Connected!");
    Serial.print("[WIFI] IP: ");
    Serial.println(WiFi.localIP());
    xEventGroupSetBits(systemEventGroup, EVENT_WIFI_CONNECTED);
  } else {
    Serial.println("\n[WIFI] Failed to connect!");
  }
}

void setupTime() {
  configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);
  
  struct tm timeinfo;
  int retries = 0;
  while(!getLocalTime(&timeinfo) && retries < 10) {
    Serial.println("[TIME] Syncing...");
    delay(1000);
    retries++;
  }
  
  if(retries < 10) {
    Serial.printf("[TIME] Synchronized: %02d:%02d:%02d\n",
                  timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    xEventGroupSetBits(systemEventGroup, EVENT_TIME_SYNCED);
  } else {
    Serial.println("[TIME] Sync failed!");
  }
}

bool isWithinAlarmHours() {
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)) {
    return false;
  }
  
  int currentHour = timeinfo.tm_hour;
  
  if(ALARM_START_HOUR > ALARM_END_HOUR) {
    return (currentHour >= ALARM_START_HOUR || currentHour < ALARM_END_HOUR);
  } else {
    return (currentHour >= ALARM_START_HOUR && currentHour < ALARM_END_HOUR);
  }
}

void triggerGoogleHome(bool activate) {
  if(WiFi.status() != WL_CONNECTED) {
    Serial.println("[GOOGLE] No WiFi connection");
    return;
  }
  
  HTTPClient http;
  WiFiClientSecure client;
  client.setInsecure();
  
  // Build IFTTT webhook URL
  String url = String(googleWebhookURL);
  if(activate) {
    url += "/" + String(IFTTT_EVENT_ON) + "/json/with/key/";
  } else {
    url += "/" + String(IFTTT_EVENT_OFF) + "/json/with/key/";
  }
  url += googleApiKey;
  
  http.begin(client, url);
  http.addHeader("Content-Type", "application/json");
  
  String payload = activate ? 
    "{\"value1\":\"INTRUDER_ALERT\",\"value2\":\"ALL_LIGHTS\",\"value3\":\"POLICE_SIREN\"}" :
    "{\"value1\":\"ALL_CLEAR\",\"value2\":\"NORMAL\",\"value3\":\"OFF\"}";
  
  int httpResponseCode = http.POST(payload);
  
  if(httpResponseCode > 0) {
    Serial.printf("[GOOGLE] Response: %d\n", httpResponseCode);
  } else {
    Serial.printf("[GOOGLE] Error: %s\n", http.errorToString(httpResponseCode).c_str());
  }
  
  http.end();
}

void alarmTimeoutCallback(TimerHandle_t xTimer) {
  Serial.println("[TIMER] Alarm timeout");
  xEventGroupSetBits(systemEventGroup, EVENT_ALARM_RESET);
  systemStats.falseAlarms++;
}

void printTaskInfo() {
  Serial.println("FreeRTOS Tasks:");
  
  TaskHandle_t tasks[] = {
    doorMonitorTaskHandle, timeKeeperTaskHandle, 
    alarmControlTaskHandle, googleHomeTaskHandle, 
    statusTaskHandle, watchdogTaskHandle
  };
  
  const char* taskNames[] = {
    "DoorMonitor", "TimeKeeper", "AlarmControl", 
    "GoogleHome", "Status", "Watchdog"
  };
  
  for(int i = 0; i < 6; i++) {
    if(tasks[i] != NULL) {
      UBaseType_t stackWaterMark = uxTaskGetStackHighWaterMark(tasks[i]);
      UBaseType_t priority = uxTaskPriorityGet(tasks[i]);
      Serial.printf("  %s: Priority=%d, Stack=%d words free\n", 
                   taskNames[i], priority, stackWaterMark);
    }
  }
  
  Serial.printf("  Total Tasks: %d\n", uxTaskGetNumberOfTasks());
}

// ============================================
// MAIN SETUP
// ============================================

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n╔══════════════════════════════════════════════╗");
  Serial.println("║     ESP32 FreeRTOS HOME SECURITY SYSTEM     ║");
  Serial.println("║            Powered by FreeRTOS v10          ║");
  Serial.println("╚══════════════════════════════════════════════╝\n");
  
  // Initialize FreeRTOS objects
  Serial.println("[RTOS] Initializing FreeRTOS objects...");
  
  stateMutex = xSemaphoreCreateMutex();
  if(stateMutex == NULL) {
    Serial.println("[ERROR] Failed to create mutex!");
    esp_restart();
  }
  
  wifiSemaphore = xSemaphoreCreateBinary();
  xSemaphoreGive(wifiSemaphore);
  
  alarmEventQueue = xQueueCreate(20, sizeof(AlarmEvent));
  if(alarmEventQueue == NULL) {
    Serial.println("[ERROR] Failed to create queue!");
    esp_restart();
  }
  
  systemEventGroup = xEventGroupCreate();
  if(systemEventGroup == NULL) {
    Serial.println("[ERROR] Failed to create event group!");
    esp_restart();
  }
  
  Serial.println("[RTOS] Objects created successfully");
  
  // Setup WiFi and Time
  setupWiFi();
  setupTime();
  
  // Create FreeRTOS Tasks
  Serial.println("\n[RTOS] Creating tasks...");
  
  // Core 0: Network Tasks
  xTaskCreatePinnedToCore(googleHomeTask, "GoogleHome", STACK_SIZE_LARGE, 
                         NULL, PRIORITY_GOOGLE_HOME, &googleHomeTaskHandle, 0);
  xTaskCreatePinnedToCore(timeKeeperTask, "TimeKeeper", STACK_SIZE_SMALL,
                         NULL, PRIORITY_TIME_KEEPER, &timeKeeperTaskHandle, 0);
  xTaskCreatePinnedToCore(statusTask, "Status", STACK_SIZE_MEDIUM,
                         NULL, PRIORITY_STATUS, &statusTaskHandle, 0);
  
  // Core 1: Real-time Tasks
  xTaskCreatePinnedToCore(doorMonitorTask, "DoorMonitor", STACK_SIZE_SMALL,
                         NULL, PRIORITY_DOOR_MONITOR, &doorMonitorTaskHandle, 1);
  xTaskCreatePinnedToCore(alarmControlTask, "AlarmControl", STACK_SIZE_MEDIUM,
                         NULL, PRIORITY_ALARM_CONTROL, &alarmControlTaskHandle, 1);
  xTaskCreatePinnedToCore(watchdogTask, "Watchdog", STACK_SIZE_SMALL,
                         NULL, PRIORITY_WATCHDOG, &watchdogTaskHandle, 1);
  
  Serial.println("[RTOS] All tasks created!");
  Serial.println("\n[SYSTEM] Task Distribution:");
  Serial.println("  Core 0: GoogleHome, TimeKeeper, Status");
  Serial.println("  Core 1: DoorMonitor, AlarmControl, Watchdog");
  
  printTaskInfo();
  
  // Set initial state
  if(isWithinAlarmHours()) {
    xSemaphoreTake
