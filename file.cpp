#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"
#include <DNSServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

MAX30105 particleSensor;
ESP8266WebServer server(80);
DNSServer dnsServer;

// Wi-Fi AP settings
const char* ssid = "HealthMonitor";
const char* password = "12345678";
const byte DNS_PORT = 53;

// Sensor data
volatile int pulse = 0;
volatile int spo2 = 0;
bool beatDetected = false;
uint32_t redBuffer[100], irBuffer[100];
uint32_t irValue = 0;
unsigned long lastBeat = 0;
bool fingerPresent = false;

// Time & Alarm
volatile unsigned long timeBase = 0;
volatile bool alarmTriggered = false;
volatile bool blinkState = true;
volatile unsigned long lastBlink = 0;

// Loops timing
unsigned long lastSensorRead = 0;
unsigned long lastSpO2Check = 0;
unsigned long lastWifiCheck = 0;
const unsigned long sensorInterval = 20;
const unsigned long spo2Interval = 5000;
const unsigned long wifiCheckInterval = 10000;

// Alarm variables
int alarmHour = -1;
int alarmMinute = -1;

// WiFi status
bool wifiInitialized = false;

// User data structures
struct PulseRecord {
  unsigned long timestamp;
  int pulseValue;
  int spo2Value;
};

struct User {
  String username;
  String password;
  int bedtimeHour;
  int bedtimeMinute;
  int wakeupHour;
  int wakeupMinute;
  PulseRecord records[20];
  int recordCount;
  bool isAdmin;
};

#define MAX_USERS 10
User users[MAX_USERS];
int userCount = 0;
int currentUserIndex = -1;

// Health norms
#define MIN_NORMAL_PULSE 60
#define MAX_NORMAL_PULSE 100
#define MIN_NORMAL_SPO2 95
#define CRITICAL_SPO2 90

// Motivational messages
const char* motivationalMessages[] = {
  "Take care of your health!",
  "Water is life",
  "Move more!",
  "Deep breathing reduces stress",
  "Smile more often!",
  "Good sleep is key to health",
  "Proper nutrition matters",
  "15 minutes of sport daily",
  "Watch your posture!",
  "Health is the main wealth"
};
#define MESSAGE_COUNT 10

unsigned long lastMessageTime = 0;
const unsigned long messageInterval = 3600000;
int currentMessageIndex = 0;

// SpO2 variables
uint8_t spo2BufferIndex = 0;
bool collectingData = false;
unsigned long lastSampleTime = 0;
const unsigned long sampleInterval = 10;

// Display update
unsigned long lastDisplayUpdate = 0;
const unsigned long displayUpdateInterval = 100;

// Sensor reading flag
bool activeSensorReading = false;

#define FINGER_THRESHOLD 5000 // порог обнаружения пальца
#define SPO2_BUFFER_SIZE 100 // размер буфера для расчета SpO2

// Добавьте переменные времени
int seconds = 0;
int minutes = 0;
int hours = 0;

// Изменённая функция проверки наличия пальца
void checkFingerPresence() {
  irValue = particleSensor.getIR();
  
  // Просто обновляем статус наличия пальца, но не останавливаем чтение сенсора
  if (irValue < FINGER_THRESHOLD) {
    fingerPresent = false;
    beatDetected = false;
  } else {
    fingerPresent = true;
    // Активируем сенсор, если он ещё не активен
    if (!activeSensorReading) {
      beginSensorReading();
    }
  }
}

// Обновлённый метод начала чтения сенсора
void beginSensorReading() {
  activeSensorReading = true;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // OLED init
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED init failed");
    while (1);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("Initializing...");
  display.display();

  // MAX30105 init
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Sensor error!");
    display.display();
    while (1);
  }
  
  particleSensor.setup(50, 4, 2, 100, 411, 4096);
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeIR(0x0A);

  // Filesystem init
  if (!LittleFS.begin()) {
    Serial.println("LittleFS mount failed");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("FS Error!");
    display.display();
    delay(2000);
  } else {
    loadUsers();
    createAdminIfNeeded();
  }

  setupWiFi();

  // Server routes
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/setTime", HTTP_GET, handleSetTime);
  server.on("/setAlarm", HTTP_GET, handleSetAlarm);
  server.on("/clearAlarm", HTTP_GET, handleClearAlarm);
  server.on("/login", HTTP_POST, handleLogin);
  server.on("/register", HTTP_POST, handleRegister);
  server.on("/logout", HTTP_GET, handleLogout);
  server.on("/setSleep", HTTP_POST, handleSetSleep);
  server.on("/admin", HTTP_GET, handleAdmin);
  server.on("/deleteUser", HTTP_GET, handleDeleteUser);
  
  // Default handler для любых других запросов - редирект на главную
  server.onNotFound([]() {
    server.sendHeader("Location", "/");
    server.send(302, "text/plain", "Переход на главную страницу...");
  });
  
  server.begin();

  display.clearDisplay();
  display.setCursor(0,0);
  display.println("System ready");
  display.println("IP: " + WiFi.softAPIP().toString());
  display.println("Open in browser!");
  display.display();
}

void setupWiFi() {
  Serial.println("Configuring Wi-Fi AP...");
  WiFi.disconnect();
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(IPAddress(192,168,4,1), IPAddress(192,168,4,1), IPAddress(255,255,255,0));
  
  if (WiFi.softAP(ssid, password)) {
    Serial.println("AP setup successful");
    wifiInitialized = true;
    
    dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
    dnsServer.start(DNS_PORT, "*", IPAddress(192,168,4,1));
    
    IPAddress ip = WiFi.softAPIP();
    Serial.print("AP IP address: "); 
    Serial.println(ip);
    
    Serial.println("======================");
    Serial.println("ВАЖНО: Для входа в административную панель:");
    Serial.println("1. Подключитесь к WiFi сети: " + String(ssid));
    Serial.println("2. Откройте в браузере http://" + ip.toString());
    Serial.println("3. Логин: admin, пароль: admin");
    Serial.println("======================");

    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("WiFi AP:");
    display.println(ssid);
    display.print("IP: ");
    display.println(ip);
    display.println("Login: admin");
    display.println("Pass: admin");
    display.display();
    delay(4000);
  } else {
    Serial.println("AP setup failed");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("WiFi AP failed!");
    display.display();
    delay(2000);
  }
}

void checkWiFi() {
  if (!wifiInitialized || WiFi.softAPgetStationNum() == 0) {
    if (WiFi.status() != WL_CONNECTED && !WiFi.softAPSSID().equals(ssid)) {
      Serial.println("WiFi AP disconnected. Reconnecting...");
      setupWiFi();
    }
  }
}

void loop() {
  // Добавляем yield() в начале цикла для улучшения отзывчивости
  yield();
  
  unsigned long now = millis();
  
  // Наивысший приоритет - обновление времени
  static unsigned long lastSecondCheck = 0;
  if (now - lastSecondCheck >= 1000) {
    lastSecondCheck = now;
    seconds++;
    if (seconds >= 60) {
      seconds = 0;
      minutes++;
      if (minutes >= 60) {
        minutes = 0;
        hours++;
        if (hours >= 24) {
          hours = 0;
        }
      }
    }
    
    // Сразу после обновления времени - проверка будильника
    // это позволяет своевременно реагировать на наступление времени будильника
    checkAlarmState();
    
    // Обеспечиваем минимальный интервал между проверками дисплея
    static unsigned long lastDisplayRefresh = 0;
    if (now - lastDisplayRefresh >= 200) {  // Обновляем дисплей не чаще 5 раз в секунду
      updateDisplay();
      lastDisplayRefresh = now;
    }
  }
  
  // Даём возможность системе обработать другие задачи после интенсивных операций
  yield();
  
  // Следующий приоритет - обработка DNS и клиентских запросов
  dnsServer.processNextRequest();
  server.handleClient();
  
  // Обязательно даем системе передохнуть после сетевых операций
  yield();
  
  // Проверяем наличие пальца, но не отключаем сенсор
  checkFingerPresence();
  
  // Еще один yield перед операциями с датчиком
  yield();
  
  // Чтение датчика с ограничением частоты для улучшения стабильности
  static unsigned long lastSensorReadTime = 0;
  if (now - lastSensorReadTime >= 30) {  // Не чаще ~33 раз в секунду
    lastSensorReadTime = now;
    readSensorData();
    
    // Рассчитываем SpO2 только если палец на датчике
    if (fingerPresent) {
      calculateSpO2();
      
      // Сохраняем измерения при наличии данных
      if (beatDetected && currentUserIndex >= 0 && pulse > 0 && spo2 > 0) {
        // Ограничиваем частоту сохранения данных
        static unsigned long lastRecordTime = 0;
        if (now - lastRecordTime >= 5000) { // Сохраняем не чаще раза в 5 секунд
          addPulseRecord(pulse, spo2);
          lastRecordTime = now;
        }
      }
    } else {
      // Если пальца нет, не сбрасываем показания полностью,
      // а постепенно уменьшаем их, чтобы избежать резких перепадов на дисплее
      static unsigned long lastValueDecayTime = 0;
      if (now - lastValueDecayTime >= 2000) { // Обновляем значения каждые 2 секунды
        lastValueDecayTime = now;
        if (pulse > 0) pulse--;
        if (spo2 > 0) spo2--;
        beatDetected = false;
      }
    }
  }
  
  // Даем системе выполнить другие задачи после интенсивных вычислений
  yield();
  
  // Периодические проверки с более низким приоритетом
  static unsigned long lastWifiCheck = 0;
  if (now - lastWifiCheck >= 10000) { // Проверка WiFi раз в 10 секунд
    checkWiFi();
    lastWifiCheck = now;
  }
  
  // Проверяем состояния уведомлений
  static unsigned long lastNotificationCheck = 0;
  if (now - lastNotificationCheck >= 3000) { // Проверка раз в 3 секунды
    lastNotificationCheck = now;
    
    // Проверки всех типов уведомлений
    checkSleepNotifications();
    
    // Мотивационные сообщения показываем с большим интервалом
    static unsigned long lastMotivationalCheck = 0;
    if (now - lastMotivationalCheck >= 60000) { // Проверка раз в минуту
      showMotivationalMessage();
      lastMotivationalCheck = now;
    }
  }
  
  // Финальный yield в конце цикла
  yield();
}

void readSensorData() {
  // Добавляем yield() в начале функции
  yield();
  
  if (checkForBeat(irValue)) {
    unsigned long delta = millis() - lastBeat;
    lastBeat = millis();
    if (delta > 300 && delta < 2000) {
      pulse = 60000 / delta;
      beatDetected = true;
      Serial.print("BPM: "); Serial.println(pulse);
    }
  }
  
  // Добавляем дополнительный yield() в конце
  yield();
}

void calculateSpO2() {
  // Добавляем yield() в начале функции для улучшения отзывчивости
  yield();
  
  // Если палец не на датчике, сбрасываем буфер
  if (irValue < FINGER_THRESHOLD) {
    collectingData = false;
    spo2BufferIndex = 0;
    spo2 = 0;
    return;
  }
  
  // Неблокирующий сбор данных
  unsigned long currentMillis = millis();
  if (currentMillis - lastSampleTime >= sampleInterval) {
    lastSampleTime = currentMillis;
    
    // Добавляем yield() для предотвращения блокировки во время сбора данных
    yield();
    
    if (spo2BufferIndex < SPO2_BUFFER_SIZE) {
      // Добавляем данные в буфер
      redBuffer[spo2BufferIndex] = particleSensor.getRed();
      irBuffer[spo2BufferIndex] = irValue;
      spo2BufferIndex++;
      
      // Добавляем yield() после каждой 10-й записи в буфер
      if (spo2BufferIndex % 10 == 0) {
        yield();
      }
    } else {
      // Буфер заполнен, выполняем расчет
      // Выполняем немедленный yield() перед интенсивным вычислением
      yield();
      
      int32_t spo2Value;
      int8_t validSPO2;
      int32_t heartRateValue;
      int8_t validHeartRate;
      
      maxim_heart_rate_and_oxygen_saturation(
        irBuffer, SPO2_BUFFER_SIZE,
        redBuffer,
        &spo2Value, &validSPO2,
        &heartRateValue, &validHeartRate
      );
      
      yield();
      
      // Обновляем значение SpO2
      if (validSPO2 == 1 && spo2Value > 0 && spo2Value <= 100) {
        spo2 = spo2Value;
        Serial.print("SpO2: ");
        Serial.print(spo2);
        Serial.println("%");
      }
      
      // Сбрасываем буфер для следующего измерения
      spo2BufferIndex = 0;
    }
  }
  
  // Финальный yield() в конце функции
  yield();
}

void checkAlarmState() {
  // Добавляем yield для предотвращения зависания
  yield();
  
  // Если будильник уже сработал, обрабатываем мигание
  if (alarmTriggered) {
    if (millis() - lastBlink > 500) {
      blinkState = !blinkState;
      lastBlink = millis();
    }
    
    // Добавляем звуковой сигнал, если есть пищалка
    // (закомментировано, так как не указано в схеме)
    // if (blinkState) {
    //   digitalWrite(BUZZER_PIN, HIGH);
    // } else {
    //   digitalWrite(BUZZER_PIN, LOW);
    // }
    
    return;
  }
  
  // Иначе проверяем, не пора ли включить будильник
  if (alarmHour >= 0) {
    // Получаем текущее время
    if (hours == alarmHour && minutes == alarmMinute && seconds < 2) {
      alarmTriggered = true;
      lastBlink = millis();
      
      // Выводим сообщение о срабатывании будильника
      Serial.println("ALARM TRIGGERED!");
      
      // Очищаем экран и показываем уведомление о будильнике
      display.clearDisplay();
      display.setTextSize(2);
      display.setCursor(0, 0);
      display.println("ALARM!");
      display.setTextSize(1);
      display.setCursor(0, 20);
      display.println("Time to wake up!");
      display.display();
    }
  }
  
  yield();
}

const char* menuTitles[] = {
  "Health Monitoring",
  "Alarm",
  "Pulse History",
  "Sleep Settings",
  "Reset Alarm"
};

void updateDisplay() {
  unsigned long now = millis();
  if (now - lastDisplayUpdate < displayUpdateInterval) {
    return;
  }
  lastDisplayUpdate = now;
  
  // Предотвращаем зависание
  yield();
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);

  // Если сработал будильник, показываем только его
  if (alarmTriggered) {
    display.setTextSize(2);
    if (blinkState) {
      display.println("ALARM!");
      display.setTextSize(1);
      display.setCursor(0, 20);
      display.println("Press Reset");
      display.println("to dismiss");
    }
    display.display();
    return;
  }

  // Заголовок системы
  display.setTextSize(1);
  display.println(menuTitles[0]); 
  display.drawLine(0, 9, display.width(), 9, WHITE);
  display.setCursor(0, 12);

  // Всегда показываем время и статус будильника
  display.printf("Time: %02d:%02d:%02d", hours, minutes, seconds);
  
  // Показываем статус будильника
  if (alarmHour >= 0) {
    display.setCursor(95, 12);
    display.print("[A]");
  }
  
  display.setCursor(0, 22);
  
  // Показываем статус пальца и значения, если они доступны
  if (!fingerPresent) {
    display.println("Place finger");
  } else {
    display.printf("Pulse: %d bpm\n", beatDetected ? pulse : 0);
    display.printf("SpO2: %d%%\n", spo2);
  }
  
  if (currentUserIndex >= 0) {
    display.print("User: ");
    display.println(users[currentUserIndex].username);
    
    // Показываем иконку будильника, если он установлен
    if (alarmHour >= 0) {
      display.setCursor(0, 55);
      display.printf("Alarm: %02d:%02d", alarmHour, alarmMinute);
    }
  } else {
    display.println("Not logged in");
  }

  display.display();
  yield();
}

void handleRoot() {
  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Expires", "-1");
  
  String html = F(R"=====(
<!DOCTYPE html><html><head>
<meta charset='UTF-8'>
<meta name='viewport' content='width=device-width,initial-scale=1'>
<title>Умный монитор здоровья</title>
<style>
:root {
  --primary: #ff9aa2;
  --primary-light: #ffb7b2;
  --bg: #fff5f5;
  --text: #5e5e5e;
  --text-light: #888;
  --card-bg: #fff;
  --accent: #ff6b6b;
  --success: #7ac142;
}
body{font-family:'Arial Rounded MT Bold',Arial,sans-serif;margin:0;padding:0;background:var(--bg);color:var(--text)}
header{background:linear-gradient(to right,var(--primary),var(--primary-light));color:#fff;padding:15px;border-radius:0 0 15px 15px;box-shadow:0 4px 10px rgba(255,170,170,0.3)}
h1{margin:0;font-size:22px;text-align:center;text-shadow:1px 1px 2px rgba(150,150,150,0.3)}
.container{max-width:800px;margin:0 auto;padding:15px}
.card{background:var(--card-bg);border-radius:15px;box-shadow:0 4px 15px rgba(0,0,0,0.05);padding:15px;margin-bottom:15px;transition:all 0.3s ease}
.card:hover{transform:translateY(-3px);box-shadow:0 7px 20px rgba(0,0,0,0.1)}
.tabs{display:flex;margin-bottom:15px;border-radius:12px;overflow:hidden;box-shadow:0 3px 10px rgba(0,0,0,0.1)}
.tab{flex:1;text-align:center;padding:10px;cursor:pointer;background:#ffeaea;color:#ff9aa2;font-weight:bold;transition:all 0.3s}
.tab:hover{background:#ffe0e0}
.tab.active{background:#ff9aa2;color:white}
.tab-content{display:none;padding:15px 5px}
.tab-content.active{display:block}
.metric{text-align:center;padding:15px;border:1px solid #ffe0e0;border-radius:15px;margin:8px;flex:1;min-width:100px;transition:all 0.3s}
.metric:hover{background:#fff8f8;transform:scale(1.03)}
.metric h3{color:#ff9aa2;margin-top:0}
.value{font-size:32px;font-weight:bold;margin:10px 0}
.form-group{margin-bottom:15px}
label{display:block;margin-bottom:5px;color:#ff9aa2;font-weight:bold}
input{width:100%;padding:10px;border:2px solid #ffe0e0;border-radius:12px;box-sizing:border-box;transition:all 0.3s}
input:focus{border-color:#ff9aa2;outline:none}
input[type="checkbox"]{width:auto}
button{background:#ff9aa2;color:white;border:none;padding:10px 15px;border-radius:12px;cursor:pointer;font-weight:bold;transition:all 0.3s;box-shadow:0 3px 8px rgba(255,154,162,0.3)}
button:hover{background:#ff8a94;transform:translateY(-2px);box-shadow:0 5px 12px rgba(255,154,162,0.4)}
button:active{transform:translateY(0)}
.warning{background:#fff0f0;color:#ff6b6b;padding:10px;border-radius:12px;margin-bottom:15px;border-left:4px solid #ff9aa2}
.health-metrics{display:flex;flex-wrap:wrap;justify-content:space-between}
.normal{color:#7ac142}
.warning-value{color:#ff6b6b}
#loginStatus{text-align:center;font-weight:bold;margin-top:5px}
.time-inputs{display:flex;gap:10px;align-items:center}
.time-inputs input{width:70px}
.time-inputs span{font-size:18px;color:var(--primary)}
.toggle-form{text-align:center;margin-top:10px;color:var(--primary);cursor:pointer;text-decoration:underline}
.toggle-form:hover{color:var(--accent)}
.admin-link {
  position: fixed;
  bottom: 15px;
  right: 15px;
  background: var(--primary);
  color: white;
  padding: 10px 15px;
  border-radius: 50px;
  text-decoration: none;
  display: flex;
  align-items: center;
  box-shadow: 0 4px 10px rgba(255,154,162,0.4);
  transition: all 0.3s;
  font-weight: bold;
  z-index: 100;
}
.admin-link:hover {
  background: var(--accent);
  transform: translateY(-3px);
  box-shadow: 0 6px 15px rgba(255,154,162,0.6);
}
</style>
</head>
<body>
    <header>
        <div class="container">
            <h1>❤️ Умный монитор здоровья ❤️</h1>
            <div id="loginStatus">Не авторизован</div>
        </div>
    </header>
    
    <div class="container">
        <div class="tabs">
            <div class="tab active" onclick="switchTab('dashboard')">Главная</div>
            <div class="tab" onclick="switchTab('settings')">Настройки</div>
            <div class="tab" onclick="switchTab('profile')">Профиль</div>
            <div class="tab" id="adminTab" style="display:none" onclick="window.location.href='/admin'">Админ</div>
        </div>
        
        <div id="dashboard" class="tab-content active">
            <div id="sensorWarning" class="warning" style="display:none">
                📌 Приложите палец к датчику для измерений
            </div>
            
            <div class="card">
                <h2 style="text-align:center;color:#ff9aa2">Текущее время: <span id="currentTime">--:--:--</span></h2>
            </div>
            
            <div class="card">
                <h2 style="text-align:center;color:#ff9aa2">Показатели здоровья</h2>
                <div class="health-metrics">
                    <div class="metric">
                        <h3>Пульс</h3>
                        <div id="pulseValue" class="value">--</div>
                        <div>уд/мин</div>
                    </div>
                    <div class="metric">
                        <h3>Кислород</h3>
                        <div id="spo2Value" class="value">--</div>
                        <div>%</div>
                    </div>
                    <div class="metric">
                        <h3>Будильник</h3>
                        <div id="alarmStatus" class="value">--</div>
                        <div id="alarmTime">--:--</div>
                    </div>
                </div>
            </div>

            <!-- Добавляем карточку для отключения сработавшего будильника -->
            <div id="alarmAlertCard" class="card" style="display:none; background-color:#ffebeb; border:2px solid #ff6b6b;">
                <h2 style="text-align:center;color:#ff3333">⏰ БУДИЛЬНИК! ⏰</h2>
                <p style="text-align:center;font-size:18px;">Время вставать! Будильник сработал!</p>
                <div style="text-align:center;margin-top:10px;">
                    <button onclick="clearAlarm()" style="background:#ff3333; font-size:18px; padding:15px 30px;">
                        Отключить будильник
                    </button>
                </div>
            </div>
        </div>
        
        <div id="settings" class="tab-content">
            <div class="card">
                <h2 style="text-align:center;color:#ff9aa2">Установка времени</h2>
                <div class="form-group">
                    <label for="timeHours">Часы:</label>
                    <input type="number" id="timeHours" min="0" max="23" placeholder="0-23">
                </div>
                <div class="form-group">
                    <label for="timeMinutes">Минуты:</label>
                    <input type="number" id="timeMinutes" min="0" max="59" placeholder="0-59">
                </div>
                <button onclick="setTime()">Установить время</button>
            </div>
            
            <div class="card">
                <h2 style="text-align:center;color:#ff9aa2">Настройка будильника</h2>
                <div class="form-group">
                    <label for="alarmEnabled">Включить будильник:</label>
                    <input type="checkbox" id="alarmEnabled">
                </div>
                <div class="form-group">
                    <label for="alarmHours">Часы:</label>
                    <input type="number" id="alarmHours" min="0" max="23" placeholder="0-23">
                </div>
                <div class="form-group">
                    <label for="alarmMinutes">Минуты:</label>
                    <input type="number" id="alarmMinutes" min="0" max="59" placeholder="0-59">
                </div>
                <button onclick="setAlarm()">Установить</button>
                <button onclick="clearAlarm()" style="background:#ff6b6b">Отключить</button>
            </div>
            
            <div class="card" id="sleepSettingsCard" style="display:none">
                <h2 style="text-align:center;color:#ff9aa2">Режим сна</h2>
                <div class="form-group">
                    <label>Время отхода ко сну:</label>
                    <div class="time-inputs">
                        <input type="number" id="bedHour" min="0" max="23" placeholder="Часы">
                        <span>:</span>
                        <input type="number" id="bedMinute" min="0" max="59" placeholder="Минуты">
                    </div>
                </div>
                <div class="form-group">
                    <label>Время пробуждения:</label>
                    <div class="time-inputs">
                        <input type="number" id="wakeHour" min="0" max="23" placeholder="Часы">
                        <span>:</span>
                        <input type="number" id="wakeMinute" min="0" max="59" placeholder="Минуты">
                    </div>
                </div>
                <button onclick="setSleepTime()">Сохранить</button>
            </div>
        </div>
        
        <div id="profile" class="tab-content">
            <div id="loginForm" class="card">
                <h2 style="text-align:center;color:#ff9aa2">Авторизация</h2>
                <div class="form-group">
                    <label for="username">Имя пользователя:</label>
                    <input type="text" id="username" placeholder="Введите логин">
                </div>
                <div class="form-group">
                    <label for="password">Пароль:</label>
                    <input type="password" id="password" placeholder="Введите пароль">
                </div>
                <button onclick="login()">Войти</button>
                <div class="toggle-form" onclick="toggleRegisterForm()">Нет аккаунта? Зарегистрироваться</div>
                <div style="text-align:center;margin-top:20px;color:#888;font-size:12px">
                    Администратор: admin / admin
                </div>
            </div>

            <div id="registerForm" class="card" style="display:none">
                <h2 style="text-align:center;color:#ff9aa2">Регистрация</h2>
                <div class="form-group">
                    <label for="newUsername">Имя пользователя:</label>
                    <input type="text" id="newUsername" placeholder="Придумайте логин">
                </div>
                <div class="form-group">
                    <label for="newPassword">Пароль:</label>
                    <input type="password" id="newPassword" placeholder="Придумайте пароль">
                </div>
                <button onclick="register()">Зарегистрироваться</button>
                <div class="toggle-form" onclick="toggleRegisterForm()">Уже есть аккаунт? Войти</div>
            </div>
            
            <div id="userProfile" style="display:none" class="card">
                <h2 style="text-align:center;color:#ff9aa2">Профиль пользователя</h2>
                <p style="text-align:center;font-size:18px;">Вы вошли как: <span id="profileUsername">--</span></p>
                <div id="adminNotice" style="display:none; margin:15px 0; padding:10px; background:#fff8f8; border-left:4px solid #ff9aa2; border-radius:5px;">
                    <p><strong>Вы администратор!</strong> У вас есть доступ к:</p>
                    <ul style="margin-left:20px;">
                        <li>Панели администратора (вкладка "Админ")</li>
                        <li>Управлению пользователями</li>
                        <li>Удалению пользователей</li>
                    </ul>
                </div>
                <button onclick="logout()">Выйти</button>
            </div>
        </div>
    </div>
    
    <!-- Кнопка быстрого доступа к админке -->
    <a href="/admin" class="admin-link" id="quickAdminLink" style="display:none">
        ⚙️ Панель администратора
    </a>

    <script>
        // Переключение вкладок
        function switchTab(tabId) {
            document.querySelectorAll('.tab-content').forEach(tab => tab.classList.remove('active'));
            document.querySelectorAll('.tab').forEach(btn => btn.classList.remove('active'));
            document.getElementById(tabId).classList.add('active');
            document.querySelector(`.tab[onclick="switchTab('${tabId}')"]`).classList.add('active');
        }
        
        // Обновление данных с сервера
        function updateData() {
            fetch('/data')
                .then(response => response.json())
                .then(data => {
                    // Обновляем время
                    document.getElementById('currentTime').textContent = data.time;
                    
                    // Обновляем показатели здоровья
                    document.getElementById('pulseValue').textContent = data.pulse;
                    document.getElementById('spo2Value').textContent = data.spo2;
                    
                    // Показываем предупреждение о датчике
                    if (data.finger_present === "0") {
                        document.getElementById('sensorWarning').style.display = 'block';
                    } else {
                        document.getElementById('sensorWarning').style.display = 'none';
                    }
                    
                    // Обновляем статус будильника
                    const alarmAlertCard = document.getElementById('alarmAlertCard');
                    if (data.alarmTriggered === "1") {
                        // Показываем карточку срабатывания будильника
                        alarmAlertCard.style.display = 'block';
                        
                        // Меняем статус будильника в метрике
                        document.getElementById('alarmStatus').textContent = 'АКТИВЕН!';
                        document.getElementById('alarmStatus').className = 'value warning-value';
                        document.getElementById('alarmTime').textContent = data.alarmTime || '--:--';
                        document.getElementById('alarmTime').style.fontWeight = 'bold';
                        document.getElementById('alarmTime').style.color = '#ff3333';
                    } else if (data.alarmEnabled === "1") {
                        // Будильник установлен, но еще не сработал
                        alarmAlertCard.style.display = 'none';
                        document.getElementById('alarmStatus').textContent = 'Включен';
                        document.getElementById('alarmStatus').className = 'value normal';
                        document.getElementById('alarmTime').textContent = data.alarmTime || '--:--';
                        document.getElementById('alarmTime').style.fontWeight = 'normal';
                        document.getElementById('alarmTime').style.color = '';
                        document.getElementById('alarmEnabled').checked = true;
                    } else {
                        // Будильник отключен
                        alarmAlertCard.style.display = 'none';
                        document.getElementById('alarmStatus').textContent = 'Выключен';
                        document.getElementById('alarmStatus').className = 'value';
                        document.getElementById('alarmTime').textContent = '--:--';
                        document.getElementById('alarmTime').style.fontWeight = 'normal';
                        document.getElementById('alarmTime').style.color = '';
                        document.getElementById('alarmEnabled').checked = false;
                    }
                    
                    // Обновляем информацию о пользователе
                    if (data.username) {
                        document.getElementById('loginStatus').textContent = `Пользователь: ${data.username}`;
                        document.getElementById('loginStatus').style.color = '#fff';
                        document.getElementById('loginStatus').style.fontWeight = 'bold';
                        
                        document.getElementById('loginForm').style.display = 'none';
                        document.getElementById('registerForm').style.display = 'none';
                        document.getElementById('userProfile').style.display = 'block';
                        document.getElementById('profileUsername').textContent = data.username;
                        document.getElementById('sleepSettingsCard').style.display = 'block';
                        
                        // Заполняем данные о режиме сна
                        if (data.bedtime && data.bedtime !== "Not set") {
                            const [bedHour, bedMin] = data.bedtime.split(':');
                            document.getElementById('bedHour').value = bedHour;
                            document.getElementById('bedMinute').value = bedMin;
                        }
                        
                        if (data.wakeup && data.wakeup !== "Not set") {
                            const [wakeHour, wakeMin] = data.wakeup.split(':');
                            document.getElementById('wakeHour').value = wakeHour;
                            document.getElementById('wakeMinute').value = wakeMin;
                        }
                        
                        // Показываем вкладку админа и кнопку быстрого доступа если пользователь админ
                        const isAdmin = data.isAdmin === "1";
                        document.getElementById('adminTab').style.display = isAdmin ? 'block' : 'none';
                        document.getElementById('quickAdminLink').style.display = isAdmin ? 'flex' : 'none';
                        document.getElementById('adminNotice').style.display = isAdmin ? 'block' : 'none';
                        
                        // Добавляем индикатор администратора в статус
                        if (isAdmin) {
                            document.getElementById('loginStatus').innerHTML = `<span style="background:#ff9aa2;color:white;padding:2px 6px;border-radius:10px;">Админ</span> ${data.username}`;
                        }
                    } else {
                        document.getElementById('loginStatus').textContent = 'Не авторизован';
                        document.getElementById('loginStatus').style.color = '#fff';
                        document.getElementById('loginStatus').style.fontWeight = 'normal';
                        
                        document.getElementById('loginForm').style.display = 'block';
                        document.getElementById('userProfile').style.display = 'none';
                        document.getElementById('sleepSettingsCard').style.display = 'none';
                        document.getElementById('adminTab').style.display = 'none';
                        document.getElementById('quickAdminLink').style.display = 'none';
                    }
                    
                    // Применяем цвета предупреждений
                    if (data.pulse > 0) {
                        if (data.pulse < 60 || data.pulse > 100) {
                            document.getElementById('pulseValue').className = 'value warning-value';
                        } else {
                            document.getElementById('pulseValue').className = 'value normal';
                        }
                    } else {
                        document.getElementById('pulseValue').className = 'value';
                    }
                    
                    if (data.spo2 > 0) {
                        if (data.spo2 < 95) {
                            document.getElementById('spo2Value').className = 'value warning-value';
                        } else {
                            document.getElementById('spo2Value').className = 'value normal';
                        }
                    } else {
                        document.getElementById('spo2Value').className = 'value';
                    }
                })
                .catch(error => console.error('Ошибка:', error));
        }
        
        // Переключение между формами входа и регистрации
        function toggleRegisterForm() {
            const loginForm = document.getElementById('loginForm');
            const registerForm = document.getElementById('registerForm');
            
            if (loginForm.style.display === 'none') {
                loginForm.style.display = 'block';
                registerForm.style.display = 'none';
            } else {
                loginForm.style.display = 'none';
                registerForm.style.display = 'block';
            }
        }
        
        // Установка времени
        function setTime() {
            const hours = document.getElementById('timeHours').value;
            const minutes = document.getElementById('timeMinutes').value;
            
            if (!hours || !minutes) {
                alert('Пожалуйста, заполните часы и минуты');
                return;
            }
            
            fetch(`/setTime?h=${hours}&m=${minutes}`)
                .then(response => {
                    if (response.ok) {
                        alert('Время успешно установлено!');
                        updateData();
                    } else {
                        alert('Ошибка при установке времени');
                    }
                })
                .catch(error => {
                    console.error('Ошибка:', error);
                });
        }
        
        // Установка будильника
        function setAlarm() {
            const hours = document.getElementById('alarmHours').value;
            const minutes = document.getElementById('alarmMinutes').value;
            
            if (!hours || !minutes) {
                alert('Пожалуйста, заполните часы и минуты');
                return;
            }
            
            fetch(`/setAlarm?h=${hours}&m=${minutes}`)
                .then(response => {
                    if (response.ok) {
                        alert('Будильник успешно установлен!');
                        updateData();
                    } else {
                        alert('Ошибка при установке будильника');
                    }
                })
                .catch(error => {
                    console.error('Ошибка:', error);
                });
        }
        
        // Отключение будильника
        function clearAlarm() {
            fetch('/clearAlarm')
                .then(response => {
                    if (response.ok) {
                        // Закрываем карточку срабатывания будильника
                        document.getElementById('alarmAlertCard').style.display = 'none';
                        updateData();
                    } else {
                        alert('Ошибка при отключении будильника');
                    }
                })
                .catch(error => {
                    console.error('Ошибка:', error);
                });
        }
        
        // Установка времени сна
        function setSleepTime() {
            const bedHour = document.getElementById('bedHour').value;
            const bedMinute = document.getElementById('bedMinute').value;
            const wakeHour = document.getElementById('wakeHour').value;
            const wakeMinute = document.getElementById('wakeMinute').value;
            
            fetch('/setSleep', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/x-www-form-urlencoded',
                },
                body: `bedH=${bedHour}&bedM=${bedMinute}&wakeH=${wakeHour}&wakeM=${wakeMinute}`
            })
            .then(response => {
                alert('Настройки сна сохранены');
                updateData();
            })
            .catch(error => {
                console.error('Ошибка:', error);
            });
        }
        
        // Регистрация нового пользователя
        function register() {
            const username = document.getElementById('newUsername').value;
            const password = document.getElementById('newPassword').value;
            
            if (!username || !password) {
                alert('Пожалуйста, заполните все поля');
                return;
            }
            
            const formData = new FormData();
            formData.append('username', username);
            formData.append('password', password);
            
            fetch('/register', {
                method: 'POST',
                body: formData
            })
            .then(response => {
                if (response.ok) {
                    alert('Регистрация успешна!');
                    updateData();
                } else {
                    alert('Ошибка регистрации. Возможно, имя пользователя уже занято.');
                }
            })
            .catch(error => {
                alert('Ошибка регистрации: ' + error);
            });
        }
        
        // Вход в систему
        function login() {
            const username = document.getElementById('username').value;
            const password = document.getElementById('password').value;
            
            const formData = new FormData();
            formData.append('username', username);
            formData.append('password', password);
            
            fetch('/login', {
                method: 'POST',
                body: formData
            })
            .then(response => {
                if (response.ok) {
                    document.getElementById('username').value = '';
                    document.getElementById('password').value = '';
                    updateData();
                } else {
                    alert('Неверное имя пользователя или пароль');
                }
            })
            .catch(error => {
                alert('Ошибка входа: ' + error);
            });
        }
        
        // Выход из системы
        function logout() {
            fetch('/logout')
                .then(() => {
                    updateData();
                });
        }
        
        // Обновление данных каждую секунду
        setInterval(updateData, 1000);
        updateData();
    </script>
</body>
</html>
)=====");
  server.send(200, "text/html", html);
}

void handleData() {
  // Добавляем yield для улучшения отзывчивости
  yield();
  
  String json = "{";
  json += "\"time\":\"" + String(hours) + ":" + (minutes < 10 ? "0" : "") + String(minutes) + ":" + (seconds < 10 ? "0" : "") + String(seconds) + "\",";
  json += "\"pulse\":\"" + String(pulse) + "\",";
  json += "\"spo2\":\"" + String(spo2) + "\",";
  json += "\"finger_present\":\"" + String(fingerPresent ? "1" : "0") + "\",";
  json += "\"sensor_active\":\"" + String(activeSensorReading ? "1" : "0") + "\",";
  json += "\"alarmEnabled\":\"" + String(alarmHour >= 0 ? "1" : "0") + "\",";
  json += "\"alarmTriggered\":\"" + String(alarmTriggered ? "1" : "0") + "\",";
  json += "\"alarmTime\":\"" + (alarmHour >= 0 ? String(alarmHour) + ":" + (alarmMinute < 10 ? "0" : "") + String(alarmMinute) : "") + "\"";
  
  // Добавляем информацию о пользователе, если авторизован
  if (currentUserIndex >= 0) {
    User* user = &users[currentUserIndex];
    json += ",\"username\":\"" + user->username + "\",";
    json += "\"isAdmin\":\"" + String(user->isAdmin ? "1" : "0") + "\"";
    
    // Добавляем информацию о времени сна/пробуждения
    if (user->bedtimeHour >= 0) {
      String bedTime = String(user->bedtimeHour) + ":";
      if (user->bedtimeMinute < 10) bedTime += "0";
      bedTime += String(user->bedtimeMinute);
      json += ",\"bedtime\":\"" + bedTime + "\"";
    } else {
      json += ",\"bedtime\":\"Not set\"";
    }
    
    if (user->wakeupHour >= 0) {
      String wakeTime = String(user->wakeupHour) + ":";
      if (user->wakeupMinute < 10) wakeTime += "0";
      wakeTime += String(user->wakeupMinute);
      json += ",\"wakeup\":\"" + wakeTime + "\"";
    } else {
      json += ",\"wakeup\":\"Not set\"";
    }
  } else {
    json += ",\"username\":\"\",";
    json += "\"isAdmin\":\"0\",";
    json += "\"bedtime\":\"Not set\",";
    json += "\"wakeup\":\"Not set\"";
  }
  
  json += "}";
  server.send(200, "application/json", json);
  
  // Добавляем yield в конце функции
  yield();
}

void handleSetTime() {
  if (server.hasArg("h") && server.hasArg("m")) {
    int h = server.arg("h").toInt();
    int m = server.arg("m").toInt();
    
    if (h >= 0 && h < 24 && m >= 0 && m < 60) {
      // Устанавливаем глобальные переменные времени
      hours = h;
      minutes = m;
      seconds = 0; // Сбрасываем секунды
      
      // Устанавливаем базу времени для корректной работы всех функций
      timeBase = millis() - (h * 3600000UL + m * 60000UL);
      
      Serial.print("Время установлено: ");
      Serial.print(h);
      Serial.print(":");
      Serial.println(m);
      
      server.send(200, "text/plain", "Time set successfully");
      return;
    }
  }
  
  server.send(400, "text/plain", "Invalid time parameters");
}

void handleSetAlarm() {
  if (server.hasArg("h") && server.hasArg("m")) {
    int h = server.arg("h").toInt();
    int m = server.arg("m").toInt();
    
    // Проверяем корректность введенных данных
    if (h >= 0 && h < 24 && m >= 0 && m < 60) {
      alarmHour = h;
      alarmMinute = m;
      alarmTriggered = false;
      
      Serial.print("Будильник установлен на: ");
      Serial.print(h);
      Serial.print(":");
      Serial.println(m);
      
      // Показываем уведомление на дисплее
      display.clearDisplay();
      display.setTextSize(1);
      display.setCursor(0, 0);
      display.println("Alarm set to:");
      display.setTextSize(2);
      display.setCursor(30, 20);
      display.printf("%02d:%02d", h, m);
      display.display();
      delay(2000);
      
      server.send(200, "text/plain", "Alarm set successfully");
      return;
    }
  }
  
  server.send(400, "text/plain", "Invalid alarm parameters");
}

void handleClearAlarm() {
  alarmHour = -1;
  alarmMinute = -1;
  alarmTriggered = false;
  
  // Показываем уведомление на дисплее
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Alarm cleared!");
  display.display();
  delay(1000);
  
  server.send(200, "text/plain", "Alarm cleared successfully");
}

// Функции для работы с пользователями
void loadUsers() {
  if (LittleFS.exists("/users.json")) {
    File file = LittleFS.open("/users.json", "r");
    if (file) {
      DynamicJsonDocument doc(4096);
      DeserializationError error = deserializeJson(doc, file);
      if (!error) {
        userCount = min((int)doc["count"].as<int>(), MAX_USERS);
        for (int i = 0; i < userCount; i++) {
          users[i].username = doc["users"][i]["username"].as<String>();
          users[i].password = doc["users"][i]["password"].as<String>();
          users[i].bedtimeHour = doc["users"][i]["bedtimeHour"] | -1;
          users[i].bedtimeMinute = doc["users"][i]["bedtimeMinute"] | -1;
          users[i].wakeupHour = doc["users"][i]["wakeupHour"] | -1;
          users[i].wakeupMinute = doc["users"][i]["wakeupMinute"] | -1;
          users[i].recordCount = min((int)doc["users"][i]["recordCount"].as<int>(), 20);
          users[i].isAdmin = doc["users"][i]["isAdmin"] | false;
          
          for (int j = 0; j < users[i].recordCount; j++) {
            users[i].records[j].timestamp = doc["users"][i]["records"][j]["timestamp"].as<unsigned long>();
            users[i].records[j].pulseValue = doc["users"][i]["records"][j]["pulse"].as<int>();
            users[i].records[j].spo2Value = doc["users"][i]["records"][j]["spo2"].as<int>();
          }
        }
      }
      file.close();
    }
  }
  
  // Создаем админа, если его нет
  createAdminIfNeeded();
}

void saveUsers() {
  DynamicJsonDocument doc(4096);
  doc["count"] = userCount;
  
  JsonArray usersArray = doc.createNestedArray("users");
  for (int i = 0; i < userCount; i++) {
    JsonObject userObj = usersArray.createNestedObject();
    userObj["username"] = users[i].username;
    userObj["password"] = users[i].password;
    userObj["bedtimeHour"] = users[i].bedtimeHour;
    userObj["bedtimeMinute"] = users[i].bedtimeMinute;
    userObj["wakeupHour"] = users[i].wakeupHour;
    userObj["wakeupMinute"] = users[i].wakeupMinute;
    userObj["recordCount"] = users[i].recordCount;
    
    JsonArray recordsArray = userObj.createNestedArray("records");
    for (int j = 0; j < users[i].recordCount; j++) {
      JsonObject recordObj = recordsArray.createNestedObject();
      recordObj["timestamp"] = users[i].records[j].timestamp;
      recordObj["pulse"] = users[i].records[j].pulseValue;
      recordObj["spo2"] = users[i].records[j].spo2Value;
    }
  }
  
  File file = LittleFS.open("/users.json", "w");
  if (file) {
    serializeJson(doc, file);
    file.close();
  }
}

int findUser(String username) {
  for (int i = 0; i < userCount; i++) {
    if (users[i].username == username) {
      return i;
    }
  }
  return -1;
}

bool addUser(String username, String password) {
  if (userCount >= MAX_USERS) {
    return false;
  }
  
  if (findUser(username) >= 0) {
    return false; // Пользователь уже существует
  }
  
  users[userCount].username = username;
  users[userCount].password = password;
  users[userCount].bedtimeHour = -1;
  users[userCount].bedtimeMinute = -1;
  users[userCount].wakeupHour = -1;
  users[userCount].wakeupMinute = -1;
  users[userCount].recordCount = 0;
  userCount++;
  saveUsers();
  return true;
}

void addPulseRecord(int pulseVal, int spo2Val) {
  if (currentUserIndex < 0 || currentUserIndex >= userCount) {
    return; // Никто не авторизован
  }
  
  User* user = &users[currentUserIndex];
  
  // Если достигли максимума записей, сдвинем все записи на одну позицию назад
  if (user->recordCount >= 20) {
    for (int i = 0; i < 19; i++) {
      user->records[i] = user->records[i+1];
    }
    user->recordCount = 19;
  }
  
  // Добавляем новую запись
  user->records[user->recordCount].timestamp = millis() - timeBase;
  user->records[user->recordCount].pulseValue = pulseVal;
  user->records[user->recordCount].spo2Value = spo2Val;
  user->recordCount++;
  
  saveUsers();
}

// Проверка для сообщений о сне
void checkSleepNotifications() {
  if (currentUserIndex < 0 || currentUserIndex >= userCount) {
    return; // Никто не авторизован
  }
  
  User* user = &users[currentUserIndex];
  unsigned long t = millis() - timeBase;
  int h = (t / 3600000) % 24;
  int m = (t / 60000) % 60;
  
  static bool sleepNotificationShown = false;
  static bool wakeupNotificationShown = false;
  
  // Проверка времени отхода ко сну
  if (user->bedtimeHour >= 0 && !sleepNotificationShown && 
      h == user->bedtimeHour && m == user->bedtimeMinute) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("TIME TO SLEEP!");
    display.println("Good night!");
    display.display();
    delay(3000);
    sleepNotificationShown = true;
  }
  
  // Сброс флага уведомления о сне через час после уведомления
  if (sleepNotificationShown && 
      ((h != user->bedtimeHour) || (m != user->bedtimeMinute))) {
    sleepNotificationShown = false;
  }
  
  // Проверка времени пробуждения
  if (user->wakeupHour >= 0 && !wakeupNotificationShown && 
      h == user->wakeupHour && m == user->wakeupMinute) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("GOOD MORNING!");
    display.println("TIME TO WAKE UP!");
    display.display();
    delay(3000);
    wakeupNotificationShown = true;
  }
  
  // Сброс флага уведомления о пробуждении через час после уведомления
  if (wakeupNotificationShown && 
      ((h != user->wakeupHour) || (m != user->wakeupMinute))) {
    wakeupNotificationShown = false;
  }
}

// Показ мотивирующих сообщений
void showMotivationalMessage() {
  unsigned long now = millis();
  if (now - lastMessageTime >= messageInterval) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println(motivationalMessages[currentMessageIndex]);
    display.display();
    delay(3000); // Показываем на 3 секунды
    
    currentMessageIndex = (currentMessageIndex + 1) % MESSAGE_COUNT;
    lastMessageTime = now;
  }
}

void handleLogin() {
  if (server.hasArg("username") && server.hasArg("password")) {
    String username = server.arg("username");
    String password = server.arg("password");
    
    int userIndex = findUser(username);
    if (userIndex >= 0 && users[userIndex].password == password) {
      // Сначала сбрасываем значения предыдущего пользователя
      pulse = 0;
      spo2 = 0;
      beatDetected = false;
      
      // Затем устанавливаем нового пользователя
      currentUserIndex = userIndex;
      
      // Показываем приветственное сообщение
      display.clearDisplay();
      display.setTextSize(1);
      display.setCursor(0, 0);
      display.println("Приветствую!");
      display.println(username);
      display.display();
      delay(1000);
      
      // Логируем вход
      Serial.println("Пользователь вошел: " + username);
      
      server.sendHeader("Location", "/");
      server.send(303);
      return;
    }
  }
  
  // Ошибка аутентификации
  server.send(401, "text/html", "Invalid credentials");
}

void handleRegister() {
  if (server.hasArg("username") && server.hasArg("password")) {
    String username = server.arg("username");
    String password = server.arg("password");
    
    if (addUser(username, password)) {
      // Автоматически авторизуем пользователя после регистрации
      currentUserIndex = findUser(username);
      server.sendHeader("Location", "/");
      server.send(303);
      return;
    }
  }
  
  // Ошибка регистрации
  server.send(400, "text/html", "Registration failed");
}

void handleLogout() {
  // Выход из аккаунта
  currentUserIndex = -1;
  
  // Сбрасываем все личные данные
  pulse = 0;
  spo2 = 0;
  beatDetected = false;
  alarmHour = -1;
  alarmMinute = -1;
  alarmTriggered = false;
  
  // Очищаем дисплей для следующего пользователя
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Выход из аккаунта");
  display.println("Успешно!");
  display.display();
  delay(1000);
  
  // Перенаправляем на главную страницу
  server.sendHeader("Location", "/");
  server.send(303);
  
  Serial.println("Пользователь вышел из аккаунта");
}

void handleSetSleep() {
  if (currentUserIndex < 0) {
    server.send(401, "text/html", "Not logged in");
    return;
  }
  
  User* user = &users[currentUserIndex];
  
  if (server.hasArg("bedH") && server.hasArg("bedM")) {
    user->bedtimeHour = server.arg("bedH").toInt();
    user->bedtimeMinute = server.arg("bedM").toInt();
  }
  
  if (server.hasArg("wakeH") && server.hasArg("wakeM")) {
    user->wakeupHour = server.arg("wakeH").toInt();
    user->wakeupMinute = server.arg("wakeM").toInt();
  }
  
  saveUsers();
  server.sendHeader("Location", "/");
  server.send(303);
}

// Функция для проверки показателей здоровья и выдачи предупреждений
String checkHealthStatus(int pulseValue, int spo2Value) {
  String warning = "";
  
  // Проверка пульса
  if (pulseValue < MIN_NORMAL_PULSE) {
    warning += "Внимание! Пульс низкий (" + String(pulseValue) + " уд/мин). ";
  } else if (pulseValue > MAX_NORMAL_PULSE) {
    warning += "Внимание! Пульс высокий (" + String(pulseValue) + " уд/мин). ";
  }
  
  // Проверка SpO2
  if (spo2Value < CRITICAL_SPO2) {
    warning += "КРИТИЧЕСКОЕ СНИЖЕНИЕ SpO2 (" + String(spo2Value) + "%)! Срочно обратитесь к врачу! ";
  } else if (spo2Value < MIN_NORMAL_SPO2) {
    warning += "Уровень SpO2 ниже нормы (" + String(spo2Value) + "%). Рекомендуется консультация врача. ";
  }
  
  return warning;
}

// Создаем административный аккаунт, если он не существует
void createAdminIfNeeded() {
  if (findUser("admin") < 0) {
    // Добавляем администратора с паролем admin
    users[userCount].username = "admin";
    users[userCount].password = "admin";
    users[userCount].bedtimeHour = -1;
    users[userCount].bedtimeMinute = -1;
    users[userCount].wakeupHour = -1;
    users[userCount].wakeupMinute = -1;
    users[userCount].recordCount = 0;
    users[userCount].isAdmin = true;
    userCount++;
    saveUsers();
    Serial.println("Админ создан");
  }
}

// Обрабатываем запрос на административную страницу
void handleAdmin() {
  // Проверяем, что пользователь авторизован и является администратором
  if (currentUserIndex < 0 || !users[currentUserIndex].isAdmin) {
    server.sendHeader("Location", "/");
    server.send(303);
    return;
  }

  String html = F(R"=====(<!DOCTYPE html><html><head>
<meta charset='UTF-8'>
<meta name='viewport' content='width=device-width,initial-scale=1'>
<title>Панель администратора</title>
<style>
:root {
  --primary: #ff9aa2;
  --primary-light: #ffb7b2;
  --bg: #fff5f5;
  --text: #5e5e5e;
  --text-light: #888;
  --card-bg: #fff;
  --accent: #ff6b6b;
  --success: #7ac142;
}
body{font-family:'Arial Rounded MT Bold',Arial,sans-serif;margin:0;padding:0;background:var(--bg);color:var(--text)}
header{background:linear-gradient(to right,var(--primary),var(--primary-light));color:#fff;padding:15px;text-align:center;box-shadow:0 4px 10px rgba(255,170,170,0.3)}
h1{margin:0;font-size:22px;text-shadow:1px 1px 2px rgba(150,150,150,0.3)}
.container{max-width:800px;margin:20px auto;padding:15px}
.card{background:var(--card-bg);border-radius:15px;box-shadow:0 4px 15px rgba(0,0,0,0.05);padding:20px;margin-bottom:15px;transition:all 0.3s ease}
.card:hover{transform:translateY(-3px);box-shadow:0 7px 20px rgba(0,0,0,0.1)}
table{width:100%;border-collapse:collapse;margin:15px 0;border-radius:10px;overflow:hidden}
th{background:var(--primary);color:white;padding:12px;text-align:left;font-weight:bold}
td{padding:12px;border-bottom:1px solid #ffe0e0;transition:all 0.2s}
tr:hover td{background:#fff8f8}
.admin-badge{background:var(--primary);color:white;padding:3px 8px;border-radius:10px;font-size:12px}
.user-badge{background:#e0e0e0;color:var(--text);padding:3px 8px;border-radius:10px;font-size:12px}
.button{display:inline-block;background:var(--accent);color:white;border:none;padding:8px 12px;border-radius:10px;text-decoration:none;cursor:pointer;font-weight:bold;transition:all 0.3s;box-shadow:0 3px 8px rgba(255,154,162,0.3);font-size:12px}
.button:hover{background:#ff5c5c;transform:translateY(-2px);box-shadow:0 5px 12px rgba(255,154,162,0.4)}
.back{display:inline-block;margin-top:15px;color:var(--primary);text-decoration:none;font-weight:bold;transition:all 0.3s}
.back:hover{color:var(--accent);transform:translateX(-3px)}
.actions{text-align:center}
.logout-section{margin-top:20px;display:flex;justify-content:space-between}
.no-users{text-align:center;padding:20px;color:var(--text-light);font-style:italic}
.modal{display:none;position:fixed;top:0;left:0;width:100%;height:100%;background:rgba(0,0,0,0.5);z-index:100;align-items:center;justify-content:center}
.modal-content{background:var(--card-bg);padding:20px;border-radius:15px;box-shadow:0 5px 25px rgba(0,0,0,0.2);max-width:400px;width:90%}
.modal-title{color:var(--primary);margin-top:0;text-align:center}
.modal-buttons{display:flex;justify-content:space-between;margin-top:20px}
.confirm-button{background:var(--accent)}
.cancel-button{background:#aaa}
</style>
</head>
<body>
<header>
  <h1>❤️ Панель администратора ❤️</h1>
</header>

<div class="container">
  <div class="card">
    <h2 style="text-align:center;color:var(--primary)">Управление пользователями</h2>
    <table>
      <tr>
        <th>Имя пользователя</th>
        <th>Роль</th>
        <th>Режим сна</th>
        <th>Режим пробуждения</th>
        <th>Действия</th>
      </tr>
)=====");

  // Проверяем, есть ли пользователи
  if (userCount == 0) {
    html += "<tr><td colspan='5' class='no-users'>Нет зарегистрированных пользователей</td></tr>";
  } else {
    // Добавляем каждого пользователя в таблицу
    for (int i = 0; i < userCount; i++) {
      html += "<tr><td>" + users[i].username + "</td><td>";
      html += users[i].isAdmin ? "<span class='admin-badge'>Админ</span>" : "<span class='user-badge'>Пользователь</span>";
      html += "</td><td>";
      
      // Добавляем время отхода ко сну
      if (users[i].bedtimeHour >= 0) {
        html += String(users[i].bedtimeHour) + ":" + (users[i].bedtimeMinute < 10 ? "0" : "") + String(users[i].bedtimeMinute);
      } else {
        html += "<i>Не задано</i>";
      }
      
      html += "</td><td>";
      
      // Добавляем время пробуждения
      if (users[i].wakeupHour >= 0) {
        html += String(users[i].wakeupHour) + ":" + (users[i].wakeupMinute < 10 ? "0" : "") + String(users[i].wakeupMinute);
      } else {
        html += "<i>Не задано</i>";
      }
      
      html += "</td><td class='actions'>";
      if (i != currentUserIndex) {
        html += "<button onclick='confirmDelete(" + String(i) + ", \"" + users[i].username + "\")' class='button'>Удалить</button>";
      } else {
        html += "<i>Текущий аккаунт</i>";
      }
      html += "</td></tr>";
    }
  }

  html += F(R"=====(
    </table>
  </div>
  <div class="logout-section">
    <a href="/" class="back">← Вернуться на главную</a>
    <a href="/logout" class="back" style="background-color:var(--accent);color:white;padding:8px 15px;border-radius:10px;">Выйти из аккаунта</a>
  </div>
</div>

<!-- Модальное окно подтверждения удаления -->
<div id="deleteModal" class="modal">
  <div class="modal-content">
    <h3 class="modal-title">Подтверждение удаления</h3>
    <p id="deleteMessage" style="text-align:center"></p>
    <div class="modal-buttons">
      <button class="button cancel-button" onclick="closeModal()">Отмена</button>
      <button class="button confirm-button" id="confirmDeleteBtn">Удалить</button>
    </div>
  </div>
</div>

<script>
  // Функция для подтверждения удаления пользователя
  function confirmDelete(userId, username) {
    document.getElementById('deleteMessage').innerText = `Вы уверены, что хотите удалить пользователя "${username}"?`;
    
    const confirmBtn = document.getElementById('confirmDeleteBtn');
    confirmBtn.onclick = function() {
      window.location.href = `/deleteUser?id=${userId}`;
    };
    
    document.getElementById('deleteModal').style.display = 'flex';
  }
  
  // Функция для закрытия модального окна
  function closeModal() {
    document.getElementById('deleteModal').style.display = 'none';
  }
  
  // Закрыть модальное окно при клике вне его содержимого
  window.onclick = function(event) {
    const modal = document.getElementById('deleteModal');
    if (event.target == modal) {
      closeModal();
    }
  }
</script>
</body>
</html>
)=====");

  server.send(200, "text/html", html);
}

// Обрабатываем запрос на удаление пользователя
void handleDeleteUser() {
  // Проверяем, что администратор авторизован
  if (currentUserIndex < 0 || !users[currentUserIndex].isAdmin) {
    server.sendHeader("Location", "/");
    server.send(303);
    return;
  }

  // Проверяем, что передан ID пользователя
  if (server.hasArg("id")) {
    int userId = server.arg("id").toInt();
    
    // Проверяем что ID валидный и не пытаемся удалить текущего пользователя
    if (userId >= 0 && userId < userCount && userId != currentUserIndex) {
      // Запоминаем имя пользователя для вывода сообщения
      String deletedUsername = users[userId].username;
      
      // Сдвигаем всех пользователей в массиве
      for (int i = userId; i < userCount - 1; i++) {
        users[i] = users[i+1];
      }
      userCount--;
      saveUsers(); // Сохраняем обновленный список
      
      // Выводим сообщение об успешном удалении
      Serial.println("Пользователь удален: " + deletedUsername);
    } else if (userId == currentUserIndex) {
      Serial.println("Попытка удаления текущего пользователя");
    } else {
      Serial.println("Неверный ID пользователя для удаления: " + String(userId));
    }
  }
  
  // Перенаправляем обратно на административную панель
  server.sendHeader("Location", "/admin");
  server.send(303);
}