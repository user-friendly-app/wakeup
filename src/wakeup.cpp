#include <Arduino.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <esp_timer.h>

#include "VL53L0X.h"
#include "DFRobotDFPlayerMini.h"  // https://github.com/DFRobot/DFRobotDFPlayerMini
#include "EdSoftLED.h"            // https://github.com/ednieuw/EdSoftLED
#include "NTPClient.h"            // https://github.com/taranais/NTPClient
#include "utils.h"  // Моя либа

VL53L0X tof; // Time of Flight
uint32_t OrangeColor = 0X00FF0000, YellowColor = 0X000000FF, WhiteColor = 0X0000FF00;
const char* contentTypeText = "text/html; charset=utf-8";
DFRobotDFPlayerMini music;
extern const uint8_t leds = 100;
const uint8_t secondsTimer = 1; // Сколько секунд ждать таймера
EdSoftLED sunrise(leds, GPIO_NUM_5);
int alarmSunriseHours = 6, alarmSunriseMinutes = 55, alarmMusicHours = 7, alarmMusicMinutes = 0, day = -1;
uint8_t volume = 25, sunriseMinutes = 5;
bool power = true; // Чтобы не разбирать будильник и не вынимать батарейку
bool alarmSunrise = true, alarmMusic = true;  
int sunriseSeconds = 0; // Сколько сейчас длится рассвет
int sunriseDurationSeconds; // Длительность всего рассвета
esp_timer_handle_t once_timer;
uint16_t distance;
// EdSoftLED lamp(1, GPIO_NUM_8);

extern const uint8_t wakeup_html[] asm("_binary_web_wakeup_html_start");
WebServer web(80);
WiFiUDP ntpUDP;
NTPClient datetime(ntpUDP, "1.ru.pool.ntp.org", 10800, 1 * 24 * 3600 * 1000);  // time server pool, offset in seconds,  update interval in milliseconds. ESP time ~1.7 секунды за сутки
esp_timer_handle_t second_timer;

void UpdateAlarmSunrise() {
  int mm = (alarmMusicHours * 60) + (alarmMusicMinutes - sunriseMinutes);  // TODO ночью (00:01 - 20) было бы отрицательное
  alarmSunriseHours = (mm / 60) % 24;                                      // correct overflow 24H to 0H
  alarmSunriseMinutes = mm % 60;
  Serial.printf("Sunrise %d:%d\n", alarmSunriseHours, alarmSunriseMinutes);
}

void SetAlarm() {
  day = -1;  // Чтобы будильник в это же утро сработал. Даже если уже был раньше утром.
  alarmMusicHours = atoi(web.arg("h").c_str());
  alarmMusicMinutes = atoi(web.arg("m").c_str());
  UpdateAlarmSunrise();
  web.send(200, contentTypeText, String(alarmMusicHours) + ":" + alarmMusicMinutes);
}

void SetSunrise() {
  sunriseMinutes = atoi(web.arg("s").c_str());  // Duration in Minutes
  sunriseDurationSeconds = sunriseMinutes * 60;
  sunriseSeconds = sunriseDurationSeconds; // Иначе влючится рассвет
  UpdateAlarmSunrise();
  web.send(200, contentTypeText, String(sunriseMinutes));
}

void SetVolume() {
  volume = atoi(web.arg("v").c_str());
  music.volume(volume);  // Set volume from 0 to 30
  web.send(200, contentTypeText, String(volume));
}

static void WiFiEvent(WiFiEvent_t event) {
  switch (event) {
    // case SYSTEM_EVENT_STA_CONNECTED: // Connected to access point
    //   break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:  // Disconnected from WiFi access point
      WiFi.reconnect();
      break;
    // case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED: // WiFi client disconnected
    //   break;
    default:
      break;
  }
}

void SetupWiFi() {
  WiFi.setHostname("Wakeup");
  WiFi.mode(WIFI_STA);
  WiFi.begin("Ah", "85k$wNRpSs=GV_S");
  WiFi.setAutoReconnect(true);  // automatically reconnect to the previously connected access point
  while (!WiFi.isConnected()) {
    Serial.print(".");
    delay(300);
  }
  WiFi.onEvent(WiFiEvent);
  Serial.printf(" Connected, IP %s\n", WiFi.localIP().toString());
}

void Alarms() {
  if (!power) return;
  auto d = datetime.getDay();
  if (d != day) {  // На следующий день включатся будильники. В полночь будет другой день.
    day = d;
    alarmSunrise = alarmMusic = true;
  }

  if (!alarmSunrise && !alarmMusic) return;

  auto hs = datetime.getHours(), ms = datetime.getMinutes();
  if (alarmSunrise && hs == alarmSunriseHours && ms == alarmSunriseMinutes) {
    alarmSunrise = false;  // Без этого будильник каждую секунду будет срабатывать
    sunriseSeconds = 0;    // Вкл рассвет
    Serial.println(F("Alarm Sunrise"));
  }
  if (alarmMusic && hs == alarmMusicHours && ms == alarmMusicMinutes) {
    alarmMusic = false; // Без этого будильник каждую секунду будет срабатывать
    ESP_ERROR_CHECK(esp_timer_start_once(once_timer, 7 * 60e6)); // 7 минут музыки и света
    music.randomAll();
    Serial.println(F("Alarm Music"));
  }
}

// Три раза поменять цвета на всех лампочках
void Sunrise() {
  if (sunriseSeconds >= sunriseDurationSeconds) return;
  uint16_t lamp = 0;
  uint32_t color = 0;
  double progress = double(sunriseSeconds) / double(sunriseDurationSeconds);
  if (progress < 0.3333333333333333) {
    color = OrangeColor;
    lamp = uint16_t(round(Map(progress, 0, 0.3333333333333333, 0, leds)));
  } else if (progress < 0.6666666666666666) {
    color = YellowColor;
    lamp = uint16_t(round(Map(progress, 0.3333333333333333, 0.6666666666666666, 0, leds)));
  } else {
    color = WhiteColor;
    lamp = uint16_t(round(Map(progress, 0.6666666666666666, 1, 0, leds)));
  }
  sunrise.update(lamp, color);
  sunriseSeconds += secondsTimer;
  // Serial.printf("Lamp %d Progress %.3f Seconds %d\n", lamp, progress, sunriseSeconds);
}

void Cancel() {
  alarmSunrise = alarmMusic = false;
  if (esp_timer_is_active(once_timer)) ESP_ERROR_CHECK(esp_timer_stop(once_timer));  // На всякий случай
  sunriseSeconds = sunriseDurationSeconds;                                           // Остановить рассвет
  music.stop();
  sunrise.off();
  Serial.println(F("Cancel"));
}

void Toggle() {
  power = !power;
  if (!power) {
    Cancel();
  }
}

void TogglePower() {
  Toggle();
  web.send(200, contentTypeText, String(power));
}

static void SecondTimerCallback(void* arg) {
  Alarms();
  Sunrise();
  // Serial.printf("🌡️ %.1f  📡 %d  Status %d  🕗 %lld  🔲 %u\n", temperatureRead(), WiFi.RSSI(), WiFi.status(), esp_timer_get_time(), getCpuFrequencyMhz());
  // digitalWrite(LED, !digitalRead(LED));  // blink on board led
}

static void CancelTimerCallback(void* arg) {
  Cancel();
}

void SetupSun() {
  sunriseDurationSeconds = sunriseMinutes * 60;
  sunriseSeconds = sunriseDurationSeconds;
  sunrise.off();
}

// ! Если в setup зависнет или ошибка будет, то для прошивки зажать кнопку ноль и воткнуть USB
void setup() {
  pinMode(GPIO_NUM_15, OUTPUT);
  // lamp.off();
  Serial.begin(76800);
  delay(1000); // Для этого: [     1][E][EdSoftLED.cpp:12] EdSoftLED(): real tick set to: 100
  Serial.println(F("\nBoot ToF"));
  Wire1.begin(); // GPIO_NUM_6, GPIO_NUM_7
  tof.setBus(&Wire1);
  if (!tof.init()) {
    Serial.println(F("Failed to detect and initialize sensor!"));
    while (1) {}
  }
  tof.startContinuous();
  distance = tof.readRangeContinuousMillimeters();
  Serial.println(F("Boot Sun"));
  SetupSun();
  Serial.print(F("Boot Wi-Fi"));
  SetupWiFi();
  Serial.print(F("Boot DNS"));
  if (MDNS.begin("wakeup")) {  // Open http://wakeup.local
    MDNS.addService("http", "tcp", 80);
    Serial.print(F(" OK\n"));
  }

  Serial.println(F("Boot Web"));
  web.on("/", HTTP_GET, [] { web.send(200, contentTypeText, String((char*)wakeup_html)); });
  web.on("/alarm", HTTP_GET, SetAlarm);
  web.on("/volume", HTTP_GET, SetVolume);
  web.on("/sunrise", HTTP_GET, SetSunrise);
  web.on("/power", HTTP_GET, TogglePower);
  web.on("/status", HTTP_GET, [] { web.send(200, contentTypeText, "{\"Present\":\"" + datetime.getFormattedDate() + "\",\"Time\":" + datetime.getEpochTime() + ",\"Alarm\":\"" + alarmMusicHours + ":" + alarmMusicMinutes + "\",\"Volume\":" + volume + ",\"Sunrise\":" + sunriseMinutes + ",\"CompileDate\":\"" + __DATE__ + "\",\"CompileTime\":\"" + __TIME__ + "\",\"Temperature\":" + temperatureRead() + ",\"Battery\":" + 0 + ",\"Power\":" + String(power) + "}"); });
  web.enableCORS(true);  // for local dev
  web.begin();

  Serial.println(F("Boot NTP"));
  datetime.begin();
  datetime.forceUpdate(); // Иначе может не обновить
  Serial.println(F("Boot Timer"));
  // create timer parameters..
  const esp_timer_create_args_t second_timer_args = {
      .callback = &SecondTimerCallback,  // link the call back
      .arg = nullptr,                    // not passing in anything
      .name = "second"                   // nice name, Optional для отладки
  };
  // create & running periodic timer
  ESP_ERROR_CHECK(esp_timer_create(&second_timer_args, &second_timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(second_timer, secondsTimer * 1e6));  // 1 раз в секунду
  // create & not running once timer
  const esp_timer_create_args_t once_timer_args = {
      .callback = &CancelTimerCallback,  // link the call back
      .arg = nullptr,                    // not passing in anything
      .name = "once"                     // nice name, Optional для отладки
  };
  // create timer, not running yet..
  ESP_ERROR_CHECK(esp_timer_create(&once_timer_args, &once_timer));
  Serial.println(F("Boot Music"));
  Serial0.begin(9600);  // MP3 Player
  music.begin(Serial0, false, true);
  music.volume(volume);
  setCpuFrequencyMhz(80);
  Serial.println(F("Boot complete"));
}

void Distance() {
  auto d2 = tof.readRangeContinuousMillimeters();
  if (d2 < float(distance * 0.85)) {
    static unsigned long cancelMillis;
    if (millis() - cancelMillis >= 600) {
      cancelMillis = millis();  // restart this TIMER
      Serial.printf("%d %d \t", d2, distance);
      Cancel();
      digitalWrite(GPIO_NUM_15, HIGH);
    }
  } else {
    digitalWrite(GPIO_NUM_15, LOW);
  }
}

void loop() {
  web.handleClient();  // Handle incoming requests
  datetime.update();
  Distance();
}
