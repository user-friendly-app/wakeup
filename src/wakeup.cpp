#include <Arduino.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <esp_timer.h>
#include "DFRobotDFPlayerMini.h"  // https://github.com/DFRobot/DFRobotDFPlayerMini
#include "EdSoftLED.h"            // https://github.com/ednieuw/EdSoftLED
#include "NTPClient.h"            // https://github.com/taranais/NTPClient
#include "espasyncbutton.hpp"     // https://github.com/vortigont/ESPAsyncButton
#include "utils.h"

uint32_t OrangeColor = 0X00FF0000, YellowColor = 0X000000FF, WhiteColor = 0X0000FF00;
extern const uint8_t ADCpin = GPIO_NUM_1;
const char* contentTypeText = "text/html; charset=utf-8";
DFRobotDFPlayerMini music;
extern const uint8_t leds = 10;
const uint8_t secondsTimer = 1; // Сколько секунд ждать таймера
EdSoftLED sunrise(leds, GPIO_NUM_3, WS2812RGB);
int alarmSunriseHours = 6, alarmSunriseMinutes = 30, alarmMusicHours = 6, alarmMusicMinutes = 40, day = -1;
uint8_t volume = 14, sunriseMinutes = 5;
bool off = false, alarmSunrise = true, alarmMusic = true;  // Чтобы не разбирать будильник и не вынимать батарейку
// Сколько сейчас длится рассвет
int sunriseSeconds = 0;
// Длительность рассвета
int sunriseDurationSeconds;

extern const uint8_t wakeup_html[] asm("_binary_web_wakeup_html_start");
WebServer web(80);
WiFiUDP ntpUDP;
NTPClient datetime(ntpUDP, "1.ru.pool.ntp.org", 10800, 1 * 24 * 3600 * 1000);  // time server pool, offset in seconds,  update interval in milliseconds. ESP time ~1.7 секунды за сутки
esp_timer_handle_t second_timer;

// A simple gpio mapped button with Logic level LOW, i.e. button shorts gpio to the ground, gpio will be pulled high via internal pull-up resistor. !!! Без debounce=true всё зависнет и неправильно работало !!!
AsyncEventButton btn(GPIO_NUM_0, true, GPIO_FLOATING, GPIO_MODE_INPUT, true);

void GetUI() {
  web.send(200, contentTypeText, String((char*)wakeup_html));
  // music.randomAll();  // music.play(1);  // Play the first mp3
}

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
  UpdateAlarmSunrise();
  web.send(200, contentTypeText, String(sunriseMinutes));
}

void SetVolume() {
  volume = atoi(web.arg("v").c_str());
  music.volume(volume);  // Set volume from 0 to 30
  web.send(200, contentTypeText, String(volume));
}

void GetStatus() {
  auto battery = (analogRead(ADCpin) * 3.3) / 8192.0;  // TODO correct it
  web.send(200, contentTypeText, "{\"Present\":\"" + datetime.getFormattedDate() + "\",\"Time\":" + datetime.getEpochTime() + ",\"Alarm\":\"" + alarmMusicHours + ":" + alarmMusicMinutes + "\",\"Volume\":" + volume + ",\"Sunrise\":" + sunriseMinutes + ",\"CompileDate\":\"" + __DATE__ + "\",\"CompileTime\":\"" + __TIME__ + "\",\"Temperature\":" + temperatureRead() + ",\"Battery\":" + battery + "}");
}

// Ожидает открытия Serial, который подключается к ПК. Это делается для того, чтобы Serial не пропустил ни одного вывода из программы. На некоторых платах это также означает, что Serial будет зависать, если к нему не подключен ПК.
void SetupDev() {
  Serial.begin(76800);
  // while (!Serial); // Так себе работает
  while (!Serial) {  // Отлично работает
    delay(500);
  }
  // delay(7000); // Без блокировки сработает, если не подключён ПК
}

void SetupWiFi() {
  WiFi.setAutoReconnect(true);  // automatically reconnect to the previously connected access point
  WiFi.persistent(true);        // Это должно быть до WiFi.mode(WIFI_STA)
  WiFi.mode(WIFI_STA);          // Не обязательно.
  WiFi.begin("Ah", "85k$wNRpSs=GV_S");
  while (!WiFi.isConnected()) {
    Serial.print(".");
    delay(300);
  }
  Serial.printf(" Connected, IP %s\n", WiFi.localIP().toString());
}

void Alarms() {
  if (off) return;
  auto d = datetime.getDay();
  if (d != day) { // Без этого будильники будут каждые 10 секунд срабатывать
    day = d;
    alarmSunrise = alarmMusic = true; 
  }

  if (!alarmSunrise && !alarmMusic) return;
  auto h = datetime.getHours(), m = datetime.getMinutes();
  if (alarmSunrise && h == alarmSunriseHours && m == alarmSunriseMinutes) {
    alarmSunrise = false;
    sunriseSeconds = 0;  // Вкл рассвет
    Serial.println("Alarm Sunrise");
  }
  if (alarmMusic && h == alarmMusicHours && m == alarmMusicMinutes) {
    alarmMusic = false;
    music.randomAll();  // Вкл радномную музыку, потом следующую рандомную итд
    Serial.println("Alarm Music");
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
    lamp = uint16_t(Map(progress, 0.0, 0.3333333333333333, 0, leds));
  } else if (progress < 0.6666666666666666) {
    color = YellowColor;
    lamp = uint16_t(Map(progress, 0.3333333333333333, 0.6666666666666666, 0, leds));
  } else {
    color = WhiteColor;
    lamp = uint16_t(Map(progress, 0.6666666666666666, 1.0, 0, leds));
  }
  sunrise.update(lamp, color);
  sunriseSeconds += secondsTimer;
  // Serial.printf("Sun Led %d, progress %.5f, S %d\t", lamp, progress, sunriseSeconds);
}

static void SecondTimerCallback(void* arg) {
  Alarms();
  Sunrise();
  int64_t time_since_boot = esp_timer_get_time();
  Serial.printf("T %.1f  RSSI %d  Connected 3=%d  Time %lld us  CPU %u MHz\n", temperatureRead(), WiFi.RSSI(), WiFi.status(), time_since_boot, getCpuFrequencyMhz());
  // digitalWrite(LED, !digitalRead(LED));  // blink on board led
}

void Cancel() {
  // alarmSunrise = alarmMusic = false;
  sunriseSeconds = sunriseDurationSeconds;  // Остановить рассвет
  music.stop();
  sunrise.off();
  Serial.println("Cancel");
}

void SetupSun() {
  sunriseDurationSeconds = sunriseMinutes * 60;
  sunriseSeconds = sunriseDurationSeconds;
  sunrise.off();
}

// ! Если в setup зависнет или ошибка будет, то для прошивки зажать кнопку ноль и воткнуть USB
void setup() {
  SetupDev();
  Serial.println("Setup Sun");
  SetupSun();
  Serial.print("Setup Wi-Fi");
  SetupWiFi();
  Serial.println("Setup DNS");
  if (MDNS.begin("wakeup")) {  // Open http://wakeup.local
    MDNS.addService("http", "tcp", 80);
    Serial.println("Setup DNS OK");
  }

  Serial.println("Setup Web");
  web.on("/", HTTP_GET, GetUI);
  web.on("/alarm", HTTP_GET, SetAlarm);
  web.on("/status", HTTP_GET, GetStatus);
  web.on("/volume", HTTP_GET, SetVolume);
  web.on("/sunrise", HTTP_GET, SetSunrise);
  web.enableCORS(true);  // for local dev
  web.begin();

  Serial.println("Setup NTP");
  datetime.begin();

  Serial.println("Setup Timer");
  // create timer parameters..
  const esp_timer_create_args_t second_timer_args = {
      .callback = &SecondTimerCallback,  // link the call back
      .arg = nullptr,                    // not passing in anything
      .name = "second"                   // nice name
  };
  // create timer, not running yet..
  ESP_ERROR_CHECK(esp_timer_create(&second_timer_args, &second_timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(second_timer, secondsTimer*1e6));  // 1 раз в секунду

  Serial.println("Setup Button");
  btn.begin();                          // activate event processing for button
  btn.timeouts.setLongPress(5 * 1000);  // for Power off Wi-Fi and go to Deep Sleep
  btn.onPress(nullptr);
  btn.onRelease(nullptr);
  btn.onMultiClick(nullptr);
  btn.onClick(Cancel);
  btn.onLongPress([]() { off = !off; Serial.print("Long, off "); Serial.println(off); });
  btn.enable();            // Button is still in "disabled" state, you need to "enable" it to start handling GPIO interrupts
  setCpuFrequencyMhz(80);  // На 4 градуса меньше греется CPU и LDO на 80 МГц, при этом Wi-Fi и Serial работают как на 240 МГц, без разницы.
  pinMode(ADCpin, INPUT);
  Serial.println("Setup Player");
  Serial0.begin(9600);  // MP3 Player
  music.begin(Serial0, true, true);
  music.volume(volume);
  Serial.println("Setup is complete");
}

void loop() {
  web.handleClient();  // Handle incoming requests
  datetime.update();
}
