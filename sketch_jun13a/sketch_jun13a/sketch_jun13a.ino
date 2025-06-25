
#define BLYNK_TEMPLATE_ID  "TMPL2wcVfnH7C"
#define BLYNK_TEMPLATE_NAME "Quickstart Template"
#define BLYNK_AUTH_TOKEN   "QJ21j7fJTIhs5NTzFwabG1Z_6egrL_I5"

#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

/* ------------------------ Hardware ---------------------------------------- */
#define PIN_SENSOR_UMIDADE 34
#define PIN_RELE_BOMBA     22
#define PIN_TERMINAL       V5        // Widget Terminal

char ssid[] = "Caic’s iPhone";
char pass[] = "caic1234";

/* ------------------------ Datastreams Blynk ------------------------------- */
#define PIN_BTN_MANUAL V0   // botão “Irrigar” (ON / OFF)

#define PIN_T_IRRIG    V7   // Tempo de irrigação (s)
#define PIN_T_ABSORB   V8   // Tempo de absorção  (s)
#define PIN_HUM_MIN    V9   // Umidade mínima (%)
#define PIN_HUM_MAX    V10  // Umidade máxima (%)

/* ------------------------ Parâmetros ajustáveis --------------------------- */
/* Valores-padrão de segurança; serão sobrescritos pelos widgets */
volatile uint32_t tIrrig  = 10'000UL;  // ms
volatile uint32_t tAbsorb = 30'000UL;  // ms
volatile float    humMin  = 35.0;      // %
volatile float    humMax  = 60.0;      // %

/* ------------------------ Variáveis globais ------------------------------- */
volatile float umidadePercentual = 0;   // atualizado na sensorTask
volatile int   btnManual         = 0;   // estado do botão V0

/* ------------------------ Fila para Terminal ------------------------------ */
static QueueHandle_t terminalQueue;

/* ------------------------ Prototipagem ------------------------------------ */
void blynkTask (void*);
void sensorTask(void*);
void pumpTask  (void*);

/* ------------------------ Callbacks Blynk --------------------------------- */
BLYNK_WRITE(PIN_BTN_MANUAL) { btnManual = param.asInt(); }

BLYNK_WRITE(PIN_T_IRRIG)    { tIrrig  = param.asInt() * 1000UL; }
BLYNK_WRITE(PIN_T_ABSORB)   { tAbsorb = param.asInt() * 1000UL; }
BLYNK_WRITE(PIN_HUM_MIN)    { humMin  = param.asFloat();        }
BLYNK_WRITE(PIN_HUM_MAX)    { humMax  = param.asFloat();        }

BLYNK_CONNECTED() {
  /* Sincroniza todos os datastreams para recuperar valores salvos no Cloud */
  Blynk.syncVirtual(PIN_BTN_MANUAL,
                    PIN_T_IRRIG, PIN_T_ABSORB,
                    PIN_HUM_MIN, PIN_HUM_MAX);
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_RELE_BOMBA, OUTPUT);
  digitalWrite(PIN_RELE_BOMBA, LOW);

  WiFi.setSleep(false);                       // liga estável
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);  // bloqueia até conectar

  terminalQueue = xQueueCreate(10, sizeof(char[64]));

  /* Cria as três tasks ---------------------------------------------------------------- */
  xTaskCreatePinnedToCore(blynkTask, "Blynk", 4096, nullptr, 2, nullptr, 1);
  xTaskCreatePinnedToCore(sensorTask,"Sensor",2048, nullptr, 1, nullptr, 0);
  xTaskCreatePinnedToCore(pumpTask,  "Pump",  2048, nullptr, 1, nullptr, 0);
}

void loop() { vTaskDelay(portMAX_DELAY); }   // nada aqui; FreeRTOS cuida de tudo

/* =============================================================================
 *  Task 1 – Blynk (única que chama Blynk.run e virtualWrite)
 * ========================================================================== */
void blynkTask(void *pv) {
  uint32_t tic = millis();

  for (;;) {
    Blynk.run();

    /* Atualiza gauge de umidade e uptime a cada 2 s */
    if (millis() - tic >= 2000) {
      tic = millis();
      Blynk.virtualWrite(V4, (int)umidadePercentual);  // gauge
      Blynk.virtualWrite(V2, millis() / 1000);         // uptime
    }

    /* Despeja fila no Terminal */
    char msg[64];
    while (xQueueReceive(terminalQueue, &msg, 0) == pdTRUE)
      Blynk.virtualWrite(PIN_TERMINAL, msg);

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

/* =============================================================================
 *  Task 2 – Sensor de umidade
 * ========================================================================== */
void sensorTask(void *pv) {
  const TickType_t intervalo = 2000 / portTICK_PERIOD_MS;

  for (;;) {
    int adc = analogRead(PIN_SENSOR_UMIDADE);
    float pct = 100.f - ((float)adc / 4095.f) * 100.f;
    pct = std::max(0.f, std::min(100.f, pct));

    umidadePercentual = pct;                       // publica

    char texto[64];
    snprintf(texto, sizeof(texto), "Umidade: %.1f%%\n", pct);
    xQueueSend(terminalQueue, &texto, 0);
    Serial.printf("Umidade (%%): %.1f\n", pct);

    vTaskDelay(intervalo);
  }
}

/* =============================================================================
 *  Task 3 – FSM de controle da bomba
 * ========================================================================== */
enum class PumpState : uint8_t {
  MONITORING, AUTO_ON, AUTO_OFF, MANUAL_ON, STOPPING
};

void pumpTask(void *pv) {
  PumpState state = PumpState::MONITORING;
  uint32_t  tStart = 0;
  bool      pumpOn = false;

  auto setPump = [&](bool on, const char *orig){
    pumpOn = on;
    digitalWrite(PIN_RELE_BOMBA, on ? HIGH : LOW);

    char txt[64];
    snprintf(txt, sizeof(txt),
             on ? "🟢 Bomba LIGADA (%s)\n" : "🔴 Bomba DESLIGADA\n", orig);
    xQueueSend(terminalQueue, &txt, 0);
    Serial.print(txt);
  };

  for (;;) {
    switch (state) {
      case PumpState::MONITORING:
        if (btnManual) {
          state = PumpState::MANUAL_ON;
          setPump(true, "manual");
        }
        else if (umidadePercentual <= humMin) {
          state  = PumpState::AUTO_ON;
          tStart = millis();
          setPump(true, "auto");
        }
        break;

      case PumpState::AUTO_ON:
        if (btnManual) {
          state = PumpState::MANUAL_ON;
        }
        else if (umidadePercentual >= humMax) {
          state = PumpState::STOPPING;
          setPump(false, "");
        }
        else if (millis() - tStart >= tIrrig) {
          state  = PumpState::AUTO_OFF;
          tStart = millis();
          setPump(false, "");
        }
        break;

      case PumpState::AUTO_OFF:
        if (btnManual) {
          state = PumpState::MANUAL_ON;
        }
        else if (umidadePercentual >= humMax) {
          state = PumpState::STOPPING;
        }
        else if (millis() - tStart >= tAbsorb) {
          state  = PumpState::AUTO_ON;
          tStart = millis();
          setPump(true, "auto");
        }
        break;

      case PumpState::MANUAL_ON:
        if (!btnManual) {
          state = PumpState::STOPPING;
          setPump(false, "");
        }
        break;

      case PumpState::STOPPING:
        if (!pumpOn) state = PumpState::MONITORING;
        else         setPump(false, "");
        break;
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);   // resolução de 100 ms
  }
}