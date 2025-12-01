#include <avr/io.h>
#include <Arduino.h>

const uint8_t  WATER_SENSOR_CHANNEL = 0;   // A0 = ADC0
const uint8_t  WATER_SAMPLES        = 16;

const uint16_t WATER_RAW_MIN = 370;  // dry
const uint16_t WATER_RAW_MAX = 580;  // wet

#define IN1 PB0
#define IN2 PB1
#define IN3 PB2
#define IN4 PB3

const int STEPS_PER_TURN = 512;  

bool isClosed = false;

void adc_init() {
  ADMUX = (1 << REFS0);

  // Enable ADC, prescaler 128 -> 125 kHz @ 16 MHz
  ADCSRA = (1 << ADEN) |
           (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC));
}

uint16_t adc_read(uint8_t channel) {
  channel &= 0x07;
  ADMUX = (ADMUX & 0xF0) | channel;  // select channel (keep REFS0)
  ADCSRA |= (1 << ADSC);           
  while (ADCSRA & (1 << ADSC));      
  uint8_t low  = ADCL;
  uint8_t high = ADCH;
  return (uint16_t)(high << 8) | low;
}

uint16_t readWaterRaw() {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < WATER_SAMPLES; i++) {
    sum += adc_read(WATER_SENSOR_CHANNEL);
  }
  return (uint16_t)(sum / WATER_SAMPLES);
}

uint8_t waterRawToPercent(uint16_t raw) {
  if (raw <= WATER_RAW_MIN) return 0;
  if (raw >= WATER_RAW_MAX) return 100;

  uint16_t span = WATER_RAW_MAX - WATER_RAW_MIN;
  uint16_t adj  = raw - WATER_RAW_MIN;

  return (uint32_t(adj) * 100UL) / span;
}

void stepMotor(int step) {
  if (step == 1) { 
    PORTB = (PORTB & 0xF0) | (1 << IN1);
    delay(2);

    PORTB = (PORTB & 0xF0) | (1 << IN2);
    delay(2);

    PORTB = (PORTB & 0xF0) | (1 << IN3);
    delay(2);

    PORTB = (PORTB & 0xF0) | (1 << IN4);
    delay(2);

    PORTB = (PORTB & 0xF0) | (1 << IN1);
    delay(2);

  } else if (step == -1) { 
    PORTB = (PORTB & 0xF0) | (1 << IN4);
    delay(2);

    PORTB = (PORTB & 0xF0) | (1 << IN3);
    delay(2);

    PORTB = (PORTB & 0xF0) | (1 << IN2);
    delay(2);

    PORTB = (PORTB & 0xF0) | (1 << IN1);
    delay(2);

    PORTB = (PORTB & 0xF0) | (1 << IN4);
    delay(2);
  }
}

void runStepperTurns(int dir, int steps) {
  for (int i = 0; i < steps; i++) {
    stepMotor(dir);
  }
  PORTB &= 0xF0;
}

void setup() {
  Serial.begin(9600);
  DDRB |= (1 << IN1) | (1 << IN2) | (1 << IN3) | (1 << IN4);

  PORTB &= 0xF0;

  adc_init();

  Serial.println(F("Water sensor + ULN2003 stepper ready"));
  Serial.println(F("Water OUT -> A0, ULN IN1..IN4 -> D8..D11"));

  isClosed = false;
}

void loop() {
  uint16_t raw = readWaterRaw();
  uint8_t  pct = waterRawToPercent(raw);

  Serial.print(F("Water raw: "));
  Serial.print(raw);
  Serial.print(F(" | Level: "));
  Serial.print(pct);
  Serial.print(F("% | State: "));
  Serial.println(isClosed ? F("CLOSED") : F("OPEN"));


  if (pct >= 50 && !isClosed) {
    Serial.println(F(">> Level high: closing (1 turn CW)..."));
    runStepperTurns(+1, STEPS_PER_TURN);  // CW
    isClosed = true;
  } else if (pct < 50 && isClosed) {
    Serial.println(F(">> Level low: opening (1 turn CCW)..."));
    runStepperTurns(-1, STEPS_PER_TURN);  // CCW
    isClosed = false;
  } else {
    // No action, already in the right state
    Serial.println(F(">> No movement, state already correct."));
  }

  delay(2000); // wait 2s before next read
}
