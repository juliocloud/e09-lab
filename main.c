#include <avr/io.h>
#include <avr/interrupt.h>
#include <Arduino.h> 

#define EMERGENCY_BUTTON PD2
#define EMERGENCY_LED    PD7

#define LM35_CHANNEL 1

#define TEMP_THRESHOLD 25.0

const uint8_t WATER_SENSOR_CHANNEL = 0;
const uint8_t WATER_SAMPLES = 16;

const uint16_t WATER_RAW_MIN = 370;
const uint16_t WATER_RAW_MAX = 580;

#define IN1 PB0
#define IN2 PB1
#define IN3 PB2
#define IN4 PB3

const int STEPS_PER_TURN = 512;

bool isClosed = false;

volatile bool emergency_stop_active = false;

#define BAUD_PRESCALER (((F_CPU / (9600 * 16UL))) - 1)

void uart_init() {
  UBRR0H = (unsigned char)(BAUD_PRESCALER >> 8);
  UBRR0L = (unsigned char)BAUD_PRESCALER;

  UCSR0B = (1 << RXEN0) | (1 << TXEN0);

  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); 
}

void uart_putc(char data) {
  while (!(UCSR0A & (1 << UDRE0)));
  UDR0 = data;
}

void uart_puts(const char *s) {
  while (*s) {
    uart_putc(*s++);
  }
}

void uart_newline() {
  uart_putc('\r');
  uart_putc('\n');
}

static void reverse(char *str, int len) {
  int i = 0, j = len - 1;
  while (i < j) {
    char temp = str[i];
    str[i] = str[j];
    str[j] = temp;
    i++;
    j--;
  }
}

void uart_puti(int n) {
  char str[12]; 
  int i = 0;
  bool isNegative = false;

  if (n == 0) {
    str[i++] = '0';
    str[i] = '\0';
    uart_puts(str);
    return;
  }

  if (n < 0) {
    isNegative = true;
    n = -n;
  }

  while (n != 0) {
    str[i++] = (n % 10) + '0';
    n = n / 10;
  }

  if (isNegative) {
    str[i++] = '-';
  }

  str[i] = '\0';
  reverse(str, i);
  uart_puts(str);
}

void uart_putf(float f, uint8_t precision) {
  if (f < 0) {
    uart_putc('-');
    f = -f;
  }

  long int_part = (long)f;
  uart_puti(int_part);
  
  if (precision > 0) {
    uart_putc('.');
    
    float frac_part = f - int_part;
    
    for (uint8_t i = 0; i < precision; i++) {
        frac_part *= 10;
    }
    
    long frac_val = (long)(frac_part + 0.5); 

    long check_val = 1;
    for (uint8_t i = 1; i < precision; i++) {
        check_val *= 10;
        if (frac_val < check_val) {
            uart_putc('0');
        }
    }

    uart_puti(frac_val);
  }
}

void adc_init() {
  ADMUX = (1 << REFS0);
  ADCSRA = (1 << ADEN) |
           (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

uint16_t adc_read(uint8_t channel) {
  channel &= 0x07;
  ADMUX = (ADMUX & 0xF0) | channel;
  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC));
  uint8_t low  = ADCL;
  uint8_t high = ADCH;
  return (uint16_t)(high << 8) | low;
}

#define LM35_SAMPLES 16 

float readLM35() {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < LM35_SAMPLES; i++) {
    sum += adc_read(LM35_CHANNEL); 
  }
  
  uint16_t raw_avg = (uint16_t)(sum / LM35_SAMPLES); 
  
  float voltage = (raw_avg * 5.0) / 1023.0; 
  return voltage * 100.0;
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
    PORTB = (PORTB & 0xF0) | (1 << IN1); delay(2);
    PORTB = (PORTB & 0xF0) | (1 << IN2); delay(2);
    PORTB = (PORTB & 0xF0) | (1 << IN3); delay(2);
    PORTB = (PORTB & 0xF0) | (1 << IN4); delay(2);
    PORTB = (PORTB & 0xF0) | (1 << IN1); delay(2);
  } else if (step == -1) {
    PORTB = (PORTB & 0xF0) | (1 << IN4); delay(2);
    PORTB = (PORTB & 0xF0) | (1 << IN3); delay(2);
    PORTB = (PORTB & 0xF0) | (1 << IN2); delay(2);
    PORTB = (PORTB & 0xF0) | (1 << IN1); delay(2);
    PORTB = (PORTB & 0xF0) | (1 << IN4); delay(2);
  }
}

void runStepperTurns(int dir, int steps) {
  for (int i = 0; i < steps; i++) {
    if (emergency_stop_active) {
      uart_puts("Stepper interrupted by emergency stop.");
      uart_newline();
      break;
    }
    stepMotor(dir);
  }
  PORTB &= 0xF0;
}

bool pwmStarted = false;

void runPWMMotor(float tempC) {
  if (emergency_stop_active) {
    OCR0A = 0;
    pwmStarted = false;
    return;
  }

  if (tempC < TEMP_THRESHOLD) {
    if (!pwmStarted) {
      DDRD |= (1 << PD6);
      TCCR0A |= (1 << COM0A1) | (1 << WGM01) | (1 << WGM00);
      TCCR0B |= (1 << CS01);

      uart_puts("PWM Motor starting up (ramping)...");
      uart_newline();
      for (uint16_t duty = 0; duty <= 255; duty++) {
        OCR0A = duty;
        delay(12);
        if (emergency_stop_active) {
          OCR0A = 0;
          return;
        }
      }

      pwmStarted = true;
    }

    OCR0A = 255;
    uart_puts("PWM Motor: ON");
    uart_newline();

  } else {
    OCR0A = 0;
    pwmStarted = false;
    uart_puts("PWM Motor: OFF (Temp >= 25 C)");
    uart_newline();
  }
}

void emergencyStop() {
  uart_puts(">>> EMERGENCY STOP ACTIVATED <<<");
  uart_newline();

  PORTB &= 0xF0;
  OCR0A = 0;

  uint32_t startTime = millis();

  while (millis() - startTime < 5000UL) {
    PORTD |= (1 << EMERGENCY_LED);
    delay(250);
    PORTD &= ~(1 << EMERGENCY_LED);
    delay(250);
  }

  PORTD &= ~(1 << EMERGENCY_LED);

  emergency_stop_active = false;
  uart_puts("Emergency stop period ended. Resuming operation...");
  uart_newline();
}

ISR(INT0_vect) {
  emergency_stop_active = true;
}

void setup() {
  uart_init();

  DDRB |= (1 << IN1) | (1 << IN2) | (1 << IN3) | (1 << IN4);
  PORTB &= 0xF0;

  DDRD |= (1 << EMERGENCY_LED);

  DDRD &= ~(1 << EMERGENCY_BUTTON);
  PORTD |= (1 << EMERGENCY_BUTTON);

  EICRA |= (1 << ISC01);
  EICRA &= ~(1 << ISC00);
  EIMSK |= (1 << INT0);

  sei();

  adc_init();

  uart_puts("Water/LM35 Sensor + Motors ready");
  uart_newline();
  uart_puts("Water OUT -> A0 | LM35 OUT -> A1");
  uart_newline();
  uart_puts("Emergency Button -> D2 | Emergency LED -> D7");
  uart_newline();

  isClosed = false;
}

void loop() {
  if (emergency_stop_active) {
    emergencyStop();
    return;
  }

  uint16_t raw = readWaterRaw();
  uint8_t pct = waterRawToPercent(raw);

  uart_puts("Water raw: ");
  uart_puti(raw);
  
  uart_puts(" | Level: ");
  uart_puti(pct);
  uart_putc('%');
  
  uart_puts(" | State: ");
  uart_puts(isClosed ? "CLOSED" : "OPEN");
  uart_newline();


  if (pct >= 50 && !isClosed) {
    uart_puts(">> Level high: closing (1 turn CW)...");
    uart_newline();
    runStepperTurns(+1, STEPS_PER_TURN);
    isClosed = true;
  } else if (pct < 50 && isClosed) {
    uart_puts(">> Level low: opening (1 turn CCW)...");
    uart_newline();
    runStepperTurns(-1, STEPS_PER_TURN);
    isClosed = false;
  } else {
    uart_puts(">> No movement, state already correct.");
    uart_newline();
  }

  float temp = readLM35();
  
  uart_puts("Temperature: ");
  uart_putf(temp, 2); 
  uart_puts(" C");
  uart_newline();

  runPWMMotor(temp);

  delay(2000);
}
