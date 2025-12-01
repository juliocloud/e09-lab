#include <avr/io.h>
#include <avr/interrupt.h>
#include <Arduino.h>

#define BOTAO_EMERGENCIA PD2
#define LED_EMERGENCIA PD7

#define CANAL_LM35 1

#define LIMITE_TEMPERATURA 30.0

const uint8_t CANAL_SENSOR_AGUA = 0;
const uint8_t AMOSTRAS_AGUA = 16;

const uint16_t AGUA_BRUTO_MIN = 370;
const uint16_t AGUA_BRUTO_MAX = 530;

#define IN1 PB0
#define IN2 PB1
#define IN3 PB2
#define IN4 PB3

const int PASSOS_POR_VOLTA = 512;

bool estaFechada = false;

volatile bool parada_de_emergencia_ativa = false;

#define BAUD_PRESCALER (((F_CPU / (9600 * 16UL))) - 1)

void uart_iniciar() {
  UBRR0H = (unsigned char)(BAUD_PRESCALER >> 8);
  UBRR0L = (unsigned char)BAUD_PRESCALER;

  UCSR0B = (1 << RXEN0) | (1 << TXEN0);
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void imprimir_char_unico(char dado) {
  while (!(UCSR0A & (1 << UDRE0)));
  UDR0 = dado;
}

void imprimir_na_uart(const char *s) {
  while (*s) {
    imprimir_char_unico(*s++);
  }
}

void nova_linha_uart() {
  imprimir_char_unico('\r');
  imprimir_char_unico('\n');
}

static void inverter(char *str, int len) {
  int i = 0, j = len - 1;
  while (i < j) {
    char temp = str[i];
    str[i] = str[j];
    str[j] = temp;
    i++;
    j--;
  }
}

void uart_imprimir_int(int n) {
  char str[12];
  int i = 0;
  bool isNegative = false;

  if (n == 0) {
    str[i++] = '0';
    str[i] = '\0';
    imprimir_na_uart(str);
    return;
  }

  if (n < 0) {
    isNegative = true;
    n = -n;
  }

  while (n != 0) {
    str[i++] = (n % 10) + '0';
    n /= 10;
  }

  if (isNegative) {
    str[i++] = '-';
  }

  str[i] = '\0';
  inverter(str, i);
  imprimir_na_uart(str);
}

void uart_imprimir_float(float f, uint8_t precisao) {
  if (f < 0) {
    imprimir_char_unico('-');
    f = -f;
  }

  long parte_int = (long)f;
  uart_imprimir_int(parte_int);

  if (precisao > 0) {
    imprimir_char_unico('.');
    float parte_frac = f - parte_int;

    for (uint8_t i = 0; i < precisao; i++) {
      parte_frac *= 10;
    }

    long valor_frac = (long)(parte_frac + 0.5);

    long valor_checar = 1;
    for (uint8_t i = 1; i < precisao; i++) {
      valor_checar *= 10;
      if (valor_frac < valor_checar) {
        imprimir_char_unico('0');
      }
    }

    uart_imprimir_int(valor_frac);
  }
}

void adc_iniciar() {
  ADMUX = (1 << REFS0);
  ADCSRA = (1 << ADEN) |
           (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

uint16_t adc_ler(uint8_t canal) {
  canal &= 0x07;
  ADMUX = (ADMUX & 0xF0) | canal;
  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC));
  uint8_t low = ADCL;
  uint8_t high = ADCH;
  return (uint16_t)(high << 8) | low;
}

#define AMOSTRAS_LM35 16

float ler_LM35() {
  uint32_t soma = 0;
  for (uint8_t i = 0; i < AMOSTRAS_LM35; i++) {
    soma += adc_ler(CANAL_LM35);
  }

  uint16_t media_bruta = soma / AMOSTRAS_LM35;

  float tensao = (media_bruta * 5.0) / 1023.0;
  return tensao * 100.0;
}

uint16_t ler_agua_bruto() {
  uint32_t soma = 0;
  for (uint8_t i = 0; i < AMOSTRAS_AGUA; i++) {
    soma += adc_ler(CANAL_SENSOR_AGUA);
  }
  return soma / AMOSTRAS_AGUA;
}

uint8_t agua_bruto_para_percentagem(uint16_t bruto) {
  if (bruto <= AGUA_BRUTO_MIN) return 0;
  if (bruto >= AGUA_BRUTO_MAX) return 100;
  uint16_t span = AGUA_BRUTO_MAX - AGUA_BRUTO_MIN;
  uint16_t adj = bruto - AGUA_BRUTO_MIN;
  return (uint32_t(adj) * 100UL) / span;
}

void motor_passo(int passo) {
  if (passo == 1) {
    PORTB = (PORTB & 0xF0) | (1 << IN1); delay(2);
    PORTB = (PORTB & 0xF0) | (1 << IN2); delay(2);
    PORTB = (PORTB & 0xF0) | (1 << IN3); delay(2);
    PORTB = (PORTB & 0xF0) | (1 << IN4); delay(2);
    PORTB = (PORTB & 0xF0) | (1 << IN1); delay(2);
  } else if (passo == -1) {
    PORTB = (PORTB & 0xF0) | (1 << IN4); delay(2);
    PORTB = (PORTB & 0xF0) | (1 << IN3); delay(2);
    PORTB = (PORTB & 0xF0) | (1 << IN2); delay(2);
    PORTB = (PORTB & 0xF0) | (1 << IN1); delay(2);
    PORTB = (PORTB & 0xF0) | (1 << IN4); delay(2);
  }
}

void rodar_voltas_motor_passo(int direcao, int passos) {
  for (int i = 0; i < passos; i++) {
    if (parada_de_emergencia_ativa) {
      imprimir_na_uart("Motor de passo interrompido por parada de emergencia.");
      nova_linha_uart();
      break;
    }
    motor_passo(direcao);
  }
  PORTB &= 0xF0;
}

bool pwm_iniciado = false;

void rodar_motor_PWM(float tempC) {
  if (parada_de_emergencia_ativa) {
    OCR0A = 0;
    pwm_iniciado = false;
    return;
  }

  if (tempC > LIMITE_TEMPERATURA) {
    if (!pwm_iniciado) {
      DDRD |= (1 << PD6);
      TCCR0A |= (1 << COM0A1) | (1 << WGM01) | (1 << WGM00);
      TCCR0B |= (1 << CS01) | (1 << CS00);
      TCCR0B &= ~(1 << CS02);

      imprimir_na_uart("Motor PWM iniciando (rampeando)...");
      nova_linha_uart();

      for (uint16_t ciclo_ativo = 0; ciclo_ativo <= 255; ciclo_ativo++) {
        OCR0A = ciclo_ativo;
        delay(12);
        if (parada_de_emergencia_ativa) {
          OCR0A = 0;
          return;
        }
      }

      pwm_iniciado = true;
    }

    OCR0A = 255;
    imprimir_na_uart("Motor PWM: LIGADO");
    nova_linha_uart();

  } else {
    OCR0A = 0;
    pwm_iniciado = false;
    imprimir_na_uart("Motor PWM: DESLIGADO (Temp >= 25 C)");
    nova_linha_uart();
  }
}

void parada_emergencia() {
  imprimir_na_uart("======================================");
  nova_linha_uart();
  imprimir_na_uart(">>> PARADA DE EMERGENCIA ATIVADA <<<");
  nova_linha_uart();
  imprimir_na_uart("======================================");
  nova_linha_uart();

  PORTB &= 0xF0;
  OCR0A = 0;

  uint32_t tempo_inicio = millis();

  while (millis() - tempo_inicio < 5000UL) {
    PORTD |= (1 << LED_EMERGENCIA);
    delay(250);
    PORTD &= ~(1 << LED_EMERGENCIA);
    delay(250);
  }

  PORTD &= ~(1 << LED_EMERGENCIA);

  parada_de_emergencia_ativa = false;
  imprimir_na_uart("Periodo de parada de emergencia encerrado. Retomando a operacao...");
  nova_linha_uart();
  nova_linha_uart();
}

ISR(INT0_vect) {
  parada_de_emergencia_ativa = true;
}

void setup() {
  uart_iniciar();

  DDRB |= (1 << IN1) | (1 << IN2) | (1 << IN3) | (1 << IN4);
  PORTB &= 0xF0;

  DDRD |= (1 << LED_EMERGENCIA);

  DDRD &= ~(1 << BOTAO_EMERGENCIA);
  PORTD |= (1 << BOTAO_EMERGENCIA);

  EICRA |= (1 << ISC01);
  EICRA &= ~(1 << ISC00);
  EIMSK |= (1 << INT0);

  sei();

  adc_iniciar();

  imprimir_na_uart("======================================");
  nova_linha_uart();
  imprimir_na_uart("Sensor de Agua/LM35 + Motores prontos!");
  nova_linha_uart();
  imprimir_na_uart("--------------------------------------");
  nova_linha_uart();
  imprimir_na_uart("Sensor de Agua OUT -> A0");
  nova_linha_uart();
  imprimir_na_uart("LM35 OUT -> A1");
  nova_linha_uart();
  imprimir_na_uart("Botao de Emergencia -> D2 (INT0)");
  nova_linha_uart();
  imprimir_na_uart("LED de Emergencia -> D7");
  nova_linha_uart();
  imprimir_na_uart("--------------------------------------");
  nova_linha_uart();

  estaFechada = false;
}

void loop() {
  if (parada_de_emergencia_ativa) {
    parada_emergencia();
    return;
  }

  uint16_t bruto = ler_agua_bruto();
  uint8_t pct = agua_bruto_para_percentagem(bruto);

  imprimir_na_uart("--------------------------------------");
  nova_linha_uart();
  imprimir_na_uart("Agua Bruto: ");
  uart_imprimir_int(bruto);

  imprimir_na_uart(" | Nivel: ");
  uart_imprimir_int(pct);
  imprimir_char_unico('%');

  imprimir_na_uart(" | Estado Atuador: ");
  imprimir_na_uart(estaFechada ? "FECHADO" : "ABERTO");
  nova_linha_uart();

  if (pct >= 50 && !estaFechada) {
    imprimir_na_uart(">> Nivel ALTO: Fechando (1 volta CW)...");
    nova_linha_uart();
    rodar_voltas_motor_passo(+1, PASSOS_POR_VOLTA);
    estaFechada = true;
  } else if (pct < 50 && estaFechada) {
    imprimir_na_uart(">> Nivel BAIXO: Abrindo (1 volta CCW)...");
    nova_linha_uart();
    rodar_voltas_motor_passo(-1, PASSOS_POR_VOLTA);
    estaFechada = false;
  } else {
    imprimir_na_uart(">> Sem movimento do atuador, estado ja correto.");
    nova_linha_uart();
  }
  nova_linha_uart();

  float temp = ler_LM35();

  imprimir_na_uart("Temperatura: ");
  uart_imprimir_float(temp, 2);
  imprimir_na_uart(" C");
  nova_linha_uart();

  rodar_motor_PWM(temp);
  nova_linha_uart();

  delay(2000);
}
