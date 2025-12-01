# Sistema de Controle Embarcado

O programa lê um sensor de nível de água, um LM35 para temperatura,
controla um motor de passo para abrir/fechar um atuador e um motor DC
via PWM.\
Há também um sistema de parada de emergência utilizando a interrupção
externa INT0.

A finalidade do projeto é acadêmica: praticar ADC, PWM, UART,
interrupções externas e controle de motor de passo usando registradores
AVR.

------------------------------------------------------------------------

## 1. Hardware Utilizado

-   ATmega328P / Arduino Uno\
-   Sensor analógico de nível de água (A0)\
-   Sensor de temperatura LM35 (A1)\
-   ULN2003 + motor de passo 28BYJ-48\
-   Motor DC controlado por PWM (PD6 / OC0A)\
-   Botão de emergência em PD2 (INT0)\
-   LED de emergência em PD7

------------------------------------------------------------------------

## 2. Comportamento Geral do Sistema

### Sensor de Água

-   Leitura baseada em 16 amostras (redução de ruído).
-   Conversão da média bruta para porcentagem usando limites calibrados.
-   Ações:
    -   Se o nível ≥ 50% e o atuador estiver aberto → fechar (1 volta
        CW).
    -   Se o nível \< 50% e o atuador estiver fechado → abrir (1 volta
        CCW).

### Sensor de Temperatura (LM35)

-   Leitura de 16 amostras no ADC1.
-   Conversão para graus Celsius (10 mV/°C).
-   Se temperatura \> 30°C → PWM é ativado e o motor DC rampa de 0 até
    255.
-   Se abaixo desse limite → PWM é desligado.

### Motor de Passo

-   Sequência full-step usando PB0--PB3.
-   512 passos por volta.
-   Bobinas são liberadas ao final da movimentação.

### Motor PWM (Timer0)

-   Saída no pino PD6 (OC0A).
-   Modo Fast PWM.
-   Prescaler configurado com CS01 + CS00.
-   Ciclo ativo controlado via OCR0A.

### Parada de Emergência (INT0)

-   Acionada por borda de descida no pino PD2.
-   Efeitos imediatos:
    -   Desliga o motor de passo e o PWM.
    -   Pisca o LED de emergência por 5 segundos.
-   Após isso, o sistema volta ao funcionamento normal.
-   A rotina de interrupção apenas seta a flag; o tratamento é feito na
    função principal.

### UART

-   Baud rate: 9600.
-   Exibe leituras de sensores, estado do atuador, estado do PWM e
    mensagens de emergência.
-   Implementada usando registradores (UDR0, UDRE0).

------------------------------------------------------------------------
## 3. Mapeamento de Pinos

| Componente            | Pino         | Observações          |
|-----------------------|--------------|-----------------------|
| Sensor de Água        | A0 (ADC0)    | Entrada analógica     |
| LM35                  | A1 (ADC1)    | Entrada analógica     |
| IN1 Stepper           | PB0          | ULN2003               |
| IN2 Stepper           | PB1          | ULN2003               |
| IN3 Stepper           | PB2          | ULN2003               |
| IN4 Stepper           | PB3          | ULN2003               |
| Motor PWM             | PD6 / OC0A   | Timer0 Fast PWM       |
| Botão de Emergência   | PD2 / INT0   | Ativo em nível baixo  |
| LED de Emergência     | PD7          | Saída digital         |
| UART TX               | PD1          | Serial 9600           |

------------------------------------------------------------------------

## 4. Estrutura do Código (Resumo)

-   `adc_iniciar()` e `adc_ler()` -- configuração e uso do ADC.\
-   `ler_LM35()` -- retorna temperatura em °C.\
-   `ler_agua_bruto()` e `agua_bruto_para_percentagem()` -- conversão do
    sensor de água.\
-   `motor_passo()` e `rodar_voltas_motor_passo()` -- controle do motor
    de passo.\
-   `rodar_motor_PWM()` -- controle do PWM baseado na temperatura.\
-   `parada_emergencia()` -- rotina de desligamento e espera.\
-   Funções UART para enviar caracteres, inteiros e floats.\
-   `ISR(INT0_vect)` -- interrupção da parada de emergência.

------------------------------------------------------------------------

## 5. Observações / Limitações

-   O ADC usa AVcc como referência; usar uma referência externa
    aumentaria a precisão.\
-   A sequência do motor de passo é básica, sem otimização para torque.\
-   O Timer0 é usado para PWM, então algumas funções do Arduino podem
    ter comportamento alterado.\
-   Os valores de calibração do sensor de água (370--530) variam
    conforme o módulo.

------------------------------------------------------------------------

## 6. Objetivo do Projeto

O projeto foi desenvolvido para reforçar conceitos de programação
embarcada em AVR, especialmente:

-   Manipulação direta de registradores\
-   Leitura e processamento via ADC\
-   Uso de interrupções externas\
-   Controle de motor de passo sem bibliotecas\
-   Geração de PWM com timers\
-   Implementação de UART sem `Serial.begin()`

O sistema é apenas para fins de estudo e não tem finalidade industrial.
