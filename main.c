#include <avr/io.h>
#include <Arduino.h>

// ---------------- NEW PIN DEFINITIONS ----------------

// Pin D2 (PD2) - Uses INT0 for external interrupt
#define EMERGENCY_BUTTON PD2
// Pin D7 (PD7) - For the emergency indicator LED
#define EMERGENCY_LED    PD7 

// ---------------- EXISTING CODE (unchanged) ----------------

const uint8_t  WATER_SENSOR_CHANNEL = 0;
const uint8_t  WATER_SAMPLES        = 16;

const uint16_t WATER_RAW_MIN = 370;
const uint16_t WATER_RAW_MAX = 580;

#define IN1 PB0
#define IN2 PB1
#define IN3 PB2
#define IN4 PB3

const int STEPS_PER_TURN = 512;

bool isClosed = false;

// ---------------- NEW GLOBAL VARIABLE ----------------
volatile bool emergency_stop_active = false;

void adc_init() {
  ADMUX = (1 << REFS0);
  ADCSRA = (1 << ADEN) |
           (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC));
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
    // ---------------- NEW: Check the stop flag ----------------
    if (emergency_stop_active) {
      Serial.println(F("Stepper interrupted by emergency stop."));
      break; // Exit the for loop immediately
    }
    // ---------------- END NEW ----------------
    
    stepMotor(dir);
  }
  PORTB &= 0xF0; // Disable coils
}

bool pwmStarted = false;

void runPWMMotor() {
  // If emergency stop is active, motor functions should immediately stop.
  if (emergency_stop_active) {
    OCR0A = 0;                     // Set PWM duty cycle to 0 (motor off)
    return;
  }
  
  // Configure PWM only once
  if (!pwmStarted) {
    DDRD |= (1 << PD6);            // PD6 = output
    TCCR0A |= (1 << COM0A1) | (1 << WGM01) | (1 << WGM00);
    TCCR0B |= (1 << CS01);         // prescaler 8 → stable PWM

    // Ramp from 0 → 255 over 3 seconds
    for (uint16_t duty = 0; duty <= 255; duty++) {
      OCR0A = duty;
      delay(12);                   // ~3000 ms total
    }

    pwmStarted = true;             // never ramp again
  }

  // After ramp, keep motor always ON
  OCR0A = 255;
}

// ---------------- NEW EMERGENCY STOP LOGIC ----------------

/**
 * @brief Handles the 5-second emergency stop sequence.
 * * Stops all motors, blinks the LED, and then resets the flag.
 */
void emergencyStop() {
  Serial.println(F(">>> EMERGENCY STOP ACTIVATED <<<"));
  
  // 1. Stop all motors immediately
  PORTB &= 0xF0; // Stepper motor coils off
  OCR0A = 0;     // PWM motor off

  uint32_t startTime = millis();
  
  // 2. Blink LED for 5 seconds (5000 ms)
  while (millis() - startTime < 5000UL) {
    PORTD |= (1 << EMERGENCY_LED);    // LED ON
    delay(250);
    PORTD &= ~(1 << EMERGENCY_LED);   // LED OFF
    delay(250);
  }
  
  // 3. Turn off LED for good
  PORTD &= ~(1 << EMERGENCY_LED); 
  
  // 4. Reset flag to allow normal operation to resume
  emergency_stop_active = false;
  Serial.println(F("Emergency stop period ended. Resuming operation..."));
}

/**
 * @brief Interrupt Service Routine for INT0 (Pin D2).
 * * This is triggered by a FALLING edge (button press, given pull-up).
 */
ISR(INT0_vect) {
  // Only set the flag; don't do complex work in the ISR
  emergency_stop_active = true;
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(9600);
  
  // Stepper Motor Pins
  DDRB |= (1 << IN1) | (1 << IN2) | (1 << IN3) | (1 << IN4);
  PORTB &= 0xF0;

  // Emergency LED Pin
  DDRD |= (1 << EMERGENCY_LED);
  
  // Emergency Button Pin (PD2)
  DDRD &= ~(1 << EMERGENCY_BUTTON);  // Set as INPUT
  PORTD |= (1 << EMERGENCY_BUTTON);   // Activate internal PULL-UP resistor

  // Configure External Interrupt INT0 (on D2/PD2)
  EICRA |= (1 << ISC01); // Set interrupt to trigger on FALLING edge
  EICRA &= ~(1 << ISC00);
  EIMSK |= (1 << INT0);  // Enable external interrupt INT0

  // Enable global interrupts
  sei();

  adc_init();

  Serial.println(F("Water sensor + ULN2003 stepper ready"));
  Serial.println(F("Water OUT -> A0, ULN IN1..IN4 -> D8..D11"));
  Serial.println(F("Emergency Button -> D2 | Emergency LED -> D7"));

  isClosed = false;
}


// ---------------- LOOP ----------------
void loop() {
  // Check if an emergency stop is active
  if (emergency_stop_active) {
    emergencyStop();
    return; // Skip the main logic for this loop iteration
  }
  
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
    runStepperTurns(+1, STEPS_PER_TURN);
    isClosed = true;
  } else if (pct < 50 && isClosed) {
    Serial.println(F(">> Level low: opening (1 turn CCW)..."));
    runStepperTurns(-1, STEPS_PER_TURN);
    isClosed = false;
  } else {
    Serial.println(F(">> No movement, state already correct."));
  }

  delay(2000);

  // ---------------- CALL NEW FUNCTION ----------------
  runPWMMotor();
}
