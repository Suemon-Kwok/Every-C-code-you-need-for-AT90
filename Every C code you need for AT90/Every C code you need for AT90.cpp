/**
 * AT90USB1287 Microcontroller Comprehensive Study Guide
 * Clock Speed: 8MHz
 * Optimization Level: None (-O0)
 * Programmer: Atmel-ICE.J41800063504
 * Interface: JTAG
 *
  * This code covers all topics from the semester:
 * - Bitmasking, Binary, Hexadecimal operations
 * - State Machines
 * - Analog to Digital Converter (ADC)
 * - USART Serial Communication
 * - Interrupt Service Routines (ISR)
 * - Real-Time Interrupts (RTI)
 * - 8-Bit Timer/Counter for PWM Signal Generation
 * - 16-Bit Timer/Counter for PWM Signal Generation
 * - Serial Peripheral Interface (SPI) Serial Communications
 * - Input Capture System
 * - Two Wire Interface (TWI) Serial Communications
 */

#include <avr/io.h>        // Contains I/O definitions for the specific device
#include <avr/interrupt.h> // Contains ISR (Interrupt Service Routine) declarations
#include <util/delay.h>    // Contains delay functions
#include <stdint.h>        // Standard integer types (uint8_t, etc.)
#include <stdbool.h>       // Boolean type support (true/false)

 // Define the CPU frequency (8MHz) for delay calculations
#define F_CPU 8000000UL

/**
 * === WEEK 1: INTRODUCTION TO MICROCONTROLLERS, BITMASKING, BINARY, HEXADECIMAL ===
 *
 * Port Registers Explained:
 * - DDRx: Data Direction Register - Controls whether pins are INPUT (0) or OUTPUT (1)
 * - PORTx: Output register - Sets output pins HIGH (1) or LOW (0) / Enables pull-up resistors for input pins
 * - PINx: Input register - Reads the current state of input pins
 *
 * Common Bitmasking Operations:
 * - Set a bit: register |= (1 << BIT_NUM)
 * - Clear a bit: register &= ~(1 << BIT_NUM)
 * - Toggle a bit: register ^= (1 << BIT_NUM)
 * - Check a bit: if (register & (1 << BIT_NUM))
 */

void bitmasking_examples(void) {
    // Set pin 3 of PORTB as output
    DDRB |= (1 << DDB3);   // Set bit 3 in Data Direction Register B

    // Set multiple pins (0, 1, and 2) of PORTD as output
    DDRD |= ((1 << DDD0) | (1 << DDD1) | (1 << DDD2));

    // Clear pin 4 of PORTB (set to LOW)
    PORTB &= ~(1 << PORTB4);

    // Toggle pin 5 of PORTC
    PORTC ^= (1 << PORTC5);

    // Check if pin 2 of PORTB is HIGH
    if (PINB & (1 << PINB2)) {
        // Pin is HIGH, do something
    }

    // Binary representation examples
    uint8_t binary_example = 0b10101010; // Binary representation (alternating 1s and 0s)

    // Hexadecimal representation examples
    uint8_t hex_example = 0xAA;          // Hexadecimal representation (same as 0b10101010)
    uint16_t hex_example2 = 0xBEEF;      // 16-bit hexadecimal value
}

/**
 * === WEEK 2: STATE MACHINES ===
 *
 * State machines are used to organize program flow by defining:
 * - States: Different operating modes of the system
 * - Transitions: Conditions that cause the system to change states
 * - Actions: Operations performed in each state or during transitions
 */

 // State machine example: Simple LED control with button
typedef enum {
    STATE_OFF,         // LED is off
    STATE_ON,          // LED is on
    STATE_BLINKING     // LED is blinking
} State_t;

void state_machine_example(void) {
    static State_t current_state = STATE_OFF;
    static uint8_t button_pressed = 0;

    // Read button state (assuming button is connected to pin 0 of PORTD)
    if (PIND & (1 << PIND0)) {
        // Button is pressed
        if (!button_pressed) {
            button_pressed = 1;  // Set the button pressed flag

            // State transition logic
            switch (current_state) {
            case STATE_OFF:
                current_state = STATE_ON;
                break;

            case STATE_ON:
                current_state = STATE_BLINKING;
                break;

            case STATE_BLINKING:
                current_state = STATE_OFF;
                break;
            }
        }
    }
    else {
        button_pressed = 0;  // Reset the button pressed flag
    }

    // State action logic
    switch (current_state) {
    case STATE_OFF:
        // Turn LED off (assuming LED is connected to pin 5 of PORTB)
        PORTB &= ~(1 << PORTB5);
        break;

    case STATE_ON:
        // Turn LED on
        PORTB |= (1 << PORTB5);
        break;

    case STATE_BLINKING:
        // Toggle LED
        PORTB ^= (1 << PORTB5);
        _delay_ms(500);  // Delay for blinking
        break;
    }
}

/**
 * === WEEK 3: ANALOG TO DIGITAL CONVERTER (ADC) ===
 *
 * ADC: Converts analog voltage levels into digital values that the microcontroller can process.
 *
 * Key ADC registers:
 * - ADMUX: ADC Multiplexer Selection Register - Selects reference voltage and input channel
 * - ADCSRA: ADC Control and Status Register A - Controls ADC operation and shows status
 * - ADCSRB: ADC Control and Status Register B - Additional ADC control bits
 * - ADCL/ADCH: ADC Data Registers - Hold the conversion result (low and high bytes)
 */

void adc_init(void) {
    // ADMUX - ADC Multiplexer Selection Register
    // REFS1:0 = 01 - Select AVCC as reference voltage
    // MUX4:0 = 00000 - Select ADC0 channel (pin PF0)
    ADMUX = (1 << REFS0);

    // ADCSRA - ADC Control and Status Register A
    // ADEN = 1 - Enable ADC
    // ADPS2:0 = 111 - Set prescaler to 128 (8MHz/128 = 62.5kHz ADC clock)
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

uint16_t adc_read(uint8_t channel) {
    // Select ADC channel (0-7)
    // ADMUX = (ADMUX & 0xF8) | (channel & 0x07);  // Clear lower 3 bits and set channel bits
    ADMUX = (ADMUX & 0xE0) | (channel & 0x1F);  // For AT90USB1287, which has more channels

    // Start conversion
    ADCSRA |= (1 << ADSC);  // ADSC: ADC Start Conversion bit

    // Wait for conversion to complete
    while (ADCSRA & (1 << ADSC));  // ADSC will be cleared when conversion is done

    // Read result (ADCL must be read first, then ADCH)
    return ADC;  // ADC is combination of ADCL and ADCH
}

/**
 * === WEEK 4: USART SERIAL COMMUNICATION ===
 *
 * USART: Universal Synchronous/Asynchronous Receiver/Transmitter
 * Used for serial communication with other devices
 *
 * Key USART registers for AT90USB1287:
 * - UBRRn: USART Baud Rate Register - Sets the baud rate
 * - UCSRnA: USART Control and Status Register A - Control flags and status flags
 * - UCSRnB: USART Control and Status Register B - Enables transmitter, receiver, interrupts
 * - UCSRnC: USART Control and Status Register C - Sets data format (data bits, parity, stop bits)
 * - UDRn: USART I/O Data Register - Buffer for transmitting and receiving data
 */

void usart_init(uint32_t baud) {
    // Calculate baud rate value based on CPU frequency
    uint16_t ubrr = F_CPU / 16 / baud - 1;

    // Set baud rate
    UBRR1H = (uint8_t)(ubrr >> 8);    // High byte
    UBRR1L = (uint8_t)ubrr;           // Low byte

    // Enable receiver and transmitter
    UCSR1B = (1 << RXEN1) | (1 << TXEN1);

    // Set frame format: 8 data bits, 1 stop bit, no parity
    UCSR1C = (1 << UCSZ11) | (1 << UCSZ10);
}

void usart_transmit(uint8_t data) {
    // Wait for empty transmit buffer
    // UDREn: USART Data Register Empty bit in UCSRnA
    while (!(UCSR1A & (1 << UDRE1)));

    // Put data into buffer, sends the data
    UDR1 = data;
}

uint8_t usart_receive(void) {
    // Wait for data to be received
    // RXCn: USART Receive Complete bit in UCSRnA
    while (!(UCSR1A & (1 << RXC1)));

    // Get and return received data from buffer
    return UDR1;
}

void usart_print_string(const char* str) {
    // Transmit each character in the string
    for (uint16_t i = 0; str[i] != '\0'; i++) {
        usart_transmit(str[i]);
    }
}

/**
 * === WEEK 5-6: INTERRUPT SERVICE ROUTINES (ISR) ===
 *
 * ISR: Special functions that execute when specific events (interrupts) occur
 *
 * Common interrupt sources:
 * - External interrupts (INTn pins)
 * - Timer/counter overflow or compare match
 * - ADC conversion complete
 * - USART data received
 *
 * Key registers:
 * - EICRA: External Interrupt Control Register A
 * - EICRB: External Interrupt Control Register B
 * - EIMSK: External Interrupt Mask Register - Enables individual external interrupts
 */

void external_interrupt_init(void) {
    // Configure INT0 (external interrupt 0, pin PD0) to trigger on falling edge
    // ISC01:0 = 10 - Falling edge of INT0 generates interrupt
    EICRA |= (1 << ISC01);
    EICRA &= ~(1 << ISC00);

    // Enable INT0 interrupt
    EIMSK |= (1 << INT0);

    // Enable global interrupts
    sei();  // Same as setting the I-bit in SREG
}

// External Interrupt 0 Service Routine
ISR(INT0_vect) {
    // Code to execute when INT0 interrupt occurs
    PORTB ^= (1 << PORTB5);  // Toggle LED on pin 5 of PORTB
}

// Button debouncing with interrupts
volatile uint8_t button_state = 0;
volatile uint32_t last_debounce_time = 0;
#define DEBOUNCE_DELAY 50  // milliseconds

// External Interrupt 0 Service Routine with debouncing
ISR(INT0_vect) {
    // Get current time (assuming a global millisecond counter is maintained)
    uint32_t current_time = /* get current time */;

    // Check if enough time has passed since the last interrupt
    if ((current_time - last_debounce_time) > DEBOUNCE_DELAY) {
        // Toggle button state
        button_state = !button_state;
        // Update debounce time
        last_debounce_time = current_time;
    }
}

/**
 * === WEEK 7: REAL-TIME INTERRUPTS (RTI) ===
 *
 * RTI: Interrupts that occur at regular intervals to maintain timing in the system
 *
 * Using Timer/Counter for regular timing:
 * - TCNTn: Timer/Counter register - Holds the current count value
 * - OCRnA/B: Output Compare Register A/B - Used for compare match
 * - TCCRnA/B: Timer/Counter Control Register A/B - Configures timer operation
 * - TIMSKn: Timer/Counter Interrupt Mask Register - Enables timer interrupts
 */

void timer1_init(void) {
    // Configure Timer/Counter 1 for CTC (Clear Timer on Compare Match) mode

    // TCCR1A - Timer/Counter Control Register 1A
    // WGM11:0 = 00 - Part of setting CTC mode (WGM13:0 = 0100)
    TCCR1A = 0;

    // TCCR1B - Timer/Counter Control Register 1B
    // WGM13:2 = 01 - Part of setting CTC mode (WGM13:0 = 0100)
    // CS12:0 = 100 - Set prescaler to 256 (8MHz/256 = 31.25kHz timer clock)
    TCCR1B = (1 << WGM12) | (1 << CS12);

    // Set compare match value for 1 Hz interrupt (31250 counts for 1 second)
    OCR1A = 31250;  // 8MHz / 256 / 1Hz = 31250

    // Enable Timer/Counter 1 Compare Match A interrupt
    TIMSK1 |= (1 << OCIE1A);  // OCIE1A: Output Compare A Match Interrupt Enable

    // Enable global interrupts
    sei();
}

// Timer/Counter 1 Compare Match A Interrupt Service Routine
ISR(TIMER1_COMPA_vect) {
    // This code will execute approximately once per second
    static uint8_t seconds = 0;

    // Increment counter
    seconds++;

    // Toggle LED to indicate 1-second intervals
    PORTB ^= (1 << PORTB5);

    // Print time via USART (if USART is initialized)
    usart_transmit('T');
    usart_transmit(':');
    usart_transmit(' ');
    usart_transmit('0' + seconds / 10);   // Tens digit
    usart_transmit('0' + seconds % 10);   // Ones digit
    usart_transmit('\r');
    usart_transmit('\n');

    // Reset counter after 60 seconds
    if (seconds >= 60) {
        seconds = 0;
    }
}

/**
 * === WEEK 8: 8-BIT TIMER/COUNTER FOR PWM SIGNAL GENERATION ===
 *
 * PWM: Pulse Width Modulation - Technique to generate analog-like signal using digital output
 * Used for controlling motor speed, LED brightness, etc.
 *
 * Timer/Counter 0 is typically used for 8-bit PWM:
 * - TCCR0A: Timer/Counter Control Register 0A - Sets PWM mode and output behavior
 * - TCCR0B: Timer/Counter Control Register 0B - Sets clock source (prescaler)
 * - OCR0A/B: Output Compare Register 0A/B - Sets PWM duty cycle
 */

void pwm_init(void) {
    // Configure Timer/Counter 0 for Fast PWM mode

    // Set pin PB7 (OC0A) as output for PWM
    DDRB |= (1 << DDB7);

    // TCCR0A - Timer/Counter Control Register 0A
    // COM0A1:0 = 10 - Clear OC0A on compare match, set at BOTTOM (non-inverting mode)
    // WGM01:0 = 11 - Part of setting Fast PWM mode (WGM02:0 = 011)
    TCCR0A = (1 << COM0A1) | (1 << WGM01) | (1 << WGM00);

    // TCCR0B - Timer/Counter Control Register 0B
    // WGM02 = 0 - Part of setting Fast PWM mode (WGM02:0 = 011)
    // CS02:0 = 011 - Set prescaler to 64 (8MHz/64 = 125kHz PWM frequency)
    TCCR0B = (1 << CS01) | (1 << CS00);

    // Set initial PWM duty cycle to 50%
    OCR0A = 127;  // 8-bit PWM, so range is 0-255
}

void set_pwm_duty_cycle(uint8_t duty_cycle) {
    // Set PWM duty cycle (0-255)
    OCR0A = duty_cycle;
}

/**
 * Main function with initialization and main loop
 */
int main(void) {
    // Initialize ports
    // Set LED pins as outputs
    DDRB |= (1 << DDB5);  // LED on PB5

    // Set button pins as inputs with pull-up resistors
    DDRD &= ~(1 << DDD0);  // Button on PD0
    PORTD |= (1 << PORTD0);  // Enable pull-up resistor

    // Initialize modules
    adc_init();
    usart_init(9600);  // 9600 baud
    external_interrupt_init();
    timer1_init();
    pwm_init();

    // Send startup message
    usart_print_string("AT90USB1287 Initialized\r\n");

    // Main loop
    while (1) {
        // Example: Read potentiometer value from ADC0 and set PWM duty cycle
        uint16_t adc_value = adc_read(0);
        uint8_t duty_cycle = adc_value >> 2;  // Convert 10-bit ADC value to 8-bit PWM value
        set_pwm_duty_cycle(duty_cycle);

        // Example: Use state machine to control LED behavior
        state_machine_example();

        // Example: Send ADC value via USART
        char buffer[20];
        // This is just an example - normally you would use sprintf, but we're keeping it simple
        usart_transmit('A');
        usart_transmit('D');
        usart_transmit('C');
        usart_transmit(':');
        usart_transmit(' ');
        // Simple way to convert 16-bit number to decimal string
        for (int16_t i = 10000; i > 0; i /= 10) {
            usart_transmit('0' + ((adc_value / i) % 10));
        }
        usart_transmit('\r');
        usart_transmit('\n');

        // Small delay
        _delay_ms(100);
    }

    return 0;  // Never reached
}

/*
 * === REGISTER DEFINITIONS AND THEIR MEANINGS ===
 *
 * Port Registers:
 * - DDRx: Data Direction Register for port x - Sets whether each pin is input or output
 * - PORTx: Output register for port x - Sets output pins HIGH/LOW or enables pull-up for inputs
 * - PINx: Input register for port x - Reads the current state of input pins
 *
 * ADC Registers:
 * - ADMUX: ADC Multiplexer Selection Register
 *   - REFS1:0: Reference Selection Bits - Selects voltage reference
 *   - MUX4:0: Analog Channel Selection Bits - Selects which analog input to use
 * - ADCSRA: ADC Control and Status Register A
 *   - ADEN: ADC Enable - Enables the ADC
 *   - ADSC: ADC Start Conversion - Starts an ADC conversion
 *   - ADATE: ADC Auto Trigger Enable - Enables auto-triggering
 *   - ADIF: ADC Interrupt Flag - Set when conversion completes
 *   - ADIE: ADC Interrupt Enable - Enables ADC interrupts
 *   - ADPS2:0: ADC Prescaler Select Bits - Sets ADC clock prescaler
 *
 * USART Registers:
 * - UBRRn: USART Baud Rate Register - Sets the baud rate
 * - UCSRnA: USART Control and Status Register A
 *   - RXCn: Receive Complete - Set when data is available
 *   - TXCn: Transmit Complete - Set when transmission is complete
 *   - UDREn: Data Register Empty - Set when transmit buffer is empty
 * - UCSRnB: USART Control and Status Register B
 *   - RXENn: Receiver Enable - Enables USART receiver
 *   - TXENn: Transmitter Enable - Enables USART transmitter
 *   - RXCIEn: RX Complete Interrupt Enable - Enables RX interrupt
 * - UCSRnC: USART Control and Status Register C
 *   - UMSELn1:0: USART Mode Select - Sets synch/asynch mode
 *   - UPMn1:0: Parity Mode - Sets parity mode
 *   - USBSn: Stop Bit Select - Sets number of stop bits
 *   - UCSZn2:0: Character Size - Sets data bit size
 * - UDRn: USART I/O Data Register - Buffer for transmitting and receiving data
 *
 * Timer/Counter Registers:
 * - TCNTn: Timer/Counter Register - Holds current timer count
 * - OCRnA/B: Output Compare Register A/B - Holds compare values
 * - TCCRnA: Timer/Counter Control Register A
 *   - COMnA1:0/COMnB1:0: Compare Output Mode - Controls OC pin behavior
 *   - WGMn1:0: Waveform Generation Mode bits (lower part) - Sets timer mode
 * - TCCRnB: Timer/Counter Control Register B
 *   - WGMn3:2: Waveform Generation Mode bits (upper part) - Sets timer mode
 *   - CSn2:0: Clock Select - Sets timer clock source and prescaler
 * - TIMSKn: Timer/Counter Interrupt Mask Register
 *   - TOIEn: Timer Overflow Interrupt Enable - Enables overflow interrupt
 *   - OCIEnA/B: Output Compare A/B Match Interrupt Enable - Enables compare match interrupts
 *
 * Interrupt Registers:
 * - EICRA/EICRB: External Interrupt Control Register A/B
 *   - ISCn1:0: Interrupt Sense Control - Sets trigger condition
 * - EIMSK: External Interrupt Mask Register
 *   - INTn: External Interrupt Request Enable - Enables specific external interrupts
 * - SREG: Status Register
 *   - I-bit (bit 7): Global Interrupt Enable - Enables all interrupts
 */

 /*************************************
  * TOPIC 9: 16-BIT TIMER/COUNTER FOR PWM
  *************************************/
  /*
   * 16-bit Timer/Counter:
   * Provides more resolution than 8-bit timers (0-65535 vs. 0-255)
   * Useful for precise timing and high-resolution PWM
   *
   * Key Timer1 Registers:
   * - TCCR1A (Timer/Counter1 Control Register A):
   *   COM1A1:0, COM1B1:0, COM1C1:0: Compare Output Mode bits
   *   WGM11:10: Waveform Generation Mode bits (lower 2 bits)
   *
   * - TCCR1B (Timer/Counter1 Control Register B):
   *   ICNC1: Input Capture Noise Canceler bit
   *   ICES1: Input Capture Edge Select bit
   *   WGM13:12: Waveform Generation Mode bits (upper 2 bits)
   *   CS12:10: Clock Select bits (prescaler)
   *
   * - OCR1A, OCR1B, OCR1C (Output Compare Registers):
   *   16-bit compare match values (0-65535)
   *
   * - ICR1 (Input Capture Register):
   *   Used to capture timer value on external events
   *   Can also be used as TOP value in some PWM modes
   *
   * PWM Modes (WGM13:0):
   * - Fast PWM (WGM13:0 = 1110): Counter counts from BOTTOM to TOP then resets
   * - Phase Correct PWM (WGM13:0 = 1010): Counter counts from BOTTOM to TOP then back to BOTTOM
   */
void pwm_16bit_init(void) {
    // Set PB5 (OC1A) as output
    // OC1A is the Timer1 Compare Match A output pin
    DDRB |= (1 << DDB5);

    // Fast PWM mode, 10-bit resolution, non-inverting
    // COM1A1 = 1, COM1A0 = 0: Clear OC1A on compare match, set at BOTTOM
    // WGM11:10 = 11: Used with WGM13:12 for Fast PWM mode with 10-bit resolution (TOP=0x03FF)
    TCCR1A = (1 << COM1A1) | (1 << WGM11) | (1 << WGM10);

    // WGM13:12 = 01: Used with WGM11:10 for Fast PWM mode with 10-bit resolution
    // CS11 = 1: Prescaler 8
    // PWM Frequency = F_CPU / (Prescaler * 1024)
    // = 8,000,000 / (8 * 1024) = 976.56 Hz
    TCCR1B = (1 << WGM12) | (1 << CS11);

    // Set initial duty cycle (50% of 1023)
    // 512/1023 ≈ 50% duty cycle
    OCR1A = 512;
}

void pwm_16bit_set_duty(uint16_t duty) {
    // Set duty cycle (0-1023 for 10-bit PWM)
    // duty = 0: 0% duty cycle (always off)
    // duty = 1023: ~100% duty cycle (always on)
    OCR1A = duty;
}/**
 * AT90USB1287 Comprehensive Microcontroller Code
 * Clock: 8MHz
 * Programmer: Atmel-ICE.J41800063504
 * Interface: JTAG
 * Optimization: None (-O0)
 */

#define F_CPU 8000000UL  // 8MHz clock speed definition for delay functions

#include <avr/io.h>       // Provides I/O port definitions and register access
#include <avr/interrupt.h> // Provides ISR() macro and interrupt vector definitions
#include <util/delay.h>   // Provides _delay_ms() and _delay_us() functions
#include <stdbool.h>      // Provides bool data type (true/false)
#include <stdint.h>       // Provides fixed-width integer types (uint8_t, uint16_t, etc.)

 /*************************************
  * TOPIC 1: MICROCONTROLLER BASICS
  *
  * - Binary: Base-2 number system (0,1)
  *   Examples: 0b00001010 = 10 decimal
  *
  * - Hexadecimal: Base-16 number system (0-9,A-F)
  *   Examples: 0x0A = 10 decimal, 0xFF = 255 decimal
  *
  * - Bitmasking: Using bitwise operations to manipulate specific bits
  *   Set bit:   PORTA |= (1 << PA0);    // Sets bit 0 of PORTA to 1
  *   Clear bit: PORTA &= ~(1 << PA0);   // Clears bit 0 of PORTA to 0
  *   Toggle:    PORTA ^= (1 << PA0);    // Inverts bit 0 of PORTA
  *   Check bit: if (PINA & (1 << PA0))  // Tests if bit 0 of PINA is 1
  *************************************/
  /*
   * Port Configuration:
   * - DDRx (Data Direction Register): Sets pin direction (1=output, 0=input)
   *   Example: DDRB - Data Direction Register for Port B
   *
   * - PORTx (Port Output Register): Sets output values or enables pull-up resistors
   *   Example: PORTB - Port B Output Register
   *
   * - PINx (Port Input Register): Reads input values (even for pins configured as outputs)
   *   Example: PINB - Port B Input Register
   *
   * - DDBn: Individual bit in DDRx register (Data Direction Bit for pin n)
   * - PORTxn: Individual bit in PORTx register (Output value for pin n)
   * - PINxn: Individual bit in PINx register (Input value from pin n)
   */
void initialize_ports(void) {
    // Set PORTB pins 0-3 as outputs (e.g., for LEDs)
    // 0b00001111 = Set pins 0-3 as outputs, leave others unchanged
    DDRB |= (1 << DDB0) | (1 << DDB1) | (1 << DDB2) | (1 << DDB3);

    // Set PORTD pins 0-3 as inputs with pull-ups (e.g., for buttons)
    // ~0b00001111 = 0b11110000 = Clear pins 0-3 (set as inputs)
    DDRD &= ~((1 << DDD0) | (1 << DDD1) | (1 << DDD2) | (1 << DDD3));
    PORTD |= (1 << PORTD0) | (1 << PORTD1) | (1 << PORTD2) | (1 << PORTD3);
}

/*************************************
 * TOPIC 2: STATE MACHINES
 *************************************/
 /*
  * State Machine Implementation:
  * - Define states as enumerated types
  * - Use switch-case statements to handle state transitions
  * - Store current state as a global variable
  */
typedef enum {
    STATE_IDLE,
    STATE_ACTIVE,
    STATE_ERROR,
    STATE_SHUTDOWN
} SystemState;

volatile SystemState current_state = STATE_IDLE;

void state_machine_tick(void) {
    // Read inputs
    bool button_pressed = !(PIND & (1 << PIND0)); // Active LOW button

    // State transition logic
    switch (current_state) {
    case STATE_IDLE:
        if (button_pressed) {
            current_state = STATE_ACTIVE;
            // Perform actions for entering active state
            PORTB |= (1 << PORTB0); // Turn on LED
        }
        break;

    case STATE_ACTIVE:
        if (button_pressed) {
            current_state = STATE_IDLE;
            // Perform actions for returning to idle state
            PORTB &= ~(1 << PORTB0); // Turn off LED
        }
        break;

    case STATE_ERROR:
        // Error handling logic
        break;

    case STATE_SHUTDOWN:
        // Shutdown logic
        break;
    }
}

/*************************************
 * TOPIC 3: ANALOG TO DIGITAL CONVERTER (ADC)
 *************************************/
 /*
  * ADC (Analog-to-Digital Converter):
  * Converts analog voltage to digital values with 10-bit resolution
  *
  * Key Registers:
  * - ADMUX (ADC Multiplexer Selection Register):
  *   Controls reference voltage selection and input channel selection
  *   REFS1:0 - Reference voltage selection bits
  *   MUX4:0 - Channel selection bits
  *
  * - ADCSRA (ADC Control and Status Register A):
  *   ADEN - ADC Enable bit
  *   ADSC - ADC Start Conversion bit
  *   ADATE - ADC Auto Trigger Enable bit
  *   ADIF - ADC Interrupt Flag
  *   ADIE - ADC Interrupt Enable
  *   ADPS2:0 - ADC Prescaler Select bits
  *
  * - ADC - ADC Data Register (combines ADCL and ADCH)
  *   Stores the 10-bit conversion result
  */
void adc_init(void) {
    // Select AVcc as reference voltage, channel 0 (ADC0)
    // REFS0 = 1: Use AVcc with external capacitor at AREF pin
    // REFS1 = 0, MUX4:0 = 00000: Select channel 0
    ADMUX = (1 << REFS0);

    // Enable ADC, set prescaler to 128 (62.5kHz @ 8MHz)
    // ADEN = 1: Enable ADC
    // ADPS2:0 = 111: Prescaler division factor = 128
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

uint16_t adc_read(uint8_t channel) {
    // Select ADC channel with safety mask
    // Keep upper 4 bits (REFS1:0, ADLAR, MUX4) and set lower 4 bits to channel
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);

    // Start single conversion
    // ADSC = 1: Start conversion
    ADCSRA |= (1 << ADSC);

    // Wait for conversion to complete (ADSC becomes 0 when done)
    while (ADCSRA & (1 << ADSC));

    // Return ADC result (10-bit value)
    return ADC;
}

/*************************************
 * TOPIC 4: USART SERIAL COMMUNICATION
 *************************************/
 /*
  * USART (Universal Synchronous/Asynchronous Receiver/Transmitter):
  * Serial communication interface for exchanging data with other devices
  *
  * Key Registers:
  * - UBRRn (USART Baud Rate Register):
  *   UBRR0H and UBRR0L: Control the baud rate of USART0
  *
  * - UCSRnA (USART Control and Status Register A):
  *   RXC0: Receive Complete Flag
  *   TXC0: Transmit Complete Flag
  *   UDRE0: Data Register Empty Flag (ready to receive new data)
  *   FE0: Frame Error Flag
  *   DOR0: Data OverRun flag
  *   UPE0: USART Parity Error
  *   U2X0: Double Transmission Speed
  *
  * - UCSRnB (USART Control and Status Register B):
  *   RXCIE0: RX Complete Interrupt Enable
  *   TXCIE0: TX Complete Interrupt Enable
  *   UDRIE0: Data Register Empty Interrupt Enable
  *   RXEN0: Receiver Enable
  *   TXEN0: Transmitter Enable
  *
  * - UCSRnC (USART Control and Status Register C):
  *   UMSEL01:0: USART Mode Select (async/sync)
  *   UPM01:0: Parity Mode bits
  *   USBS0: Stop Bit Select (1 or 2 stop bits)
  *   UCSZ01:0: Character Size bits
  *
  * - UDRn (USART Data Register):
  *   UDR0: Transmit/receive data register
  */
void usart_init(uint32_t baud_rate) {
    // Calculate UBRR value for baud rate
    // Formula: UBRR = (F_CPU / (16 * baud_rate)) - 1
    uint16_t ubrr = (F_CPU / (16UL * baud_rate)) - 1;

    // Set baud rate by writing to UBRR0H (high byte) and UBRR0L (low byte)
    UBRR0H = (uint8_t)(ubrr >> 8);  // High byte of UBRR value
    UBRR0L = (uint8_t)ubrr;         // Low byte of UBRR value

    // Enable transmitter and receiver
    // TXEN0 = 1: Enable transmitter
    // RXEN0 = 1: Enable receiver
    UCSR0B = (1 << TXEN0) | (1 << RXEN0);

    // Set frame format: 8 data bits, 1 stop bit, no parity
    // UCSZ01:0 = 11: 8-bit data
    // USBS0 = 0, UPM01:0 = 00: 1 stop bit, no parity
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void usart_transmit(uint8_t data) {
    // Wait for empty transmit buffer
    // UDRE0 = 1 when transmit buffer is empty
    while (!(UCSR0A & (1 << UDRE0)));

    // Put data into buffer, sends the data
    UDR0 = data;
}

uint8_t usart_receive(void) {
    // Wait for data to be received
    // RXC0 = 1 when unread data exists in receive buffer
    while (!(UCSR0A & (1 << RXC0)));

    // Get and return received data from buffer
    return UDR0;
}

void usart_transmit_string(const char* str) {
    // Send each character in string until null terminator
    while (*str) {
        usart_transmit(*str++);
    }
}

/*************************************
 * TOPIC 5-6: INTERRUPT SERVICE ROUTINES (ISR)
 *************************************/
 /*
  * ISR (Interrupt Service Routines):
  * Functions that execute when specific hardware events occur
  *
  * Key Registers for External Interrupts:
  * - EICRA (External Interrupt Control Register A):
  *   ISC11:0, ISC01:0: External Interrupt Sense Control bits
  *     00: Low level generates interrupt
  *     01: Any logical change generates interrupt
  *     10: Falling edge generates interrupt
  *     11: Rising edge generates interrupt
  *
  * - EIMSK (External Interrupt Mask Register):
  *   INTn: External Interrupt Request Enable bits
  *
  * Global Interrupt Control:
  * - sei(): Enable global interrupts (Set Global Interrupt Enable bit)
  * - cli(): Disable global interrupts (Clear Global Interrupt Enable bit)
  *
  * Interrupt Vectors:
  * - INT0_vect: External Interrupt Request 0
  * - TIMER0_COMPA_vect: Timer/Counter0 Compare Match A
  * - ADC_vect: ADC Conversion Complete
  */
void interrupt_init(void) {
    // Configure external interrupt INT0 (falling edge)
    // ISC01 = 1, ISC00 = 0: Interrupt on falling edge
    EICRA |= (1 << ISC01);  // Falling edge

    // Enable external interrupt INT0
    EIMSK |= (1 << INT0);   // Enable INT0

    // Enable global interrupts
    sei();  // Set Global Interrupt Enable bit in SREG
}

// External Interrupt 0 ISR
// This function automatically runs when INT0 is triggered
ISR(INT0_vect) {
    // Handle button press interrupt
    PORTB ^= (1 << PORTB1); // Toggle LED (XOR operation)
}

// Example: ADC Conversion Complete ISR
// This function automatically runs when ADC conversion completes
// (if ADC interrupt is enabled with ADCSRA |= (1 << ADIE))
ISR(ADC_vect) {
    // Handle ADC conversion complete
    uint16_t adc_value = ADC;
    // Process ADC value
}

/*************************************
 * TOPIC 7: REAL-TIME INTERRUPTS (RTI)
 *************************************/
 /*
  * RTI (Real-Time Interrupts):
  * Timer-based interrupts that occur at regular intervals for time-critical operations
  *
  * Key Timer0 Registers:
  * - TCCR0A (Timer/Counter Control Register A):
  *   COM0A1:0, COM0B1:0: Compare Output Mode bits
  *   WGM01:0: Waveform Generation Mode bits
  *     00: Normal mode
  *     01: PWM, Phase Correct mode
  *     10: CTC (Clear Timer on Compare Match) mode
  *     11: Fast PWM mode
  *
  * - TCCR0B (Timer/Counter Control Register B):
  *   FOC0A, FOC0B: Force Output Compare bits
  *   WGM02: Waveform Generation Mode bit (in conjunction with WGM01:0)
  *   CS02:0: Clock Select bits (prescaler)
  *     000: No clock source (Timer stopped)
  *     001: No prescaling (clk_IO)
  *     010: clk_IO/8
  *     011: clk_IO/64
  *     100: clk_IO/256
  *     101: clk_IO/1024
  *
  * - OCR0A (Output Compare Register A):
  *   Compare match value for Timer/Counter0
  *
  * - TIMSK0 (Timer/Counter0 Interrupt Mask Register):
  *   OCIE0A: Output Compare A Match Interrupt Enable bit
  *   OCIE0B: Output Compare B Match Interrupt Enable bit
  *   TOIE0: Timer Overflow Interrupt Enable bit
  */
void rti_init(void) {
    // Configure Timer/Counter 0 for RTI
    // WGM01 = 1, WGM00 = 0, WGM02 = 0: CTC mode (Clear Timer on Compare Match)
    TCCR0A = (1 << WGM01);

    // CS02 = 1, CS00 = 1: Prescaler 1024
    // 8MHz / 1024 = 7812.5 Hz tick rate
    TCCR0B = (1 << CS02) | (1 << CS00);

    // Set compare match value for approximately 100ms interval
    // (8MHz / 1024 prescaler / 10Hz = 781.25)
    OCR0A = 78; // Slightly under 100ms (78 * 1024 / 8MHz = 0.09984s ≈ 100ms)

    // Enable compare match interrupt
    // OCIE0A = 1: Enable Timer/Counter0 Compare Match A interrupt
    TIMSK0 |= (1 << OCIE0A);
}

// Timer0 Compare Match A ISR (100ms Real-Time Interrupt)
// This function automatically runs when Timer0 reaches OCR0A value
ISR(TIMER0_COMPA_vect) {
    static uint8_t counter = 0;

    // Increment counter
    counter++;

    // Execute task every 1 second (10 * 100ms)
    if (counter >= 10) {
        counter = 0;
        PORTB ^= (1 << PORTB2); // Toggle LED
    }
}

/*************************************
 * TOPIC 8: 8-BIT TIMER/COUNTER FOR PWM
 *************************************/
 /*
  * PWM (Pulse Width Modulation):
  * Technique to generate square waves with varying duty cycles
  * Used for motor control, LED brightness control, DAC, etc.
  *
  * 8-bit Timer0 PWM Modes:
  * - Fast PWM: Counter counts from 0 to 255 and resets to 0
  *   Good for high frequency applications
  *
  * - Phase Correct PWM: Counter counts from 0 to 255 and back to 0
  *   Better for motor control due to symmetric operation
  *
  * Key Registers:
  * - TCCR0A, TCCR0B: Already described in RTI section
  *   COM0A1:0 bits control PWM output behavior:
  *     00: Normal port operation, OC0A disconnected
  *     10: Clear OC0A on compare match, set at BOTTOM (non-inverting mode)
  *     11: Set OC0A on compare match, clear at BOTTOM (inverting mode)
  *
  * - OCR0A: Controls duty cycle (0-255)
  *   0: 0% duty cycle, 255: 100% duty cycle (for non-inverting mode)
  */
void pwm_8bit_init(void) {
    // Set PB3 (OC0A) as output
    // OC0A is the Timer0 Compare Match A output pin
    DDRB |= (1 << DDB3);

    // Fast PWM mode, non-inverting
    // COM0A1 = 1, COM0A0 = 0: Clear OC0A on compare match, set at BOTTOM
    // WGM01 = 1, WGM00 = 1: Fast PWM mode
    TCCR0A = (1 << COM0A1) | (1 << WGM01) | (1 << WGM00);

    // Start timer with prescaler 64
    // CS01 = 1, CS00 = 1: clk_IO/64 prescaler
    // PWM Frequency = F_CPU / (Prescaler * 256)
    // = 8,000,000 / (64 * 256) = 488.28 Hz
    TCCR0B = (1 << CS01) | (1 << CS00);

    // Set initial duty cycle (50%)
    // 127/255 ≈ 50% duty cycle
    OCR0A = 127;
}

void pwm_8bit_set_duty(uint8_t duty) {
    // Set duty cycle (0-255)
    // duty = 0: 0% duty cycle (always off)
    // duty = 255: 100% duty cycle (always on)
    OCR0A = duty;
}

/*************************************
 * TOPIC 9: 16-BIT TIMER/COUNTER FOR PWM
 *************************************/
 /*
  * 16-bit PWM Configuration:
  * - Configure timer mode (Fast PWM, Phase Correct PWM)
  * - Set prescaler for desired PWM frequency
  * - Configure compare output mode for PWM generation
  * - Set OCR1A/OCR1B for duty cycle control
  */
void pwm_16bit_init(void) {
    // Set PB5 (OC1A) as output
    DDRB |= (1 << DDB5);

    // Fast PWM mode, 10-bit resolution, non-inverting
    TCCR1A = (1 << COM1A1) | (1 << WGM11) | (1 << WGM10);
    TCCR1B = (1 << WGM12) | (1 << CS11);  // Prescaler 8

    // Set initial duty cycle (50% of 1023)
    OCR1A = 512;
}

void pwm_16bit_set_duty(uint16_t duty) {
    OCR1A = duty;
}

/*************************************
 * TOPIC 10: SERIAL PERIPHERAL INTERFACE (SPI)
 *************************************/
 /*
  * SPI (Serial Peripheral Interface):
  * Synchronous serial communication protocol for short-distance communication
  * Uses 4 wires: MOSI, MISO, SCK, SS
  *   - MOSI (Master Out Slave In): Data from master to slave
  *   - MISO (Master In Slave Out): Data from slave to master
  *   - SCK (Serial Clock): Clock signal generated by master
  *   - SS (Slave Select): Signal to select specific slave device
  *
  * Key SPI Registers:
  * - SPCR (SPI Control Register):
  *   SPIE: SPI Interrupt Enable bit
  *   SPE: SPI Enable bit
  *   DORD: Data Order bit (MSB first or LSB first)
  *   MSTR: Master/Slave Select bit (1=master, 0=slave)
  *   CPOL: Clock Polarity bit (idle high or idle low)
  *   CPHA: Clock Phase bit (sample on leading or trailing edge)
  *   SPR1:0: SPI Clock Rate Select bits
  *
  * - SPSR (SPI Status Register):
  *   SPIF: SPI Interrupt Flag
  *   WCOL: Write Collision Flag
  *   SPI2X: Double SPI Speed bit
  *
  * - SPDR (SPI Data Register):
  *   Holds the data to be transmitted/received
  */
void spi_master_init(void) {
    // Set MOSI (PB2), SCK (PB1), SS (PB0) as outputs
    // MOSI: Master Out Slave In data line
    // SCK: Serial Clock output from master
    // SS: Slave Select (active low)
    DDRB |= (1 << DDB2) | (1 << DDB1) | (1 << DDB0);

    // Enable SPI, Master mode, set clock rate to fck/16
    // SPE = 1: SPI Enable
    // MSTR = 1: Master mode
    // SPR0 = 1: fck/16 clock rate
    // SPI Frequency = F_CPU / 16 = 8MHz / 16 = 500kHz
    SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
}

uint8_t spi_master_transmit(uint8_t data) {
    // Start transmission by writing data to SPDR
    SPDR = data;

    // Wait for transmission to complete
    // SPIF flag is set when a serial transfer is complete
    while (!(SPSR & (1 << SPIF)));

    // Return received data (automatically stored in SPDR after transmission)
    return SPDR;
}

/*************************************
 * TOPIC 11: INPUT CAPTURE SYSTEM
 *************************************/
 /*
  * Input Capture:
  * System to record the time when an external event occurs
  * Useful for measuring frequency, pulse width, or time between events
  *
  * Input Capture Operation:
  * 1. Timer runs continuously
  * 2. When an event occurs on ICP pin, timer value is stored in ICR register
  * 3. Input Capture Flag (ICF) is set
  * 4. Optional interrupt can be triggered
  *
  * Key Input Capture Registers:
  * - TCCR1B (Timer/Counter1 Control Register B):
  *   ICNC1: Input Capture Noise Canceler bit (1=enabled)
  *   ICES1: Input Capture Edge Select bit (1=rising edge, 0=falling edge)
  *
  * - ICR1 (Input Capture Register):
  *   16-bit register that stores timer value when capture occurs
  *
  * - TIMSK1 (Timer/Counter1 Interrupt Mask Register):
  *   ICIE1: Input Capture Interrupt Enable bit
  */
void input_capture_init(void) {
    // Set ICP1 (PD4) as input
    // ICP1: Input Capture Pin for Timer/Counter1
    DDRD &= ~(1 << DDD4);

    // Configure Timer/Counter 1 in normal mode
    // Normal mode: Timer counts from 0 to 0xFFFF and overflows
    TCCR1A = 0;

    // Enable input capture noise canceler, trigger on rising edge, prescaler 8
    // ICNC1 = 1: Enable Input Capture Noise Canceler (4 consecutive equal samples)
    // ICES1 = 1: Capture on rising edge
    // CS11 = 1: clk_IO/8 prescaler
    TCCR1B = (1 << ICNC1) | (1 << ICES1) | (1 << CS11);

    // Enable input capture interrupt
    // ICIE1 = 1: Enable Input Capture Interrupt
    TIMSK1 |= (1 << ICIE1);
}

// Input Capture ISR
// This function automatically runs when an input capture event occurs
ISR(TIMER1_CAPT_vect) {
    // Read the captured timestamp
    uint16_t capture_value = ICR1;

    // Process captured value (e.g., measure frequency or pulse width)
    // Example: Calculate period between two rising edges
    static uint16_t last_capture = 0;
    uint16_t period;

    if (capture_value >= last_capture) {
        period = capture_value - last_capture;
    }
    else {
        // Timer overflow occurred
        period = (0xFFFF - last_capture) + capture_value + 1;
    }

    // Store current capture for next calculation
    last_capture = capture_value;

    // Period in microseconds (with 8MHz / 8 prescaler):
    // period * (prescaler / F_CPU) * 1000000
    // period * (8 / 8000000) * 1000000 = period
    // Frequency in Hz = 1000000 / period
}

/*************************************
 * TOPIC 12: TWO WIRE INTERFACE (TWI/I²C)
 *************************************/
 /*
  * TWI (Two Wire Interface)/I²C:
  * Serial communication protocol using only two wires:
  *   - SCL (Serial Clock Line): Provides clock signal
  *   - SDA (Serial Data Line): Carries data
  *
  * Features:
  * - Multi-master, multi-slave support
  * - 7-bit or 10-bit addressing
  * - Data transfer rates up to 400kHz
  *
  * Key TWI Registers:
  * - TWBR (TWI Bit Rate Register):
  *   Controls SCL frequency along with prescaler
  *   SCL frequency = F_CPU / (16 + 2*TWBR*prescaler)
  *
  * - TWCR (TWI Control Register):
  *   TWINT: TWI Interrupt Flag (set by hardware when TWI completes operation)
  *   TWEA: TWI Enable Acknowledge bit
  *   TWSTA: TWI START Condition bit
  *   TWSTO: TWI STOP Condition bit
  *   TWEN: TWI Enable bit
  *
  * - TWSR (TWI Status Register):
  *   TWS7:3: TWI Status bits
  *   TWPS1:0: TWI Prescaler bits
  *
  * - TWDR (TWI Data Register):
  *   Holds data to be transmitted or received
  *
  * - TWAR (TWI Address Register):
  *   TWA6:0: TWI slave address
  *   TWGCE: TWI General Call Recognition Enable bit
  */
void twi_master_init(void) {
    // Set SCL frequency to 100kHz with 8MHz system clock
    // SCL freq = F_CPU / (16 + 2*TWBR*prescaler)
    // 100kHz = 8MHz / (16 + 2*TWBR*1)
    // TWBR = (F_CPU/SCL - 16) / 2 = (8MHz/100kHz - 16) / 2 = 32
    TWSR = 0;                  // Prescaler 1 (TWPS1:0 = 00)
    TWBR = 32;                 // Bit rate register value for 100kHz

    // Enable TWI
    // TWEN = 1: Enable TWI interface
    TWCR = (1 << TWEN);
}

void twi_start(void) {
    // Send START condition
    // TWINT = 1: Clear interrupt flag
    // TWSTA = 1: Generate START condition
    // TWEN = 1: Keep TWI enabled
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);

    // Wait for TWINT flag to be set
    // TWINT is set when START condition is transmitted
    while (!(TWCR & (1 << TWINT)));

    // Check if START was sent successfully (status code 0x08)
    // TWI Status = TWSR & 0xF8 (mask out prescaler bits)
    // if ((TWSR & 0xF8) != 0x08) { /* Error handling */ }
}

void twi_stop(void) {
    // Send STOP condition
    // TWINT = 1: Clear interrupt flag
    // TWSTO = 1: Generate STOP condition
    // TWEN = 1: Keep TWI enabled
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);

    // No need to wait for TWINT flag after STOP
    // The TWSTO bit is cleared automatically when STOP is transmitted
}

void twi_write(uint8_t data) {
    // Load data into TWDR
    TWDR = data;

    // Start transmission
    // TWINT = 1: Clear interrupt flag
    // TWEN = 1: Keep TWI enabled
    TWCR = (1 << TWINT) | (1 << TWEN);

    // Wait for TWINT flag to be set
    // TWINT is set when data transmission completes
    while (!(TWCR & (1 << TWINT)));

    // Check if data was sent successfully (status code 0x28)
    // if ((TWSR & 0xF8) != 0x28) { /* Error handling */ }
}

uint8_t twi_read_ack(void) {
    // Start reception and send ACK after reception
    // TWINT = 1: Clear interrupt flag
    // TWEA = 1: Send ACK after reception
    // TWEN = 1: Keep TWI enabled
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);

    // Wait for TWINT flag to be set
    // TWINT is set when byte reception completes
    while (!(TWCR & (1 << TWINT)));

    // Return received data
    return TWDR;
}

uint8_t twi_read_nack(void) {
    // Start reception and send NACK after reception
    // TWINT = 1: Clear interrupt flag
    // TWEN = 1: Keep TWI enabled
    // TWEA = 0: Send NACK after reception (for last byte)
    TWCR = (1 << TWINT) | (1 << TWEN);

    // Wait for TWINT flag to be set
    // TWINT is set when byte reception completes
    while (!(TWCR & (1 << TWINT)));

    // Return received data
    return TWDR;
}

/*************************************
 * SYSTEM INTEGRATION AND MAIN FUNCTION
 *************************************/

void system_init(void) {
    // Initialize all peripherals
    initialize_ports();     // Configure I/O pins
    adc_init();             // Configure Analog-to-Digital Converter
    usart_init(9600);       // Configure USART with 9600 baud rate
    interrupt_init();       // Configure external interrupts
    rti_init();             // Configure Real-Time Interrupts
    pwm_8bit_init();        // Configure 8-bit PWM (Timer0)
    pwm_16bit_init();       // Configure 16-bit PWM (Timer1)
    spi_master_init();      // Configure SPI as master
    input_capture_init();   // Configure Input Capture system
    twi_master_init();      // Configure TWI/I²C as master

    // Enable global interrupts
    // sei(): Set Global Interrupt Enable bit in SREG
    sei();
}

int main(void) {
    // Initialize system
    system_init();

    // Main application loop
    while (1) {
        // Update state machine
        state_machine_tick();

        // Example: Read ADC and adjust PWM
        // Read analog value from ADC channel 0
        uint16_t adc_value = adc_read(0);

        // Scale 10-bit ADC value (0-1023) to 8-bit PWM (0-255)
        // Right shift by 2 bits (divide by 4)
        pwm_8bit_set_duty(adc_value >> 2);

        // Example: Send ADC value via USART
        // Send high byte first, then low byte
        usart_transmit((uint8_t)(adc_value >> 8));  // High byte
        usart_transmit((uint8_t)adc_value);         // Low byte

        // Delay to control loop rate
        // 100ms delay using _delay_ms() function
        _delay_ms(100);
    }

    return 0;  // Never reached in embedded applications
}
