#define D0 eS_PORTD2
#define D1 eS_PORTB2
#define D2 eS_PORTB5
#define D3 eS_PORTD3
#define D4 eS_PORTD4
#define D5 eS_PORTD5
#define D6 eS_PORTD6
#define D7 eS_PORTD7
#define RS eS_PORTB0
#define EN eS_PORTB4
#define F_CPU 16000000UL
#include <avr/io.h>
#include "lcd.h"
#include <util/delay.h>
#include <string.h>
#include <avr/interrupt.h>
#include <stdio.h>

#define LED_PIN PB3
#define MOTOR_PIN PB1
#define BAUDRATE 9600
#define BAUD_PRESCALLER (((F_CPU / (BAUDRATE * 16UL))) - 1)

// Function to map a value from one range to another
long map(long x, long in_min, long in_max, long out_min, long out_max) {
	// Maps the input value from the input range to a 0-1 range
	float normalized = (float)(x - in_min) / (in_max - in_min);

	// Apply a non-linear transformation (e.g. squaring)
	normalized = normalized * normalized;

	// Map the transformed value back to the output range
	return (long)(normalized * (out_max - out_min) + out_min);
}

// Initialize USART for communication
void USART_init(void){
	UBRR0H = (uint8_t)(BAUD_PRESCALLER>>8); // Set baud rate
	UBRR0L = (uint8_t)(BAUD_PRESCALLER);
	UCSR0B = (1<<RXEN0)|(1<<TXEN0); // Enable transmission
	UCSR0C = (3<<UCSZ00); // Set to 8-bit data format (default)
}

// Function to send data through USART
void USART_send(unsigned char data){
	while(!(UCSR0A & (1<<UDRE0))); // Wait until the buffer is empty
	UDR0 = data;
}

// Initialize ADC for analog reading
void ADC_Init() {
	// Select pin A0 as the analog input
	ADMUX = (1 << REFS0); // Use AVcc as the reference voltage

	// Enable ADC and set the prescaler
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

// Read value from ADC
uint16_t Read_ADC(uint8_t channel) {
	// Select ADC channel to read
	ADMUX = (ADMUX & 0xF8) | (channel & 0x05);

	// Start a single conversion
	ADCSRA |= (1 << ADSC);

	// Wait for conversion to complete
	while (ADCSRA & (1 << ADSC));

	// Return the ADC value
	return ADC;
}

// Initialize PWM for LED
void PWM_LED_Init() {
	// Set PB3 as an output mode
	DDRB |= (1 << LED_PIN);

	// Configure Timer 2 for Fast PWM
	TCCR2A |= (1 << COM2A1) | (1 << WGM21) | (1 << WGM20); // Set to Fast PWM mode
	TCCR2B |= (1 << CS20);  // No prescaling
}

// Set the duty cycle for the LED PWM
void Set_LED_PWM(uint8_t duty) {
	OCR2A = duty;  // Use OCR2A to control the brightness of the LED
}

// Initialize the servo motor
void initServo() {
	DDRB |= (1 << MOTOR_PIN); // Set PB1 as output

	// Set Timer 1 for Fast PWM mode, non-inverted
	TCCR1A |= (1 << COM1A1) | (1 << WGM11);
	TCCR1B |= (1 << WGM12) | (1 << WGM13);

	ICR1 = 4999; // Set the TOP value to 4999 for a 20ms PWM period

	// Set prescaler to 64
	TCCR1B |= (1 << CS11) | (1 << CS10);

	OCR1A = 250; // Initialize to 0-degree position (1ms pulse width)
}

// Set the servo motor angle
void setServoAngle(int angle) {
	int pulseWidth = map(angle, 0, 180, 250, 500); // Map angle to pulse width
	OCR1A = pulseWidth; // Set the PWM pulse width
}

// Interrupt Service Routine for INT0
ISR(INT0_vect) {
	cli(); // Disable global interrupts
	// Add code here for what needs to be done when interrupt occurs
	sei(); // Enable global interrupts
}

int main(void) {
	PWM_LED_Init();    // Initialize LED using Timer 0
	initServo();       // Initialize servo motor using Timer 1
	ADC_Init();
	USART_init(); // Initialize USART with a baud rate of 9600
	char receivedData;
	
	char buffer[16];
	DDRD = 0xFC | DDRD;
	DDRB = DDRB | 0x35;
	int i;
	Lcd8_Init();
	
	// Configure INT0 for button press interrupt
	DDRD &= ~(1 << DDD2);  // Set INT0 pin as input
	PORTD |= (1 << PORTD2); // Enable pull-up resistor on INT0 pin
	EICRA |= (1 << ISC01);  // Set INT0 to trigger on falling edge
	EICRA &= ~(1 << ISC00);
	EIMSK |= (1 << INT0);   // Enable INT0 interrupt
	sei();                  // Enable global interrupts
	
	while(1) {
		// Read ADC value from channel 0
		uint16_t adcValue = Read_ADC(0);
		
		// Set servo angle based on ADC value
		if (adcValue < 500) {
			setServoAngle(180); // If light intensity is below a threshold, set motor angle to 180 degrees
			} else {
			setServoAngle(0);   // Otherwise, reset servo to 0 degrees
		}
		
		// Map ADC value inversely to PWM value for LED brightness (0-255)
		uint8_t pwmValue = map(1023 - adcValue, 0, 1023, 0, 255);
		Set_LED_PWM(pwmValue); // Set LED brightness
		
		// Read light level and map to 0-100% for display
		uint16_t ldrValue = Read_ADC(0); // Re-read ADC for potentially different data processing
		int lightLevel = map(ldrValue, 0, 1023, 0, 100);
		
		// Format and display light level on LCD
		snprintf(buffer, sizeof(buffer), "Light: %d%%    ", lightLevel);
		Lcd8_Set_Cursor(1,1);
		Lcd8_Write_String(buffer);
		
		// Send formatted string over USART
		for (int i = 0; buffer[i] != '\0'; i++) {
			USART_send(buffer[i]);
		}
		
		// Check if data is received via USART
		if (UCSR0A & (1 << RXC0)) {
			unsigned char cmd = UDR0; // Read the received data
			
			// Perform actions based on received command
			if (cmd == '0') {
				// Turn off LED and reset servo position if '0' command is received
				Set_LED_PWM(0);
				setServoAngle(0);
			}
		}
		
		// Delay for a short period before updating readings
		_delay_ms(500);
	}
	return 0;
}
