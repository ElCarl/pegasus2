#include <SPI.h>

// Input constants
const uint8_t N_PINS = 14;
const uint8_t N_ENCS = 7;   // This should be N_PINS / 2!
const uint8_t INPUT_PINS[] = {2, 3, 4, 5, 6, 7, 8, 9, 15, 14, 17, 16, 19, 18};  // Order is important! It defines which pins belong to which encoder

// Lookup table for encoder changes
const int8_t LOOKUP_TABLE[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

// Serial communication constants
const unsigned long BAUDRATE = 1000000;
const byte REQUEST_ENCODER_COUNTS = 248;
const byte SEND_MESSAGE           = 252;
const byte END_MESSAGE            = 255;

// Global variables
uint8_t pin_states[]  = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t enc_vals[]    = {0, 0, 0, 0, 0, 0, 0};
long encoder_counts[] = {0, 0, 0, 0, 0, 0, 0};
uint8_t state;

// Debug globals
uint16_t counter = 0;

// Define the TX data structure
struct ENCODER_DATA_STRUCTURE {
    uint32_t tick_stamp_ms;  // Timestamp in ms since program start
    int32_t encoder_counts[N_ENCS];
};

// SPI communication global variables
volatile uint8_t transmission_pos = 0;
volatile uint8_t checksum;

uint32_t handshake_time_ms;

// Encoder struct declaration & info
ENCODER_DATA_STRUCTURE encoder_counts_struct;
uint8_t struct_len = sizeof(encoder_counts_struct);
uint8_t * struct_addr = (uint8_t*)&encoder_counts_struct;

void setup() {
    // Pin modes
    for (uint8_t pin = 0; pin < N_PINS; pin++) {
        pinMode(INPUT_PINS[pin], INPUT);
    }
    
    // Set pin modes for SPI communication
    pinMode(MISO, OUTPUT);
    pinMode(MOSI, INPUT);
    pinMode(SCK, INPUT);
    pinMode(SS, INPUT);

    // Set bits in the SPI register
    SPCR |= bit(SPE);   // Enable SPI in slave mode
    SPCR |= bit(SPIE);  // Enable SPI interrupts

    // Begin serial connection: for debugging only
    Serial.begin(BAUDRATE);
    Serial.println("Started");
}

// Interrupt service routine to handle SPI communications
ISR (SPI_STC_vect) {
    // Read the data from the SPI register
    volatile uint8_t data = SPDR;
    Serial.print("SPDR: ");
    Serial.println(data);

    // If encoder data is requested
    if (data == REQUEST_ENCODER_COUNTS) {
        // Load the data length into the SPI register
        SPDR = struct_len;

        // Reset the transmission position, in case any communication
        // is lost from the master and it restarts
        transmission_pos = 0;

        // Load the encoder data into the struct
        for (volatile uint8_t encoder = 0; encoder < N_ENCS; encoder++) {
            encoder_counts_struct.encoder_counts[encoder] = encoder_counts[encoder];
        }

        // Calculate checksum. This could go in the per-byte transmission if
        // the setup in the first interrupt is too long
        checksum = struct_len;
        for (volatile uint8_t i = 0; i < struct_len; i++) {
            checksum ^= *(struct_addr + i);
        }
    }
    // If main message body data is requested
    else if (data == SEND_MESSAGE) {
        // Check if we have reached the end of the message
        if (transmission_pos >= struct_len) {
            // If so, load the checksum into the SPI register
            SPDR = checksum;
        }
        // If there's still data to send
        else {
            // Load the next byte of the struct into the register
            SPDR = *(struct_addr + transmission_pos++);
        }
    }
    else { //if (data == END_MESSAGE) OR if we receive an unrecognised message
        transmission_pos = 0;
    }
}

void loop_DISABLE() {
    for (uint8_t pin = 0; pin < N_PINS; pin++) {
        state = digitalRead(INPUT_PINS[pin]);
        // If the pin state has changed
        if (state != pin_states[pin]) {
            // Update the pin state
            pin_states[pin] = state;
            // And update the encoder count accordingly
            encoder_pin_change(pin / 2);  // (pin / 2) gives the encoder number:
        }                                 // 0 & 1 -> 0, 2 & 3 -> 1, etc.
    }
}

void loop() {// Debug loop
    for (uint8_t i = 0; i < N_ENCS; i++) {
        encoder_counts[i] = (counter + 1) * i;
    }
    counter++;
    Serial.print("Count: "); Serial.println(counter);
    delay(1000);
}

void encoder_pin_change(uint8_t encoder) {
    enc_vals[encoder] <<= 2;
    enc_vals[encoder] |= ((pin_states[2 * encoder] << 1) + pin_states[(2 * encoder) + 1]);
    encoder_counts[encoder] += LOOKUP_TABLE[enc_vals[encoder] & 0b1111];
}

void transmit_encoder_counts_serial() {
    uint8_t send_byte;
    uint8_t serial_checksum = struct_len;

    encoder_counts_struct.tick_stamp_ms = millis() - handshake_time_ms;

    for (uint8_t encoder = 0; encoder < N_ENCS; encoder++) {
        encoder_counts_struct.encoder_counts[encoder] = encoder_counts[encoder];
    }

    // Start message
    Serial.write(SEND_MESSAGE);

    // Send struct length
    Serial.write(struct_len);

    // Send struct data & calculate checksum
    for (uint8_t i = 0; i < struct_len; i++) {
        send_byte = *(struct_addr + i);
        Serial.write(send_byte);
        serial_checksum ^= send_byte;
    }

    // Send checksum
    Serial.write(serial_checksum);
}
