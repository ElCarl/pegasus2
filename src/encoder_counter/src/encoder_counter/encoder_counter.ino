// Input constants
const uint8_t N_PINS = 18;
const uint8_t N_ENCS = 9;   // This should be N_PINS / 2!
const uint8_t INPUT_PINS[] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 15, 14, 17, 16, 19, 18};  // Order is important!

// Lookup table for encoder changes
const int8_t LOOKUP_TABLE[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

// Serial communication constants
const unsigned long BAUDRATE = 57600UL;
const unsigned long TRANSMIT_FREQUENCY = 10;  // How frequently to send the encoder counts
const byte BEGIN_MESSAGE_BYTE = 252;
const byte END_MESSAGE_BYTE   = 255;

// Global variables
uint8_t pin_states[]  = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t enc_vals[]    = {0, 0, 0, 0, 0, 0, 0, 0, 0};
long encoder_counts[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t state;
unsigned long transmissions = 0;  // How many times the program has sent the encoder counts

// Define the TX data structure
struct ENCODER_DATA_STRUCTURE {
    uint32_t tick_stamp_ms;  // Timestamp in ms since program start
    int32_t encoder_counts[N_ENCS];
};

ENCODER_DATA_STRUCTURE encoder_counts_struct;

uint32_t handshake_time_ms;

void setup() {
    // Pin modes
    for (uint8_t pin = 0; pin < N_PINS; pin++) {
        pinMode(INPUT_PINS[pin], INPUT);
    }
    
    // Begin serial connection
    Serial.begin(BAUDRATE);
}

void loop() {
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
    
    // Checks to see if the encoder counts should be transmitted. Compares number
    // of actual transmissions to number of expected transmissions; if it has
    // transmitted fewer than expected, then it will transmit.
    if (transmissions * 1000 < millis() * TRANSMIT_FREQUENCY) {
        // multiply by 1000 to change from s to ms. Multiplication faster than
        // division, at the cost of a little clarity.
        transmit_encoder_counts_new();
        transmissions++;
    }  
}

void encoder_pin_change(uint8_t encoder) {
    enc_vals[encoder] <<= 2;
    enc_vals[encoder] |= ((pin_states[2 * encoder] << 2) + pin_states[(2 * encoder) + 1]);
    encoder_counts[encoder] += LOOKUP_TABLE[enc_vals[encoder] & 0b1111];
}

void transmit_encoder_counts_new() {
    uint8_t send_byte;
    uint8_t struct_len = sizeof(encoder_counts_struct);
    uint8_t checksum = struct_len;
    uint8_t * struct_addr = (uint8_t*)&encoder_counts_struct;

    encoder_counts_struct.tick_stamp_ms = millis() - handshake_time_ms;

    for (uint8_t encoder = 0; encoder < N_ENCS; encoder++) {
        encoder_counts_struct.encoder_counts[encoder] = encoder_counts[encoder];
    }

    // Start message
    Serial.write(BEGIN_MESSAGE_BYTE);

    // Send struct length
    Serial.write(struct_len);

    // Send struct data & calculate checksum
    for (uint8_t i = 0; i < struct_len; i++) {
        send_byte = *(struct_addr + i);
        Serial.write(send_byte);
        checksum ^= send_byte;
    }

    // Send checksum
    Serial.write(checksum);

    // End message
    Serial.write(END_MESSAGE_BYTE);
}

