// Input constants
const uint8_t N_PINS = 18;
const uint8_t N_ENCS = 9;   // This should be N_PINS / 2!
const uint8_t INPUT_PINS[] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 15, 14, 17, 16, 19, 18};  // Order is important!

// Lookup table for encoder changes
const int8_t LOOKUP_TABLE[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

// Serial communication constants
const unsigned long BAUDRATE = 38400UL;
const unsigned long TRANSMIT_FREQUENCY = 10;  // How frequently to send the encoder counts
const byte SERIAL_READY_BYTE  = 251;
const byte BEGIN_MESSAGE_BYTE = 252;
const byte END_MESSAGE_BYTE   = 255;

// Global variables
uint8_t pin_states[]  = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t enc_vals[]    = {0, 0, 0, 0, 0, 0, 0, 0, 0};
long encoder_counts[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t state;
unsigned long transmissions = 0;  // How many times the program has sent the encoder counts

void setup() {
    for (uint8_t pin = 0; pin < N_PINS; pin++) {
        pinMode(INPUT_PINS[pin], INPUT);
    }
    
    Serial.begin(BAUDRATE);
    
    while (Serial.read() != SERIAL_READY_BYTE) {
        delay(100);
        Serial.write(SERIAL_READY_BYTE);
    }
}

void loop() {
    for (uint8_t pin = 0; pin < N_PINS; pin++) {
        state = digitalRead(INPUT_PINS[pin]);
        // If the pin state has changed
        if (state != pin_states[pin]) {
            // Update the pin state
            pin_states[pin] = state;
            // And update the encoder count accordingly
            encoder_pin_change(pin / 2);  // (pin / 2) gives the encoder number: 0 & 1 -> 0, 2 & 3 -> 1, etc.
        }
    }
    
    // Check to see if the encoder counts should be transmitted. Compares number of actual transmissions to number of expected transmissions; if it has transmitted fewer than expected, then it will transmit
    if (transmissions * 1000 < millis() * TRANSMIT_FREQUENCY) {
        // multiply by 1000 to change from s to ms. Multiplication faster than division, at the cost of a little clarity.
        // Using millis since micros overflows after 70 minutes, which could easily occur when running the rover. Millis takes 50 days to overflow - somewhat less likely! And we don't need the additional accuracy anyway, unless TRANSMIT_FREQUENCY nears 1000
        transmit_encoder_counts();
    }  
}

void encoder_pin_change(uint8_t encoder) {
    enc_vals[encoder] <<= 2;
    //enc_vals[encoder] = enc_vals[encoder] << 2;  // Here just in case <<= doesn't work!
    enc_vals[encoder] |= ((pin_states[2 * encoder] << 2) + pin_states[(2 * encoder) + 1]);
    //enc_vals[encoder] = enc_vals[encoder] | ((pin_states[2 * encoder] << 2) + pin_states[(2 * encoder) + 1]);  // Same as above, backup!
    encoder_counts[encoder] += LOOKUP_TABLE[enc_vals[encoder] & 0b1111];
}

void transmit_encoder_counts() {
    Serial.write(BEGIN_MESSAGE_BYTE);
    // First write all encoder values
    byte long_buffer[4];
    for (uint8_t encoder = 0; encoder < N_ENCS; encoder++) {
        // Must convert long to series of bytes for transmission
        long_to_bytes(encoder_counts[encoder], long_buffer);
        Serial.write(long_buffer, 4);
    }
    // Then calculate and write the checksum
    long checksum = calculate_checksum();
    long_to_bytes(checksum, long_buffer);
    Serial.write(long_buffer, 4);

    Serial.write(END_MESSAGE_BYTE);
    transmissions++;
}

long calculate_checksum() {
    // Very simplistic checksum - just sum all the encoder values. It's checking for transmission
    // errors, which should cause pretty large deviations, so I don't think it needs to be too advanced.
    long checksum = 0;
    
    // Just sum all the encoder values. This may cause overflow, but it should overflow identically
    // on the other end, so it shouldn't really be an issue.
    for (uint8_t encoder = 0; encoder < N_ENCS; encoder++) {
        checksum += encoder_counts[encoder];
    }
    
    return checksum;
}

// Converts a long to an array of 4 bytes for serial transmission
void long_to_bytes(long val, byte bytes[]) {
    bytes[0] =  val        & 255;
    bytes[1] = (val >> 8)  & 255;
    bytes[2] = (val >> 16) & 255;
    bytes[3] = (val >> 24) & 255;
}
