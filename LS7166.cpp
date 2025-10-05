#include "LS7166.h"

LS7166::LS7166(uint8_t cs1_pin, uint8_t cs2_pin) {
    _cs1_pin = cs1_pin;
    _cs2_pin = cs2_pin;
}

void LS7166::begin() {
    // Initialize SPI
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV8);
    SPI.setDataMode(SPI_MODE0);
    SPI.setBitOrder(MSBFIRST);

    // Configure CS pins
    pinMode(_cs1_pin, OUTPUT);
    pinMode(_cs2_pin, OUTPUT);
    digitalWrite(_cs1_pin, HIGH);
    digitalWrite(_cs2_pin, HIGH);

    // Initialize both encoders
    init_encoders();
}

void LS7166::write_LS7166(uint8_t cs_pin, uint8_t reg, uint8_t data) {
    digitalWrite(cs_pin, LOW);
    SPI.transfer(reg | 0x80);
    SPI.transfer(data);
    digitalWrite(cs_pin, HIGH);
}

uint8_t LS7166::read_LS7166(uint8_t cs_pin, uint8_t reg) {
    uint8_t data;
    digitalWrite(cs_pin, LOW);
    SPI.transfer(reg);
    data = SPI.transfer(0x00);
    digitalWrite(cs_pin, HIGH);
    return data;
}

void LS7166::init_encoder(uint8_t cs_pin) {
    // Configure MDR0: 4X quadrature, free running, index disabled
    write_LS7166(cs_pin, MCR_MDR0, 0x00);
    write_LS7166(cs_pin, MDR0, 0x03);

    // Configure MDR1: 32-bit counter, enable counting
    write_LS7166(cs_pin, MCR_MDR1, 0x00);
    write_LS7166(cs_pin, MDR1, 0x00);

    // Clear counter
    write_LS7166(cs_pin, CMD, CLR_CNTR);
}

void LS7166::init_encoder1() {
    init_encoder(_cs1_pin);
}

void LS7166::init_encoder2() {
    init_encoder(_cs2_pin);
}

void LS7166::init_encoders() {
    init_encoder1();
    init_encoder2();
}

int32_t LS7166::read_encoder1() {
    int32_t count = 0;

    // Latch counter to output register
    write_LS7166(_cs1_pin, CMD, LATCH_CNTR);

    // Read 32-bit counter value
    digitalWrite(_cs1_pin, LOW);
    SPI.transfer(OTR);
    count = SPI.transfer(0x00);
    count |= ((int32_t)SPI.transfer(0x00) << 8);
    count |= ((int32_t)SPI.transfer(0x00) << 16);
    count |= ((int32_t)SPI.transfer(0x00) << 24);
    digitalWrite(_cs1_pin, HIGH);

    return count;
}

int32_t LS7166::read_encoder2() {
    int32_t count = 0;

    // Latch counter to output register
    write_LS7166(_cs2_pin, CMD, LATCH_CNTR);

    // Read 32-bit counter value
    digitalWrite(_cs2_pin, LOW);
    SPI.transfer(OTR);
    count = SPI.transfer(0x00);
    count |= ((int32_t)SPI.transfer(0x00) << 8);
    count |= ((int32_t)SPI.transfer(0x00) << 16);
    count |= ((int32_t)SPI.transfer(0x00) << 24);
    digitalWrite(_cs2_pin, HIGH);

    return count;
}

void LS7166::reset_encoder1() {
    write_LS7166(_cs1_pin, CMD, CLR_CNTR);
}

void LS7166::reset_encoder2() {
    write_LS7166(_cs2_pin, CMD, CLR_CNTR);
}

void LS7166::reset_encoders() {
    reset_encoder1();
    reset_encoder2();
}

int32_t LS7166::get_encoder_difference() {
    int32_t enc1 = read_encoder1();
    int32_t enc2 = read_encoder2();
    return enc1 - enc2;
}

int32_t LS7166::get_encoder_average() {
    int32_t enc1 = read_encoder1();
    int32_t enc2 = read_encoder2();
    return (enc1 + enc2) / 2;
}