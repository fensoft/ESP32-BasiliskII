/*
 *  opcode_sampler.cpp - lightweight 68k opcode sampler
 *
 *  Designed for low overhead in the CPU hot loop. Records a sampled stream
 *  of opcodes into a ring buffer and reports top opcodes every few seconds.
 */

#include "opcode_sampler.h"
#include <Arduino.h>
#include <algorithm>
#include "esp_attr.h"  // For DRAM_ATTR

// Ring buffer size (power of two)
#ifndef OPCODE_SAMPLE_BUF_SIZE
#define OPCODE_SAMPLE_BUF_SIZE 4096
#endif

// Report interval
#ifndef OPCODE_REPORT_INTERVAL_MS
#define OPCODE_REPORT_INTERVAL_MS 5000
#endif

#if (OPCODE_SAMPLE_BUF_SIZE & (OPCODE_SAMPLE_BUF_SIZE - 1)) != 0
#error "OPCODE_SAMPLE_BUF_SIZE must be a power of two"
#endif

// Ring buffer
DRAM_ATTR static uint16_t opcode_samples[OPCODE_SAMPLE_BUF_SIZE];
DRAM_ATTR static volatile uint32_t opcode_head = 0;
DRAM_ATTR static volatile uint32_t opcode_tail = 0;
static uint32_t dropped_samples = 0;

// Report state
static uint32_t last_report_ms = 0;

#ifdef ARDUINO
IRAM_ATTR
#endif
void opcode_sampler_record(uint16_t opcode)
{
    uint32_t head = opcode_head;
    uint32_t next = (head + 1) & (OPCODE_SAMPLE_BUF_SIZE - 1);
    if (next == opcode_tail) {
        dropped_samples++;
        return;
    }

    opcode_samples[head] = opcode;
    opcode_head = next;
}

void opcode_sampler_report(void)
{
    uint32_t now = millis();
    if (now - last_report_ms < OPCODE_REPORT_INTERVAL_MS) {
        return;
    }
    last_report_ms = now;

    // Drain ring buffer into a local array
    static uint16_t local[OPCODE_SAMPLE_BUF_SIZE];
    uint32_t count = 0;
    while (opcode_tail != opcode_head && count < OPCODE_SAMPLE_BUF_SIZE) {
        local[count++] = opcode_samples[opcode_tail];
        opcode_tail = (opcode_tail + 1) & (OPCODE_SAMPLE_BUF_SIZE - 1);
    }

    if (count == 0) {
        return;
    }

    // Sort and count frequencies
    std::sort(local, local + count);

    struct TopEntry {
        uint16_t opcode;
        uint32_t count;
    };
    TopEntry top[10] = {};

    uint16_t current = local[0];
    uint32_t current_count = 1;

    auto consider = [&](uint16_t op, uint32_t cnt) {
        // Insert into top-10 if large enough
        int min_idx = -1;
        uint32_t min_count = 0xFFFFFFFF;
        for (int i = 0; i < 10; i++) {
            if (top[i].count == 0) {
                top[i].opcode = op;
                top[i].count = cnt;
                return;
            }
            if (top[i].count < min_count) {
                min_count = top[i].count;
                min_idx = i;
            }
        }
        if (cnt > min_count && min_idx >= 0) {
            top[min_idx].opcode = op;
            top[min_idx].count = cnt;
        }
    };

    for (uint32_t i = 1; i < count; i++) {
        if (local[i] == current) {
            current_count++;
        } else {
            consider(current, current_count);
            current = local[i];
            current_count = 1;
        }
    }
    consider(current, current_count);

    // Sort top entries by count desc (simple bubble sort, size 10)
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 9 - i; j++) {
            if (top[j].count < top[j + 1].count) {
                TopEntry tmp = top[j];
                top[j] = top[j + 1];
                top[j + 1] = tmp;
            }
        }
    }

    Serial.println("[OPCODE] Top 10 sampled opcodes:");
    for (int i = 0; i < 10; i++) {
        if (top[i].count == 0) break;
        uint32_t pct = (top[i].count * 100U) / count;
        Serial.printf("[OPCODE]   %04X: %u (%u%%)\n", top[i].opcode, top[i].count, pct);
    }
    if (dropped_samples > 0) {
        Serial.printf("[OPCODE] Dropped samples: %u (buffer full)\n", dropped_samples);
        dropped_samples = 0;
    }
}
