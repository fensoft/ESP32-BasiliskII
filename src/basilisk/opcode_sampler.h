/*
 *  opcode_sampler.h - lightweight 68k opcode sampler
 *
 *  Collects a sampled stream of opcodes from the hot CPU loop and
 *  reports top opcodes periodically from the main loop.
 */

#ifndef OPCODE_SAMPLER_H
#define OPCODE_SAMPLER_H

#include <stdint.h>

#ifndef OPCODE_SAMPLER_ENABLED
#define OPCODE_SAMPLER_ENABLED 1
#endif

#ifdef __cplusplus
extern "C" {
#endif

// Record a single opcode (sampled internally)
void opcode_sampler_record(uint16_t opcode);

// Periodic reporting (call from main loop)
void opcode_sampler_report(void);

#ifdef __cplusplus
}
#endif

#endif // OPCODE_SAMPLER_H
