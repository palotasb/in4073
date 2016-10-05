#ifndef PROFILE_H
#define PROFILE_H

#include <inttypes.h>

typedef struct profile {
    uint32_t time;
    uint32_t tag;
    uint32_t max_delta;
    uint32_t max_tag;
    uint32_t last_delta;
    uint32_t last_tag;
} profile_t;

static inline void profile_init(profile_t* p);
static inline void profile_start(profile_t* p, uint32_t time);
static inline void profile_start_tag(profile_t* p, uint32_t time, uint32_t tag);
static inline void profile_end(profile_t* p, uint32_t time);

static inline void profile_init(profile_t* p) {
    p->time = 0;
    p->tag = UINT32_MAX;
    p->max_delta = 0;
    p->max_tag = UINT32_MAX;
    p->last_delta = 0;
    p->last_tag = UINT32_MAX;
}

static inline void profile_start(profile_t* p, uint32_t time) {
    p->time = time;
}

static inline void profile_start_tag(profile_t* p, uint32_t time, uint32_t tag) {
    p->time = time;
    p->tag = tag;
}

static inline void profile_end(profile_t* p, uint32_t time) {
    p->last_delta = time - p->time;
    p->last_tag = p->tag;
    if (p->max_delta < p->last_delta) {
        p->max_delta = p->last_delta;
        p->max_tag = p->tag;
    }
}

#endif // PROFILE_H
