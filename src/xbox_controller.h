#ifndef XBOX_CONTROLLER_H
#define XBOX_CONTROLLER_H

//#define DEBUG
//#define VERBOSE

//#if defined(DEBUG) || defined(VERBOSE)
#if 0
#define VERBOSE_LOG(...) printf(__VA_ARGS__)
#else
#define VERBOSE_LOG(...)
#endif

#if defined(DEBUG)
#define DEBUG_LOG(...) printf(__VA_ARGS__)
#else
#define DEBUG_LOG(...)
#endif


#if 0
#define FUNCTION_CALL_LOG(...) printf(__VA_ARGS__)
#else
#define FUNCTION_CALL_LOG(...)
#endif

static inline void update_led();
static inline bool get_bit_status(uint val, uint bit_no);
static inline void xbox_controller_event_handler(const uint8_t* data, uint16_t size);

typedef struct
{
    uint8_t lx;
    uint8_t ly;
    uint8_t rx;
    uint8_t ry;
    uint8_t lt;
    uint8_t rt;
    uint8_t buttons;
    uint8_t hat;
    bool button_a;
    bool button_b;
    bool button_x;
    bool button_y;
} ControllerData;



#endif