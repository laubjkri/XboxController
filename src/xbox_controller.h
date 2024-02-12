#ifndef XBOX_CONTROLLER_H
#define XBOX_CONTROLLER_H

// #define DEBUG
#define VERBOSE

#if defined(DEBUG) || defined(VERBOSE)
#define VERBOSE_LOG(...) printf(__VA_ARGS__)
#else
#define VERBOSE_LOG(...)
#endif


#if defined(DEBUG)
#define FUNCTION_CALL_LOG(...) printf(__VA_ARGS__)
#else
#define FUNCTION_CALL_LOG(...)
#endif

void update_led();


#endif