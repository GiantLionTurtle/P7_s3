
#ifndef P7_COMMON_HPP_
#define P7_COMMON_HPP_

#define BAUD_RATE 115200
#define COMMAND_DURATION_MS 100
#define N_ACCELS_SAMPLES 10

// Must be kept in sync with rpi's State.hpp
enum class State { Ready, Stabilize, ReturnHome, TakingTree, Swinging, JustGonnaSendIt, Drop };

#endif