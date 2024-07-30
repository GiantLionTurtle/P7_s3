
#ifndef P7_COMMON_HPP_
#define P7_COMMON_HPP_

#define BAUD_RATE 115200
#define COMMAND_DURATION_MS 100
#define N_ACCELS_SAMPLES 10
#define UPDATE_RATE_MS 25

// Must be kept in sync with rpi's State.hpp
enum class State { Ready, Stabilize, ReturnHome, TakingTree, Swinging, LastSwing, JustGonnaSendIt, JustGonnaSmoothIt, GetToDrop, Drop, ShortCircuitForward, ShortCircuitBackward, Error };

constexpr double startPos = 0.0;
constexpr double wheelRadius = 0.05; // m

#define JSON_TIME "t"
#define JSON_GOAL "g"
#define JSON_WHEEL "w"
#define JSON_DWHEEL "dw"
#define JSON_DDWHEEL "ddw"
#define JSON_PENDULUM "p"
#define JSON_DPENDULUM "dp"
#define JSON_ATGOAL "k"
#define JSON_STATE "s"
#define JSON_VOLTAGE "v"
#define JSON_CURRENT "c"
#define JSON_COMMAND_VELOCITIES "a"
#define JSON_COMMAND_START "z"
#define JSON_PID_P "kp"
#define JSON_PID_I "ki"
#define JSON_PID_D "kd"
#define JSON_SEND "d"

#endif
