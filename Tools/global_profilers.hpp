/**
 * @file global_profilers.hpp
 * @author Haoru Xue (haoru.xue@autoware.org)
 * @brief
 * @version 0.1
 * @date 2022-04-05
 *
 * @copyright Copyright 2022 Triton AI
 *
 */

#include "profiler.hpp"

namespace tritonai {
namespace gkc {
Profiler COMM_PROFILER("Comm");
Profiler CONTROL_PROFILER("Control");
Profiler Sensor_PROFILER("Sensor");
} // namespace gkc
} // namespace tritonai
