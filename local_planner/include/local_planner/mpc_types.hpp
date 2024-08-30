#pragma once

#include <string>

namespace mpc {
namespace STATE_SPACE {
    static constexpr int x = 0;
    static constexpr int y = 1;
    static constexpr int yaw = 2;
    static constexpr int vel = 3;
    static constexpr int steer = 4;
    static constexpr int dim = 5;
} // STATE_SPACE

namespace CONTROL_SPACE {
    static constexpr int steer = 0;
    static constexpr int accel = 1;
    static constexpr int dim = 2;
} // CONTROL_SPACE

namespace mpc_types {
    using State = Eigen::Matrix<double, STATE_SPACE::dim, 1>;
    using Control = Eigen::Matrix<double, CONTROL_SPACE::dim, 1>;
} // mpc_types

} // mpc