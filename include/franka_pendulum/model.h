#pragma once

#include <stddef.h>

namespace franka_pendulum
{
    ///Available hardware configurations
    enum class Model
    {
        D0, ///< Franka with no pendulum
        D1, ///< Franka holding 1-DOF pendulum with one revolute joint
        D2, ///< Franka holding 2-DOF pendulum with two revolute joints
        D2b,///< Franka holding 3-DOF pendulum (2-DOF + rotation) with "needle in cup" design
        D2c,///< Franka holding 3-DOF pendulum (2-DOF + rotation) with "needle in deepening" design
    };

    ///Gets number of DOFs of the model
    size_t get_model_freedom(Model model);
}