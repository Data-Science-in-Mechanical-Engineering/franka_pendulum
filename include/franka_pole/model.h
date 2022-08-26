#pragma once

#include <stddef.h>

namespace franka_pole
{
    ///Available hardware configurations
    enum class Model
    {
        D0, ///< Franka with no pole
        D1, ///< Franka holding 1-DOF pole with one revolute joint
        D2, ///< Franka holding 2-DOF pole with two revolute joints
        D2b,///< Franka holding 3-DOF pole (2-DOF + rotation) with "needle in cup" design
        D2c,///< Franka holding 3-DOF pole (2-DOF + rotation) with "needle in deepening" design
    };

    ///Gets number of DOFs of the model
    size_t get_model_freedom(Model model);
}