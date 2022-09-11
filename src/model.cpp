#include <franka_pendulum/model.h>

size_t franka_pendulum::get_model_freedom(Model model)
{
    switch (model)
    {
        case Model::D0: return 0;
        case Model::D1: return 1;
        default: return 2;
    } 
}