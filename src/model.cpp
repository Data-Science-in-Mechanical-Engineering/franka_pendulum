#include <franka_pole/model.h>

size_t franka_pole::get_model_freedom(Model model)
{
    switch (model)
    {
        case Model::D0: return 0;
        case Model::D1: return 1;
        default: return 2;
    } 
}