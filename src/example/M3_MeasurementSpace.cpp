/**
(C) Copyright 2020-2023 Murilo Marques Marinho (www.murilomarinho.info)

This file is part of adaptive_control_example.

    DQ Robotics is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    adaptive_control_example is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with adaptive_control_example.  If not, see <http://www.gnu.org/licenses/>.

Author:
    Murilo M. Marinho (murilomarinho@ieee.org)

Contributors (aside from author):
    None
*/
#include <stdexcept>

#include "marinholab/papers/tro2022/adaptive_control/M3_MeasurementSpace.h"

int get_measure_space_dimension(const M3_MeasureSpace &measure_space)
{
    switch(measure_space)
    {
    case M3_MeasureSpace::None:
        return 0;
    case M3_MeasureSpace::Pose:
        return 8;
    case M3_MeasureSpace::Rotation:
        return 4;
    case M3_MeasureSpace::Translation:
        return 4;
    case M3_MeasureSpace::Distance:
        return 1;
    }
    throw std::runtime_error("Not supposed to be reachable");
}



