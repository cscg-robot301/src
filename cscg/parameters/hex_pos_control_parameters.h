//
// Created by weijin on 19-1-18.
//

#ifndef CSCG_HEX_POS_CONTROL_PARAMETERS_H
#define CSCG_HEX_POS_CONTROL_PARAMETERS_H

#include <iostream>
#include <cmath>

namespace cscg{

    struct HEXPosCtrl{

        float HEX_THR_MIN;
        float HEX_THR_MAX;
        float HEX_THR_HOVER;
        float HEX_THR_CURVE;
        float HEX_HRI_P;
        float HEX_Z_VEL_P;
        float HEX_Z_VEL_I;
        float HEX_Z_VEL_D;
        float HEX_XY_VEL_P;
        float HEX_XY_VEL_I;
        float HEX_XY_VEL_D;

    };

}
#endif //CSCG_HEX_POS_CONTROL_PARAMETERS_H
