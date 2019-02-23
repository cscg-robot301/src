//
// Created by weijin on 19-1-21.
//

#ifndef CSCG_LIMITS_HPP
#define CSCG_LIMITS_HPP

#include <iostream>
#include <cmath>
#include <float.h>

#ifndef MATH_PI
#define MATH_PI		3.141592653589793238462643383280
#endif
namespace cscg {
    namespace math {

        template<typename _Tp>
        constexpr const _Tp &min(const _Tp &a, const _Tp &b);

        template<typename _Tp>
        constexpr const _Tp &max(const _Tp &a, const _Tp &b);

        template<typename _Tp>
        constexpr const _Tp &constrain(const _Tp &val, const _Tp &min_val, const _Tp &max_val);

/** Constrain float values to valid values for int16_t.
 * Invalid values are just clipped to be in the range for int16_t. */
        inline int16_t constrainFloatToInt16(float value);


        template<typename _Tp>
        inline constexpr bool isInRange(const _Tp &val, const _Tp &min_val, const _Tp &max_val);

        template<typename T>
        constexpr T radians(const T degrees);

        template<typename T>
        constexpr T degrees(const T radians);

/** Save way to check if float is zero */
        inline bool isZero(const float val);

/** Save way to check if double is zero */
        inline bool isZero(const double val);

    }
}
#endif //CSCG_LIMITS_HPP
