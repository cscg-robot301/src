//
// Created by weijin on 19-1-21.
//

#include "Limits.h"

namespace cscg{
    namespace math{

        template<typename _Tp>
        constexpr const _Tp &min(const _Tp &a, const _Tp &b)
        {
            return (a < b) ? a : b;
        }

        template<typename _Tp>
        constexpr const _Tp &max(const _Tp &a, const _Tp &b)
        {
            return (a > b) ? a : b;
        }

        template<typename _Tp>
        constexpr const _Tp &constrain(const _Tp &val, const _Tp &min_val, const _Tp &max_val)
        {
            return (val < min_val) ? min_val : ((val > max_val) ? max_val : val);
        }

/** Constrain float values to valid values for int16_t.
 * Invalid values are just clipped to be in the range for int16_t. */
        inline int16_t constrainFloatToInt16(float value)
        {
            return (int16_t)math::constrain(value, (float)INT16_MIN, (float)INT16_MAX);
        }


        template<typename _Tp>
        inline constexpr bool isInRange(const _Tp &val, const _Tp &min_val, const _Tp &max_val)
        {
            return (min_val <= val) && (val <= max_val);
        }

        template<typename T>
        constexpr T radians(const T degrees)
        {
            return degrees * (static_cast<T>(MATH_PI) / static_cast<T>(180));
        }

        template<typename T>
        constexpr T degrees(const T radians)
        {
            return radians * (static_cast<T>(180) / static_cast<T>(MATH_PI));
        }

/** Save way to check if float is zero */
        inline bool isZero(const float val)
        {
            return fabsf(val - 0.0f) < FLT_EPSILON;
        }

/** Save way to check if double is zero */
        inline bool isZero(const double val)
        {
            return fabs(val - 0.0) < DBL_EPSILON;
        }
    }
}
