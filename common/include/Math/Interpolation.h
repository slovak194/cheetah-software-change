/*! @file Interpolation.h
 *  @brief Utility functions to interpolate between two values
 *
 */

#ifndef PROJECT_INTERPOLATION_H
#define PROJECT_INTERPOLATION_H

#include <assert.h>
#include <type_traits>

namespace Interpolate {

/*!
 * Linear interpolation between y0 and yf.  x is between 0 and 1
 */
template <typename y_t, typename x_t>
y_t lerp(y_t y0, y_t yf, x_t x) {
  static_assert(std::is_floating_point<x_t>::value,
                "must use floating point value");
  assert(x >= 0 && x <= 1);
  if(x < 0)x = 0;
  if(x > 1)x = 1;
  return y0 + (yf - y0) * x;
}

/*!
 * Cubic bezier interpolation between y0 and yf.  x is between 0 and 1
 */
template <typename y_t, typename x_t>
y_t cubicBezier(y_t y0, y_t yf, x_t x) {
  static_assert(std::is_floating_point<x_t>::value,
                "must use floating point value");

  assert(x >= 0 && x <= 1);
  if(x < 0)x = 0;
  if(x > 1)x = 1;

  y_t yDiff = yf - y0;
  x_t bezier = x * x * x + x_t(3) * (x * x * (x_t(1) - x));
  return y0 + bezier * yDiff;
}

/*!
 * Cubic bezier interpolation derivative between y0 and yf.  x is between 0 and
 * 1
 */
template <typename y_t, typename x_t>
y_t cubicBezierFirstDerivative(y_t y0, y_t yf, x_t x) {
  static_assert(std::is_floating_point<x_t>::value,
                "must use floating point value");
  assert(x >= 0 && x <= 1);
  if(x < 0)x = 0;
  if(x > 1)x = 1;

  y_t yDiff = yf - y0;
  x_t bezier = x_t(6) * x * (x_t(1) - x);
  return bezier * yDiff;
}

/*!
 * Cubic bezier interpolation derivative between y0 and yf.  x is between 0 and
 * 1
 */
template <typename y_t, typename x_t>
y_t cubicBezierSecondDerivative(y_t y0, y_t yf, x_t x) {
  static_assert(std::is_floating_point<x_t>::value,
                "must use floating point value");
  assert(x >= 0 && x <= 1);
  if(x < 0)x = 0;
  if(x > 1)x = 1;
  
  y_t yDiff = yf - y0;
  x_t bezier = x_t(6) - x_t(12) * x;
  return bezier * yDiff;
}

//一种先加速，后减速，前半截快，后半截慢的贝塞尔曲线
const float y[101] = {
0.0000, 0.0002, 0.0007, 0.0028, 0.0063, 0.0109, 0.0166, 0.0234, 0.0311, 0.0398, 
0.0494, 0.0598, 0.0709, 0.0828, 0.0953, 0.1084, 0.1221, 0.1363, 0.1510, 0.1661,
0.1816, 0.1974, 0.2135, 0.2299, 0.2465, 0.2633, 0.2803, 0.2974, 0.3146, 0.3319, 
0.3493, 0.3666, 0.3839, 0.4012, 0.4185, 0.4357, 0.4528, 0.4697, 0.4865, 0.5032, 
0.5197, 0.5360, 0.5521, 0.5681, 0.5837, 0.5992, 0.6144, 0.6293, 0.6440, 0.6584, 
0.6725, 0.6863, 0.6999, 0.7131, 0.7260, 0.7386, 0.7509, 0.7629, 0.7746, 0.7860,
0.7970, 0.8077, 0.8181, 0.8282, 0.8379, 0.8474, 0.8565, 0.8653, 0.8738, 0.8820, 
0.8898, 0.8974, 0.9047, 0.9116, 0.9183, 0.9247, 0.9308, 0.9366, 0.9421, 0.9474, 
0.9523, 0.9571, 0.9615, 0.9657, 0.9696, 0.9733, 0.9767, 0.9799, 0.9829, 0.9856,
0.9880, 0.9903, 0.9923, 0.9940, 0.9956, 0.9969, 0.9980, 0.9988, 0.9994, 0.9999, 
1.0000 };
const float y1[101] = {
0.0000, 0.0161, 0.1517, 0.2803, 0.4020, 0.5171, 0.6257, 0.7280, 0.8242, 0.9145, 
0.9990, 1.0779, 1.1515, 1.2198, 1.2831, 1.3414, 1.3951, 1.4441, 1.4888, 1.5292, 
1.5655, 1.5978, 1.6263, 1.6512, 1.6725, 1.6905, 1.7052, 1.7168, 1.7254, 1.7312, 
1.7342, 1.7347, 1.7326, 1.7283, 1.7217, 1.7129, 1.7022, 1.6895, 1.6751, 1.6590, 
1.6413, 1.6221, 1.6015, 1.5796, 1.5565, 1.5322, 1.5070, 1.4808, 1.4537, 1.4259, 
1.3973, 1.3681, 1.3383, 1.3081, 1.2774, 1.2463, 1.2149, 1.1833, 1.1515, 1.1196, 
1.0875, 1.0555, 1.0234, 0.9914, 0.9595, 0.9277, 0.8961, 0.8646, 0.8334, 0.8025, 
0.7718, 0.7415, 0.7115, 0.6818, 0.6524, 0.6235, 0.5949, 0.5667, 0.5390, 0.5116, 
0.4846, 0.4580, 0.4318, 0.4060, 0.3805, 0.3554, 0.3307, 0.3063, 0.2821, 0.2583, 
0.2347, 0.2114, 0.1883, 0.1653, 0.1425, 0.1197, 0.0970, 0.0744, 0.0517, 0.0289,
0.0000 };
const float y2[101] = {
14.6556, 13.9215, 13.2071, 12.5121, 11.8364, 11.1796, 10.5415, 9.9218, 9.3203, 
8.7368, 8.1709, 7.6224, 7.0911, 6.5767, 6.0790, 5.5977, 5.1325, 4.6832, 4.2496, 
3.8313, 3.4282, 3.0399, 2.6663, 2.3071, 1.9619, 1.6307, 1.3130, 1.0087, 0.7175,
 0.4391, 0.1733, -0.0801, -0.3214, -0.5510, -0.7689, -0.9756, -1.1711, -1.3559, 
-1.5301, -1.6940, -1.8478, -1.9919, -2.1264, -2.2516, -2.3678, -2.4752, -2.5741,
 -2.6647, -2.7473, -2.8222, -2.8895, -2.9496, -3.0026, -3.0490, -3.0888, -3.1224, 
-3.1501, -3.1720, -3.1884, -3.1996, -3.2058, -3.2073, -3.2044, -3.1973, -3.1862, 
-3.1714, -3.1532, -3.1318, -3.1075, -3.0805, -3.0510, -3.0194, -2.9859, -2.9507,
 -2.9141, -2.8764, -2.8377, -2.7984, -2.7587, -2.7189, -2.6792, -2.6399, -2.6012,
 -2.5634, -2.5267, -2.4914, -2.4578, -2.4260, -2.3964, -2.3692, -2.3447, -2.3230,
 -2.3046, -2.2895, -2.2782, -2.2707, -2.2675, -2.2687, -2.2746, -2.2854, -2.3015
};
/*!
 * Ease Cubic bezier interpolation between y0 and yf.  x is between 0 and 1
 */
template <typename y_t, typename x_t>
y_t easeCubicBezier(y_t y0, y_t yf, x_t x) {
  static_assert(std::is_floating_point<x_t>::value,
                "must use floating point value");

  assert(x >= 0 && x <= 1);
  if(x < 0)x = 0;
  if(x > 1)x = 1;

  y_t yDiff = yf - y0;
  x_t bezier = y[(int)(x*100)];
  return y0 + bezier * yDiff;
}
/*!
 * Ease Cubic bezier interpolation derivative between y0 and yf.  x is between 0 and
 * 1
 */
template <typename y_t, typename x_t>
y_t easeCubicBezierFirstDerivative(y_t y0, y_t yf, x_t x) {
  static_assert(std::is_floating_point<x_t>::value,
                "must use floating point value");
  assert(x >= 0 && x <= 1);
  if(x < 0)x = 0;
  if(x > 1)x = 1;

  y_t yDiff = yf - y0;
  x_t bezier = y1[(int)(x*100)];
  return bezier * yDiff;
}

/*!
 * Ease Cubic bezier interpolation derivative between y0 and yf.  x is between 0 and
 * 1
 */
template <typename y_t, typename x_t>
y_t easeCubicBezierSecondDerivative(y_t y0, y_t yf, x_t x) {
  static_assert(std::is_floating_point<x_t>::value,
                "must use floating point value");
  assert(x >= 0 && x <= 1);
  if(x < 0)x = 0;
  if(x > 1)x = 1;
  
  y_t yDiff = yf - y0;
  x_t bezier = y2[(int)(x*100)];
  return bezier * yDiff;
}

}  // namespace Interpolate

#endif  // PROJECT_INTERPOLATION_H
