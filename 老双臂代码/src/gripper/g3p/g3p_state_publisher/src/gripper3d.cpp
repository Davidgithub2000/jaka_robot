/*
 * File: gripper3d.c
 *
 * MATLAB Coder version            : 4.3
 * C/C++ source code generated on  : 14-Aug-2020 16:31:57
 */

/* Include Files */
//#include "gripper3d.h"
#include <math.h>

/* Function Definitions */

/*
 * Arguments    : double dalta
 *                double itheta[4]
 * Return Type  : void
 */
void gripper3d(double dalta, double itheta[4])
{
  double e;
  double itheta_tmp;
  if (dalta < 0.0) {
    dalta = 0.0;
  } else {
    if (dalta > 11.0) {
      dalta = 11.0;
    }
  }

  e = sqrt((60.0 - dalta) * (60.0 - dalta) + 324.0);
  itheta_tmp = e * e;
  itheta[0] = atan(18.0 / (60.0 - dalta)) + acos(((itheta_tmp + 400.0) - 2916.0)
    / (2.0 * e * 20.0));
  itheta[1] = 1.5707963267948966;
  itheta[2] = atan((60.0 - dalta) / 18.0) + acos(((itheta_tmp + 2916.0) - 400.0)
    / (2.0 * e * 54.0));
  itheta[3] = acos((3316.0 - itheta_tmp) / 2160.0);
}

/*
 * File trailer for gripper3d.c
 *
 * [EOF]
 */
