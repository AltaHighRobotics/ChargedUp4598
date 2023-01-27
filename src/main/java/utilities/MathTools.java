// Usefull stuff for autonomous.

/*
  @Hacker
  @New Hawks
*/

package utilities;

import java.lang.Math;

public final class MathTools {

    // The gryo angle ranges from 180 to -180. This function convects that to a normal angle.
    public static final double makeNonNegAngle(double angle) {
        if (angle >= 0.0) {
            return angle;
        }

        return 360.0 + angle;
    }

    // Find the distance between two angles.
    public static final double angleDis(double a1, double a2) {
        double a, b;
        a = makeNonNegAngle(a1);
        b = makeNonNegAngle(a2);
        
        return 180.0 - Math.abs(Math.abs(a - b) - 180.0);
    }
}
