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

        double dir = b - a;

        if (Math.abs(dir) > 180.0) {
            dir = -(Math.signum(dir) * 360.0) + dir;
        }

        return dir;
    }

    // Find angle set point while handling angle wrapping.
    public static final double getAngleSetPoint(double desiredAngle, double currentAngle) {
        return currentAngle + angleDis(currentAngle, desiredAngle);
    }
}
