package frc.robot.Util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class Limits {
    /**
     * Checks if an angle in degrees is within a given range
     * 
     * @param min   minimum angle in degrees
     * @param max   maximum angle in degrees
     * @param value value in degrees to check
     * @return true if the value is within the range
     */
    public static boolean inRangeDeg(double min, double max, double value) {
        // Sanitize the value
        // TODO improve sanitization method
        while (value > 360)
            value -= 360;
        while (value < -360)
            value += 360;

        boolean result = value > min && value < max;
        result |= value > min + 360 && value < max + 360;
        result |= value > min - 360 && value < max - 360;

        return result;
    }

    /**
     * Checks if an angle in degrees is within tolerance of a target
     * 
     * @param target target angle in degrees
     * @param tol    target tolerance in degrees
     * @param value  value in degrees to check
     * @return true if the value is within tolerance of the target
     */
    public static boolean inTolDeg(double target, double tol, double value) {
        return inRangeDeg(target - tol, target + tol, value);
    }

    /**
     * Checks if value is withing a given range
     * 
     * @param min   minimum value
     * @param max   maximum value
     * @param value value to check
     * @return true if the value is within the range
     */
    public static boolean inRange(Rotation2d min, Rotation2d max, Rotation2d value) {
        return inRangeDeg(min.getDegrees(), max.getDegrees(), value.getDegrees());
    }

    /**
     * Checks if a value is within tolerance of a target
     * 
     * @param target target value
     * @param tol    target tolerance
     * @param value  value to check
     * @return true if the value is within tolerance of the target
     */
    public static boolean inTol(Rotation2d target, Rotation2d tol, Rotation2d value) {
        return inTolDeg(target.getDegrees(), tol.getDegrees(), value.getDegrees());
    }

    /**
     * Checks if value is withing a given range
     * 
     * @param min   minimum value
     * @param max   maximum value
     * @param value value to check
     * @return true if the value is within the range
     */
    public static boolean inRange(Angle min, Angle max, Angle value) {
        return inRangeDeg(min.in(Degrees), max.in(Degrees), value.in(Degrees));
    }

    /**
     * Checks if a value is within tolerance of a target
     * 
     * @param target target value
     * @param tol    target tolerance
     * @param value  value to check
     * @return true if the value is within tolerance of the target
     */
    public static boolean inTol(Angle target, Angle tol, Angle value) {
        return inTolDeg(target.in(Degrees), tol.in(Degrees), value.in(Degrees));
    }

    /**
     * Checks if value is withing a given range
     * 
     * @param min   minimum value
     * @param max   maximum value
     * @param value value to check
     * @return true if the value is within the range
     */
    public static boolean inRange(double min, double max, double value) {
        return min < value && max > value;
    }

    /**
     * Checks if a value is within tolerance of a target
     * 
     * @param target target value
     * @param tol    target tolerance
     * @param value  value to check
     * @return true if the value is within tolerance of the target
     */
    public static boolean inTol(double target, double tol, double value) {
        return inRange(target - tol, target + tol, value);
    }

    /**
     * Checks if value is withing a given range
     * 
     * @param min   minimum value
     * @param max   maximum value
     * @param value value to check
     * @return true if the value is within the range
     */
    public static boolean inRange(Distance min, Distance max, Distance value) {
        return inRange(min.in(Meters), max.in(Meters), value.in(Meters));
    }

    /**
     * Checks if a value is within tolerance of a target
     * 
     * @param target target value
     * @param tol    target tolerance
     * @param value  value to check
     * @return true if the value is within tolerance of the target
     */
    public static boolean inTol(Distance target, Distance tol, Distance value) {
        return inTol(target.in(Meters), tol.in(Meters), value.in(Meters));
    }

    /**
     * Checks if value is withing a given range
     * 
     * @param min   minimum value
     * @param max   maximum value
     * @param value value to check
     * @return true if the value is within the range
     */
    public static boolean inRange(LinearVelocity min, LinearVelocity max, LinearVelocity value) {
        return inRange(min.in(MetersPerSecond), max.in(MetersPerSecond), value.in(MetersPerSecond));
    }

    /**
     * Checks if a value is within tolerance of a target
     * 
     * @param target target value
     * @param tol    target tolerance
     * @param value  value to check
     * @return true if the value is within tolerance of the target
     */
    public static boolean inTol(LinearVelocity target, LinearVelocity tol, LinearVelocity value) {
        return inTol(target.in(MetersPerSecond), tol.in(MetersPerSecond), value.in(MetersPerSecond));
    }

    /**
     * Checks if value is withing a given range
     * 
     * @param min   minimum value
     * @param max   maximum value
     * @param value value to check
     * @return true if the value is within the range
     */
    public static boolean inRange(AngularVelocity min, AngularVelocity max, AngularVelocity value) {
        return inRange(min.in(DegreesPerSecond), max.in(DegreesPerSecond), value.in(DegreesPerSecond));
    }

    /**
     * Checks if a value is within tolerance of a target
     * 
     * @param target target value
     * @param tol    target tolerance
     * @param value  value to check
     * @return true if the value is within tolerance of the target
     */
    public static boolean inTol(AngularVelocity target, AngularVelocity tol, AngularVelocity value) {
        return inTol(target.in(DegreesPerSecond), tol.in(DegreesPerSecond), value.in(DegreesPerSecond));
    }

    /**
     * Checks if a value is within tolerance of a target
     * 
     * @param target target value
     * @param tol    target tolerance
     * @param value  value to check
     * @return true if the value is within tolerance of the target
     */
    public static boolean inTol(Translation2d target, double tol, Translation2d value) {
        return target.getDistance(value) > tol;
    }

    /**
     * Checks if a value is within tolerance of a target
     * 
     * @param target target value
     * @param tol    target tolerance
     * @param value  value to check
     * @return true if the value is within tolerance of the target
     */
    public static boolean inTol(Translation2d target, Distance tol, Translation2d value) {
        return inTol(target, tol.in(Meters), value);
    }
}
