package frc.robot.simulation;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Newtons;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.NewtonMeters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Value;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularAcceleration;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutForce;
import edu.wpi.first.units.measure.MutTorque;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Torque;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Util.FFParam;

public class SwerveSim {
    private final DCMotor driveMotor;
    private final DCMotor angleMotor;

    private final Dimensionless driveGR;
    private final Dimensionless angleGR;

    private final Distance wheelDiam;
    private final Distance wheelCircum;

    private final Torque driveFriction;
    private final Torque angleFriction;

    private final MomentOfInertia angleMOI;

    private final MutAngularVelocity driveAngVel;
    private final MutCurrent driveCurrent;
    private final MutTorque driveTorque;
    private final MutForce driveForce;
    private final MutAngle driveRot;

    private final MutCurrent angleCurrent;
    private final MutTorque angleTorque;
    private final MutAngularVelocity angleVel;
    private final MutAngularAcceleration angleAccel;
    private final MutAngle angleRot;

    /**
     * Constructor
     * @param driveMotor    Drive motor simulation
     * @param angleMotor    Angle motor simulation
     * @param driveGR       Drive gear ratio
     * @param angleGR       Angle gear ratio
     * @param angleFF       Angle Feed Forward constants
     */
    public SwerveSim(DCMotor driveMotor, DCMotor angleMotor, Distance wheelDiam, Dimensionless driveGR, Dimensionless angleGR, FFParam driveFF, FFParam angleFF, MomentOfInertia angleMOI) {
        this.driveMotor = driveMotor;
        this.angleMotor = angleMotor;

        this.wheelDiam = wheelDiam;
        this.wheelCircum = wheelDiam.times(Math.PI);

        this.driveGR = driveGR;
        this.angleGR = angleGR;
        
        driveFriction = NewtonMeters.of(driveMotor.stallTorqueNewtonMeters/driveMotor.freeSpeedRadPerSec * driveMotor.KvRadPerSecPerVolt*driveFF.kS);
        angleFriction = NewtonMeters.of(angleMotor.stallTorqueNewtonMeters/angleMotor.freeSpeedRadPerSec * angleMotor.KvRadPerSecPerVolt*angleFF.kS);

        this.angleMOI = angleMOI;

        driveAngVel = RotationsPerSecond.mutable(0);
        driveTorque = NewtonMeters.mutable(0);
        driveCurrent = Amps.mutable(0);
        driveForce = Newtons.mutable(0);
        driveRot = Rotations.mutable(0);

        angleCurrent = Amps.mutable(0);
        angleTorque = NewtonMeters.mutable(0);
        angleVel = RadiansPerSecond.mutable(0);
        angleAccel = RadiansPerSecondPerSecond.mutable(0);
        angleRot = Radians.mutable(0);
    }

    /**
     * Convert from linear velocity of module to drive wheel angular rate
     * @param moduleSpeed   linear velocity of the swerve module at the center of the drive wheel
     * @return  angular velocity of the drive wheel
     */
    public AngularVelocity getDriveRate(LinearVelocity moduleSpeed) {
        return driveAngVel.mut_setMagnitude(moduleSpeed.in(MetersPerSecond) / wheelCircum.in(Meters) * 2 * Math.PI);
    }

    /**
     * Calculate the drive motor current 
     * @param wheelSpeed    current speed of the drive wheel
     * @param voltage       voltage applied to the drive motor
     * @return  current through the drive motor
     */
    public Current getDriveCurrent(AngularVelocity wheelSpeed, Voltage voltage) {
        return driveCurrent.mut_setMagnitude(driveMotor.getCurrent(wheelSpeed.in(RadiansPerSecond) * driveGR.in(Value), voltage.in(Volts)));
    }

    /**
     * Calculate the working torque applied to the drive wheel (torque from friction has been accounted for)
     * @param wheelSpeed    current speed of the drive wheel
     * @param current       current through the drive motor
     * @return  force applied by the drive wheel
     */
    public Torque getDriveTorque(AngularVelocity wheelSpeed, Current current) {
        double motorTorque = driveMotor.getTorque(current.in(Amps));
        double outputTorque = 0;
        
        if(Math.abs(wheelSpeed.in(RadiansPerSecond)) > 0) {
            outputTorque = motorTorque + (wheelSpeed.in(RadiansPerSecond) > 0 ? -1 : 1) * driveFriction.in(NewtonMeters);
        } else {
            if(Math.abs(motorTorque) > driveFriction.in(NewtonMeters)) {
                outputTorque = motorTorque + (motorTorque > 0 ? -1 : 1) * driveFriction.in(NewtonMeters);
            }
        }

        return driveTorque.mut_setMagnitude(outputTorque * driveGR.in(Value));
    }

    /**
     * Calculate the drive wheel force
     * @param torque    torque applied to the drive wheel
     * @return  force applied by the drive wheel
     */
    public Force getDriveForce(Torque torque) {
        return driveForce.mut_setMagnitude(torque.in(NewtonMeters) / wheelDiam.in(Meters));
    }

    /**
     * Calculate the rotation of the motor based on the distance traveled
     * @param driveDist distance traveled by the swerve module
     * @return  Rotations of the motor
     */
    public Angle getDriveDistance(Distance driveDist) {
        return driveRot.mut_setMagnitude(driveDist.in(Meters) / wheelCircum.in(Meters) * driveGR.in(Value));
    }
    
    /**
     * Calculate the current through the angle motor
     * @param angleRate current angle rate of the swerve module
     * @param voltage   applied voltage to the angle motor
     * @return  current through the angle motor
     */
    public Current getAngleCurrent(AngularVelocity angleRate, Voltage voltage) {
        return angleCurrent.mut_setMagnitude(driveMotor.getCurrent(angleRate.in(RadiansPerSecond) * angleGR.in(Value), voltage.in(Volts)));
    }

    /**
     * Calculate the working torque applied to the module angle (torque from friction has been accounted for)
     * @param angleRate current angle rate of the angle motor
     * @param current   current through angle motor
     * @return  torque applied by the angle motor
     */
    public Torque getAngleTorque(AngularVelocity angleRate, Current current) {
        double motorTorque = angleMotor.getTorque(current.in(Amps));
        double outputTorque = 0;
        
        if(Math.abs(angleRate.in(RadiansPerSecond)) > 0) {
            outputTorque = motorTorque + (angleRate.in(RadiansPerSecond) > 0 ? -1 : 1) * angleFriction.in(NewtonMeters);
        } else {
            if(Math.abs(motorTorque) > angleFriction.in(NewtonMeters)) {
                outputTorque = motorTorque + (motorTorque > 0 ? -1 : 1) * angleFriction.in(NewtonMeters);
            }
        }

        return angleTorque.mut_setMagnitude(outputTorque * angleGR.in(Value));
    }

    /**
     * Calculate the angular acceleration of the module
     * @param torque    working torque applied to the rotation axis 
     * @return
     */
    public AngularAcceleration getAngleAccel(Torque torque) {
        return angleAccel.mut_setMagnitude(torque.in(NewtonMeters) / angleMOI.in(KilogramSquareMeters));
    }

    /**
     * Calculate the updated angular velocity
     * @param rate      current angle rate
     * @param accel     angular acceleration
     * @param period     update period
     * @return  updated angle rate
     */
    public AngularVelocity getAngleRate(AngularVelocity rate, AngularAcceleration accel, Time period) {
        return angleVel.mut_setMagnitude(rate.in(RadiansPerSecond) + accel.in(RadiansPerSecondPerSecond) * period.in(Seconds));
    }

    /**
     * Calculate the angle change in the swerve module
     * @param rate      starting angle rate
     * @param accel     acceleration of the module
     * @param period    update period
     * @return  change in angle of the swerve module
     */
    public Angle getAngleChange(AngularVelocity rate, AngularAcceleration accel, Time period) {
        double deltaSec = period.in(Seconds);
        return angleRot.mut_setMagnitude(rate.in(RadiansPerSecond) * deltaSec + 
            accel.in(RadiansPerSecondPerSecond) * deltaSec * deltaSec);
    }
}
