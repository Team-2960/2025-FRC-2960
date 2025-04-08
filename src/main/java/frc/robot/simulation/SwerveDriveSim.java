package frc.robot.simulation;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Translation2d;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.NewtonMeters;
import static edu.wpi.first.units.Units.Newtons;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularAcceleration;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutForce;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutTorque;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Time;
import frc.robot.subsystems.Swerve;

public class SwerveDriveSim {
    /**
     * Class defining a force vector
     */
    public class ForceVector {
        public final MutForce mag; // < Magnitude of the force
        public final MutAngle dir; // < Direction of the force

        /**
         * Constructor
         */
        public ForceVector() {
            mag = Newtons.mutable(0);
            dir = Radians.mutable(0);
        }
    }

    /**
     * Class to track module states
     */
    public class State {
        public Swerve module;

        public final MutVoltage driveVoltage;
        public final MutVoltage angleVoltage;
        public final MutLinearVelocity driveVel;
        public final MutAngularVelocity wheelSpeed;
        public final MutAngularVelocity angleVel;
        public final MutCurrent driveCurrent;
        public final MutCurrent angleCurrent;
        public final MutTorque driveTorque;
        public final MutTorque angleTorque;
        public final MutAngularAcceleration angleAccel;
        public final MutLinearVelocity driveFinalVel;
        public final MutAngularVelocity angleFinalVel;
        public final MutDistance driveChange;
        public final MutAngle angleChange;
        public final ForceVector driveForce;

        public final MutTorque robotTorque;

        /**
         * Constructor
         * 
         * @param module swerve module
         */
        public State(Swerve module) {
            this.module = module;

            driveVoltage = Volts.mutable(0);
            angleVoltage = Volts.mutable(0);
            driveVel = MetersPerSecond.mutable(0);
            wheelSpeed = RadiansPerSecond.mutable(0);
            angleVel = RadiansPerSecond.mutable(0);
            driveCurrent = Amps.mutable(0);
            angleCurrent = Amps.mutable(0);
            driveTorque = NewtonMeters.mutable(0);
            angleTorque = NewtonMeters.mutable(0);
            angleAccel = RadiansPerSecondPerSecond.mutable(0);
            driveFinalVel = MetersPerSecond.mutable(0);
            angleFinalVel = RadiansPerSecond.mutable(0);
            driveChange = Meters.mutable(0);
            angleChange = Radians.mutable(0);

            driveForce = new ForceVector();

            robotTorque = NewtonMeters.mutable(0);
        }

        /**
         * Update module states
         */
        public void update(Time period) {
            // Update drive parameters
            driveVoltage.mut_replace(module.getSimDriveVoltage());
            angleVoltage.mut_replace(module.getSimAngleVoltage());
            driveVel.mut_replace(module.getDriveVelocity());
            angleVel.mut_replace(module.getAngleRate());

            wheelSpeed.mut_replace(module.sim.getDriveRate(driveVel));
            driveCurrent.mut_replace(module.sim.getDriveCurrent(wheelSpeed, driveVoltage));
            driveTorque.mut_replace(module.sim.getDriveTorque(wheelSpeed, driveCurrent));
            driveForce.mag.mut_replace(module.sim.getDriveForce(driveTorque));

            angleCurrent.mut_replace(module.sim.getAngleCurrent(angleVel, angleVoltage));
            angleTorque.mut_replace(module.sim.getAngleTorque(angleVel, angleCurrent));
            angleAccel.mut_replace(module.sim.getAngleAccel(angleTorque));
            angleFinalVel.mut_replace(module.sim.getAngleRate(angleVel, angleAccel, period));
            driveForce.dir.mut_replace(module.getAnglePos().getRadians(), Radians);

            double forceTanAngle = driveForce.dir.in(Radians) - (module.translation.getAngle().getRadians());
            double tangentForce = Math.cos(forceTanAngle) * driveForce.mag.in(Newtons);
            double dist = module.translation.getDistance(new Translation2d(0, 0));
            robotTorque.mut_setMagnitude(tangentForce * dist);

            CurrentSimTracking.updateCurrent(module.name + " Drive", driveCurrent);
            CurrentSimTracking.updateCurrent(module.name + " Angle", angleCurrent);
        }

        /**
         * Update the drive distance and velocity
         * @param period    update period
         * @param xVel      x velocity
         * @param yVel      y velocity
         * @param rVel      angular velocity
         */
        public void updateDriveDist(Time period, LinearVelocity xVel, LinearVelocity yVel, AngularVelocity rVel) {
            double linMag = Math.sqrt(Math.pow(xVel.in(MetersPerSecond), 2) + Math.pow(yVel.in(MetersPerSecond), 2));
            double linAngle = Math.atan2(yVel.in(MetersPerSecond), xVel.in(MetersPerSecond));

            driveFinalVel.mut_setMagnitude(Math.cos(linAngle - driveForce.dir.in(Radians)) * linMag);

            double dist = module.translation.getDistance(new Translation2d(0, 0));
            double rotMag = rVel.in(RadiansPerSecond) * dist;
            double rotAngle = module.translation.getAngle().getRadians();

            angleFinalVel.mut_setMagnitude(Math.cos(rotAngle - driveForce.dir.in(Radians)) * rotMag);

            driveChange.mut_setMagnitude(driveFinalVel.in(MetersPerSecond) * period.in(Seconds));

            module.setSimState(driveChange, driveFinalVel, rDelta, angleFinalVel);
        }
    }

    private final State[] states;

    private final Mass robotMass;
    private final MomentOfInertia robotMOI;

    private final ForceVector forceVector = new ForceVector();
    private final MutTorque robotTorque = NewtonMeters.mutable(0);

    public final MutLinearVelocity xVelMut = MetersPerSecond.mutable(0);
    public final MutLinearVelocity yVelMut = MetersPerSecond.mutable(0);
    public final MutAngularVelocity rVelMut = RadiansPerSecond.mutable(0);

    public final MutAngle rDelta = Radians.mutable(0);

    /**
     * Constructor
     * 
     * @param modules   Swerve module objects
     * @param robotMass Simulated mass of the robot
     * @param robotMOI  Simulated Moment of Inertia of the robot
     */
    public SwerveDriveSim(Swerve[] modules, Mass robotMass, MomentOfInertia robotMOI) {
        states = new State[modules.length];
        for (int i = 0; i < modules.length; i++)
            states[i] = new State(modules[i]);

        this.robotMass = robotMass;
        this.robotMOI = robotMOI;
    }

    /**
     * Update the swerve drive simulation
     * @param xVelInit  initial x linear velocity
     * @param yVelInit  initial y linear velocity
     * @param rVelInit  initial angular velocity
     * @param period    update period
     * @return angular change of the robot position
     */
    public Angle update(LinearVelocity xVelInit, LinearVelocity yVelInit, AngularVelocity rVelInit, Time period) {
        // Zero force accumulators
        forceVector.mag.mut_setMagnitude(0);
        forceVector.dir.mut_setMagnitude(0);
        robotTorque.mut_setMagnitude(0);

        // Update module states
        for (var state : states) {
            state.update(period);

            forceVector.mag.mut_plus(state.driveForce.mag);
            forceVector.dir.mut_plus(state.driveForce.dir);

            robotTorque.mut_plus(state.robotTorque);
        }

        // Calculate the average force angle
        forceVector.dir.mut_divide(4);
        
        // Calculate 
        xVelMut.mut_replace(xVelInit);
        yVelMut.mut_replace(yVelInit);
        rVelMut.mut_replace(rVelInit);

        double linAccel = forceVector.mag.in(Newtons) * robotMass.in(Kilograms);
        double angAccel = robotTorque.in(NewtonMeters) * robotMOI.in(KilogramSquareMeters);

        xVelMut.mut_plus(linAccel * Math.cos(forceVector.dir.in(Radians) * period.in(Seconds)), MetersPerSecond);
        yVelMut.mut_plus(linAccel * Math.sin(forceVector.dir.in(Radians) * period.in(Seconds)), MetersPerSecond);
        rVelMut.mut_plus(angAccel * period.in(Seconds), RadiansPerSecond);

        for (var state : states) {
            state.updateDriveDist(period, xVelMut, yVelMut, rVelMut);
        }

        return rDelta.mut_setMagnitude(rVelMut.in(RadiansPerSecond) * period.in(Seconds));
    }

}
