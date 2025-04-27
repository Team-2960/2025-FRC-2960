package frc.lib2960.subsystems;

import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Velocity;
import frc.lib2960.util.FFParam;
import frc.lib2960.util.PIDParam;

public class MotorMechanismSettings {
    public final boolean isInverted;
    public final PIDController pidController;
    public final SimpleMotorFeedforward ffController;
    public final double trapezoidalRampDistance;
    public final AngularVelocity maxSpeed;
    public final boolean defaultHold;


    public MotorMechanismSettings(boolean isInverted, boolean defaultHold){
        this.isInverted = isInverted;
        this.pidController = new PIDController(0, 0, 0);
        this.ffController = new SimpleMotorFeedforward(0, 0, 0);
        this.trapezoidalRampDistance = 1;
        this.defaultHold = defaultHold;
        maxSpeed = RotationsPerSecond.of(0);
    }

    public MotorMechanismSettings(boolean isInverted, boolean defaultHold, PIDController pidController, SimpleMotorFeedforward ffController){
        this.isInverted = isInverted;
        this.pidController = pidController;
        this.ffController = ffController;
        this.trapezoidalRampDistance = 1;
        this.defaultHold = defaultHold;
        maxSpeed = RotationsPerSecond.of(0);
    }

    public MotorMechanismSettings(boolean isInverted, boolean defaultHold, PIDController pidController, SimpleMotorFeedforward ffController, double trapezoidalRampDistance, AngularVelocity maxSpeed){
        this.isInverted = isInverted;
        this.pidController = pidController;
        this.ffController = ffController;
        this.trapezoidalRampDistance = trapezoidalRampDistance;
        this.defaultHold = defaultHold;
        this.maxSpeed = maxSpeed;
    }
}
