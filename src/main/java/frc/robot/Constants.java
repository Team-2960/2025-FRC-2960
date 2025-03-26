package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Value;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Pounds;

import org.opencv.core.Point;
import org.opencv.core.Scalar;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import frc.robot.Util.*;

public class Constants {

    public static final Voltage motorOff = Volts.of(0);

    public static class CommonConst {
        public static final Dimensionless revTBEncCountPerRev = Value.of(4096);
    }

    // Robot constants
    public static class RobotConst {
        public static final Time updatePeriod = Seconds.of(0.02);
        
        public static final Distance frameWidth = Inches.of(29.5);
        public static final Distance frameLength = Inches.of(29.5);
        public static final Distance frameDiag = Inches.of(Math.sqrt(Math.pow(frameWidth.in(Inches), 2) + Math.pow(frameLength.in(Inches), 2)));

        public static final Distance bumperThickness = Inches.of(3.25);
        public static final Distance wheelInset = Inches.of(1.75);

        public static final Distance fullWidth = frameWidth.plus(bumperThickness.times(2));
        public static final Distance fullLength = frameLength.plus(bumperThickness.times(2));
        public static final Distance fullDiag = frameDiag.plus(bumperThickness.times(2));

        public static final ModuleConfig moduleConfig = new ModuleConfig(0.0381, 5.450, 1, new DCMotor(12, 4.69, 257, 1.5, 106.33, 1), 257, 1);

        public static final Distance coralOffsetX = fullLength.div(2);
        public static final Distance coralOffsetY = Meters.of(0.276);
        public static final Angle coralOffsetR = Degrees.of(0);

        public static final Distance algaeOffsetX = fullLength.div(2).unaryMinus();
        public static final Distance algaeOffsetY = Meters.of(0);
        public static final Angle algaeOffsetR = Degrees.of(180);

        public static final Distance climberOffsetX = Inches.of(1.75);
        public static final Distance climberOffsetY = fullWidth.div(2).unaryMinus();
        public static final Angle climberOffsetR = Degrees.of(-90);

        public static final Transform2d coralOffset = new Transform2d(coralOffsetX, coralOffsetY, new Rotation2d(coralOffsetR));
        public static final Transform2d algeaOffset = new Transform2d(algaeOffsetX, algaeOffsetY, new Rotation2d(algaeOffsetR));
        public static final Transform2d climberOffset = new Transform2d(climberOffsetX, climberOffsetY, new Rotation2d(climberOffsetR));
    }

    // CAN IDs
    public static class CAN_IDS {
        public static final int elevatorMotor = 11;

        public static final int coralMotor = 13;

        public static final int algaeAngleMotor = 3;
        public static final int algaeRollerMotor = 4;

        public static final int climberMotor = 14;

        public static final int armMotor = 12;

        public static final int frontLeftDriveM = 9;
        public static final int frontLeftAngleM = 10;
        public static final int frontRightDriveM = 7;
        public static final int frontRightAngleM = 8;

        public static final int backLeftDriveM = 1;
        public static final int backLeftAngleM = 2;
        public static final int backRightDriveM = 5;
        public static final int backRightAngleM = 6;
    }

    // Digital Input Ports
    public static class DIO_PORTS {
        public static final int coralPresentPE = 9;
    }

    // PWM Ports
    public static class PWM_PORTS {
        public static final int ledPort = 0;
    }

    // Auton
    public static class AutonConst {
        public static final Distance autoClearance = Meters.of(.25);
        public static final Distance autonRampDownSpeed = Meters.of(0.5);  
        public static final LinearVelocity minSpeed = MetersPerSecond.of(2);

        //TODO Change MOI
        public static final RobotConfig robotConfig = new RobotConfig(52.16, 6.883, RobotConst.moduleConfig, RobotConst.fullWidth.in(Meters));
    }
    


    // Drive
    public static class DriveConst {
        public static final Dimensionless driveGearRatio = Value.of(5.08);
        public static final Distance wheelDiameter = Inches.of(2.95);
        public static final Distance wheelRadius = wheelDiameter.div(2);
        public static final Distance wheelCirc = wheelDiameter.times(Math.PI);
        public static final Distance distRatio = wheelCirc.div(driveGearRatio);
        public static final LinearVelocity velRatio = distRatio.div(Seconds.of(60));

        public static final Distance wheelXOffset = RobotConst.frameLength.div(2).minus(RobotConst.wheelInset);
        public static final Distance wheelYOffset = RobotConst.frameWidth.div(2).minus(RobotConst.wheelInset);

        public static final Translation2d frontLeftLocation = new Translation2d(
            wheelXOffset,
            wheelYOffset
        );

        public static final Translation2d frontRightLocation = new Translation2d(
            wheelXOffset,
            wheelYOffset.unaryMinus()
        );
        
        public static final Translation2d backLeftLocation = new Translation2d(
            wheelXOffset.unaryMinus(),
            wheelYOffset
        );
        
        public static final Translation2d backRightLocation = new Translation2d(
            wheelXOffset.unaryMinus(),
            wheelYOffset.unaryMinus()
        );
        
        public static final LinearVelocity maxSpeed = MetersPerSecond.of(4.5);
        public static final AngularVelocity maxAngularSpeed = RadiansPerSecond.of(1.5 * 2 * Math.PI);
        public static final AngularVelocity maxAutoAngularSpeed = RadiansPerSecond.of(0.5 * 2 * Math.PI);

        public static final Angle angleRampDistance = Degrees.of(10);
        public static final PIDParam angleAlignPID = new PIDParam(4, 0, 0.3);
    }

    // Swerve
    public static class SwerveConst {
        public static PIDParam drivePID = new PIDParam(.5, 0.0, 0.0);
        public static FFParam driveFF = FFParam.simpleMotor(0.0, 2.25, 0.0);

        public static PIDParam anglePID = new PIDParam(0.05, 0.0, 0.001);
        public static FFParam angleFF = FFParam.simpleMotor(0.1, 0.1, 0);

        public static final Angle angleRampRate = Degrees.of(7); 
        public static final Angle angleRampDist = Degrees.of(30);

        public static final AngularVelocity maxAngularSpeed = RadiansPerSecond.of(Math.PI * 4);
        public static final AngularAcceleration maxAngularAccel = RadiansPerSecondPerSecond.of(Math.PI * 10);
    }

    // Arm
    public static class ArmConst  {
        public static final PIDParam pid = new PIDParam(0.002, 0.0, 0.0);
        public static final FFParam ff = FFParam.arm(0.01 * 2, 0.02948 * 1.0, 0.025, 0.0);

        public static final Angle rampDownDist = Degrees.of(20);

        public static final AngularVelocity maxSpeed = DegreesPerSecond.of(90); 
        public static final AngularVelocity maxAutoSpeed = DegreesPerSecond.of(90);

        public static final Angle topLim = Degrees.of(95);
        public static final Angle botLim = Degrees.of(0);
    
        public static final Angle intakeAngle = Degrees.of(95);
        public static final Angle travelAngle = Degrees.of(85);
        public static final Angle L1CoralScoreAngle = Degrees.of(73);
        public static final Angle coralScoreAngle = Degrees.of(60);
        public static final Angle coralL4Angle = Degrees.of(60);
        public static final Angle algaeRemoveAngle = Degrees.of(25);
    }

    //Elevator
    public static class ElevConst {
        public static PIDParam pid = new PIDParam(0.0, 0.0, 0.0);
        public static FFParam ff = FFParam.elevator(0.22643, 4.4162, 0.33045, 0.32248);
        
        public static final Dimensionless gearRatio = Value.of(1.0/12.0 * 2.0);
        public static final Distance outputDiam = Inches.of(1.751);
        public static final Distance outputCircum = outputDiam.times(Math.PI);

        public static final Distance distScale = outputCircum.times(gearRatio);
        public static final LinearVelocity velScale = distScale.div(Seconds.of(60));
        public static final LinearVelocity maxAutoSpeed = InchesPerSecond.of(2);
        public static final Distance rampDownDist = Inches.of(10);
        public static final Distance posTol = Inches.of(.5);
        public static final Mass carrageMass = Pounds.of(10);

        public static final Distance topLim = Inches.of(57.5);
        public static final Distance botLim = Inches.of(0.25);

        public static final Distance intakePos = Inches.of(0);
        public static final Distance L1Pos = Inches.of(0);
        public static final Distance L2Pos = Inches.of(16);
        public static final Distance L3Pos = Inches.of(31.5);
        public static final Distance L4Pos = Inches.of(55.5);
        public static final Distance lowAlgaePos = Inches.of(17.7);
        public static final Distance highAlgaePos = Inches.of(36);
    }

    //End Effector
    public static class EndEffectorConst {
        public static final Voltage coralEjectVolt = Volts.of(4);
        public static final Voltage coralIntakeVolt = Volts.of(1.5);
        public static final Voltage algaeRemovalVolt = Volts.of(12);
        public static final Time coralEjectTime = Seconds.of(1);
    }

    //Algae Roller
    public static class AlgaeConst {
        public static final PIDParam anglePID = new PIDParam(0.0, 0.0, 0.0);
        public static final FFParam angleFF = FFParam.arm(0.15488, 5.0985, 0.64833, 0.48409);
        
        public static final AngularVelocity maxAngleAutoSpeed = RadiansPerSecond.of(.5); 
        public static final Angle angleRampDownDist = Degrees.of(30);

        public static final Angle topLim = Degrees.of(88);
        public static final Angle botLim = Degrees.of(10);

        public static final Voltage ejectVolt = Volts.of(-3);
        public static final Voltage intakeVolt = Volts.of(3);
    }

    // Climber
    public static class ClimberConst {
        public static final Dimensionless gearRatio = Value.of(1.0/75.0);
        public static final Distance spoolDiam = Inches.of(1.5);
        public static final Distance spoolCircum = spoolDiam.times(Math.PI);
        public static final Distance posScale = spoolCircum.times(gearRatio);

        public static final Angle extAngle = Rotations.of(0.325);
        public static final Angle retAngle = Rotations.of(0.127);

        public static final Voltage extVolt = Volts.of(-6);      
        public static final Voltage retVolt = Volts.of(12);     
        public static final Voltage resetVolt = Volts.of(6);    
    }

    // Cameras
    public static class CameraConst {
        public static final Transform3d robotToFrontCamera = new Transform3d(
            Inches.of(14), 
            Inches.of(0.125), 
            Inches.of(8.875),
            new Rotation3d(
                Degrees.of(0), 
                Degrees.of(-10), 
                Degrees.of(0)
            )
        );
        
        public static final Transform3d robotToLeftRearCamera = new Transform3d(
            Inches.of(-13.944), 
            Inches.of(13.944), 
            Inches.of(8.791), 
            new Rotation3d(
                Degrees.of(14.6),
                Degrees.of(14.6), 
                Degrees.of(135)
            )
        ); 
        
        public static final Transform3d robotToRightRearCamera = new Transform3d(
            Inches.of(-13.944), 
            Inches.of(-13.944), 
            Inches.of(8.791), 
            new Rotation3d(
                Degrees.of(-14.6),
                Degrees.of(14.6), 
                Degrees.of(-135)
            )
        );
    }

    // Driver Camera
    public static final class DriverCameraConst {
        public static final int width = 640;    // pixels
        public static final int height = 480;   // pixels

        public static final Point leftTop = new Point(100, 0);
        public static final Point leftBottom = new Point(100, height);
        public static final Point rightTop = new Point(width - 100, 0);
        public static final Point rightBottom = new Point(width - 100, height);

        public static final Scalar lineColor = new Scalar(0, 255, 0);
        public static final int lineWidth = 1;  // pixel
    }

    // LED Control
    public static final class LEDConst {
        public static final int frontLEDCount = 30;
        public static final int leftLEDCount = 30;
        public static final int rightLEDCount = 30;
    }
}
