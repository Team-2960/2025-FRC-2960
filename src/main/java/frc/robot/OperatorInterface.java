package frc.robot;

import frc.robot.Constants.DriveConst;
import frc.robot.Util.FieldLayout;
import frc.robot.subsystems.AlgaeAngle;
import frc.robot.subsystems.AlgaeRoller;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.ElevArmControl;
import frc.robot.subsystems.EndEffector;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class OperatorInterface {
    // INSTANCE
    public static OperatorInterface oi = null;

    // JOYSTICKS
    private CommandXboxController driverController;
    private CommandXboxController operatorController;

    private MutLinearVelocity xSpeed;
    private MutLinearVelocity ySpeed;
    private MutAngularVelocity rSpeed;

    /**
     * Constructor
     */
    public OperatorInterface(
        Drive drive, 
        ElevArmControl elevArmControl, 
        EndEffector endEffector, 
        AlgaeAngle algaeAngle, 
        AlgaeRoller algaeRoller, 
        Climber climber
    ) {
        // Create Joysticks
        driverController = new CommandXboxController(0);
        operatorController = new CommandXboxController(1);

        xSpeed = MetersPerSecond.mutable(0);
        ySpeed = MetersPerSecond.mutable(0);
        rSpeed = RadiansPerSecond.mutable(0);

        driveTriggers(drive);
        coralPlacementTriggers(elevArmControl, endEffector);
        algaeGrabberTriggers(algaeAngle, algaeRoller);
        climberTriggers(climber);
    }
    
    /**
     * Setup drivetrain triggers
     */
    private void driveTriggers(Drive drive){
        // Set drive default command
        drive.setDefaultCommand(drive.new RateControlCommand(this::getXSpeed, this::getYSpeed, this::getAngleRate));

        // Goto nearest branch
        driverController.y()
            .onTrue(drive.new GotoNearestBranchCommand(Constants.RobotConst.coralOffset));

        // Align to -45 degrees 
        driverController.x()
            .onTrue(
                drive.new AngleAlignCommand(
                    this::getXSpeed, 
                    this::getYSpeed, 
                    Rotation2d.fromDegrees(-45)
                )
            );

        // Align to Feeder station
        driverController.b()
            .onTrue(
                drive.new AngleAlignCommand(
                    this::getXSpeed, 
                    this::getYSpeed, 
                    Rotation2d.fromDegrees(54)
                )
            );

        // Align to driver to Cage
        driverController.a()
            .onTrue(
                drive.new AngleAlignCommand(
                    this::getXSpeed, 
                    this::getYSpeed, 
                    Rotation2d.fromDegrees(-90)
                )
            );

    }

    /**
     * Setup coral placement triggers
     */
    private void coralPlacementTriggers(ElevArmControl elevArmControl, EndEffector endEffector){
        //Elevator Arm Triggers
        operatorController.y().onTrue(elevArmControl.getGoToL4Command());
        operatorController.x().onTrue(elevArmControl.getGoToL3Command());
        operatorController.b().onTrue(elevArmControl.getGoToL2Command());
        operatorController.a().onTrue(elevArmControl.getGoToIntakeCommand());
        operatorController.pov(90).onTrue(elevArmControl.getGoToLowAlgaeCommand());
        operatorController.pov(270).onTrue(elevArmControl.getGoToHighAlgaeCommand());

        driverController.axisMagnitudeGreaterThan(3, 0.1)
            .whileTrue(endEffector.new EjectCmd());
        
        driverController.axisMagnitudeGreaterThan(2, 0.1)
            .whileTrue(endEffector.new IntakeCmd());

        operatorController.axisMagnitudeGreaterThan(3, 0.1)
            .whileTrue(endEffector.new ReverseCmd());
            
        operatorController.axisMagnitudeGreaterThan(2, 0.1)
            .whileTrue(endEffector.new EjectCmd());
    }

    /**
     * Setup Algea Grabber Triggers
     */
    private void algaeGrabberTriggers(AlgaeAngle algaeAngle, AlgaeRoller algaeRoller){
        // TODO move algea presets to constants
        operatorController.pov(0)
            .onTrue(algaeAngle.new AngleCommand(Rotation2d.fromDegrees(20)));
        
        operatorController.pov(180)
            .onTrue(algaeAngle.new AngleCommand(Rotation2d.fromDegrees(80)));

        operatorController.rightBumper().whileTrue(algaeRoller.new EjectCmd());
        operatorController.leftBumper().whileTrue(algaeRoller.new IntakeCmd());
    }

    /**
     * Setup Climber Triggers
     */
    private void climberTriggers(Climber climber){
        driverController.pov(90).whileTrue(climber.new ExtendCmd());
        driverController.pov(270).whileTrue(climber.new RetractCmd());

        operatorController.start().whileTrue(climber.new SetExtPosCommand());

        operatorController.back().whileTrue(climber.new SetRetPosCommand());
    }
    /**
     * Source method for getting the target x speed
     * @return  target x speed
     */
    public LinearVelocity getXSpeed() {
        double axis = MathUtil.applyDeadband(driverController.getLeftX(), 0.05);
        return xSpeed.mut_replace(axis * getMaxSpeed() * getAllianceDir(), MetersPerSecond);
    }

    /**
     * Source method for getting the target y speed
     * @return  target y speed
     */
    public LinearVelocity getYSpeed() {
        double axis = MathUtil.applyDeadband(driverController.getLeftY(), 0.05);
        return ySpeed.mut_replace(axis * getMaxSpeed() * getAllianceDir(), MetersPerSecond);
    }

    /**
     * Source method for getting the target angle rate
     * @return  target angle rate
     */
    public AngularVelocity getAngleRate() {
        double axis = MathUtil.applyDeadband(driverController.getRightY(), 0.1);
        return rSpeed.mut_replace(axis * getMaxAngleSpeed() * -1, RadiansPerSecond);
    }

    /**
     * Checks if the robot should be in slowdown mode
     * @return
     */
    public boolean isSlowDown() {
        return driverController.getHID().getRawButton(5);
    }

    /**
     * Gest the maximum linear speed of the robot
     * @return
     */
    public double getMaxSpeed() {
        return (isSlowDown() ? .5 : 1) * DriveConst.maxSpeed.in(MetersPerSecond);
    }

    public double getMaxAngleSpeed() {
        return (isSlowDown() ? .5 : 1) * DriveConst.maxAngularSpeed.in(RadiansPerSecond);
    }

    public double getAllianceDir() {
        return FieldLayout.isRedAlliance() ? 1 : -1;
    }
}