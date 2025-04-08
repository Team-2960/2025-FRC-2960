package frc.robot;

import frc.robot.Util.FieldLayout;
import frc.robot.Util.FieldLayout.ReefFace;
import frc.robot.subsystems.AlgaeAngle;
import frc.robot.subsystems.AlgaeRoller;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.ElevArmControl;
import frc.robot.subsystems.EndEffector;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class OperatorInterface extends SubsystemBase {
    // INSTANCE
    public static OperatorInterface oi = null;

    // JOYSTICKS
    private CommandXboxController driverController;
    private CommandXboxController operatorController;


    //Manual Control Class Variables
    double xSpeed;
    double ySpeed;
    double rSpeed;
    
    // Shuffleboard Entries
    private GenericEntry sb_driveX;
    private GenericEntry sb_driveY;
    private GenericEntry sb_driveR;
    private GenericEntry sb_driveFR;

    private GenericEntry sb_matchTimer;

    /**
     * Constructor
     */
    private OperatorInterface() {
        // Create Joysticks
        driverController = new CommandXboxController(0);
        operatorController = new CommandXboxController(1);

        // Setup Shuffleboard
        var drive_layout = Shuffleboard.getTab("OI")
                .getLayout("Drive", BuiltInLayouts.kList)
                .withSize(2, 4);
        sb_driveX = drive_layout.add("X Speed", 0).getEntry();
        sb_driveY = drive_layout.add("Y Speed", 0).getEntry();
        sb_driveR = drive_layout.add("R Speed", 0).getEntry();
        sb_driveFR = drive_layout.add("Field Relative", true).getEntry();

        var match_info = Shuffleboard.getTab("OI")
            .getLayout("Match Info", BuiltInLayouts.kList)
            .withSize(2, 4);

        sb_matchTimer = match_info.add("Match Timer", -1).getEntry();


        driveTriggers();
        coralPlacementTriggers();
        algaeGrabberTriggers();
        climberTriggers();

    }
    

    private void driveTriggers(){
        Drive drive = Drive.getInstance();

        
        driverController.y()
            .onTrue(drive.linearDriveCommands
                .new LinearGoToReefCommand(
                    Constants.centerOffset))
            .onTrue(drive.rotationDriveCommands
                .new RotGoToReefCommand(Rotation2d.fromDegrees(0))
        );

        driverController.y().and(driverController.rightBumper())
            .whileTrue(
                drive.linearDriveCommands
                .new LinearGoToReefCommand(
                    Constants.rightBranchOffset))
            .whileTrue(drive.rotationDriveCommands
                .new RotGoToReefCommand(Rotation2d.fromDegrees(0))
        );
        
        driverController.y().and(driverController.leftBumper())
            .whileTrue(
                drive.linearDriveCommands
                .new LinearGoToReefCommand(
                    Constants.leftBranchOffset))
            .whileTrue(drive.rotationDriveCommands
                .new RotGoToReefCommand(Rotation2d.fromDegrees(0))
        );
        
        driverController.b()
            .onTrue(
                new RunCommand(() -> drive.allianceAngleCalc(Rotation2d.fromDegrees(54)), drive.rotationDriveCommands)
        );

        driverController.b()
            .onTrue(
                new InstantCommand(() -> drive.pathFindtoPath(drive.getPath("Left HP Teleop"), 
                    new PathConstraints(4.5, 7, 9.42478, 12.5664, 12)),
                    drive.linearDriveCommands,
                    drive.rotationDriveCommands
                )
            );

        driverController.x()
            .onTrue(
                new InstantCommand(() -> drive.pathFindtoPath(drive.getPath("Right HP Teleop"), 
                    new PathConstraints(4.5, 7, 9.42478, 12.5664, 12)),
                    drive.linearDriveCommands,
                    drive.rotationDriveCommands
                )
            );
    }

    private void coralPlacementTriggers(){
        EndEffector endEffector = EndEffector.getInstance();
        ElevArmControl elevArmControl = ElevArmControl.getInstance();

        //Elevator Arm Triggers
        operatorController.y().onTrue(elevArmControl.getGoToL4Command());
        operatorController.x().onTrue(elevArmControl.getGoToL3Command());
        operatorController.b().onTrue(elevArmControl.getGoToL2Command());
        operatorController.a().onTrue(elevArmControl.getGoToIntakeCommand());
        operatorController.pov(90).onTrue(elevArmControl.getGoToLowAlgaeCommand());


        driverController.axisMagnitudeGreaterThan(3, 0.1)
            .whileTrue(endEffector.new EjectCmd());
        
        driverController.axisMagnitudeGreaterThan(2, 0.1)
            .whileTrue(endEffector.new IntakeCmd());

        operatorController.axisMagnitudeGreaterThan(3, 0.1)
            .whileTrue(endEffector.new ReverseCmd());
            
        operatorController.axisMagnitudeGreaterThan(2, 0.1)
            .whileTrue(endEffector.new EjectCmd());
        
        operatorController.pov(90).onTrue(elevArmControl.getGoToLowAlgaeCommand());
        
        operatorController.pov(270).onTrue(elevArmControl.getGoToHighAlgaeCommand());

    }

    private void algaeGrabberTriggers(){
        AlgaeAngle algaeAngle = AlgaeAngle.getInstance();
        AlgaeRoller algaeRoller = AlgaeRoller.getInstance();

        operatorController.pov(0)
            .onTrue(algaeAngle.new AngleCommand(Rotation2d.fromDegrees(20)));
        
        operatorController.pov(180)
            .onTrue(algaeAngle.new AngleCommand(Rotation2d.fromDegrees(80)));

        operatorController.rightBumper().whileTrue(algaeRoller.new EjectCmd());
        operatorController.leftBumper().whileTrue(algaeRoller.new IntakeCmd());
    }

    private void climberTriggers(){
        Climber climber = Climber.getInstance();
        driverController.pov(90).whileTrue(climber.new ExtendCmd());
        driverController.pov(270).whileTrue(climber.new RetractCmd());

        operatorController.start().whileTrue(climber.new SetExtPosCommand());

        operatorController.back().whileTrue(climber.new SetRetPosCommand());
    }
    /**
     * Updates the controls for the drivetrain
     */
    private void updateDrive() {
        Drive drive = Drive.getInstance();
        boolean slowSpeed = driverController.getHID().getRawButton(5);
        double maxSpeed = (slowSpeed ? .5 : 1) * Constants.maxSpeed;
        double maxAngleRate = (slowSpeed ? .5 : 1) * Constants.maxAngularSpeed;

        boolean fieldRelative = true;
        var alliance = DriverStation.getAlliance();
        double alliance_dir = alliance.isPresent() && alliance.get() == Alliance.Red ? 1 : -1;

        double xAxis = MathUtil.applyDeadband(driverController.getRawAxis(1), 0.05);
        double yAxis = MathUtil.applyDeadband(driverController.getRawAxis(0), 0.05);
        double rAxis = MathUtil.applyDeadband(driverController.getRawAxis(4), 0.1);

        double xSpeed = xAxis * maxSpeed * alliance_dir;
        double ySpeed = yAxis * maxSpeed * alliance_dir;
        double rSpeed = rAxis * maxAngleRate * -1;

        double presetMirror = (drive.isRedAlliance() ? -1 : 1);
        Rotation2d rotationMirror = (drive.isRedAlliance() ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0));


        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.rSpeed = rSpeed;


        if (Math.abs(xAxis) > 0 || Math.abs(yAxis) > 0){
            drive.setDriveRate(xSpeed, ySpeed);
            drive.setLinearManualDrive(true);
        }else{
            drive.setLinearManualDrive(false);
        }

        if (Math.abs(rAxis) > 0){
            drive.setRotationRate(rSpeed);
            drive.setRotManualDrive(true);
        }else{
            drive.setRotManualDrive(false);
        }

        if(driverController.getHID().getPOV() == 0 && driverController.getHID().getStartButton()){
            drive.setPresetPose(
                FieldLayout.getReef(ReefFace.ZERO).plus(new Transform2d(Constants.robotLength/2 * -presetMirror, 0, rotationMirror)));
        }

        // Update Shuffleboard
        sb_driveX.setDouble(xSpeed);
        sb_driveY.setDouble(ySpeed);
        sb_driveR.setDouble(rSpeed);
        sb_driveFR.setBoolean(fieldRelative);

    }


    private void sysIdTest(){
        //Elevator elevator = Elevator.getInstance();
        //AlgaeAngle algaeAngle = AlgaeAngle.getInstance();
        Arm arm = Arm.getInstance();
        // if (operatorController.getAButton()){
        //     elevator.setSysIdCommandQuasiUp();

        // } else if(operatorController.getBButton()){
        //     elevator.setSysIdCommandQuasiDown();

        // } else if(operatorController.getXButton()){
        //     elevator.setSysIdCommandDynUp();

        // } else if(operatorController.getYButton()){
        //     elevator.setSysIdCommandDynDown();

        // }
        
    }

    /**
     * Subsystem Period Method
     */
    @Override
    public void periodic() {
        if (DriverStation.isTeleop()) {
            updateDrive();
        } else if(DriverStation.isTest()) {
            updateDrive();
            //sysIdTest();
        }

        updateUI();
    }

    private void updateUI(){
        sb_matchTimer.setDouble(DriverStation.getMatchTime());
    }

    /**
     * Static Initializer
     * 
     * @return common instance of the OperatorInterface class
     */
    public static OperatorInterface getInstance() {
        if (oi == null) {
            oi = new OperatorInterface();
        }

        return oi;
    }
}