package frc.robot;

import frc.robot.Util.FieldLayout;
import frc.robot.Util.FieldLayout.ReefFace;
import frc.robot.subsystems.AlgaeAngle;
import frc.robot.subsystems.AlgaeRoller;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class OperatorInterface extends SubsystemBase {
    // JOYSTICKS
    private XboxController driverController;
    private XboxController operatorController;

    // LED Control
    int led_count = 69;
    AddressableLED leds;
    AddressableLEDBuffer led_idle;
    AddressableLEDBuffer led_note;
    AddressableLEDBuffer led_endgame1;
    AddressableLEDBuffer led_endgame2;
    Timer ledTimer = new Timer();
    
    // Shuffleboard Entries
    private GenericEntry sb_driveX;
    private GenericEntry sb_driveY;
    private GenericEntry sb_driveR;
    private GenericEntry sb_driveFR;

    private GenericEntry sb_armRate;
    private GenericEntry sb_armExtendManual;

    private GenericEntry sb_rumblePower;
    private GenericEntry sb_rumbleTimer;
    private GenericEntry sb_isEndGame;

    /**
     * Constructor
     */
    OperatorInterface() {
        // Create Joysticks
        driverController = new XboxController(0);
        operatorController = new XboxController(1);

        // Setup LEDs
        leds = new AddressableLED(0);
        leds.setLength(led_count);

        led_idle = new AddressableLEDBuffer(led_count);
        led_note = new AddressableLEDBuffer(led_count);
        led_endgame1 = new AddressableLEDBuffer(led_count);
        led_endgame2 = new AddressableLEDBuffer(led_count);

        for (int i = 0; i < led_count; i++) {
            led_idle.setRGB(i, 148, 148, 148);
            led_note.setRGB(i, 0, 0, 127);

            if (i % 2 == 0) {
                led_endgame1.setRGB(i, 255, 255, 255);
                led_endgame2.setRGB(i, 0, 0, 127);
            } else {

                led_endgame1.setRGB(i, 0, 0, 127);
                led_endgame2.setRGB(i, 255, 255, 255);
            }
        }

        leds.setData(led_idle);
        leds.start();

        // Setup Shuffleboard
        var drive_layout = Shuffleboard.getTab("OI")
                .getLayout("Drive", BuiltInLayouts.kList)
                .withSize(2, 4);
        sb_driveX = drive_layout.add("X Speed", 0).getEntry();
        sb_driveY = drive_layout.add("Y Speed", 0).getEntry();
        sb_driveR = drive_layout.add("R Speed", 0).getEntry();
        sb_driveFR = drive_layout.add("Field Relative", true).getEntry();

        var arm_layout = Shuffleboard.getTab("OI")
                .getLayout("Arm", BuiltInLayouts.kList)
                .withSize(2, 4);
        sb_armRate = arm_layout.add("Arm Manual Rate", 0).getEntry();
        sb_armExtendManual = arm_layout.add("Arm Extend", 0).getEntry();

        var rumble_layout = Shuffleboard.getTab("OI")
                .getLayout("Rumble", BuiltInLayouts.kList)
                .withSize(2, 4);

        sb_rumblePower = rumble_layout.add("Rumble Power", 0).getEntry();
        sb_rumbleTimer = rumble_layout.add("Rumble Timer", 0).getEntry();
        sb_isEndGame = rumble_layout.add("Is End Game", false).getEntry();
    }
    

    

    /**
     * Updates the controls for the drivetrain
     */
    private void updateDrive() {
        Drive drive = RobotContainer.getInstance().drive;

        boolean slowSpeed = driverController.getRawButton(5);
        double maxSpeed = (slowSpeed ? .5 : 1) * Constants.maxSpeed;
        double maxAngleRate = (slowSpeed ? .5 : 1) * Constants.maxAngularSpeed;

        boolean fieldRelative = true;
        var alliance = DriverStation.getAlliance();
        double alliance_dir = alliance.isPresent() && alliance.get() == Alliance.Red ? 1 : -1;

        double xAxis = MathUtil.applyDeadband(driverController.getRawAxis(1), 0.05);
        double yAxis = MathUtil.applyDeadband(driverController.getRawAxis(0), 0.05);
        double rAxis = MathUtil.applyDeadband(driverController.getRawAxis(4), 0.1);

        double xSpeed = MathUtil.applyDeadband(driverController.getRawAxis(1), 0.05) * maxSpeed * alliance_dir;
        double ySpeed = MathUtil.applyDeadband(driverController.getRawAxis(0), 0.05) * maxSpeed * alliance_dir;
        double rSpeed = rAxis * maxAngleRate * -1;

        if (driverController.getPOV() == 0){
            drive.presetPosition(FieldLayout.getReef(ReefFace.ZERO).plus(new Transform2d(-Constants.robotLength/2, 0, new Rotation2d())));
        }

        if (!driverController.getRawButton(3) && !driverController.getRawButton(4)){
            if (Math.abs(xAxis) > 0.05 || Math.abs(yAxis) > 0.05 ){
                drive.setDriveRate(xSpeed, ySpeed);

            } else if (driverController.getRawButton(3)){
                drive.setGoToPoint(new Translation2d(0, 0));
                
            }else{
                drive.setDriveRate(0, 0);
            }


            if (Math.abs(rAxis) > 0.05){
                drive.setRotationRate(rSpeed);

            }
            else if (driverController.getRawButton(1)) {
                drive.setAngleAlign(Rotation2d.fromDegrees(0));

            }
            else if (driverController.getRawButton(2)){
                drive.setPointAlign(new Translation2d(0, 0), Rotation2d.fromDegrees(0));

            } else{
                drive.setRotationRate(0);
            }
        }else{
            if (driverController.getRawButton(3)){
                drive.setGoToPoint(new Translation2d(0, 0));
                drive.setAngleAlign(Rotation2d.fromDegrees(90));
                
            } else if (driverController.getRawButton(4)){
                //Shuffle distance should be 0.15875
                if (driverController.getRawButton(5)){
                    drive.goToReef(new Pose2d(-Constants.robotLength/2, 0.3, Rotation2d.fromDegrees(0)));

                } else if (driverController.getRawButton(6)){
                    drive.goToReef(new Pose2d(-Constants.robotLength/2, -0.3, Rotation2d.fromDegrees(0)));
                }else{
                    drive.goToReef(new Pose2d(-Constants.robotLength/2, 0, Rotation2d.fromDegrees(0)));
                }
            }
        }

        // Update Shuffleboard
        sb_driveX.setDouble(xSpeed);
        sb_driveY.setDouble(ySpeed);
        sb_driveR.setDouble(rSpeed);
        sb_driveFR.setBoolean(fieldRelative);
    }

    /**
     * updates controls for elevator, arm, and end effector
     */
    private void updateCoralPlacement(){//TODO finish whatever is needed for this
        Elevator elevator = RobotContainer.getInstance().elevator;
        EndEffector endEffector = RobotContainer.getInstance().endEffector;
        // Arm arm = Arm.getInstance();

        // //L2 = button A | L3 = button B | L4 = button Y
        // boolean elevatorL2 = operatorController.getRawButton(1);
        // boolean elevatorL3 = operatorController.getRawButton(2);
        // boolean elevatorL4 = operatorController.getRawButton(4);

        double elevatorRate = MathUtil.applyDeadband(operatorController.getRightY(), 0.05);
        if(Math.abs(elevatorRate) > 0){
            elevator.setRateCommand(elevatorRate);
        }else if (operatorController.getAButton()){
            elevator.setPosCommand(30);
        }else if(operatorController.getBButton()){
            elevator.setPosCommand(0);
        }
        //elevator.setRateCommand(elevatorRate);
        
    }


    /**
     * updates controls for algae angle and algae roller
     */
    private void updateAlgaeGrabber() {
        AlgaeAngle algaeAngle = RobotContainer.getInstance().algaeAngle;
        AlgaeRoller algaeRoller = RobotContainer.getInstance().algaeRoller;

        boolean algaeUp = operatorController.getPOV() == 180;
        boolean algaeDown = operatorController.getPOV() == 0;

        boolean algaeIntake = operatorController.getRightBumperButton();
        boolean algaeEject = operatorController.getLeftBumperButton();

       
        //sets algae intake angle position (D-pad up = down | D-pad down = up)
        if(algaeUp){
            algaeAngle.setAngleCommand(Rotation2d.fromDegrees(20));
        }else if(algaeDown){
            algaeAngle.setAngleCommand(Rotation2d.fromDegrees(80));
        }

        //runs algae intake/eject rollers (right trigger = run intake | left trigger = run reverse intake)
        if(algaeIntake){
            algaeRoller.runIntake();
        }else if(algaeEject){
            algaeRoller.runEject();
        }else{
            algaeRoller.stop();
        }
    }

    /**
     * Updates the controls for the climber
     */
    private void updateClimber() {
        // TODO Implement
    }

    /**
     * Updates the driver feedback state
     */
    private void updateDriverFeedback() {
        // TODO Implement
    }


    /**
     * updates drive (test mode)
     */
    private void updateDriveTest() {
        // TODO Implement test mode for drivetrain

        updateDrive();
    }

    /**
     * updates elevator, arm, and end effector controls (test mode)
     */
    private void updateCoralPlacementTest(){
        Arm arm = RobotContainer.getInstance().arm;
        Elevator elevator = RobotContainer.getInstance().elevator;
        //EndEffector end_effector = RobotContainer.getInstance().endEffector;

        double arm_percent = MathUtil.applyDeadband(operatorController.getLeftY(), .05);
        double armMultiplier = 0;
        double elev_percent = -MathUtil.applyDeadband(operatorController.getRightY(), .05);
        if( operatorController.getRightBumperButton()){
            armMultiplier = 1;
        } else{
            armMultiplier = 12;
        }
        //arm.setVoltCommand(arm_percent * armMultiplier);
        //arm.setRateCommand(arm_percent * 90);
        //elevator.setVoltCommand(elev_percent * 12);

        // if(operatorController.getAButton()) {
        //     end_effector.runEject();
        // } else if(operatorController.getBButton()) {
        //     end_effector.runIntake();
        // }else {
        //     end_effector.stop();
        // }

        if (operatorController.getAButton()){
            elevator.setPosCommand(3);
        }else if(operatorController.getBButton()){
            elevator.setPosCommand(0.5);
        }else if (Math.abs(elev_percent) > 0.06){
            elevator.setRateCommand(elev_percent);
        }
        
        if (operatorController.getXButton()){
            arm.setAngleCommand(Rotation2d.fromDegrees(80));
        }else if(operatorController.getYButton()){
            arm.setAngleCommand(Rotation2d.fromDegrees(45));
        } else if (operatorController.getLeftBumperButton()){
            arm.setRateCommand(10);
        } else{
            arm.setRateCommand(arm_percent * 90);
        }
    }
    
    /**
     * updates controls for algae angle and algae roller (test mode)
     */
    private void updateAlgaeGrabberTest() {
        AlgaeAngle algae_angle = RobotContainer.getInstance().algaeAngle;
        AlgaeRoller algae_roller = RobotContainer.getInstance().algaeRoller;

        double angle_percent_pos = MathUtil.applyDeadband(operatorController.getRightTriggerAxis(), .05)/4;
        double angle_percent_neg = -MathUtil.applyDeadband(operatorController.getLeftTriggerAxis(), .05)/4;
        double angle_percent = angle_percent_pos + angle_percent_neg;

        double anglePercent = MathUtil.applyDeadband(operatorController.getLeftY(), 0.05);
        //algae_angle.setVoltCommand(angle_percent * 12);
        if(operatorController.getAButton()){
            algae_angle.setAngleCommand(Rotation2d.fromDegrees(20));
        }else if(operatorController.getBButton()){
            algae_angle.setAngleCommand(Rotation2d.fromDegrees(80));
        }else{
            algae_angle.setRateCommand(anglePercent);
        }

        if(operatorController.getXButton()) {
            algae_roller.runEject();
        } else if (operatorController.getYButton()) {
            algae_roller.runIntake();
        } else {
            algae_roller.stop();
        }
    }

    /**
     * updates controls for climber (test mode)
     */
    private void updateClimberTest() {
        Climber climber = RobotContainer.getInstance().climber;

        // if(operatorController.getStartButton()) {
        //     climber.runExtend();
        // } else if (operatorController.getBackButton()) {
        //     climber.runRetract();
        // } else if (operatorController.getPOV() == 0){
        //     climber.runReset();
        // } else {
        //     climber.stop();
        // }
        
        //Runs the climber with the Left Stick Y Axis on the Operator Controller
        double posClimberVolt = MathUtil.applyDeadband(driverController.getLeftTriggerAxis(), 0.05) * 12;
        double negClimberVolt = -MathUtil.applyDeadband(driverController.getRightTriggerAxis(), 0.05) * 12;
        climber.setClimberVoltage(posClimberVolt + negClimberVolt);
    }

    private void sysIdTest(){
        //Elevator elevator = Elevator.getInstance();
        //AlgaeAngle algaeAngle = AlgaeAngle.getInstance();
        Arm arm = RobotContainer.getInstance().arm;
        // if (operatorController.getAButton()){
        //     elevator.setSysIdCommandQuasiUp();

        // } else if(operatorController.getBButton()){
        //     elevator.setSysIdCommandQuasiDown();

        // } else if(operatorController.getXButton()){
        //     elevator.setSysIdCommandDynUp();

        // } else if(operatorController.getYButton()){
        //     elevator.setSysIdCommandDynDown();

        // }
        if (operatorController.getPOV() == 180){
            arm.setSysIdCommandGroup();
        }
    }

    /**
     * Subsystem Period Method
     */
    @Override
    public void periodic() {
        if (DriverStation.isTeleop()) {
            updateDrive();
            updateCoralPlacement();
            updateAlgaeGrabber();
            //updateClimber();
            //updateEndEffector();
            //updateDriverFeedback();
            //updateClimberTest();
        } else if(DriverStation.isTest()) {
            //updateDriveTest();
            updateCoralPlacementTest();
            //updateAlgaeGrabberTest();
            //updateClimberTest();
            //sysIdTest();
        }
    }
}