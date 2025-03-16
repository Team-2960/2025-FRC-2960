package frc.robot;

import frc.robot.Util.FieldLayout;
import frc.robot.Util.FieldLayout.ReefFace;
import frc.robot.subsystems.AlgaeAngle;
import frc.robot.subsystems.AlgaeRoller;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.ElevArmControl;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class OperatorInterface extends SubsystemBase {
    // INSTANCE
    public static OperatorInterface oi = null;

    // JOYSTICKS
    private XboxController operatorController;
    private CommandXboxController driverController;
    private CommandXboxController operatorController1;


    //Manual Control Class Variables
    double xSpeed;
    double ySpeed;
    double rSpeed;

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

    private GenericEntry sb_matchTimer;
    private GenericEntry sb_coralPresent;

    /**
     * Constructor
     */
    private OperatorInterface() {
        // Create Joysticks
        operatorController = new XboxController(1);
        driverController = new CommandXboxController(0);
        operatorController1 = new CommandXboxController(1);


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

        boolean slowSpeed = driverController.getHID().getRawButton(6);
        double maxSpeed = (slowSpeed ? .5 : 1) * Constants.maxSpeed;
        double maxAngleRate = (slowSpeed ? .5 : 1) * Constants.maxAngularSpeed;

        boolean fieldRelative = true;
        var alliance = DriverStation.getAlliance();
        double alliance_dir = alliance.isPresent() && alliance.get() == Alliance.Red ? 1 : -1;

        double xAxis = MathUtil.applyDeadband(driverController.getRawAxis(1), 0.05);
        double yAxis = MathUtil.applyDeadband(driverController.getRawAxis(0), 0.05);
        double rAxis = MathUtil.applyDeadband(driverController.getRawAxis(4), 0.05);

        driverController.pov(0).whileTrue(drive.new PresetPoseCommand(
            FieldLayout.getReef(ReefFace.ZERO).plus(new Transform2d(-Constants.robotLength/2, 0, new Rotation2d()))));
        
        driverController.y()
            .onTrue(drive.linearDriveCommands
                .new LinearGoToReefCommand(
                    new Translation2d(-Constants.robotLength/2, 0)))
            .onTrue(drive.rotationDriveCommands
                .new RotGoToReefCommand(Rotation2d.fromDegrees(0))
        );

        driverController.y().and(driverController.rightBumper())
            .whileTrue(
                drive.linearDriveCommands
                .new LinearGoToReefCommand(
                    new Translation2d(-Constants.robotLength/2, -0.41)))
            .whileTrue(drive.rotationDriveCommands
                .new RotGoToReefCommand(Rotation2d.fromDegrees(0))
        );
        
        driverController.y().and(driverController.leftBumper())
            .whileTrue(
                drive.linearDriveCommands
                .new LinearGoToReefCommand(
                    new Translation2d(-Constants.robotLength/2, -0.1)))
            .whileTrue(drive.rotationDriveCommands
                .new RotGoToReefCommand(Rotation2d.fromDegrees(0))
        );

        driverController.x()
            .onTrue(
                drive
                .rotationDriveCommands
                .new AngleAlignCommand(Rotation2d.fromDegrees(-54))
        );
        
        driverController.b()
            .onTrue(
                drive
                .rotationDriveCommands
                .new AngleAlignCommand(Rotation2d.fromDegrees(54))
        );

        driverController.a()
            .onTrue(
                drive
                .rotationDriveCommands
                .new AngleAlignCommand(Rotation2d.fromDegrees(-90))
        );

        driverController.pov(180)
            .onTrue(
                drive
            .linearDriveCommands
            .new TrapLinearGoToReefCommand(
                new Translation2d(-Constants.robotLength/2, 0))
        );
    }

    private void coralPlacementTriggers(){
        EndEffector endEffector = EndEffector.getInstance();
        ElevArmControl elevArmControl = ElevArmControl.getInstance();

        //Elevator Arm Triggers
        operatorController1.y().onTrue(elevArmControl.getGoToL4Command());
        operatorController1.x().onTrue(elevArmControl.getGoToL3Command());
        operatorController1.b().onTrue(elevArmControl.getGoToL2Command());
        operatorController1.a().onTrue(elevArmControl.getGoToIntakeCommand());
        operatorController1.pov(90).onTrue(elevArmControl.getGoToLowAlgaeCommand());


        driverController.axisMagnitudeGreaterThan(3, 0.1)
            .whileTrue(endEffector.new EjectCmd());
        
        driverController.axisMagnitudeGreaterThan(2, 0.1)
            .whileTrue(endEffector.new IntakeCmd());

        operatorController1.axisMagnitudeGreaterThan(3, 0.1)
            .whileTrue(endEffector.new ReverseCmd());
            
        operatorController1.axisMagnitudeGreaterThan(2, 0.1)
            .whileTrue(endEffector.new EjectCmd());
        
        operatorController1.pov(90).onTrue(elevArmControl.getGoToLowAlgaeCommand());
        
        operatorController1.pov(270).onTrue(elevArmControl.getGoToHighAlgaeCommand());

    }

    private void algaeGrabberTriggers(){
        AlgaeAngle algaeAngle = AlgaeAngle.getInstance();
        AlgaeRoller algaeRoller = AlgaeRoller.getInstance();

        operatorController1.pov(0)
            .onTrue(algaeAngle.new AngleCommand(Rotation2d.fromDegrees(20)));
        
        operatorController1.pov(180)
            .onTrue(algaeAngle.new AngleCommand(Rotation2d.fromDegrees(80)));

        operatorController1.rightBumper().whileTrue(algaeRoller.new EjectCmd());
        operatorController1.leftBumper().whileTrue(algaeRoller.new IntakeCmd());
    }

    private void climberTriggers(){
        Climber climber = Climber.getInstance();
        driverController.rightBumper().whileTrue(climber.new ExtendCmd());
        driverController.leftBumper().whileTrue(climber.new RetractCmd());
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

        // Update Shuffleboard
        sb_driveX.setDouble(xSpeed);
        sb_driveY.setDouble(ySpeed);
        sb_driveR.setDouble(rSpeed);
        sb_driveFR.setBoolean(fieldRelative);

    }

    /**
     * updates controls for elevator, arm, and end effector
     */
    private void 
    
    updateCoralPlacement(){//TODO finish whatever is needed for this
        Elevator elevator = Elevator.getInstance();
        EndEffector endEffector = EndEffector.getInstance();
        Arm arm = Arm.getInstance();
        ElevArmControl elevArmControl = ElevArmControl.getInstance();

        // //L2 = button A | L3 = button B | L4 = button Y
        // boolean elevatorL2 = operatorController.getRawButton(1);
        // boolean elevatorL3 = operatorController.getRawButton(2);
        // boolean elevatorL4 = operatorController.getRawButton(4);

        double elevatorRate = MathUtil.applyDeadband(operatorController.getRightY(), 0.05);
        double armRate = MathUtil.applyDeadband(operatorController.getLeftY(), 0.06);
        boolean eject = operatorController1.getRightTriggerAxis() > 0.1;
        boolean intake = operatorController1.getLeftTriggerAxis() > 0.1;



        // if (operatorController.getRightBumperButton()){
            if(Math.abs(elevatorRate) > 0){
                elevator.setRateCommand(elevatorRate);
            }else if (operatorController1.getHID().getAButton()){
                elevator.setPosCommand(30);
            }else if(operatorController1.getHID().getBButton()){
                elevator.setPosCommand(0);
            }

            if(Math.abs(armRate) > 0.05){
                arm.setRateCommand(armRate * 90);
            }else{
                arm.setHoldCommand();
            }

            if(eject){
                endEffector.setEject();
            } else if(intake){
                endEffector.setIntake();
            } else{
                endEffector.setStop();
            }

        // }else{
        //     if (operatorController.getYButton()){
        //         elevArmControl.getGoToL4Command().schedule();
        //     } else if (operatorController.getBButton()){
        //         elevArmControl.getGoToL2Command().schedule();
        //     } else if(operatorController.getXButton()){
        //         elevArmControl.getGoToL3Command().schedule();
        //     } else if (operatorController.getAButton()){
        //         elevArmControl.getGoToIntakeCommand().schedule();
        //     }

        //     if(eject){
        //         endEffector.setEject();
        //     } else if(intake){
        //         endEffector.setIntake();
        //     } else{
        //         endEffector.setStop();
        //     }
        // }
        //elevator.setRateCommand(elevatorRate);

    }


    /**
     * updates controls for algae angle and algae roller
     */
    private void updateAlgaeGrabber() {
        AlgaeAngle algaeAngle = AlgaeAngle.getInstance();
        AlgaeRoller algaeRoller = AlgaeRoller.getInstance();

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
        Arm arm = Arm.getInstance();
        Elevator elevator = Elevator.getInstance();
        //EndEffector end_effector = EndEffector.getInstance();

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
        AlgaeAngle algae_angle = AlgaeAngle.getInstance();
        AlgaeRoller algae_roller = AlgaeRoller.getInstance();

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
        Climber climber = Climber.getInstance();

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
            //updateCoralPlacement();
            //updateAlgaeGrabber();
            //updateClimber();
            //updateDriverFeedback();
            //updateClimberTest();
        } else if(DriverStation.isTest()) {
            //updateDriveTest();
            //updateCoralPlacementTest();
            //updateAlgaeGrabberTest();
            updateClimberTest();
            updateCoralPlacement();
            updateDrive();
            //sysIdTest();
        }
        if (DriverStation.getAlliance().get() == Alliance.Red){
            offsetMirror = -1;
        }else{
            offsetMirror = 1;
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