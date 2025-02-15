package frc.robot;

import frc.robot.Util.FieldLayout;
import frc.robot.Util.FieldLayout.ReefFace;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Climber.ClimberStates;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.path.GoalEndState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class OperatorInterface extends SubsystemBase {
    // INSTANCE
    public static OperatorInterface oi = null;

    // JOYSTICKS
    private Joystick driverController;
    private Joystick operatorController;

    private int lastOpPOV;

    // LED Control
    int led_count = 69;
    AddressableLED leds;
    AddressableLEDBuffer led_idle;
    AddressableLEDBuffer led_note;
    AddressableLEDBuffer led_endgame1;
    AddressableLEDBuffer led_endgame2;
    Timer ledTimer = new Timer();

    // Robot State Tracking
    private Timer rumbleTimer = new Timer();
    private boolean lastIsNotePresent = true;
    private boolean lastIsEndGame = false;

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
    private OperatorInterface() {
        // Create Joysticks
        driverController = new Joystick(0);
        operatorController = new Joystick(1);

        lastOpPOV = -1;

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
     * Subsystem Period Method
     */
    @Override
    public void periodic() {
        if (DriverStation.isTeleop()) {
            updateDrive();
            updateArm();
            updateClimber();
            updateDriverFeedback();
        }
    }

    /**
     * Updates the controls for the drivetrain
     */
    private void updateDrive() {
        Drive drive = Drive.getInstance();

        boolean slowSpeed = driverController.getRawButton(5);
        double maxSpeed = (slowSpeed ? .5 : 1) * Constants.maxSpeed;
        double maxAngleRate = (slowSpeed ? .5 : 1) * Constants.maxAngularSpeed;

        boolean fieldRelative = true;// !driverController.getRawButton(1);
        var alliance = DriverStation.getAlliance();
        double alliance_dir = alliance.isPresent() && alliance.get() == Alliance.Red ? 1 : -1;

        double xAxis = MathUtil.applyDeadband(driverController.getRawAxis(1), 0.05);
        double yAxis = MathUtil.applyDeadband(driverController.getRawAxis(0), 0.05);
        double rAxis = MathUtil.applyDeadband(driverController.getRawAxis(4), 0.1);

        double xSpeed = MathUtil.applyDeadband(driverController.getRawAxis(1), 0.05) * maxSpeed * alliance_dir;
        double ySpeed = MathUtil.applyDeadband(driverController.getRawAxis(0), 0.05) * maxSpeed * alliance_dir;
        double rSpeed = rAxis * maxAngleRate * -1;

        Climber climber = Climber.getInstance();

        if (driverController.getPOV() == 0){
            //drive.presetPosition(new Pose2d(0.0, 0.0, new Rotation2d()));
            drive.presetPosition(FieldLayout.getReef(ReefFace.ZERO).plus(new Transform2d(-Constants.robotLength/2, 0, new Rotation2d())));
        }


        if (driverController.getRawButton(8)) {
            climber.setClimbState(ClimberStates.CLIMB);
        }

        
        climber.setRatchet(driverController.getRawButton((5)));

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

            // } else if (driverController.getRawButton(4)){
            //     drive.setReefAlign(new Rotation2d());

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
                    drive.goToReef(new Pose2d(-Constants.robotLength/2, 0.3, Rotation2d.fromDegrees(180)));

                } else if (driverController.getRawButton(6)){
                    drive.goToReef(new Pose2d(-Constants.robotLength/2, -0.3, Rotation2d.fromDegrees(180)));
                }else{
                    drive.goToReef(new Pose2d(-Constants.robotLength/2, 0, Rotation2d.fromDegrees(180)));
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
     * Updates the controls for the arm
     */
    private void updateArm() {
        Arm arm = Arm.getInstance();

        // Set Arm Presets
        if (operatorController.getRawButton(1)) {
            arm.setStateCommand("Speaker");
        } else if (operatorController.getRawButton(2)) {
            arm.setStateCommand("lineSpeaker");
        } else if (operatorController.getRawButton(3)) {
            arm.setStateCommand("Amp");// amp
        } else if (operatorController.getRawButton(4)) {
            arm.setStateCommand("Intake");
        } else if (operatorController.getPOV() == 90 && operatorController.getPOV() != 270) {
            arm.armAutoAlign();
        } else if (operatorController.getPOV() == 270 && operatorController.getPOV() != 90) {
            arm.setStateCommand("home");
        }

        // TODO Map Podium shot preset

        // Manual Arm Angle Control
        double armManual = operatorController.getRawAxis(1);
        double armManualRate = armManual * Constants.maxArmSpeed;
        double tolerance = .1;

        if (Math.abs(armManual) > tolerance) {
            arm.setTolRateCommand(armManualRate, (tolerance + 0.3)* Constants.maxArmSpeed);
        }

        sb_armRate.setDouble(armManualRate);
    }


    /**
     * Updates the controls for the climber
     */
    private void updateClimber() {
        Climber climber = Climber.getInstance();
        // TODO implement climber controls
        if (operatorController.getRawButton(7) || driverController.getRawButton(7)) {
            climber.setClimbState(ClimberStates.CLIMB_START);
        } else if (operatorController.getRawButton(8) || driverController.getRawButton(8)) {
            climber.setClimbState(ClimberStates.CLIMB);
        } else {
            climber.setClimbState(ClimberStates.IDLE);
        }

    }

    /**
     * Updates the driver feedback state
     */
    private void updateDriverFeedback() {
        double rumblePower = 0;
        
        AddressableLEDBuffer ledColor = led_idle;

        boolean isEndGame = DriverStation.isTeleop() && DriverStation.getMatchTime() <= 50 && DriverStation.getMatchType() != MatchType.None;


        // Rumble the controllers at full power for 1 second when the end game is about
        // to start
        if (isEndGame) {
            if (!lastIsEndGame) {
                rumbleTimer.restart();
                ledTimer.restart();
            }

            // if(rumbleTimer.get() < 1) rumblePower = 1;TODO add this back later

            double ledTime = ledTimer.get();
            if (ledTime < 1) {
                if (ledTime % .2 < .1) {
                    ledColor = led_endgame1;
                } else {
                    ledColor = led_endgame2;
                }
            }
        }

        // Update controller rumble
        driverController.setRumble(RumbleType.kBothRumble, rumblePower);
        operatorController.setRumble(RumbleType.kBothRumble, rumblePower);

        // Update LEDs
        leds.setData(ledColor);

        // Update state transition checks
        lastIsEndGame = isEndGame;

        // Update UI
        sb_rumblePower.setDouble(rumblePower);
        sb_rumbleTimer.setDouble(rumbleTimer.get());
        sb_isEndGame.setBoolean(isEndGame);
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