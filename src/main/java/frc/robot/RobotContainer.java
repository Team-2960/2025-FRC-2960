package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeAngle;
import frc.robot.subsystems.AlgaeRoller;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Cameras;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.DriverCamera;
import frc.robot.subsystems.ElevArmControl;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.LEDControl;

/**
 * Contains all the subsystems
 */
public class RobotContainer {
    // TODO Remove singleton instance once named commands and auto chooser are moved
    private static RobotContainer instance = null;

    public final Drive drive = new Drive();
    public final OperatorInterface oi = new OperatorInterface();
    public final Arm arm = new Arm();
    public final AlgaeAngle algaeAngle = new AlgaeAngle();
    public final AlgaeRoller algaeRoller = new AlgaeRoller();
    public final ElevArmControl elevArmControl = new ElevArmControl();
    public final Elevator elevator = new Elevator();
    public final EndEffector endEffector = new EndEffector();
    public final Climber climber = new Climber();
    public final Cameras cameras = new Cameras();
    public final DriverCamera driverCamera = new DriverCamera();
    public final LEDControl ledControl = new LEDControl();

    private final SendableChooser<Command> autoChooser;

    private RobotContainer(){
        // TODO Move to their respective subsystems
        NamedCommands.registerCommand("driveAlignCommand", drive.linearDriveCommands.new GoToPointCommand(new Translation2d()));
        NamedCommands.registerCommand("goToIntakeCommand", elevArmControl.getGoToIntakeCommand());
        NamedCommands.registerCommand("goToL1Command", elevArmControl.getGoToL1Command());
        NamedCommands.registerCommand("goToL2Command", elevArmControl.getGoToL2Command());
        NamedCommands.registerCommand("goToL3Command", elevArmControl.getGoToL3Command());
        NamedCommands.registerCommand("goToL4Command", elevArmControl.getGoToL4Command());
        NamedCommands.registerCommand("goToLowAlgaeCommand", elevArmControl.getGoToLowAlgaeCommand());
        NamedCommands.registerCommand("goToHighAlgaeCommand", elevArmControl.getGoToHighAlgaeCommand());
        NamedCommands.registerCommand("coralPresentCommand", endEffector.new CoralPresentCommand());
        NamedCommands.registerCommand("coralNotPresentCommand", endEffector.new CoralNotPresentCommand());
        NamedCommands.registerCommand("ejectCommand", endEffector.new EjectCmd());
        NamedCommands.registerCommand("intakeCommand", endEffector.new IntakeCmd());

        // TODO Move to Robot.java
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Choose Auto", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }    

    
    public static RobotContainer getInstance() {
        if(instance == null) instance = new RobotContainer();

        return instance;
    }
}