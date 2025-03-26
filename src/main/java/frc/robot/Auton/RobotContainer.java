package frc.robot.Auton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.OperatorInterface;
import frc.robot.Util.FieldLayout;
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

public class RobotContainer {
    private static SendableChooser<Command> autoChooser;

    // Subsystems
    public static Drive drive = new Drive();
    
    public static Arm arm = new Arm();
    public static Elevator elevator = new Elevator();
    public static ElevArmControl elevArmControl = new ElevArmControl(arm, elevator);
    public static EndEffector endEffector = new EndEffector();

    public static AlgaeAngle algaeAngle = new AlgaeAngle();
    public static AlgaeRoller algaeRoller = new AlgaeRoller();

    public static Climber climber = new Climber();

    public static Cameras cameras = new Cameras(drive);
    public static DriverCamera driverCamera = new DriverCamera();

    public static LEDControl ledControl = new LEDControl(endEffector);

    public static OperatorInterface oi = new OperatorInterface(drive, elevArmControl, endEffector, algaeAngle, algaeRoller, climber);

    // PathPlanner
    public static RobotConfig config;
    public static AutoBuilder autoBuilder;

    public static PathConstraints pathConstraints;

    /**
     * Constructor
     */
    static {

        initPathPlanner();

        initNamedCommands();

        autoChooser = AutoBuilder.buildAutoChooser();
        
        SmartDashboard.putData("Choose Auto", autoChooser);
    }

    /**
     * Initializes PathPlanner
     */
    private static void initPathPlanner() {
        // Get Path Planner RobotConfig
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }

        autoBuilder = new AutoBuilder();
        AutoBuilder.configure(
            drive::getPose,
            drive::presetPosition,
            drive::getChassisSpeeds,
            (speeds, feedforwards) -> drive.pathPlannerKinematics(speeds),
            new PPHolonomicDriveController(
                    new PIDConstants(5, 0, 0),
                    new PIDConstants(5, 0, 0)),
            config,
            FieldLayout::isRedAlliance,
            drive

        );

        pathConstraints = PathConstraints.unlimitedConstraints(12);
    }

    /**
     * Adds Named Commands to PathPlanner
     */
    private static void initNamedCommands() {
        NamedCommands.registerCommand("gotoNearestBranch", drive.new GotoNearestBranchCommand(Constants.RobotConst.coralOffset));
        NamedCommands.registerCommand("goToIntakeCommand", elevArmControl.getGoToIntakeCommand());
        // TODO Implement left & right branch align
        //NamedCommands.registerCommand("leftBranchAlign", leftBranchAlign);
        //NamedCommands.registerCommand("rightBranchAlign", rightBranchAlign);
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
        NamedCommands.registerCommand("elevatorHoldCommand", elevator.new HoldCommand());
        NamedCommands.registerCommand("armHoldCommand", arm.new ArmHoldCommand());
    }

    /**
     * Gets the currently selected auton command
     * @return currently selected Auton command
     */
    public static Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}