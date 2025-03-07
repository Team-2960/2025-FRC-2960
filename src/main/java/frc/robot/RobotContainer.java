package frc.robot;

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
    
    public static RobotContainer get() {
        if(instance == null) instance = new RobotContainer();

        return instance;
    }
}