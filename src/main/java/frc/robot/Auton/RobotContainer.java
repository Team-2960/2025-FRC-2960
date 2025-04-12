package frc.robot.Auton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
//import frc.robot.subsystems.Drive.LinearDriveCommands.LinearGoToReefCommand;
import frc.robot.subsystems.ElevArmControl;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;

public class RobotContainer {
  SendableChooser<Command> autoChooser;
  Command chosenAuto;
  private Command elevIntakeCommand;
  private Command elevL4PosCommand;
  private static RobotContainer robotContainer = null;

  public RobotContainer(){
    Drive drive = Drive.getInstance();
    ElevArmControl elevArmControl = ElevArmControl.getInstance();
    Elevator elevator = Elevator.getInstance();
    Arm arm = Arm.getInstance();
    EndEffector endEffector = EndEffector.getInstance();

    elevIntakeCommand = new RunCommand(() -> elevator.setElevatorPos(Constants.elevIntakePos), elevator);
    elevL4PosCommand = new RunCommand(() -> elevator.setElevatorPos(Constants.elevL4Pos), elevator);

    NamedCommands.registerCommand("rightBranchAlign", drive.new AutonReefAlign(new Pose2d(Constants.rightBranchOffset, new Rotation2d())));
    NamedCommands.registerCommand("leftBranchAlign", drive.new AutonReefAlign(new Pose2d(Constants.leftBranchOffset, new Rotation2d())));
    NamedCommands.registerCommand("goToIntakeCommand", elevArmControl.getGoToIntakeCommand());
    NamedCommands.registerCommand("goToL1Command", elevArmControl.getGoToL1Command());
    NamedCommands.registerCommand("goToL2Command", elevArmControl.getGoToL2Command());
    NamedCommands.registerCommand("goToL3Command", elevArmControl.getGoToL3Command());
    NamedCommands.registerCommand("goToL4Command", elevArmControl.getGoToL4Command());
    NamedCommands.registerCommand("goToLowAlgaeCommand", elevArmControl.getGoToLowAlgaeCommand());
    NamedCommands.registerCommand("goToHighAlgaeCommand", elevArmControl.getGoToHighAlgaeCommand());
    NamedCommands.registerCommand("coralPresentCommand", endEffector.new CoralPresentCommand());
    NamedCommands.registerCommand("coralNotPresentCommand", endEffector.new CoralNotPresentCommand());
    NamedCommands.registerCommand("coralInEndEffector", endEffector.new CoralInEndEffectorCommand());
    NamedCommands.registerCommand("ejectCommand", endEffector.new EjectCmd());
    NamedCommands.registerCommand("intakeCommand", endEffector.new AutonIntakeCmd());
    NamedCommands.registerCommand("elevatorHoldCommand", elevator.new ElevatorHoldCommand());
    NamedCommands.registerCommand("armHoldCommand", arm.new ArmHoldCommand());
    NamedCommands.registerCommand("armIntakeAngle", new RunCommand(() -> arm.setArmAngle(Constants.armIntakeAngle), arm));
    NamedCommands.registerCommand("elevIntakePos", elevIntakeCommand);
    NamedCommands.registerCommand("elevL4Pos", elevL4PosCommand);
    NamedCommands.registerCommand("driveDoNothing", new RunCommand(() -> drive.setChassisSpeeds(new ChassisSpeeds(0,0,0), false), drive));

    autoChooser = AutoBuilder.buildAutoChooser();
    
    SmartDashboard.putData("Choose Auto", autoChooser);
  }

  public Command getAutonomousCommand() {

    chosenAuto = autoChooser.getSelected();
    
    // Load the path you want to follow using its name in the GUI
    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return chosenAuto;
  }

  public static RobotContainer getInstance(){
    if (robotContainer == null){
      robotContainer = new RobotContainer();
    }
    return robotContainer;
  }

  
}