package frc.robot.Auton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.ElevArmControl;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
public class RobotContainer {
  SendableChooser<Command> autoChooser;
  Command chosenAuto;
  private static RobotContainer robotContainer = null;

  public RobotContainer(){
    Drive drive = Drive.getInstance();
    ElevArmControl elevArmControl = ElevArmControl.getInstance();
    Elevator elevator = Elevator.getInstance();
    Arm arm = Arm.getInstance();
    EndEffector endEffector = EndEffector.getInstance();

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
    NamedCommands.registerCommand("elevatorHoldCommand", elevator.new ElevatorHoldCommand());
    NamedCommands.registerCommand("armHoldCommand", arm.new ArmHoldCommand());

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