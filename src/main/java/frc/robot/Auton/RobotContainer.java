package frc.robot.Auton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Drive.LinearDriveCommands.LinearGoToReefCommand;
import frc.robot.subsystems.ElevArmControl;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;

public class RobotContainer {
  SendableChooser<Command> autoChooser;
  Command chosenAuto;
  Command leftBranchAlign;
  Command rightBranchAlign;
  private Command testAuton;
  private static RobotContainer robotContainer = null;

  public RobotContainer(){
    Drive drive = Drive.getInstance();
    ElevArmControl elevArmControl = ElevArmControl.getInstance();
    Elevator elevator = Elevator.getInstance();
    Arm arm = Arm.getInstance();
    EndEffector endEffector = EndEffector.getInstance();

    // leftBranchAlign = new ParallelCommandGroup(
    //   drive.linearDriveCommands.new LinearGoToReefCommand(
    //     Constants.leftBranchOffset),
    //   drive.rotationDriveCommands
    //     .new RotGoToReefCommand(Rotation2d.fromDegrees(0))
    // );

    // rightBranchAlign = new ParallelCommandGroup(
    //   drive.linearDriveCommands
    //     .new LinearGoToReefCommand(
    //       Constants.rightBranchOffset),
    //   drive.rotationDriveCommandsl
    //     .new RotGoToReefCommand(Rotation2d.fromDegrees(0))
    // );

    // rightBranchAlign = new RunCommand(() -> {
    //     PPHolonomicDriveController.overrideXFeedback(() -> drive.getCalcToPoint(drive.calcGoToReef(new Pose2d(Constants.rightBranchOffset, Rotation2d.fromDegrees(0))).getTranslation()).get(1));
    //     PPHolonomicDriveController.overrideYFeedback(() -> drive.getCalcToPoint(drive.calcGoToReef(new Pose2d(Constants.rightBranchOffset, Rotation2d.fromDegrees(0))).getTranslation()).get(2));
    //   }
    //   , drive.linearDriveCommands);

    // rightBranchAlign = new ParallelCommandGroup(
    //   drive.linearDriveCommands.new AutonLinearGoToReefCommand(Constants.rightBranchOffset),
    //   drive.rotationDriveCommands.new AutonRotGoToReefCommand(Rotation2d.fromDegrees(0))
    // );

    // leftBranchAlign = new ParallelCommandGroup(
    //   drive.linearDriveCommands.new AutonLinearGoToReefCommand(Constants.leftBranchOffset),
    //   drive.rotationDriveCommands.new AutonRotGoToReefCommand(Rotation2d.fromDegrees(0))
    // );

    testAuton = new SequentialCommandGroup(
      AutoBuilder.resetOdom(new Pose2d(7.255, 1.899, Rotation2d.fromDegrees(180))),
      drive.followPath("Copy of RA 1st Coral"),
      // new ParallelCommandGroup(
      //   elevArmControl.getGoToL4Command(),
      //   drive.linearDriveCommands.new AutonLinearGoToReefCommand(Constants.leftBranchOffset),
      //   drive.rotationDriveCommands.new RotGoToReefCommand(new Rotation2d())
      // ),

      endEffector.new EjectCmd()
    );



    NamedCommands.registerCommand("goToIntakeCommand", elevArmControl.getGoToIntakeCommand());
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
    NamedCommands.registerCommand("elevatorHoldCommand", elevator.new ElevatorHoldCommand());
    NamedCommands.registerCommand("armHoldCommand", arm.new ArmHoldCommand());

    autoChooser = AutoBuilder.buildAutoChooser();
    
    SmartDashboard.putData("Choose Auto", autoChooser);

  }

  public Command getAutonomousCommand() {

    chosenAuto = autoChooser.getSelected();
    
    // Load the path you want to follow using its name in the GUI
    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return testAuton;
  }

  public static RobotContainer getInstance(){
    if (robotContainer == null){
      robotContainer = new RobotContainer();
    }
    return robotContainer;
  }

  
}