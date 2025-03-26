// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Auton.RobotContainer;
//import frc.robot.Auton.AutonList;
import frc.robot.subsystems.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    private Command autonomousCommand;
    

    @Override
    public void robotInit() {
        Drive.getInstance();
        OperatorInterface.getInstance();
        Climber.getInstance();
        Arm.getInstance();
        AlgaeAngle.getInstance();
        AlgaeRoller.getInstance();
        ElevArmControl.getInstance();
        Elevator.getInstance();
        EndEffector.getInstance();
        LEDControl.getInstance();
        Cameras.getInstance();

        CameraServer.startAutomaticCapture();
        
        autonomousCommand = RobotContainer.getAutonomousCommand();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

    }

    @Override
    public void autonomousInit() {
        autonomousCommand = RobotContainer.getAutonomousCommand();

        if(autonomousCommand != null) autonomousCommand.schedule();
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) autonomousCommand.cancel();
    }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void testInit() {
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
    }

}
