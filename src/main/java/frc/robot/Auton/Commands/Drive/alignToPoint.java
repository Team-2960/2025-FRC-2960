// package frc.robot.Auton.Commands.Drive;

// import edu.wpi.first.math.geometry.*;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.subsystems.Drive;

// /**
//  * Sets the robot to track a point on the field
//  */
// public class alignToPoint extends Command {
//     Translation2d targetPoint;
//     Rotation2d offsetAngle;
//     Rotation2d tolerance;
//     boolean enableFinished;

//     Drive drive;

//     public alignToPoint(double x, double y, Rotation2d tolerance, boolean enableFinished) {
//         this(new Translation2d(x, y), tolerance, enableFinished);
//     }

//     /**
//      * Constructor
//      * @param   targetPoint     Point for the robot to track
//      * @param   tolerance       Angle tolerance on tracking the target
//      */
//     public alignToPoint(Translation2d targetPoint, Rotation2d tolerance, boolean enableFinished) {
//         this(targetPoint, new Rotation2d(), tolerance, enableFinished);
//     }

//     /**
//      * Constructor
//      * @param   targetPoint     Point for the robot to track
//      * @param   offsetAngle     Angle offset from target point
//      * @param   tolerance       Angle tolerance on tracking the target
//      */
//     public alignToPoint(Translation2d targetPoint, Rotation2d offsetAngle, Rotation2d tolerance, boolean enableFinished) {
//         this.targetPoint = targetPoint;
//         this.offsetAngle = offsetAngle;
//         this.tolerance = tolerance;
//         this.enableFinished = enableFinished;

//         drive = Drive.getInstance();
//     }

//     /**
//      * Gets the current target angle
//      * @return  current target angle
//      */
//     public Rotation2d getTargetAngle() {
//         return targetPoint.minus(drive.getEstimatedPos().getTranslation()).getAngle().plus(offsetAngle);
//     }

//     /**
//      * gets the current target angle error
//      * @return  current target angle error
//      */
//     public Rotation2d getTargetError() {
//         return getTargetAngle().minus(drive.getEstimatedPos().getRotation());
//     }

//     /**
//      * Initialize command
//      */
//     @Override
//     public void initialize() {
//         // Set robot to target point tracking mode
//         drive.setTargetPoint(targetPoint, offsetAngle);
//     }

//     /**
//      * Command is finished
//      */
//     @Override
//     public boolean isFinished() {
//         // Check if the robot is at the target angle
//         return Math.abs(getTargetError().getRadians()) < tolerance.getRadians() && enableFinished;
//     }
// }