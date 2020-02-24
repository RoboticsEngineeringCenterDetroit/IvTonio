/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.robot.Robot;

public class DriveXY extends Command {

  public static final double TARGET_TOLERANCE = 0.05;
  Pose2d target;
  Transform2d delta;
  long startTimeMicroSeconds;


  public Transform2d getDelta() {
    return target.minus(Robot.driveTrain.getPose());
  }

  public double getDistanceToTarget() {
    return getDelta().getTranslation().getNorm();
  }

  public DriveXY(double x, double y, double angleDegrees) {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.driveTrain);
    target = new Pose2d(new Translation2d(x, y), Rotation2d.fromDegrees(angleDegrees));
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    startTimeMicroSeconds = RobotController.getFPGATime();
  }

  public double getElapsedTime() {
    return (RobotController.getFPGATime()  - startTimeMicroSeconds) / 1000000.0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Translation2d translation = getDelta().getTranslation();
    translation = translation.div(translation.getNorm());

    double scale = 1.0;
    double dx = getDistanceToTarget();
    double time = getElapsedTime();

    if(dx < 1.0) {
      scale = dx/1.0;
    }

    if(time < 1.0)
    {
      scale = time/1.0;
    }

    translation = translation.times(scale);

    Robot.driveTrain.drive(translation, 0.0, true);
  }


  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Math.abs(getDistanceToTarget()) < TARGET_TOLERANCE;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.driveTrain.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.driveTrain.stop();
  }
}
