/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import org.frcteam2910.common.robot.Utilities;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ManualShooterCommand extends Command {
  /**
   * Creates a new ManualShooterCommand.
   */
  public ManualShooterCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    requires(Robot.shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double setpoint;

    // if(Robot.oi.shooterController.getPOV() == 0) {
    //   setpoint += 500;
    // } else if(Robot.oi.shooterController.getPOV() == 180) {
    //   setpoint -= 500;
    // }

    setpoint = Robot.oi.shooterController.getRawAxis(3);
    Robot.shooter.setMotor(setpoint);

    double feedspeed = Utilities.deadband(Robot.oi.shooterController.getRawAxis(2));
    Robot.shooter.setFeedSpeed(feedspeed);
  }

  

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
