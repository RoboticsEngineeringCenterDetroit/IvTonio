/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import org.frcteam2910.common.robot.Utilities;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Robot;
import frc.robot.subsystems.Shooter;

public class ManualShooterCommand extends Command {
  /**
   * Creates a new ManualShooterCommand.
   */

  double pctSetpoint = 0.0;

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

    if(Robot.oi.shooterController.getPOV() == 0) {
      pctSetpoint += 0.05;
    } else if(Robot.oi.shooterController.getPOV() == 180) {
      pctSetpoint -= 0.05;
    }

    pctSetpoint = MathUtil.clamp(pctSetpoint, 0.0, 1.0);
    Robot.shooter.setMotorPercent(pctSetpoint);
    SmartDashboard.putNumber("Shooter Setpoint", pctSetpoint);
    // setpoint = Robot.oi.shooterController.getRawAxis(3);
    // setpoint *= Shooter.MAX_VEL;
    // Robot.shooter.setMotorVelocity(setpoint);

    double feedspeed = Utilities.deadband(Robot.oi.shooterController.getRawAxis(2), 0.10);
    Robot.shooter.setFeedSpeed(feedspeed);
  }

  

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
