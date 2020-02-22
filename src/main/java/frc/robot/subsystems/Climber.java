/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ClimbCommand;

public class Climber extends Subsystem {
  /**
   * Creates a new Climber.
   */

  TalonFX climberMotor;

  public Climber() {
    climberMotor = new TalonFX(14);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void climb(double speed) {
    climberMotor.set(ControlMode.PercentOutput, speed);
    SmartDashboard.putNumber("Climber", speed);
    
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ClimbCommand());
  }

 
}
