/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ClimbCommand;

public class Climber extends Subsystem {
  private static final int CLIMBER_CAN_ID = 14;
  private static final int EXTENDER_CAN_ID = 11;
  /**
   * Creates a new Climber.
   */

  TalonFX climberMotor;
  CANSparkMax extenderMotor;

  public Climber() {
    climberMotor = new TalonFX(CLIMBER_CAN_ID);
    extenderMotor = new CANSparkMax(EXTENDER_CAN_ID, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climb Current", climberMotor.getSupplyCurrent());
  }

  public void climb(double speed) {
    climberMotor.set(ControlMode.PercentOutput, speed);
    SmartDashboard.putNumber("Climber", speed);
    
  }

  public void extend(double speed) {
    extenderMotor.set(speed);
    SmartDashboard.putNumber("Extender", speed);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ClimbCommand());
  }

 
}
