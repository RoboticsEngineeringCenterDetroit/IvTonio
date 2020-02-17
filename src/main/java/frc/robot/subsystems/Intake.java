/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;


public class Intake extends SubsystemBase {
  /**
   * Creates a new Intake.
   */

  public static final int LEFT_INTAKE_CAN_ID  = 9;
  public static final int RIGHT_INTAKE_CAN_ID = 10;
  
  TalonSRX leftIntakeMotor, rightIntakeMotor;

  public Intake() {
    leftIntakeMotor = new TalonSRX(LEFT_INTAKE_CAN_ID);
    rightIntakeMotor = new TalonSRX(RIGHT_INTAKE_CAN_ID);
    leftIntakeMotor.setNeutralMode(NeutralMode.Coast);
    rightIntakeMotor.setNeutralMode(NeutralMode.Coast);
  }


  public void setIntakeSpeed(double speed) {
    leftIntakeMotor.set(ControlMode.PercentOutput, speed);
    rightIntakeMotor.set(ControlMode.PercentOutput, speed);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    double inSpeed = Robot.oi.shooterController.getRawAxis(1);
    setIntakeSpeed(inSpeed);
  }
}
