/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  private final int SHOOTER_MOTOR_CAN_ID = 15;
  /**
   * Creates a new shooter.
   */

  TalonFX shooterMotor;

  public Shooter() {
    shooterMotor = new TalonFX(SHOOTER_MOTOR_CAN_ID);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    int countsPerHundredMs = shooterMotor.getSelectedSensorVelocity();

    double rpm = countsPerHundredMs * 60000 / (100 * 4096);

    SmartDashboard.putNumber("Shooter RPM", rpm);
  }

  public void setSpeed(double speedPercent) {
      shooterMotor.set(ControlMode.PercentOutput,speedPercent);
  }
}
