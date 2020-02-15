/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  private ShuffleboardTab tab;
  private NetworkTableEntry rpmSetpointEntry;
  private NetworkTableEntry rpmEntry;
  private Double rpmSetpoint = 0.0;

  private final int SHOOTER_MOTOR_CAN_ID = 15;
  /**
   * Creates a new shooter.
   */

  TalonFX shooterMotor;

  public Shooter() {
    shooterMotor = new TalonFX(SHOOTER_MOTOR_CAN_ID);
   shooterMotor.setNeutralMode(NeutralMode.Coast);

    tab = Shuffleboard.getTab("Shooter");
    rpmSetpointEntry = tab.add("RPM setpoint", 0).getEntry();
    rpmEntry = tab.add("RPM", 0).getEntry();
  }

  public Double getRpm() {
    int countsPerHundredMs = shooterMotor.getSelectedSensorVelocity();

    Double rpm = countsPerHundredMs * 60000.0 / (100 * 2048);

    return rpm;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    rpmSetpoint = rpmSetpointEntry.getDouble(0.0);
    setRpm(rpmSetpoint);

    rpmEntry.setDouble(getRpm());
  }

  public void setRpm(double rpm) {

    double velocity = (rpm * 100.0 * 2048.0 )/ 60000.0;

    shooterMotor.set(ControlMode.Velocity, velocity);
    System.out.println("shooter velocity = " + velocity);
  }

  public void setSpeed(double speed) {

    shooterMotor.set(ControlMode.PercentOutput, speed);
  }
}
