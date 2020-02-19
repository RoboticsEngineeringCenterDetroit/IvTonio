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
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.commands.ManualShooterCommand;
import edu.wpi.first.wpiutil.math.*;

public class Shooter extends Subsystem {

  private ShuffleboardTab tab;
  private NetworkTableEntry rpmSetpointEntry;
  private NetworkTableEntry rpmEntry;
  private Double rpmSetpoint;

  private final int SHOOTER_MOTOR_CAN_ID = 15;
  private final int FEED_MOTOR_CAN_ID = 12;

  public static final double DEFAULT_RPM = 0.0;
  public static final double MAX_RPM = 6300.0;

  /**
   * Creates a new shooter.
   */

  TalonFX shooterMotor;
  TalonSRX feedMotor;

  public Shooter() {
    
    shooterMotor = new TalonFX(SHOOTER_MOTOR_CAN_ID);
    shooterMotor.setNeutralMode(NeutralMode.Coast);

    feedMotor = new TalonSRX(FEED_MOTOR_CAN_ID);

    tab = Shuffleboard.getTab("Shooter");
    rpmSetpointEntry = tab.add("RPM setpoint", 0).getEntry();
    rpmEntry = tab.add("RPM", 0).getEntry();

    rpmSetpoint = DEFAULT_RPM;
  }

  public Double getRpm() {
    int countsPerHundredMs = shooterMotor.getSelectedSensorVelocity();
    Double rpm = countsPerHundredMs * 60000.0 / (100 * 2048);

    return rpm;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run


    //rpmSetpoint = MAX_RPM * Robot.oi.shooterController.getRawAxis(3);
    feedMotor.set(ControlMode.PercentOutput, Robot.oi.shooterController.getRawAxis(2));

    setMotor(rpmSetpoint);

    rpmEntry.setDouble(this.getRpm());
  }

  public void setMotor(double rpm) {

    double velocity = (rpm * 100.0 * 2048.0 )/ 60000.0;

    shooterMotor.set(ControlMode.Velocity, velocity);
    System.out.println("shooter velocity = " + velocity);
  }

  public double getRpmSetpoint() {
    return rpmSetpoint;
  }

  public void setRpm(double rpm)
  {
    rpmSetpoint = MathUtil.clamp(rpm, 0.0, MAX_RPM);
  }

  public void setFeedSpeed(double speed) {
    feedMotor.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ManualShooterCommand());
  }
}
