package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import org.frcteam2910.common.robot.Utilities;

public class DriveCommand extends Command {

    public DriveCommand() {
        requires(frc.robot.Robot.driveTrain);
    }

    @Override
    protected void execute() {

        double drive = Robot.oi.driveController.getRawAxis(1);
        double rotate = Robot.oi.driveController.getRawAxis(4);

        Robot.driveTrain.leftFrontDriveMotor.set(drive);
        Robot.driveTrain.leftFrontRotateMotor.set(rotate);
        Robot.driveTrain.rightFrontDriveMotor.set(drive);
        Robot.driveTrain.rightFrontRotateMotor.set(rotate);
        Robot.driveTrain.leftBackDriveMotor.set(drive);
        Robot.driveTrain.leftBackRotateMotor.set(rotate);
        Robot.driveTrain.rightBackDriveMotor.set(drive);
        Robot.driveTrain.rightBackRotateMotor.set(rotate);
        

    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
