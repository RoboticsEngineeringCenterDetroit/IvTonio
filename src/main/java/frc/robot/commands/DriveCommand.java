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
        double forward = -Robot.oi.driveController.getRawAxis(1);
        forward = Utilities.deadband(forward);
        // Square the forward stick
        forward = Math.copySign(Math.pow(forward, 2.0), forward);

        double strafe = -Robot.oi.driveController.getRawAxis(0);
        strafe = Utilities.deadband(strafe);
        // Square the strafe stick
        strafe = Math.copySign(Math.pow(strafe, 2.0), strafe);

        double rotation = -Robot.oi.driveController.getRawAxis(4);
        rotation = Utilities.deadband(rotation);
        // Square the rotation stick
        rotation = Math.copySign(Math.pow(rotation, 2.0), rotation);

        Robot.driveTrain.drive(new Translation2d(forward, strafe), rotation, false);

        double shooterSpeed;

        shooterSpeed = Robot.oi.shooterController.getRawAxis(3);
        Robot.shooter.setSpeed(shooterSpeed);

    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
