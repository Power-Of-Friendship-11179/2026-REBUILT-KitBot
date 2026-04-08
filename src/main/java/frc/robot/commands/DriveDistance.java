package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;

public class DriveDistance extends Command {
    private final CANDriveSubsystem drive;
    private final double meters;
    private final double xSpeed;
    private Pose2d start;

    public DriveDistance(final double meters, final double speed, final CANDriveSubsystem drive) {
        this.drive = drive;
        this.meters = meters;
        this.xSpeed = Math.copySign(speed, this.meters);
        this.addRequirements(this.drive);
    }

    @Override
    public void initialize() {
        this.start = drive.getPose();
    }

    @Override
    public void execute() {
        drive.gyroStabilizedArcadeDrive(xSpeed, 0.0);
    }

    @Override
    public boolean isFinished() {
        final double distanceTraveled = start.getTranslation().getDistance(drive.getPose().getTranslation());
        return Math.abs(distanceTraveled) >= Math.abs(this.meters);
    }

    @Override
    public void end(final boolean interrupted) {
        this.drive.stop();
    }
}
