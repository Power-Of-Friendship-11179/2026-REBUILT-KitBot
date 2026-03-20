package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.CANDriveSubsystem;

/**
 * Turn to angle command for blue orgin angles. The angle is rotated when on red
 * side. This makes it work well for auto.
 */
public class TurnToAngleGSAD extends Command {
    private final CANDriveSubsystem drive;
    private final double targetHeadingBlue;
    private  double targetHeading;

    public TurnToAngleGSAD(final double targetHeadingBlue, final CANDriveSubsystem drive) {
        this.targetHeadingBlue = targetHeadingBlue;
        this.drive = drive;
        this.addRequirements(this.drive);
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("TurnToAngle", true);
        Commands.runOnce(() -> targetHeading = drive.automodeOnlyForceGSADTargetHeading(targetHeadingBlue));
    }

    @Override
    public void execute() {
        drive.gyroStabilizedArcadeDrive(0.1, 0.0);
    }

    @Override
    public boolean isFinished() {
        double error = MathUtil.inputModulus(
            targetHeading - drive.getPose().getRotation().getDegrees(),
            -180.0,
            180.0);
        return Math.abs(error) < DriveConstants.kTurnToleranceDeg;
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("TurnToAngle", false);
        drive.stop();
    }
}
