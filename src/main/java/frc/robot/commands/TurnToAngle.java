package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.CANDriveSubsystem;

/**
 * Turn to angle command for blue orgin angles. The angle is rotated when on red
 * side. This makes it work well for auto.
 */
public class TurnToAngle extends Command {
    private final CANDriveSubsystem drive;
    private final double targetHeadingBlue;
    private  double targetHeading;
    private final ProfiledPIDController turnController = new ProfiledPIDController(
            DriveConstants.kTurnP,
            0.0, // Do not mess with i and izone in time we have
            DriveConstants.kTurnD,
            new TrapezoidProfile.Constraints(
                    DriveConstants.kMaxTurnRateDegPerS,
                    DriveConstants.kMaxTurnAccelerationDegPerSSquared));
    private final SimpleMotorFeedforward turnFeedforward = new SimpleMotorFeedforward(
            DriveConstants.ksVolts,
            DriveConstants.kvVoltSecondsPerDegree,
            DriveConstants.kaVoltSecondsSquaredPerDegree);

    public TurnToAngle(final double targetHeadingBlue, final CANDriveSubsystem drive) {
        this.targetHeadingBlue = targetHeadingBlue;
        this.drive = drive;
        // Set the controller to be continuous (because it is an angle controller)
        turnController.enableContinuousInput(-180, 180);
        this.addRequirements(this.drive);
    }

    @Override
    public void initialize() {
        targetHeading = targetHeadingBlue;
        final Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        if (alliance == Alliance.Red) {
            targetHeading = Rotation2d.fromDegrees(targetHeadingBlue)
                    .rotateBy(FieldConstants.ROTATE_AROUND_FOR_RED.getRotation()).getDegrees();
        }
        turnController.setP(SmartDashboard.getNumber("Turn kP", DriveConstants.kTurnP));
        turnController.setD(SmartDashboard.getNumber("Turn kD", DriveConstants.kTurnD));
        // Set the controller tolerance - the delta tolerance ensures the robot is
        // stationary at the setpoint before it is considered as having reached the
        // reference
        turnController.setTolerance(
                SmartDashboard.getNumber("Turn Tol Deg", DriveConstants.kTurnToleranceDeg),
                SmartDashboard.getNumber("Turn Rate Tol Deg Per Sec", DriveConstants.kTurnRateToleranceDegPerS));
        turnController.reset(drive.getPose().getRotation().getDegrees());
    }

    @Override
    public void execute() {
        drive.driveArcade(
                0.0,
                turnController.calculate(drive.getPose().getRotation().getDegrees(), targetHeading)
                        // Divide feedforward voltage by battery voltage to normalize it to [-1, 1]
                        + turnFeedforward.calculate(turnController.getSetpoint().velocity)
                                / RobotController.getBatteryVoltage());
    }

    @Override
    public boolean isFinished() {
        return turnController.atGoal();
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}
