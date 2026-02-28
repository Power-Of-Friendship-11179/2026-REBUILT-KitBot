package frc.robot.auto;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;

/**
 * Instances of this class are are added to the automode chooser. The chosen
 * supplier is used to construct just one command (rather than all as is usually
 * done), and provide the odometry starting pose.
 * 
 * <p>
 * The starting pose passed to the constructor is blue origin. Assuming a
 * rotation around the center of the field is appropriate, the valued returned
 * from {@link #getStartingPose2d()} is alliance adjusted.
 */
public class AutoSupplier {
    private final Supplier<Command> commandSupplier;
    private final Pose2d startingPose2d;
    private final Pose2d rotateAroundForRed = new Pose2d(
            FieldConstants.FIELD_LAYOUT.getFieldLength() / 2.0,
            FieldConstants.FIELD_LAYOUT.getFieldWidth() / 2.0,
            Rotation2d.fromDegrees(180.0));

    /**
     * Creates a new auto supplier for the chooser.
     * 
     * @param commandSupplier the supplier should create a new command instance each
     *                        time it is called.
     * @param startingPose2d  the intended starting pose using the blue origin.
     */
    public AutoSupplier(final Supplier<Command> commandSupplier, final Pose2d startingPose2d) {
        this.commandSupplier = commandSupplier;
        this.startingPose2d = startingPose2d;
    }

    public Command getCommand() {
        return this.commandSupplier.get();
    }

    /**
     * @return the starting pose as appropriate to the alliance.
     */
    public Pose2d getStartingPose2d() {
        final Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        if (alliance == Alliance.Red) {
            return this.startingPose2d.rotateAround(
                    rotateAroundForRed.getTranslation(),
                    rotateAroundForRed.getRotation());
        }
        return this.startingPose2d;
    }
}
