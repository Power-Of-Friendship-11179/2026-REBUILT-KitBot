package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotConstants;

/**
 * An autonomous command that does nothing. The robot should be placed to the
 * far left of the field as see from the driver station with the intake facing
 * the alliance wall. The battery side bumper outside edge should be even with
 * the far side of the robot starting line.
 * 
 * <p>
 * We may have need to update this starting location due to alliance partner
 * needs, but hopefully, wew never run this one.
 */
public class DoNothing extends SequentialCommandGroup {
    public static final AutoSupplier getAutoSupplier() {
        return new AutoSupplier(
                DoNothing::new,
                new Pose2d(
                        FieldConstants.TO_STARTING_LINE_METERS + FieldConstants.LINE_WIDTHS_METERS
                                - (RobotConstants.LENGTH_WITH_BUMPERS_METERS / 2.0),
                        FieldConstants.FIELD_LAYOUT.getFieldWidth() - (RobotConstants.WIDTH_WITH_BUMPERS_METERS / 2.0),
                        Rotation2d.fromDegrees(180.0)));
    };

    private DoNothing() {
    } // stay still
}
