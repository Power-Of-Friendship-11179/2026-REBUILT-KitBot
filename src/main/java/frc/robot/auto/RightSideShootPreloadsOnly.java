// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.CANShooter;

/**
 * An autonomous command that starts to the right of the hub but pointed at the
 * hub, backs off a bit, shoots the preloaded fuel, and ends. The robot should
 * be placed with the back left corner of the bumpers (the corner the battery is
 * closest to) just pverlapping the start line and be sure it is not touching
 * the ramp (touching the ramp is against the rules). The robot should be placed
 * at a 45 degree angle (opposite corners with be square with the field and its
 * elements). It should be placed at a distance from the hub such that this 45
 * degree angle is pointed at the center of the hub.
 */
public class RightSideShootPreloadsOnly extends SequentialCommandGroup {
  public static final AutoSupplier getAutoSupplier(
      final CANDriveSubsystem driveSubsystem,
      final CANFuelSubsystem ballSubsystem,
      final CANShooter shooterSubsystem) {
    return new AutoSupplier(
        () -> new RightSideShootPreloadsOnly(driveSubsystem, ballSubsystem, shooterSubsystem),
        new Pose2d(
            FieldConstants.TO_STARTING_LINE_METERS + FieldConstants.LINE_WIDTHS_METERS
                - (RobotConstants.DIAGONAL_WITH_BUMPERS_METERS / 2.0),
            (FieldConstants.FIELD_LAYOUT.getFieldWidth() / 2.0)
                - FieldConstants.SIDE_PRELOADS_ONLY_Y_OFFSET_METERS,
            Rotation2d.fromDegrees(-135.0)));
  }

  private RightSideShootPreloadsOnly(
      final CANDriveSubsystem driveSubsystem,
      final CANFuelSubsystem ballSubsystem,
      final CANShooter shooterSubsystem) {
    addCommands(
        // TODO validate this. No driving may be needed.
        new DriveAway(driveSubsystem, 0.5, .5),
        new ShootPreloadsSequence(ballSubsystem, shooterSubsystem));
  }
}
