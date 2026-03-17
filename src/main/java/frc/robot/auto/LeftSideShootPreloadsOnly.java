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
 * See {@link RightSideShootPreloadsOnly} and position similarly but to the left
 * of the hub.
 */
public class LeftSideShootPreloadsOnly extends SequentialCommandGroup {
  public static final AutoSupplier getAutoSupplier(
      final CANDriveSubsystem driveSubsystem,
      final CANFuelSubsystem ballSubsystem,
      final CANShooter shooterSubsystem) {
    return new AutoSupplier(
        () -> new LeftSideShootPreloadsOnly(driveSubsystem, ballSubsystem, shooterSubsystem),
        new Pose2d(
            FieldConstants.TO_STARTING_LINE_METERS + FieldConstants.LINE_WIDTHS_METERS
                - (RobotConstants.DIAGONAL_WITH_BUMPERS_METERS / 2.0),
            (FieldConstants.FIELD_LAYOUT.getFieldWidth() / 2.0)
                + FieldConstants.SIDE_PRELOADS_ONLY_Y_OFFSET_METERS,
            Rotation2d.fromDegrees(-135.0)));
  }

  private LeftSideShootPreloadsOnly(
      final CANDriveSubsystem driveSubsystem,
      final CANFuelSubsystem ballSubsystem,
      final CANShooter shooterSubsystem) {
    addCommands(
        new DriveAway(driveSubsystem, 0.5, .5),
        new ShootPreloadsSequence(ballSubsystem, shooterSubsystem));
  }
}
