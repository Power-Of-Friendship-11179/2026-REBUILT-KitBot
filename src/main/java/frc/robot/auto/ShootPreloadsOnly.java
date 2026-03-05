// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.AutoDrive;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.CANShooter;

/**
 * An autonomous command that starts at the hub, backs off a bit, shoots the
 * preloaded fuel, and ends. The robot should be placed centered on the hub with
 * the battery side bumpers up against the hub.
 */
public class ShootPreloadsOnly extends SequentialCommandGroup {
  public static final AutoSupplier getAutoSupplier(
      final CANDriveSubsystem driveSubsystem,
      final CANFuelSubsystem ballSubsystem,
      final CANShooter shooterSubsystem) {
    return new AutoSupplier(
        () -> new ShootPreloadsOnly(driveSubsystem, ballSubsystem, shooterSubsystem),
        new Pose2d(
            FieldConstants.TO_STARTING_LINE_METERS + FieldConstants.LINE_WIDTHS_METERS
                - (RobotConstants.LENGTH_WITH_BUMPERS_METERS / 2.0),
            FieldConstants.FIELD_LAYOUT.getFieldWidth() / 2.0,
            Rotation2d.fromDegrees(180.0)));
  }

  private ShootPreloadsOnly(
      final CANDriveSubsystem driveSubsystem,
      final CANFuelSubsystem ballSubsystem,
      final CANShooter shooterSubsystem) {
    addCommands(
        new AutoDrive(driveSubsystem, 0.7, 0.0).withTimeout(.5),
        new ShootPreloadsSequence(ballSubsystem, shooterSubsystem));
  }
}
