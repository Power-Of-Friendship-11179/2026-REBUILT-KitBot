// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoDrive;
import frc.robot.subsystems.CANDriveSubsystem;

/**
 * An autonomous command that starts at the hub, backs off a bit, shoots the
 * preloaded fuel, and ends. The robot should be placed centered on the hub with
 * the battery side bumpers up against the hub.
 */
public class DriveAway extends SequentialCommandGroup {
  public DriveAway(
      final CANDriveSubsystem driveSubsystem,
      final double xSpeed,
      final double seconds) {
    addCommands(
        new AutoDrive(driveSubsystem, xSpeed, 0.0).withTimeout(seconds));
  }
}
