// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.Intake;
import frc.robot.commands.TurnToAngle;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.CANShooter;

/**
 * A sequence to shoot from right or left, get more fuel from neutral zone over
 * the bump, return, and shoot.
 */
public class ShootPreloadsNeutralShoot extends SequentialCommandGroup {
    private static final double BLUE_TO_NEUTRAL_HEADING = 0.0;
    public ShootPreloadsNeutralShoot(
        final double blueShotHeading,
        final CANDriveSubsystem driveSubsystem,
        final CANFuelSubsystem ballSubsystem,
        final CANShooter shooterSubsystem) {
    addCommands(
        new DriveAway(driveSubsystem, 0.5, .5),
        new QuickShootSequence(ballSubsystem, shooterSubsystem),
        new TurnToAngle(BLUE_TO_NEUTRAL_HEADING, driveSubsystem).withTimeout(2.0),
        Commands.runOnce(() -> driveSubsystem.automodeOnlyForceGSADTargetHeading(BLUE_TO_NEUTRAL_HEADING)),
        new ParallelDeadlineGroup(
            new DriveDistance(FieldConstants.DRIVE_TO_CENTER_OVER_RAMP_METERS,
                    driveSubsystem),
            new Intake(ballSubsystem, shooterSubsystem)),
        Commands.runOnce(() -> driveSubsystem.automodeOnlyForceGSADTargetHeading(BLUE_TO_NEUTRAL_HEADING)),
        new ParallelCommandGroup(
            new DriveDistance(-FieldConstants.DRIVE_TO_CENTER_OVER_RAMP_METERS,
                    driveSubsystem),
            new Intake(ballSubsystem, shooterSubsystem).withTimeout(0.5)),
        new TurnToAngle(blueShotHeading, driveSubsystem).withTimeout(3.0),
        // TODO should we add a small GSAD movement to align better?
        new QuickShootSequence(ballSubsystem, shooterSubsystem));
    }
}
