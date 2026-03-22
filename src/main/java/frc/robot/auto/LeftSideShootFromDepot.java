// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.Intake;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.CANShooter;

/**
 * See {@link RightSideShootPreloadsOnly} and position similarly but to the left
 * of the hub.
 */
public class LeftSideShootFromDepot extends SequentialCommandGroup {
    private static final double BLUE_TO_DEPOT_HEADING = 157.5;
    private static final double DRIVE_TO_DEPOT_METERS = Inches.of(115.0).in(Meters);
    private static final double BLUE_FROM_DEPOT_HEADING = 146.25;
    private static final double DRIVE_FROM_DEPOT_METERS = Inches.of(115.0).in(Meters);;

    private static final Pose2d BLUE_POSE = new Pose2d(
            FieldConstants.TO_STARTING_LINE_METERS + FieldConstants.LINE_WIDTHS_METERS
                    - (RobotConstants.DIAGONAL_WITH_BUMPERS_METERS / 2.0),
            (FieldConstants.FIELD_LAYOUT.getFieldWidth() / 2.0)
                    + FieldConstants.SIDE_PRELOADS_ONLY_Y_OFFSET_METERS,
            Rotation2d.fromDegrees(135.0));

    public static final AutoSupplier getAutoSupplier(
            final CANDriveSubsystem driveSubsystem,
            final CANFuelSubsystem ballSubsystem,
            final CANShooter shooterSubsystem) {
        return new AutoSupplier(
                () -> new LeftSideShootFromDepot(driveSubsystem, ballSubsystem, shooterSubsystem),
                BLUE_POSE);
    }

    private LeftSideShootFromDepot(
            final CANDriveSubsystem driveSubsystem,
            final CANFuelSubsystem ballSubsystem,
            final CANShooter shooterSubsystem) {
        addCommands(
                new ParallelDeadlineGroup(
                        new DriveAway(driveSubsystem, 0.5, .5),
                        shooterSubsystem.idle()),
                new ParallelDeadlineGroup(
                        new QuickShootSequence(ballSubsystem, shooterSubsystem, 3.0),
                        driveSubsystem.run(driveSubsystem::stop)),

                Commands.runOnce(() -> driveSubsystem.automodeOnlyForceGSADTargetHeading(BLUE_TO_DEPOT_HEADING)),
                new ParallelDeadlineGroup(
                        new DriveDistance(DRIVE_TO_DEPOT_METERS,
                                driveSubsystem).withTimeout(6.0),
                        new Intake(ballSubsystem, shooterSubsystem)),

                Commands.runOnce(() -> driveSubsystem.automodeOnlyForceGSADTargetHeading(BLUE_FROM_DEPOT_HEADING)),
                new ParallelDeadlineGroup(
                        new DriveDistance(DRIVE_FROM_DEPOT_METERS,
                                driveSubsystem),
                        shooterSubsystem.idle()),

                // TODO should we add a small GSAD movement to align better?
                new ParallelDeadlineGroup(
                        new QuickShootSequence(ballSubsystem, shooterSubsystem, 5.0),
                        driveSubsystem.run(driveSubsystem::stop)));
    }
}
