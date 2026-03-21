// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.OperatorConstants.DRIVER_CONTROLLER_PORT;
import static frc.robot.Constants.OperatorConstants.OPERATOR_CONTROLLER_PORT;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.FieldConstants;
import frc.robot.auto.AutoSupplier;
import frc.robot.auto.DoNothing;
import frc.robot.auto.DriveAway;
import frc.robot.auto.LeftSideShootFromNeutral;
import frc.robot.auto.LeftSideShootPreloadsOnly;
import frc.robot.auto.RightSideShootFromNeutral;
import frc.robot.auto.RightSideShootPreloadsOnly;
import frc.robot.auto.ShootPreloadsOnly;
import frc.robot.auto.ShootPreloadsSequence;
import frc.robot.commands.Agitate;
import frc.robot.commands.Drive;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.Eject;
import frc.robot.commands.FeedPlayers;
import frc.robot.commands.Intake;
import frc.robot.commands.LaunchSequence;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.CANShooter;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final CANDriveSubsystem driveSubsystem = new CANDriveSubsystem();
  private final CANFuelSubsystem fuelSubsystem = new CANFuelSubsystem();
  private final CANShooter shooterSubsystem = new CANShooter();

  private final CommandXboxController driverController = new CommandXboxController(DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operatorController = new CommandXboxController(OPERATOR_CONTROLLER_PORT);

  private final SendableChooser<AutoSupplier> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    configureBindings();
    autoChooser.setDefaultOption("Do Nothing", DoNothing.getAutoSupplier());
    autoChooser.addOption("Shoot Preloads Only", ShootPreloadsOnly.getAutoSupplier(driveSubsystem, fuelSubsystem, shooterSubsystem));
    autoChooser.addOption("Right Side Shoot Preloads Only", RightSideShootPreloadsOnly.getAutoSupplier(driveSubsystem, fuelSubsystem, shooterSubsystem));
    autoChooser.addOption("Left Side Shoot Preloads Only", LeftSideShootPreloadsOnly.getAutoSupplier(driveSubsystem, fuelSubsystem, shooterSubsystem));
    autoChooser.addOption("Right Side With Neutral", RightSideShootFromNeutral.getAutoSupplier(driveSubsystem, fuelSubsystem, shooterSubsystem));
    autoChooser.addOption("Left Side With Neutral", LeftSideShootFromNeutral.getAutoSupplier(driveSubsystem, fuelSubsystem, shooterSubsystem));
    SmartDashboard.putData(autoChooser);
  }

  private void configureBindings() {
    operatorController.leftBumper().whileTrue(new Intake(fuelSubsystem, shooterSubsystem));
    operatorController.rightBumper().whileTrue(new LaunchSequence(fuelSubsystem, shooterSubsystem));
    operatorController.a().whileTrue(new Eject(fuelSubsystem, shooterSubsystem));
    operatorController.x().whileTrue(new Agitate(fuelSubsystem));
    //operatorController.rightTrigger().whileTrue(long shot);

    // TESTING ONLY
    driverController.rightBumper().whileTrue(new FeedPlayers(fuelSubsystem));
    operatorController.start().onTrue(Commands.runOnce(driveSubsystem::testingOnlyReset));

    // TEMP TESTING
    //driverController.povUp().whileTrue(new TurnToAngle(0.0, driveSubsystem));
    //driverController.povRight().whileTrue(new TurnToAngle(-90.0, driveSubsystem));
    //driverController.povDown().whileTrue(new TurnToAngle(180.0, driveSubsystem));
    //driverController.povLeft().whileTrue(new TurnToAngle(90.0, driveSubsystem));
    driverController.x().whileTrue(new DriveDistance(FieldConstants.DRIVE_TO_CENTER_OVER_RAMP_METERS/4.0,
                    driveSubsystem));
    driverController.povUp().whileTrue(new DriveAway(driveSubsystem, 0.5, .5));
    driverController.povDown().whileTrue(new ShootPreloadsSequence(fuelSubsystem, shooterSubsystem));

    // NOTE: don't bind to driver left bumper
    driveSubsystem.setDefaultCommand(new Drive(driveSubsystem, driverController));
    fuelSubsystem.setDefaultCommand(fuelSubsystem.run(() -> fuelSubsystem.stop()));
    shooterSubsystem.setDefaultCommand(shooterSubsystem.idle());
  }

  public Command getAutonomousCommand() {
    final AutoSupplier autoSupplier = autoChooser.getSelected();
    driveSubsystem.setRobotStartingPose(autoSupplier.getStartingPose2d());
    return autoSupplier.getCommand();
  }
}
