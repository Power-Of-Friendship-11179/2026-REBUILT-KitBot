// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import static frc.robot.Constants.OperatorConstants.*;

import frc.robot.auto.AutoSupplier;
import frc.robot.auto.DoNothing;
import frc.robot.auto.ShootPreloadsOnly;
import frc.robot.commands.Drive;
import frc.robot.commands.Eject;
import frc.robot.commands.Intake;
import frc.robot.commands.LaunchSequence;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;

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

  private final CommandXboxController driverController = new CommandXboxController(DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operatorController = new CommandXboxController(OPERATOR_CONTROLLER_PORT);

  private final SendableChooser<AutoSupplier> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    configureBindings();
    autoChooser.setDefaultOption("Do Nothing", DoNothing.getAutoSupplier());
    autoChooser.addOption("Shoot Preloads Only", ShootPreloadsOnly.getAutoSupplier(driveSubsystem, fuelSubsystem));
  }

  private void configureBindings() {
    operatorController.leftBumper().whileTrue(new Intake(fuelSubsystem));
    operatorController.rightBumper().whileTrue(new LaunchSequence(fuelSubsystem));
    operatorController.a().whileTrue(new Eject(fuelSubsystem));

    driveSubsystem.setDefaultCommand(new Drive(driveSubsystem, driverController));
    fuelSubsystem.setDefaultCommand(fuelSubsystem.run(() -> fuelSubsystem.stop()));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected().getCommand();
  }
}
