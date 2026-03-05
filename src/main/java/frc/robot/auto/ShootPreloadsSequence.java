package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Agitate;
import frc.robot.commands.LaunchSequence;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.CANShooter;

public class ShootPreloadsSequence extends SequentialCommandGroup {
    public ShootPreloadsSequence(CANFuelSubsystem ballSubsystem, CANShooter shooterSubsystem) {
        addCommands(
                new LaunchSequence(ballSubsystem, shooterSubsystem).withTimeout(6.0),
                new Agitate(ballSubsystem).withTimeout(3.0),
                new LaunchSequence(ballSubsystem, shooterSubsystem).withTimeout(6.0));
    }
}
