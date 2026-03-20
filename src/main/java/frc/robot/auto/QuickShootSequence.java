package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FuelConstants;
import frc.robot.commands.Launch;
import frc.robot.commands.SpinUp;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.CANShooter;

public class QuickShootSequence extends ParallelDeadlineGroup {
    public QuickShootSequence(CANFuelSubsystem ballSubsystem, CANShooter shooterSubsystem, double shootDurationSec) {
        super(new SequentialCommandGroup(
                new SpinUp(ballSubsystem).withTimeout(FuelConstants.SPIN_UP_SECONDS),
                new Launch(ballSubsystem).withTimeout(shootDurationSec)),
        shooterSubsystem.shoot());
    }
}
