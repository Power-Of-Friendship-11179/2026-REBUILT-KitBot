package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FuelConstants;
import frc.robot.commands.Agitate;
import frc.robot.commands.Launch;
import frc.robot.commands.SpinUp;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.CANShooter;

public class ShootPreloadsSequence extends ParallelCommandGroup {
    public ShootPreloadsSequence(CANFuelSubsystem ballSubsystem, CANShooter shooterSubsystem) {
        addCommands(
                shooterSubsystem.shoot(),
                new SequentialCommandGroup(
                        new SpinUp(ballSubsystem).withTimeout(FuelConstants.SPIN_UP_SECONDS),
                        new Launch(ballSubsystem).withTimeout(3.0),
                        new Agitate(ballSubsystem).withTimeout(2.0),
                        new Launch(ballSubsystem).withTimeout(3.0),
                        new Agitate(ballSubsystem).withTimeout(2.0),
                        new Launch(ballSubsystem).withTimeout(3.0),
                        new Agitate(ballSubsystem).withTimeout(2.0),
                        new Launch(ballSubsystem).withTimeout(3.0)));
    }
}
