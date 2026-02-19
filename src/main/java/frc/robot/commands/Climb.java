package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;

public class Climb extends Command {
    ClimberSubsystem climb;
    Boolean direction_up;
   
    public Climb(ClimberSubsystem climb, Boolean direction_up) {
        addRequirements(climb);
        this.climb = climb;
        this.direction_up = direction_up;
    }

    @Override
    public void initialize() {
        double v = ClimberConstants.CLIMBER_VOLTAGE;
        v = (!direction_up) ? -v : v;
        climb.setPower(v);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        climb.setPower(0);
    }
}
