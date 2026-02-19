package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ClimberConstants.*;

public class ClimberSubsystem extends SubsystemBase {
    private SparkMax climbMotor;

    public enum Position { COMPRESS, EXPAND }

    public ClimberSubsystem() {
        climbMotor = new SparkMax(CLIMBER_MOTOR_ID, MotorType.kBrushed);
    }

    public void setPower(double p) {
        climbMotor.set(p);
    }

    @Override
    public void periodic() {}
}
