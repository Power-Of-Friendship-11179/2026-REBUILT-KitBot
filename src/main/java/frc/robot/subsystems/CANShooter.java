package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.SHOOTER_CAN_ID;
import static frc.robot.Constants.ShooterConstants.SHOOTER_CURRENT_LIMIT;
import static frc.robot.Constants.ShooterConstants.SHOOTER_IDLE_VOLTAGE;
import static frc.robot.Constants.ShooterConstants.SHOOTER_SHOOTING_VOLTAGE;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CANShooter extends SubsystemBase {
    private SparkFlex shooterMotor = new SparkFlex(SHOOTER_CAN_ID, MotorType.kBrushless);

    public CANShooter() {
        final SparkFlexConfig shooterConfig = new SparkFlexConfig();
        shooterConfig.voltageCompensation(11.0);
        shooterConfig.smartCurrentLimit(SHOOTER_CURRENT_LIMIT);
        shooterConfig.idleMode(IdleMode.kCoast);
        shooterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SmartDashboard.putNumber("Shooter shooting Voltage", SHOOTER_SHOOTING_VOLTAGE);
    }

    public Command stop() {
        return run(() -> shooterMotor.set(0));
    }

    public Command idle() {
        return run(() -> shooterMotor.setVoltage(SHOOTER_IDLE_VOLTAGE));
    }

    public Command shoot() {
        return run(() -> shooterMotor.setVoltage(SmartDashboard.getNumber("Shooter shooting Voltage", SHOOTER_SHOOTING_VOLTAGE)));
    }
}