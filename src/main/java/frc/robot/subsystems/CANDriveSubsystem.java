// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import static frc.robot.Constants.DriveConstants.DRIVE_MOTOR_CURRENT_LIMIT;
import static frc.robot.Constants.DriveConstants.LEFT_FOLLOWER_ID;
import static frc.robot.Constants.DriveConstants.LEFT_LEADER_ID;
import static frc.robot.Constants.DriveConstants.RIGHT_FOLLOWER_ID;
import static frc.robot.Constants.DriveConstants.RIGHT_LEADER_ID;
import static frc.robot.Constants.DriveConstants.TRACK_WIDTH_METERS;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

/**
 * Provides the ability to drive around the field and maintain odometry.
 * 
 * <p>
 * See
 * https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
 * for details on the wpilib coordinate system. Odometry is maintained using
 * this system. In particular, note:
 * 
 * <ol>
 * <li>We are using a counterclockwise gyro as wpilib expects.
 * <li>Also, wpilib generally expects the yaw angle to be in the range (-180.0,
 * 180.0].
 * <li>For ease of April tag processing, we will use the "Always blue origin"
 * odometry scheme.
 * <li>Odometry will be particularly chanllenging when going over the bump. We
 * use the gyro to detect being on the bump and will work to give April tags
 * more influence on odometry for a short time (TODO a few good reads?)
 * after coming off the bump. This is TODO.
 * </ol>
 * 
 * <p>
 * The methods in this class handle the inconsistancies between our selected
 * hardware and the wpilib coordinate system. Use the methods defined here. Do
 * not use the devices directly.
 */
public class CANDriveSubsystem extends SubsystemBase {
  private final SparkMax leftLeader = new SparkMax(LEFT_LEADER_ID, MotorType.kBrushed);
  private final SparkMax leftFollower = new SparkMax(LEFT_FOLLOWER_ID, MotorType.kBrushed);
  private final SparkMax rightLeader = new SparkMax(RIGHT_LEADER_ID, MotorType.kBrushed);
  private final SparkMax rightFollower = new SparkMax(RIGHT_FOLLOWER_ID, MotorType.kBrushed);

  private final DifferentialDrive drive = new DifferentialDrive(leftLeader, rightLeader);

  private final ADIS16470_IMU gyro = new ADIS16470_IMU();
  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;

  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(TRACK_WIDTH_METERS);
  private final DifferentialDrivePoseEstimator odometry;
  private final Field2d field = new Field2d();

  // variables used in gyro stablized arcade drive (GSAD) and nowhere else.
  private boolean gsadActive = false;
  private double gsadTargetHeadingDegrees = 0.0;
  private static final double GSAD_KP = 0.025; // Just P if PID is used

  public CANDriveSubsystem() {
    leftLeader.setCANTimeout(250);
    rightLeader.setCANTimeout(250);
    leftFollower.setCANTimeout(250);
    rightFollower.setCANTimeout(250);

    SparkMaxConfig config = new SparkMaxConfig();
    config.voltageCompensation(12);
    config.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);

    // Set configuration to follow each leader and then apply it to corresponding
    // follower. Resetting in case a new controller is swapped
    // in and persisting in case of a controller reset due to breaker trip
    config.idleMode(IdleMode.kBrake);
    config.follow(leftLeader);
    leftFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    config.follow(rightLeader);
    rightFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Remove following, then apply config to right leader
    config.disableFollowerMode();
    config.encoder.inverted(DriveConstants.RIGHT_ENCODER_REVERSED);
    config.encoder.countsPerRevolution(DriveConstants.ENCODER_CPR);
    config.encoder.positionConversionFactor(DriveConstants.WHEEL_TRAVEL_METERS_PER_ROT);
    config.encoder.velocityConversionFactor(DriveConstants.WHEEL_VELOCITY_RPM_TO_MPS);
    rightLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // Set config to inverted and then apply to left leader. Set Left side inverted
    // so that postive values drive both sides forward
    config.inverted(true);
    config.encoder.inverted(DriveConstants.LEFT_ENCODER_REVERSED);
    leftLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    leftEncoder = leftLeader.getEncoder();
    rightEncoder = rightLeader.getEncoder();

    // The code here will handle deadbands.
    drive.setDeadband(0.0);

    // TODO add custom standard deviations if needed.
    odometry = new DifferentialDrivePoseEstimator(
      kinematics,
      getYaw(),
      0,
      0,
      Pose2d.kZero);

    SmartDashboard.putData(drive);
    SmartDashboard.putData(field);
  }

  /**
   * The yaw relative to the raw gyro (aka robot) physical starting angle. This is
   * a continuous angle.
   * 
   * @return the current counterclockwise positive robot yaw.
   */
  private Rotation2d getYaw() {
    return Rotation2d.fromDegrees(getYawImpl());
  }

  /**
   * The yaw relative to the raw gyro (aka robot) physical starting angle. Can be
   * used internally when the raw degrees are needed. This is a continuous angle.
   * 
   * @return the current counterclockwise positive robot yaw.
   */
  private double getYawImpl() {
    return gyro.getAngle();
  }

  /**
   * This is typically called when the automode command is chosen and ready to
   * run. The selected automode should be mappable to an intented starting
   * {@link Pose2d}. This must be in the "Always blue origin" odometry scheme.
   * 
   * @param startingPose the robot's starting pose.
   */
  public void setRobotStartingPose(Pose2d startingPose) {
    odometry.resetPosition(getYaw(), leftEncoder.getPosition(), rightEncoder.getPosition(), startingPose);
    gsadActive = false; // force GSAD to get updated heading info on first input.
  }

  public void testingOnlyReset() {
    setRobotStartingPose(Pose2d.kZero);
    leftEncoder.setPosition(0.0);
    rightEncoder.setPosition(0.0);
  }

  /**
   * @return the current odometry calculate robot pose. This is a "blue origin"
   *         pose with the rotation adjusted for starting angle. The angle is
   *         continuous.
   */
  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  }

  @Override
  public void periodic() {
    updateOdometry();
    Pose2d currentPose2d = getPose();
    field.setRobotPose(currentPose2d);
    SmartDashboard.putNumber("Raw Yaw", getYawImpl());
    SmartDashboard.putNumber("Pose Heading", currentPose2d.getRotation().getDegrees());
    SmartDashboard.putNumber("Left Side Position", leftEncoder.getPosition());
    SmartDashboard.putNumber("Right Side Position", rightEncoder.getPosition());
    SmartDashboard.putBoolean("GSAD", gsadActive);
    SmartDashboard.putNumber("GSAD Target", gsadTargetHeadingDegrees);
  }

  public void stop() {
    drive.arcadeDrive(0.0, 0.0);
  }

  public void driveArcade(double xSpeed, double zRotation) {
    double driveStraightFudgeRotation = 0.0;
    if (Math.abs(zRotation) < RobotDriveBase.kDefaultDeadband) {
      driveStraightFudgeRotation = -1 * xSpeed * 0.20;
    }
    drive.arcadeDrive(xSpeed, zRotation + driveStraightFudgeRotation);
  }

  public void gyroStabilizedArcadeDrive(double xSpeed, double zRotation) {
    xSpeed = MathUtil.applyDeadband(xSpeed, RobotDriveBase.kDefaultDeadband);
    zRotation = MathUtil.applyDeadband(zRotation, RobotDriveBase.kDefaultDeadband);
    if ((zRotation != 0.0) || (xSpeed == 0.0)) {
      // Driver (human or auto) is rotating the robot or stopped.
      gsadActive = false;
    } else if (xSpeed != 0.0) {
      // Driving but with zero rotation.
      if (!gsadActive) {
        // Just stopped rotating or first time since starting pose set.
        // Grab the target heading and set gsad active.
        // Use raw yaw is better since it is continuous and we avoid the
        // 180 to -180 jump which can cause uncontrolled spinning. It is
        // also okay to use since we are only using the delta and not
        // the angle value itself.
        gsadTargetHeadingDegrees = getYawImpl();
        gsadActive = true;
      }
      // Calculate corrective gsad rotation value.
      zRotation = (gsadTargetHeadingDegrees - getYawImpl()) * GSAD_KP;
    }
    drive.arcadeDrive(xSpeed, zRotation);
  }

  public void driveTank(double leftSpeed, double rightSpeed) {
    drive.tankDrive(leftSpeed, rightSpeed);
  }

  /**
   * Updates the field-relative position.
   */
  private void updateOdometry() {
    odometry.update(
        this.getYaw(), leftEncoder.getPosition(), rightEncoder.getPosition());
  }
}
