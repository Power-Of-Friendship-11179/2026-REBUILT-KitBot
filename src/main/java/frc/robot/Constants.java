// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Motor controller IDs for drivetrain motors
    public static final int LEFT_LEADER_ID = 3;
    public static final int LEFT_FOLLOWER_ID = 4;
    public static final int RIGHT_LEADER_ID = 5;
    public static final int RIGHT_FOLLOWER_ID = 2;

    // Current limit for drivetrain motors. 60A is a reasonable maximum to reduce
    // likelihood of tripping breakers or damaging CIM motors
    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 50;

    public static final boolean LEFT_ENCODER_REVERSED = false;
    public static final boolean RIGHT_ENCODER_REVERSED = true;

    public static final int ENCODER_CPR = 8192;
    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(6.29);
    public static final double WHEEL_TRAVEL_METERS_PER_ROT = WHEEL_DIAMETER_METERS * Math.PI;
    public static final double WHEEL_VELOCITY_RPM_TO_MPS = WHEEL_TRAVEL_METERS_PER_ROT * (1.0 / 60.0);

    public static final double TRACK_WIDTH_METERS = Inches.of(21.75).in(Meters);
    
    public static final double kTurnP = 0.036;
    public static final double kTurnI = 0.08;
    public static final double kTurnD = 0.0;

    public static final double kTurnToleranceDeg = 2.0;
    public static final double kTurnRateToleranceDegPerS = 10; // degrees per second

    public static final double kMaxTurnRateDegPerS = 100;
    public static final double kMaxTurnAccelerationDegPerSSquared = 300;

    public static final double ksVolts = 0.1;
    public static final double kvVoltSecondsPerDegree = 0.08;
    public static final double kaVoltSecondsSquaredPerDegree = 0.015;

    public static final double DRIVE_DISTANCE_DUTY_CYCLE = 0.6;
  }

  public static final class FuelConstants {
    // Motor controller IDs for Fuel Mechanism motors
    public static final int FEEDER_MOTOR_ID = 6;
    public static final int INTAKE_LAUNCHER_MOTOR_ID = 1;   

    // Current limit and nominal voltage for fuel mechanism motors.
    public static final int FEEDER_MOTOR_CURRENT_LIMIT = 50;
    public static final int LAUNCHER_MOTOR_CURRENT_LIMIT = 50;

    // Voltage values for various fuel operations. These values may need to be tuned
    // based on exact robot construction.
    // See the Software Guide for tuning information
    public static final double AGITATION_FEEDER_VOLTAGE = 9;
    public static final double INTAKING_FEEDER_VOLTAGE = 9;
    public static final double INTAKING_INTAKE_VOLTAGE = -5;
    public static final double LAUNCHING_FEEDER_VOLTAGE = -9;
    public static final double LAUNCHING_INTAKE_VOLTAGE = -7.95;
    public static final double SPIN_UP_FEEDER_VOLTAGE = 6;
    public static final double SPIN_UP_SECONDS = 1;
  }

  public static final class ShooterConstants {
    public static final int SHOOTER_CAN_ID = 7;
    public static final int SHOOTER_FOLLOWER_CAN_ID = 9;
    public static final int SHOOTER_CURRENT_LIMIT = 60;

    public static final double SHOOTER_IDLE_VOLTAGE = -1.5;
    public static final double SHOOTER_SHOOTING_VOLTAGE = -5.125;
  }

  public static final class OperatorConstants {
    // Port constants for driver and operator controllers. These should match the
    // values in the Joystick tab of the Driver Station software
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    // Joystick input deadband. Same as default for now.
    public static final double DRIVE_DEADBAND = RobotDriveBase.kDefaultDeadband;

    // This value is multiplied by the joystick value when rotating the robot to
    // help avoid turning too fast and beign difficult to control
    public static final double DRIVE_SCALING = 1.0;
    public static final double ROTATION_SCALING = 0.875;
  }

  public static final class FieldConstants {
    // TODO Andymark layout is used in NC. At world champs, we will probably need to
    // switch. Note that all the measurements in this class could change too.
    public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout
        .loadField(AprilTagFields.k2026RebuiltAndymark);
      
    // Rotate a blue origin Pose2d about this point to make it work for red.
    public static final Pose2d ROTATE_AROUND_FOR_RED = new Pose2d(
            FieldConstants.FIELD_LAYOUT.getFieldLength() / 2.0,
            FieldConstants.FIELD_LAYOUT.getFieldWidth() / 2.0,
            Rotation2d.fromDegrees(180.0));

    public static final double TO_STARTING_LINE_METERS = Inches.of(156.06).in(Meters);
    public static final double LINE_WIDTHS_METERS = Inches.of(2.0).in(Meters);
    public static final double HUB_X_DEPTH_METERS = Inches.of(47.0).in(Meters);
    // Half 45 deg robot plus half hub depth = sides of position and aim triangle.
    public static final double SIDE_PRELOADS_ONLY_Y_OFFSET_METERS = (RobotConstants.DIAGONAL_WITH_BUMPERS_METERS / 2.0)
        + (HUB_X_DEPTH_METERS / 2.0);
    
    public static final double RAMP_FIELD_WIDTH_METERS = Inches.of(44.4).in(Meters);
    public static final double RAMP_ANGLE_RADIANS = Math.toRadians(15.0);
    public static final double RAMP_DRIVE_OVER_METERS = RAMP_FIELD_WIDTH_METERS / Math.cos(RAMP_ANGLE_RADIANS);
    public static final double RAMP_TO_CENTER_LINE_METERS = Inches.of(121.3).in(Meters);
    // Estimate in meters of distance from robot center to ramp after left or right
    // side drive back and rotation.
    public static final double ROBOT_TO_RAMP_EST_METERS = Inches.of(10.0).in(Meters)
        + (RobotConstants.LENGTH_WITH_BUMPERS_METERS / 2.0);
    public static final double DRIVE_TO_CENTER_OVER_RAMP_METERS = ROBOT_TO_RAMP_EST_METERS + RAMP_DRIVE_OVER_METERS
        + RAMP_TO_CENTER_LINE_METERS - 1.5;
  }

  public static final class RobotConstants {
    public static final double FRAME_WIDTH_METERS = Inches.of(26.5).in(Meters);
    public static final double FRAME_LENGTH_METERS = Inches.of(26.5).in(Meters);
    public static final double BUMPER_WIDTH_METERS = Inches.of(3.25).in(Meters);
    public static final double WIDTH_WITH_BUMPERS_METERS = FRAME_WIDTH_METERS + (2.0 * BUMPER_WIDTH_METERS);
    public static final double LENGTH_WITH_BUMPERS_METERS = FRAME_LENGTH_METERS + (2.0 * BUMPER_WIDTH_METERS);
    public static final double DIAGONAL_WITH_BUMPERS_METERS = Math.sqrt(Math.pow(WIDTH_WITH_BUMPERS_METERS, 2.0)
        + Math.pow(LENGTH_WITH_BUMPERS_METERS, 2.0));
  }
}
