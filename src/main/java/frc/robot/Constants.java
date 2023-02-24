// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

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
    public static final int kFrontLeftDriveMotorPort = 9;
    public static final int kRearLeftDriveMotorPort = 12;
    public static final int kFrontRightDriveMotorPort = 5;
    public static final int kRearRightDriveMotorPort = 2;

    public static final int kFrontLeftTurningMotorPort = 7;
    public static final int kRearLeftTurningMotorPort = 11;
    public static final int kFrontRightTurningMotorPort = 4;
    public static final int kRearRightTurningMotorPort = 16;

    public static final int kFrontLeftTurningEncoderPort = 19;
    public static final int kRearLeftTurningEncoderPort = 20;
    public static final int kFrontRightTurningEncoderPort = 18;
    public static final int kRearRightTurningEncoderPort = 17;

    public static final double kFrontLeftTurningEncoderOffset = 356;
    public static final double kRearLeftTurningEncoderOffset = 122;
    public static final double kFrontRightTurningEncoderOffset = 256;
    public static final double kRearRightTurningEncoderOffset = 328;

    public static final boolean kFrontLeftDriveMotorReversed = false;
    public static final boolean kRearLeftDriveMotorReversed = false;
    public static final boolean kFrontRightDriveMotorReversed = true;
    public static final boolean kRearRightDriveMotorReversed = true;

    /** Distance between centers of right and left wheels on robot. */
    public static final double kTrackWidth = 0.57;

    /** Distance between front and back wheels on robot. */
    public static final double kWheelBase = 0.6;

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final double kMaxSpeedMetersPerSecond = 3.66;
    public static final double kMaxAngularSpeedRadiansPerSecond = 8.76;

    public static final double kPTranslation = 5;
    public static final double kPRotation = 5;
  }

  public static final class ModuleConstants {
    public static final double kWheelDiameterMeters = 0.1;

    /** Gear ratio between the motor and the wheel. */
    public static final double kDrivingGearRatio = 8.14;
    public static final double kPModuleTurningController = 0.3;

  }

  public static final class ArmConstants {
    public static final int kPivotMotorPort = 0; // TODO: fix arm ports
    public static final int kElevatorMotorPort = 0;
    public static final int kPivotEncoderPort = 0;
    public static final double kEncoderOffset = 0;

    public static final int kRestingLimitPort = 0;
    public static final int kTopLimitPort = 0;
    public static final int kMidLimitPort = 0;
    public static final int kStationLimitPort = 0;

    public static final double kMaxPivotSpeedPercent = 0.25;
    public static final double kMaxElevatorSpeedPercent = 0.25;

    public static final double kArmDriveSpeedPercent = 0.2; // Use a low value so that we don't overshoot

    public static final double kRestingPivot = 0; // TODO: find correct angle values for pivot
    public static final double kMidPivot = 0;
    public static final double kTopPivot = 0;
    public static final double kStationPivot = 0;

    public static final int kRestingEH = 0b1000;
    public static final int kMidEH = 0b1100;
    public static final int kTopEH = 0b1110;
    public static final int kStationEH = 0b1111;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    public static final double kControllerDeadband = 0.2;
    public static final double kSlowModeScalar = 0.8;
  }

  public static final double kTurningStopTime = 0.2; // TODO: tune heading correction stop time
}
