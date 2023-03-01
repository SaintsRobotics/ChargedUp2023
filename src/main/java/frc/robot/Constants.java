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
    public static final int kFrontLeftDriveMotorPort = 28;
    public static final int kRearLeftDriveMotorPort = 27;
    public static final int kFrontRightDriveMotorPort = 30;
    public static final int kRearRightDriveMotorPort = 33;

    public static final int kFrontLeftTurningMotorPort = 29;
    public static final int kRearLeftTurningMotorPort = 26;
    public static final int kFrontRightTurningMotorPort = 31;
    public static final int kRearRightTurningMotorPort = 32;

    public static final int kFrontLeftTurningEncoderPort = 3;
    public static final int kRearLeftTurningEncoderPort = 5;
    public static final int kFrontRightTurningEncoderPort = 4;
    public static final int kRearRightTurningEncoderPort = 6;

    public static final double kFrontLeftTurningEncoderOffset = 237;
    public static final double kRearLeftTurningEncoderOffset = 86.8;
    public static final double kFrontRightTurningEncoderOffset = 297.5;
    public static final double kRearRightTurningEncoderOffset = 225.3;

    public static final boolean kFrontLeftDriveMotorReversed = false;
    public static final boolean kRearLeftDriveMotorReversed = false;
    public static final boolean kFrontRightDriveMotorReversed = true;
    public static final boolean kRearRightDriveMotorReversed = true;

    /** Distance between centers of right and left wheels on robot (in meters). */
    public static final double kTrackWidth = 0.48;

    /** Distance between front and back wheels on robot (in meters). */
    public static final double kWheelBase = 0.63;

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final double kMaxSpeedMetersPerSecond = 3.6576;
    public static final double kMaxAngularSpeedRadiansPerSecond = 15.24;

    public static final double kTurningStopTime = 0.2; // TODO: tune heading correction stop time

    public static final double kPTranslation = 0.5;
    public static final double kPRotation = 0.5;
    public static final double kPSnapRotate = 6;

    public static final double kToleranceBalance = 3.5;
    public static final double kToleranceSnapRotate = 0.02;
  }

  public static final class ModuleConstants {
    public static final double kWheelDiameterMeters = 0.1;

    /** Gear ratio between the motor and the wheel. */

    public static final double kDrivingGearRatio = 8.14; // SDS MK4i's in L1 configuration

    public static final double kPModuleTurningController = -0.3;

  }

  public static final class ArmConstants {
    public static final int kPivotMotorPort = 36;
    public static final int kElevatorMotorPort = 34;
    public static final int kPivotEncoderPort = 7;
    public static final double kPivotEncoderOffset = -191;

    public static final double kMaxPivotSpeedPercent = 20;
    public static final double kMaxElevatorSpeedPercent = 0.2;

    public static final int kMinLimitPort = 9;
    public static final int kMaxLimitPort = 8;

    public static final double kRestingPivot = 0; // TODO: find correct angle values for pivot, degrees
    public static final double kMidPivot = 0;
    public static final double kTopPivot = 0;
    public static final double kStationPivot = 0;

    public static final double kRestingPos = 0; // TODO: measure correct positions, meters
    public static final double kMidPos = 0;
    public static final double kTopPos = 0;
    public static final double kStationPos = 0;

    public static final int sproketTeeth = 22;
    public static final double inchPerTeeth = 0.25;
    public static final double gearRatio = 4 / 1;

    public static final double kMinSwitchPos = 0;
    public static final double kMaxSwitchPos = 0.61595 - 0.05;

    public static final double kMinPivotPos = 0; // TODO find accurate min and max for pivot
    public static final double kMaxPivotPos = 92; // 0 is straight up
  }

  public static final class GrabberConstants {
    public static final int kIntakeLeftSolenoidPort = 0;
    public static final int kIntakeRightSolenoidPort = 1;
    public static final int kPneumaticsHubID = 2;

    public static final int kCompressorModuleID = 1;

    public static final int kCompressorMinimumPressure = 70;
    public static final int kCompressorMaximumPressure = 110;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    public static final double kControllerDeadband = 0.2;
    public static final double kSlowModeScalar = 0.8;
  }

  public static final class AutonConstants {
    public static final int maxVelocity = 4;
    public static final int maxAcceleration = 3;
  }
}
