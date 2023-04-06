// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation;

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

  /**
   * Constants related to auto align
   */
  public static final class AlignConstants {

    /**
     * Align positions, starting from the right on blue side
     */
    public static final double[] kAlignPositions = new double[] {
        0.369358333, // Cone, DS 1
        0.80856667, // Cube, DS 1
        1.412875, // Cone, DS 1
        1.9695583330000002, // Cone, DS 2
        2.4087666700000003, // Cube, DS 2
        3.013075, // Cone, DS 2
        3.950758333, // Cone, DS 3
        4.38996667, // Cube, DS 3
        4.994275, // Cone, DS 3
        5.534, // HP right most driver side
        5.934 // HP left most driver side
    }; // TODO: tune align positions

    /** Max speed when aligning */
    public static final double kMaxSpeedMetersPerSecond = 1;

    /**
     * How close we need to be to the align position (X) for the robot to be in a
     * legal state
     */
    public static final double kXPositionToleranceMeters = 0.1;

    /** Tolerance for PID setpoint */
    public static final double kPIDTolerance = 0.01;

    /** X position of grid align positions */
    public static final double kXPositionGrid = 1.42875; // TODO: tune grid position

    /** X position of loading align positions */
    public static final double kXPositionLoad = 0.3556; // TODO: tune load position

    /** How long we need to be at the setpoint for the command to terminate */
    public static final double kAtSetpointTimeSeconds = 1;
  }

  public static final class ArmConstants {
    public static final int kPivotMotorPort = 36;
    public static final int kElevatorMotorPort = 34;
    public static final int kPivotEncoderPort = 7;
    public static final int kLowLimitSwitchPort = 9;
    public static final int kHighLimitSwitchPort = 8;

    public static final boolean kPivotMotorInverted = false;
    public static final boolean kElevatorMotorInverted = true;

    public static final double kPivotEncoderOffset = 169;
    public static final double kElevatorEncoderPositionConversionFactor = 0.035;

    public static final double kElevatorStartingPosition = 1.219;
    public static final double kElevatorLowPosition = 1.378;
    public static final double kElevatorHighPosition = 1.999;

    public static final double kPivotMaxPosition = 87;
    public static final double kPivotMinPosition = 12;

    /** Maximum distance the elevator can extend before hitting the stop. */
    public static final double kElevatorMaxPosition = 1.99;
    public static final double kElevatorMinPosition = 1.25;

    public static final double kMotorMountPivotLimit = 45;
    public static final double kMotorMountElevatorLimit = 1.42;

    /** Maximum height legal for the game. */
    public static final double kMaxGameHeight = 1.427;
    public static final double kMaxGameExtension = 1.681;

    public static final double kPivotFeedForwardCoefficient = 0.03;
    public static final double kElevatorFeedForwardCoefficient = 0.05;

    public static final double kPivotMaxSpeed = 0.25;
    public static final double kElevatorMaxSpeed = 0.1;

    public static final double kPPivotPID = 0.2;
    public static final double kPElevatorPID = 30;

    public static final double kPivotTolerance = 1;
    public static final double kElevatorTolerance = 0.01;
  }

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
    public static final double kMaxAngularSpeedRadiansPerSecond = 15.24 / 3;

    public static final double kTurningStopTime = 0.2; // TODO: tune heading correction stop time
    public static final double kSpeedIncreasePerPeriod = 0.15;

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

  public static final class GrabberConstants {
    public static final int kIntakeLeftSolenoidPort = 0;
    public static final int kIntakeRightSolenoidPort = 1;
    public static final int kPneumaticsHubID = 2;

    public static final int kCompressorModuleID = 1;

    public static final int kCompressorMinimumPressure = 85;
    public static final int kCompressorMaximumPressure = 110;
  }

  public static final class LEDConstants {
    public static final int kLEDPort = 0;
    public static final int kLEDLength = 28;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    public static final double kControllerDeadband = 0.2;
    public static final double kSlowModeScalar = 0.8;
  }

  public static final class AutonConstants {
    public static final double maxVelocity = 2;
    public static final double maxAcceleration = 1.5;
  }

  public static final class VisionConstants {

    public static final String kCameraName = "Microsoft_LifeCam_HD-3000";

    // X and Y are from true center of the robot, Angle is from front of the robot.
    public static final Transform3d kCameraOffset = new Transform3d(new Translation3d(0.5, 0, 0.5),
        new Rotation3d(0, 0, 0));
    // Currently set as cam mounted facing forward, half a meter forward of center,
    // half a meter up from center. // TODO adjust vision offset values

    public static final AprilTagFieldLayout kAprilTagFieldLayout = loadFieldLayout();

    // Need this method to catch error thrown by AprilTagFieldLayout because it
    // complains about the file path
    private static AprilTagFieldLayout loadFieldLayout() {
      try {
        return AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
      } catch (IOException ioe) {
        DriverStation.reportError("Failed to load AprilTagFieldLayout, no vision estimation available",
            ioe.getStackTrace());
        return null;
      }
    }
  }
}
