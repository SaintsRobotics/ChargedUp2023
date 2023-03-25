// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;

public class DriveSubsystem extends SubsystemBase {
  private final SwerveModule m_frontLeft = new SwerveModule(
      DriveConstants.kFrontLeftDriveMotorPort,
      DriveConstants.kFrontLeftTurningMotorPort,
      DriveConstants.kFrontLeftTurningEncoderPort,
      DriveConstants.kFrontLeftDriveMotorReversed,
      DriveConstants.kFrontLeftTurningEncoderOffset);

  private final SwerveModule m_rearLeft = new SwerveModule(
      DriveConstants.kRearLeftDriveMotorPort,
      DriveConstants.kRearLeftTurningMotorPort,
      DriveConstants.kRearLeftTurningEncoderPort,
      DriveConstants.kRearLeftDriveMotorReversed,
      DriveConstants.kRearLeftTurningEncoderOffset);

  private final SwerveModule m_frontRight = new SwerveModule(
      DriveConstants.kFrontRightDriveMotorPort,
      DriveConstants.kFrontRightTurningMotorPort,
      DriveConstants.kFrontRightTurningEncoderPort,
      DriveConstants.kFrontRightDriveMotorReversed,
      DriveConstants.kFrontRightTurningEncoderOffset);

  private final SwerveModule m_rearRight = new SwerveModule(
      DriveConstants.kRearRightDriveMotorPort,
      DriveConstants.kRearRightTurningMotorPort,
      DriveConstants.kRearRightTurningEncoderPort,
      DriveConstants.kRearRightDriveMotorReversed,
      DriveConstants.kRearRightTurningEncoderOffset);

  private final AHRS m_gyro = new AHRS();
  private double m_gyroAngle;

  private final PIDController m_headingCorrectionPID = new PIDController(5, 0, 0); // TODO: tune this
  private final Timer m_headingCorrectionTimer;

  private final SwerveDrivePoseEstimator m_swervePoseEstimator;
  private final PhotonCameraWrapper m_photonCamera = new PhotonCameraWrapper();
  private Optional<EstimatedRobotPose> result;

  private SwerveModulePosition[] m_swerveModulePositions = new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_rearLeft.getPosition(),
      m_rearRight.getPosition()
  };

  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
      m_gyro.getRotation2d(), m_swerveModulePositions);

  private final Field2d m_field = new Field2d();

  private final SlewRateLimiter m_xLimiter = new SlewRateLimiter(
      DriveConstants.maxDirectionalAcceleration);
  private final SlewRateLimiter m_yLimiter = new SlewRateLimiter(
      DriveConstants.maxDirectionalAcceleration);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(
      DriveConstants.maxRotationalAcceleration);

  /** Creates a new {@link DriveSubsystem}. */
  public DriveSubsystem() {
    SmartDashboard.putData("Field", m_field);

    m_headingCorrectionPID.enableContinuousInput(-Math.PI, Math.PI);
    m_headingCorrectionPID.setSetpoint(MathUtil.angleModulus(m_gyro.getRotation2d().getRadians()));
    m_headingCorrectionTimer = new Timer();
    m_headingCorrectionTimer.start();

    m_swervePoseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics, m_gyro.getRotation2d(),
        m_swerveModulePositions, m_odometry.getPoseMeters());
  }

  @Override
  public void periodic() {
    m_swerveModulePositions = new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
    };

    m_odometry.update(Robot.isReal() ? m_gyro.getRotation2d() : new Rotation2d(m_gyroAngle), m_swerveModulePositions);

    m_field.setRobotPose(m_odometry.getPoseMeters());

    SmartDashboard.putNumber("gyro angle", m_gyro.getAngle());
    SmartDashboard.putNumber("pitch", m_gyro.getPitch());
    SmartDashboard.putNumber("yaw", m_gyro.getYaw());
    SmartDashboard.putNumber("roll", m_gyro.getRoll());

    // Check if we have a camera
    if (!m_photonCamera.camera.isConnected()) {
      SmartDashboard.putString("Vision Status", "No camera connected: " +
          "Check if coprocessor is connected and that camera is plugged in.");
    }

    // Check if we have april tags
    result = m_photonCamera.getEstimatedGlobalPose(m_odometry.getPoseMeters());

    if (!m_photonCamera.camera.getLatestResult().hasTargets()) {
      SmartDashboard.putString("Vision Status", "No AprilTags detected");
    } else {
      SmartDashboard.putString("Vision Status", "AprilTags detected");
    }

    // if we have an estimated pose, update pose using pose estimator
    if (result.isPresent()) {
      EstimatedRobotPose camPose = result.get();
      m_swervePoseEstimator.resetPosition(m_gyro.getRotation2d(), m_swerveModulePositions, m_odometry.getPoseMeters());
      m_swervePoseEstimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
    }
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Robot.isReal() ? m_gyro.getRotation2d() : new Rotation2d(m_gyroAngle),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    if (fieldRelative != isRel){
      xSpeed = 0;
      ySpeed = 0;
      rot = 0;
      m_xLimiter.reset(0);
      m_yLimiter.reset(0);
      m_rotLimiter.reset(0);
    }

    isRel = fieldRelative;

    if (rot != 0) {
      m_headingCorrectionTimer.reset();
    }

    double rotation = rot;

    double currentAngle = MathUtil.angleModulus(m_gyro.getRotation2d().getRadians());

    if ((xSpeed == 0 && ySpeed == 0) || m_headingCorrectionTimer.get() < DriveConstants.kTurningStopTime) {
      m_headingCorrectionPID.setSetpoint(currentAngle);
    } else {
      rotation = m_headingCorrectionPID.calculate(currentAngle);
    }

    xSpeed = m_xLimiter.calculate(xSpeed);
    ySpeed = m_yLimiter.calculate(ySpeed);
    rotation = m_rotLimiter.calculate(rotation);

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotation,
                Robot.isReal() ? m_gyro.getRotation2d() : new Rotation2d(m_gyroAngle))
            : new ChassisSpeeds(xSpeed, ySpeed, rotation));
    setModuleStates(swerveModuleStates);
  }

  private boolean isRel = false;
  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);

    // Takes the integral of the rotation speed to find the current angle for the
    // simulator
    m_gyroAngle += DriveConstants.kDriveKinematics.toChassisSpeeds(desiredStates).omegaRadiansPerSecond
        * Robot.kDefaultPeriod;
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
    m_gyroAngle = 0;
  }

  /**
   * Gets current gyro pitch angle in degrees
   * 
   * @return Current gyro pitch angle
   */
  public double getGyroPitch() {
    return m_gyro.getPitch();
  }
}
