// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Robot;

public class SwerveModule {
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final CANCoder m_turningEncoder;

  private final PIDController m_turningPIDController = new PIDController(
      ModuleConstants.kPModuleTurningController, 0, 0);

  private SwerveModuleState m_state = new SwerveModuleState();
  private double m_distance;

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel     The channel of the drive motor.
   * @param turningMotorChannel   The channel of the turning motor.
   * @param turningEncoderChannel The channel of the turning encoder.
   * @param driveMotorReversed    Whether the drive motor is reversed.
   * @param turningEncoderOffset  Offset of the turning encoder.
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderChannel,
      boolean driveMotorReversed,
      double turningEncoderOffset) {
    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);
    m_turningEncoder = new CANCoder(turningEncoderChannel);

    // converts default units to meters per second
    m_driveMotor.getEncoder().setVelocityConversionFactor(
        ModuleConstants.kWheelDiameterMeters * Math.PI / 60 / ModuleConstants.kDrivingGearRatio);
    m_driveMotor.setIdleMode(IdleMode.kBrake);
    m_driveMotor.setInverted(driveMotorReversed);

    m_turningMotor.setIdleMode(IdleMode.kBrake);

    // converts default units to radians
    m_turningEncoder.configFeedbackCoefficient(Math.toRadians(0.087890625), "radians", SensorTimeBase.PerSecond);
    m_turningEncoder.configMagnetOffset(-turningEncoderOffset);

    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    m_distance += m_state.speedMetersPerSecond * Robot.kDefaultPeriod;
    return Robot.isReal()
        ? new SwerveModulePosition(m_driveMotor.getEncoder().getPosition(),
            new Rotation2d(m_turningEncoder.getPosition()))
        : new SwerveModulePosition(m_distance, m_state.angle);
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    m_state = SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningEncoder.getPosition()));

    final double driveOutput = m_driveMotor.getEncoder().getVelocity();

    final double turnOutput = m_turningPIDController.calculate(m_turningEncoder.getPosition(),
        m_state.angle.getRadians());

    m_driveMotor.set(driveOutput);
    m_turningMotor.set(turnOutput);
  }
}
