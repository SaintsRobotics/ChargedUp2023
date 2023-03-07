// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  private final CANSparkMax m_pivotMotor = new CANSparkMax(ArmConstants.kPivotMotorPort, MotorType.kBrushless);
  private final CANSparkMax m_elevatorMotor = new CANSparkMax(ArmConstants.kElevatorMotorPort, MotorType.kBrushless);

  private final CANCoder m_pivotEncoder = new CANCoder(ArmConstants.kPivotEncoderPort);

  /** Returns false when arm is detected. */
  private final DigitalInput m_lowLimitSwitch = new DigitalInput(ArmConstants.kLowLimitSwitchPort);
  /** Returns false when arm is detected. */
  private final DigitalInput m_highLimitSwitch = new DigitalInput(ArmConstants.kHighLimitSwitchPort);

  private double m_pivotSpeed;
  private double m_elevatorSpeed;

  /** Creates a new {@link ArmSubsystem}. */
  public ArmSubsystem() {
    m_pivotMotor.setInverted(ArmConstants.kPivotMotorInverted);
    m_elevatorMotor.setInverted(ArmConstants.kElevatorMotorInverted);

    m_pivotEncoder.configMagnetOffset(ArmConstants.kPivotEncoderOffset);
    m_pivotEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

    m_elevatorMotor.getEncoder().setPositionConversionFactor(ArmConstants.kElevatorEncoderPositionConversionFactor);
    m_elevatorMotor.getEncoder().setPosition(ArmConstants.kElevatorStartingPosition);
  }

  @Override
  public void periodic() {
    // Sets the position of the elevator encoder when it sees a limit switch
    if (!m_lowLimitSwitch.get()) {
      m_elevatorMotor.getEncoder().setPosition(ArmConstants.kElevatorLowPosition);
    }
    if (!m_highLimitSwitch.get()) {
      m_elevatorMotor.getEncoder().setPosition(ArmConstants.kElevatorHighPosition);
    }

    m_pivotMotor.set(m_pivotSpeed);
    m_elevatorMotor.set(m_elevatorSpeed + (Math.cos(Math.toRadians(m_pivotEncoder.getAbsolutePosition())) * 0.05));

    // Remove before merging
    SmartDashboard.putNumber("pivot position", m_pivotEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("elevator position", m_elevatorMotor.getEncoder().getPosition());
  }

  /**
   * Sets the speeds of the arm.
   * 
   * @param pivotSpeed    Pivot speed, -1.0 to 1.0.
   * @param elevatorSpeed Elevator speed, -1.0 to 1.0
   */
  public void set(double pivotSpeed, double elevatorSpeed) {
    m_pivotSpeed = pivotSpeed;
    m_elevatorSpeed = elevatorSpeed;
  }

  /**
   * Gets the position of the pivot.
   * 
   * @return The position of the pivot, -180 to 180.
   */
  public double getPivotPosition() {
    return m_pivotEncoder.getAbsolutePosition();
  }

  /**
   * Gets the position of the elevator.
   * 
   * @return The position of the elevator in meters.
   */
  public double getElevatorPosition() {
    return m_elevatorMotor.getEncoder().getPosition();
  }
}
