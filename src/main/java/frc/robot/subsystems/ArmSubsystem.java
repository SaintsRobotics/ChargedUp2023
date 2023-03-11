// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
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

    // Prevents pivot from extending beyond reasonable limits
    if (m_pivotEncoder.getAbsolutePosition() < ArmConstants.kPivotMinPosition && m_pivotSpeed < 0) {
      m_pivotSpeed = 0;
    } else if (m_pivotEncoder.getAbsolutePosition() > ArmConstants.kPivotMaxPosition && m_pivotSpeed > 0) {
      m_pivotSpeed = 0;
    }

    // Prevents elevator from extending until it hits the stop
    if (m_elevatorMotor.getEncoder().getPosition() < ArmConstants.kElevatorMinPosition && m_elevatorSpeed < 0) {
      m_elevatorSpeed = 0;
    } else if (m_elevatorMotor.getEncoder().getPosition() > ArmConstants.kElevatorMaxPosition && m_elevatorSpeed > 0) {
      m_elevatorSpeed = 0;
    }

    // If arm is above legal height, bring it down
    double heightOverLimit = m_elevatorMotor.getEncoder().getPosition()
        - (ArmConstants.kMaxGameHeight / Math.cos(Math.toRadians(m_pivotEncoder.getAbsolutePosition())));
    if (heightOverLimit > 0 && m_elevatorSpeed > -heightOverLimit * 2) {
      m_elevatorSpeed = -heightOverLimit * 2;
    }

    // If arm is above legal extension, bring it in
    double extensionOverLimit = m_elevatorMotor.getEncoder().getPosition()
        - (ArmConstants.kMaxGameExtension / Math.sin(Math.toRadians(m_pivotEncoder.getAbsolutePosition())));
    if (extensionOverLimit > 0 && m_elevatorSpeed > -extensionOverLimit * 2) {
      m_elevatorSpeed = -extensionOverLimit * 2;
    }

    // Counters the effect of gravity with a small feed forward
    m_pivotMotor.set(m_pivotSpeed
        - (Math.sin(Math.toRadians(m_pivotEncoder.getAbsolutePosition()))
            * ArmConstants.kPivotFeedForwardCoefficient));
    m_elevatorMotor.set(m_elevatorSpeed
        + (Math.cos(Math.toRadians(m_pivotEncoder.getAbsolutePosition()))
            * ArmConstants.kElevatorFeedForwardCoefficient));
  }

  /**
   * Sets the speeds of the arm.
   * 
   * @param pivotSpeed    Pivot speed, -1.0 to 1.0.
   * @param elevatorSpeed Elevator speed, -1.0 to 1.0
   */
  public void set(double pivotSpeed, double elevatorSpeed) {
    // Prevents setting the speed of the arm above the max
    m_pivotSpeed = MathUtil.clamp(pivotSpeed * ArmConstants.kPivotMaxSpeed, -ArmConstants.kPivotMaxSpeed,
        ArmConstants.kPivotMaxSpeed);
    m_elevatorSpeed = MathUtil.clamp(elevatorSpeed * ArmConstants.kElevatorMaxSpeed, -ArmConstants.kElevatorMaxSpeed,
        ArmConstants.kElevatorMaxSpeed);
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
