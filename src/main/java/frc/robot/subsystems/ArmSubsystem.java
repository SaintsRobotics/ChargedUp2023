// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  private final CANSparkMax m_pivotMotor = new CANSparkMax(36, MotorType.kBrushless);
  private final CANSparkMax m_elevatorMotor = new CANSparkMax(34, MotorType.kBrushless);

  private final CANCoder m_pivotEncoder = new CANCoder(7);

  /** Returns false when arm is detected. */
  private final DigitalInput m_minLimit = new DigitalInput(9);
  /** Returns false when arm is detected. */
  private final DigitalInput m_maxLimit = new DigitalInput(8);

  /** Creates a new {@link ArmSubsystem}. */
  public ArmSubsystem() {
    m_elevatorMotor.setInverted(true);

    m_pivotEncoder.configMagnetOffset(-191);
    m_pivotEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

    // 4:1 gear ratio
    // 22 tooth sprocket
    // 1/4 inch chain
    // 39.37 inches in a meter
    // rotation * 4 = 22 teeth * 1/4 inch = 5.5 inches per 4 rotations = 1.375
    // inches per rotation
    // meters = rotation * 1.375 / 39.37
    m_elevatorMotor.getEncoder().setPositionConversionFactor((22 * 0.25 / 4.0) / 39.37);
    m_elevatorMotor.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
    if (!m_minLimit.get()) {
      m_elevatorMotor.getEncoder().setPosition(0);
    }
    if (!m_maxLimit.get()) {
      m_elevatorMotor.getEncoder().setPosition(0.616);
    }
  }

  /**
   * Sets the speeds of the arm.
   * 
   * @param pivotSpeed    Pivot speed, -1.0 to 1.0.
   * @param elevatorSpeed Elevator speed, -1.0 to 1.0
   */
  public void set(double pivotSpeed, double elevatorSpeed) {
    m_pivotMotor.set(pivotSpeed);
    m_elevatorMotor.set(elevatorSpeed);
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
