// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  private double m_pivotSpeed;
  private double m_elevatorSpeed;

  private final CANSparkMax m_pivotMotor = new CANSparkMax(ArmConstants.kPivotMotorPort,
      MotorType.kBrushless);
  private final CANSparkMax m_elevatorMotor = new CANSparkMax(ArmConstants.kElevatorMotorPort,
      MotorType.kBrushless);

  private final CANCoder m_pivotEncoder = new CANCoder(ArmConstants.kPivotEncoderPort);
  private final CANCoder m_elevatorEncoder = new CANCoder(ArmConstants.kElevatorEncoderPort); // TODO change to 4 limit switches

  /**
   * Constructs a {@link ArmSubsystem}.
   *
   * 
   */
  public ArmSubsystem() {
    m_pivotSpeed = 0;
    m_elevatorSpeed = 0;
  }

  @Override
  public void periodic() {
    m_pivotMotor.set(m_pivotSpeed);
    m_elevatorMotor.set(m_elevatorSpeed);
  }

  /**
   * Sets the speed of the arm pivot motor.
   *
   * @param pivotSpeed The desired speed of the motor.
   */
  public void setPivotSpeed(double pivotSpeed) {
    m_pivotSpeed = pivotSpeed;
  }

  /**
   * Sets the speed of the arm elevator motor.
   *
   * @param elevatorSpeed The desired speed of the elevator motor.
   */
  public void setElevatorSpeed(double elevatorSpeed) {
    m_elevatorSpeed = elevatorSpeed;
  }

  /**
   * Sets the speeds of the arm elevator and pivot motors.
   *
   * @param pivotSpeed The desired speedo of the pivot motor.
   * @param elevatorSpeed The desired speed of the elevator motor.
   */
  public void setArmSpeeds(double pivotSpeed, double elevatorSpeed) {
    m_pivotSpeed = pivotSpeed;
    m_elevatorSpeed = elevatorSpeed;
  }

  /**
   * Returns the encoder value of the pivot motor.
   *
   * @return The pivot encoder position.
   */
  public double getPivotPosition() {
    return m_pivotEncoder.getPosition();
  }

  /**
   * Returns the encoder value of the elevator motor.
   *
   * @return The elevator encoder position.
   */
  public double getElevatorPosition() {
    return m_elevatorEncoder.getPosition();
  }

}
