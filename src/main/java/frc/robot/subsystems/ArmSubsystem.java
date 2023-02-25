// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  private final CANSparkMax m_pivotMotor = new CANSparkMax(ArmConstants.kPivotMotorPort,
      MotorType.kBrushless);
  private final CANSparkMax m_elevatorMotor = new CANSparkMax(ArmConstants.kElevatorMotorPort,
      MotorType.kBrushless);

  private final CANCoder m_pivotEncoder = new CANCoder(ArmConstants.kPivotEncoderPort);

  private final DigitalInput m_minLimit = new DigitalInput(ArmConstants.kMinLimitPort);
  private final DigitalInput m_maxLimit = new DigitalInput(ArmConstants.kMaxLimitPort);

  // PID for correcting arm angle (both when changing pivot and to counteract gravity)
  private final PIDController m_pivotPID = new PIDController(0, 0, 0); // TODO: tune arm PID controllers
  private final PIDController m_elevatorPID = new PIDController(0, 0, 0);
  
  /**
   * Constructs a {@link ArmSubsystem}.
   *
   * 
   */
  public ArmSubsystem() {
    m_pivotPID.enableContinuousInput(-Math.PI, Math.PI);
    m_pivotEncoder.configMagnetOffset(-ArmConstants.kPEncoderOffset);
    
    m_elevatorPID.setSetpoint(ArmConstants.kRestingPos);
    m_pivotPID.setSetpoint(ArmConstants.kRestingPivot);

    }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Output Current (Amps)", m_elevatorMotor.getOutputCurrent());
    SmartDashboard.putNumber("Pivot Output Current (Amps)", m_pivotMotor.getOutputCurrent());
    SmartDashboard.putNumber("Pivot Encoder", m_pivotEncoder.getAbsolutePosition());

    // Correct arm angle
    m_pivotMotor.set(m_pivotPID.calculate(Math.toRadians(m_pivotEncoder.getAbsolutePosition())) * ArmConstants.kMaxPivotSpeedPercent);

    if (m_minLimit.get()) m_elevatorMotor.set(ArmConstants.kMaxElevatorSpeedPercent);
    else if (m_maxLimit.get()) m_elevatorMotor.set(-ArmConstants.kMaxElevatorSpeedPercent);
    else m_elevatorMotor.set(m_elevatorPID.calculate((m_elevatorMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition() * 4 / 5.5) / 39.37));
  }

  /**
   * Makes the arm go into resting position
   */
  public void goResting() {
    m_pivotPID.setSetpoint(ArmConstants.kRestingPivot);
    m_elevatorPID.setSetpoint(ArmConstants.kRestingPos);
  }

  /**
   * Makes the arm go into top position
   */
  public void goTop() {
    m_pivotPID.setSetpoint(ArmConstants.kTopPivot);
    m_elevatorPID.setSetpoint(ArmConstants.kTopPos);
  }

  /**
   * Makes the arm go into mid position
   */
  public void goMid() {
    m_pivotPID.setSetpoint(ArmConstants.kMidPivot);
    m_elevatorPID.setSetpoint(ArmConstants.kMidPos);
  }

  /**
   * Makes the arm go into station position
   */
  public void goStation() {
    m_pivotPID.setSetpoint(ArmConstants.kStationPivot);
    m_elevatorPID.setSetpoint(ArmConstants.kStationPos);
  }

  /**
   * Sets the speeds of the arm elevator and pivot motors.
   *
   * @param pivotSpeed    The desired speed (in percent) of the pivot motor.
   * @param elevatorSpeed The desired speed (in percent) of the elevator motor.
   */
  public void setArmSpeeds(double pivotSpeed, double elevatorSpeed) {

    // Don't adjust arm if no input
    if (pivotSpeed == 0 && elevatorSpeed == 0) {
      return;
    }

    m_pivotMotor.set(pivotSpeed);
    m_elevatorMotor.set(elevatorSpeed);

    // Set new setpoint for arm angle if we are no longer moving the arm pivot
    if (pivotSpeed == 0) {
      m_pivotPID.setSetpoint(Math.toRadians(m_pivotEncoder.getAbsolutePosition()));
    }

    if (elevatorSpeed == 0)
      m_elevatorPID.setSetpoint((m_elevatorMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition() * 4 / 5.5) / 39.37);
  }
}
