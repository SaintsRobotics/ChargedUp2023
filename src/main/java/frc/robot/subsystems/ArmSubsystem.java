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
      MotorType.kBrushless); //TODO: TEST IF ELEVATOR AND PIVOT MOTORS NEED TO BE INVERTED
  private final CANSparkMax m_elevatorMotor = new CANSparkMax(ArmConstants.kElevatorMotorPort,
      MotorType.kBrushless);

  private final CANCoder m_pivotEncoder = new CANCoder(ArmConstants.kPivotEncoderPort);

  private final DigitalInput m_minLimit = new DigitalInput(ArmConstants.kMinLimitPort);
  private final DigitalInput m_maxLimit = new DigitalInput(ArmConstants.kMaxLimitPort);

  // PID for correcting arm angle (both when changing pivot and to counteract
  // gravity)
  private final PIDController m_pivotPID = new PIDController(0, 0, 0); // TODO: tune arm PID controllers
  /** setpoint is in meters */
  private final PIDController m_elevatorPID = new PIDController(0, 0, 0);

  private boolean seenSwitch = false; // Elevator won't move down until we see the switch
  private double m_encoderOffset = 0; // Encoder offset, updtated whenever a limit switch goes HIGH, in meters
  private Double m_elevatorSpeed = 0.0;
  private Double m_pivotSpeed = 0.0;

  /**
   * Constructs a {@link ArmSubsystem}.
   * Does not move arm above limit switch automatically and always moves arm to
   * resting position
   * 
   */
  public ArmSubsystem() {
    m_pivotPID.enableContinuousInput(-Math.PI, Math.PI);
    m_pivotEncoder.configMagnetOffset(-ArmConstants.kPEncoderOffset);

    m_elevatorPID.setSetpoint(ArmConstants.kRestingPos);
    m_pivotPID.setSetpoint(ArmConstants.kRestingPivot);
  }

  /**
   * Converts elevator spark max encoder value to meters
   * 
   * @param encoderValue Number of rotations motor has made
   * @return Current elevator position in meters
   */
  private double encoderToMeters(double encoderValue) {
    // 4:1 gear ratio
    // 22 tooth sproket
    // 1/4 inch chain
    // 39.37 inches in a meter
    // rotation * 4 = 22 teeth * 1/4 inch = 5.5 inches per 4 rotations = 1.375
    // inches per rotation
    // meters = rotation * 1.375 / 39.37
    return m_encoderOffset + (encoderValue * (ArmConstants.sproketTeeth * ArmConstants.inchPerTeeth / ArmConstants.gearRatio) / 39.37);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Output Current (Amps)", m_elevatorMotor.getOutputCurrent());
    SmartDashboard.putNumber("Pivot Output Current (Amps)", m_pivotMotor.getOutputCurrent());
    SmartDashboard.putNumber("Pivot Encoder", m_pivotEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("Elevator Encoder", m_elevatorMotor.getEncoder().getPosition());

    if (m_minLimit.get() || m_maxLimit.get()) {
      seenSwitch = true;
    }
    
    if (m_minLimit.get()){
      m_encoderOffset = 0;
      m_encoderOffset = ArmConstants.kMinSwitchPos - encoderToMeters(m_elevatorMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition());
    }

    if (m_maxLimit.get()){
      m_encoderOffset = 0;
      m_encoderOffset = ArmConstants.kMaxSwitchPos - encoderToMeters(m_elevatorMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition());
    }

    if (m_elevatorSpeed != 0 || m_pivotSpeed != 0){
      if (seenSwitch){
        m_elevatorMotor.set(m_elevatorSpeed);
        m_pivotMotor.set(m_pivotSpeed);
      }
      else if (m_elevatorSpeed > 0) m_elevatorMotor.set(m_elevatorSpeed);
    }

    else if (m_elevatorSpeed != null && m_elevatorSpeed == 0 && m_pivotSpeed == 0) {
      m_pivotPID.setSetpoint(Math.toRadians(m_pivotEncoder.getAbsolutePosition()));
      m_elevatorPID.setSetpoint(encoderToMeters(m_elevatorMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition()));
    }

    else{
      double elevatorSpeed = m_elevatorPID.calculate(encoderToMeters(m_elevatorMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition()));
      if (seenSwitch){
        m_elevatorMotor.set(elevatorSpeed);
        m_pivotMotor.set(Math.toRadians(m_pivotEncoder.getAbsolutePosition()));
      }
      else if (elevatorSpeed > 0) m_elevatorMotor.set(elevatorSpeed);
    }
  }

  /**
   * Makes the arm go into resting position
   */
  public void goResting() {
    m_pivotPID.setSetpoint(ArmConstants.kRestingPivot);
    m_elevatorPID.setSetpoint(ArmConstants.kRestingPos);
    m_elevatorSpeed = null;
  }

  /**
   * Makes the arm go into top position
   */
  public void goTop() {
    m_pivotPID.setSetpoint(ArmConstants.kTopPivot);
    m_elevatorPID.setSetpoint(ArmConstants.kTopPos);
    m_elevatorSpeed = null;
  }

  /**
   * Makes the arm go into mid position
   */
  public void goMid() {
    m_pivotPID.setSetpoint(ArmConstants.kMidPivot);
    m_elevatorPID.setSetpoint(ArmConstants.kMidPos);
    m_elevatorSpeed = null;
  }

  /**
   * Makes the arm go into station position
   */
  public void goStation() {
    m_pivotPID.setSetpoint(ArmConstants.kStationPivot);
    m_elevatorPID.setSetpoint(ArmConstants.kStationPos);
    m_elevatorSpeed = null;
  }

  /**
   * Sets the speeds of the arm elevator and pivot motors.
   *
   * @param pivotSpeed    The desired speed (in percent) of the pivot motor.
   * @param elevatorSpeed The desired speed (in percent) of the elevator motor.
   */
  public void setArmSpeeds(double pivotSpeed, double elevatorSpeed) {
    if (m_elevatorSpeed == null && elevatorSpeed == 0 && pivotSpeed == 0) return;
    m_pivotSpeed = pivotSpeed;
    m_elevatorSpeed = elevatorSpeed;
  }
}
