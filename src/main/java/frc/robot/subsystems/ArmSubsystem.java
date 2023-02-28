// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Robot;

public class ArmSubsystem extends SubsystemBase {
  private final CANSparkMax m_pivotMotor = new CANSparkMax(ArmConstants.kPivotMotorPort,
      MotorType.kBrushless); // TODO: TEST IF PIVOT MOTOR NEEDS TO BE INVERTED
  private final CANSparkMax m_elevatorMotor = new CANSparkMax(ArmConstants.kElevatorMotorPort,
      MotorType.kBrushless);

  private final CANCoder m_pivotEncoder = new CANCoder(ArmConstants.kPivotEncoderPort);

  /** These limit switches go LOW when they detect something */
  private final DigitalInput m_minLimit = new DigitalInput(ArmConstants.kMinLimitPort);
  /** These limit switches go LOW when they detect something */
  private final DigitalInput m_maxLimit = new DigitalInput(ArmConstants.kMaxLimitPort);

  // PID for correcting arm angle (both when changing pivot and to counteract
  // gravity)
  private final PIDController m_pivotPID = new PIDController(-0.05, 0, 0); // TODO: tune pivot PID controller
  /** setpoint is in meters */
  public final PIDController m_elevatorPID = new PIDController(1.5, 0, 0);


  public boolean seenSwitch = false; // Elevator won't move down until we see the switch
  private double m_encoderOffset = -0.16; // Encoder offset, updtated whenever a limit switch goes LOW, in meters

  /**
   * Constructs a {@link ArmSubsystem}.
   * Does not move arm above limit switch automatically and always moves arm to
   * resting position
   * 
   */
  public ArmSubsystem() {
    m_pivotPID.enableContinuousInput(-180, 180);
    m_pivotEncoder.configMagnetOffset(ArmConstants.kPivotEncoderOffset);
    m_pivotEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

    m_elevatorMotor.setInverted(true);
  }

  /** Runs on robot enable and resets PID controller and sets seenSwitch to false */
  public void enable(){
    m_pivotPID.reset();
    m_pivotPID.setSetpoint(m_pivotEncoder.getPosition());

    m_elevatorPID.reset();
    m_elevatorPID.setSetpoint(getElevatorEncoder());

    seenSwitch = false;
  }

  @Override
  public void periodic() {
    double ps = m_pivotPID.calculate(m_pivotEncoder.getPosition());
    SmartDashboard.putNumber("Elevator Output Current (Amps)", m_elevatorMotor.getOutputCurrent());
    SmartDashboard.putNumber("Pivot Output Current (Amps)", m_pivotMotor.getOutputCurrent());
    SmartDashboard.putNumber("Pivot Encoder", m_pivotEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("Elevator Relative Encoder m", getElevatorEncoder()); // this encoder has a conversion factor method. We want non absolute encoder because values can go past 360 and we need to know that
    SmartDashboard.putBoolean("Min Limit Switch", !m_minLimit.get());
    SmartDashboard.putBoolean("Max Limit Switch", !m_maxLimit.get());
    SmartDashboard.putBoolean("Seen Switch", seenSwitch);
    SmartDashboard.putNumber("elevator speed", m_elevatorMotor.get());
    SmartDashboard.putNumber("Elevator PID error", m_elevatorPID.getPositionError());
    SmartDashboard.putNumber("Elevator PID setpoint", m_elevatorPID.getSetpoint());
    SmartDashboard.putNumber("Pivot Amps", m_pivotMotor.getOutputCurrent());
    SmartDashboard.putNumber("Pivot PID out", m_encoderOffset);

    if (!m_minLimit.get() || !m_maxLimit.get()) {
      seenSwitch = true;
    }
    
    if (!m_minLimit.get()) {
      m_encoderOffset = ArmConstants.kMinSwitchPos - encoderToMeters(m_elevatorMotor.getEncoder().getPosition());
      m_pivotPID.setSetpoint(30);
    }
    if (!m_maxLimit.get()) {
      m_encoderOffset = ArmConstants.kMaxSwitchPos - encoderToMeters(m_elevatorMotor.getEncoder().getPosition());
    }

    if (m_elevatorPID.getSetpoint() < (ArmConstants.kMinSwitchPos + 0.075) && seenSwitch) {
      m_elevatorPID.setSetpoint(ArmConstants.kMinSwitchPos + 0.075);
    }
    if (m_elevatorPID.getSetpoint() > ArmConstants.kMaxSwitchPos) {
      m_elevatorPID.setSetpoint(ArmConstants.kMaxSwitchPos);
    }

    if (m_elevatorPID.getSetpoint() < getElevatorEncoder() && !seenSwitch) {
      m_elevatorPID.setSetpoint(getElevatorEncoder());
    }

    if (m_pivotPID.getSetpoint() < ArmConstants.kMinPivotPos) {
      m_pivotPID.setSetpoint(ArmConstants.kMinPivotPos);
    }
    if (m_elevatorPID.getSetpoint() > ArmConstants.kMaxPivotPos) {
      m_pivotPID.setSetpoint(ArmConstants.kMaxPivotPos);
    }

    if (seenSwitch) {
      m_pivotMotor.set(MathUtil.clamp(ps, -0.05, 0.05)); //TODO: adjust clamp value
    }
    
   m_elevatorMotor.set(MathUtil.clamp(m_elevatorPID.calculate(getElevatorEncoder()), -0.25, 0.25) + (0.03 * Math.cos(m_pivotEncoder.getPosition()))); //TODO: take angle into account

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
   * @param pivotSpeed    The desired speed of the pivot motor.
   * @param elevatorSpeed The desired speed of the elevator motor.
   */
  public void setArmSpeeds(double pivotSpeed, double elevatorSpeed) {
    if (pivotSpeed == 0 && elevatorSpeed == 0) {
      return;
    }

    m_pivotPID.setSetpoint(m_pivotEncoder.getPosition() + (pivotSpeed * Robot.kDefaultPeriod));
    m_elevatorPID.setSetpoint(m_elevatorPID.getSetpoint() + (elevatorSpeed * Robot.kDefaultPeriod));
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
    return encoderValue * (ArmConstants.sproketTeeth * ArmConstants.inchPerTeeth / ArmConstants.gearRatio) / 39.37;
  }
  
  /** 
   * Gets the elevator encoder value in meters with the offset.
   * 
   * @return The elevator encoder value.
   */
  private double getElevatorEncoder() {
    return m_encoderOffset + encoderToMeters(m_elevatorMotor.getEncoder().getPosition());
    // should setPositionConversionFactor on the motor rather than using encoderToMeters
  }
}

