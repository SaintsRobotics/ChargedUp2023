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

  private boolean seenSwitch = false; //Elevator won't move down until we see the switch
  private final boolean m_initArmPos; //Whether or not to move arm above limit switch on initilization

  private double eco = 0; //Encoder Correction Offset, updtated whenever a limit switch goes HIGH
  
  /**
   * Constructs a {@link ArmSubsystem}.
   *  Does not move arm above limit switch automatically and always moves arm to resting position
   * 
   */
  public ArmSubsystem() {
    m_pivotPID.enableContinuousInput(-Math.PI, Math.PI);
    m_pivotEncoder.configMagnetOffset(-ArmConstants.kPEncoderOffset);
    
    m_elevatorPID.setSetpoint(ArmConstants.kRestingPos);
    m_pivotPID.setSetpoint(ArmConstants.kRestingPivot);

    m_initArmPos = false;
  }

  /**
   * Subsystem constructer but with more options
   * @param initArmPos Whether or not to move arm above limit switch on init
   * @param armPos Starting arm pos, a negative value will use the default (resting position)
   * @param pivot Starting pivot, a negative value will use the default (resting position)
   */
  public ArmSubsystem(boolean initArmPos, double armPos, double pivot){
    m_pivotPID.enableContinuousInput(-Math.PI, Math.PI);
    m_pivotEncoder.configMagnetOffset(-ArmConstants.kPEncoderOffset);
    
    m_elevatorPID.setSetpoint(armPos < 0 ? ArmConstants.kRestingPos : armPos);
    m_pivotPID.setSetpoint(pivot < 0 ? ArmConstants.kRestingPivot : pivot);

    m_initArmPos = initArmPos;
  }

  /**
   * Converts elevator encoder value to meters
   * @param in Number of rotations motor has made
   * @return Current elevator position in meters
   */
  private double encoder2meters(double in){
    //4:1 gear ratio
    //22 tooth sproket
    //1/4 inch chain
    //39.37 inches in a meter
    //rotation * 4 = 22 teeth * 1/4 inch = 5.5 inches per 4 rotations = 1.375 inches per rotation
    //meters = rotation * 1.375 / 39.37
    return (in * (ArmConstants.sproketTeeth * ArmConstants.inchPerTeeth / ArmConstants.gearRatio) / 39.37 + eco);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Output Current (Amps)", m_elevatorMotor.getOutputCurrent());
    SmartDashboard.putNumber("Pivot Output Current (Amps)", m_pivotMotor.getOutputCurrent());
    SmartDashboard.putNumber("Pivot Encoder", m_pivotEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("Elevator Encoder", m_elevatorMotor.getEncoder().getPosition());

    //Check if we want to move arm above limit switch
    if (m_initArmPos && !seenSwitch) m_elevatorMotor.set(ArmConstants.kMaxElevatorSpeedPercent);

    // Correct arm angle
    if (m_minLimit.get() || m_maxLimit.get()) seenSwitch = true;
    
    //Correct encoder using eoc
    if (m_minLimit.get()){
      eco = 0;
      eco = ArmConstants.kMinSwitchPos - encoder2meters(m_elevatorMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition());
    }
    else if (m_maxLimit.get()){
      eco = 0;
      eco = ArmConstants.kMaxSwitchPos + encoder2meters(m_elevatorMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition());
    }

    double elevatorSpeed = m_elevatorPID.calculate(encoder2meters(m_elevatorMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition()));

    if (m_minLimit.get()) m_elevatorMotor.set(ArmConstants.kMaxElevatorSpeedPercent);
    else if (m_maxLimit.get()) m_elevatorMotor.set(-ArmConstants.kMaxElevatorSpeedPercent);
    else if (seenSwitch || elevatorSpeed > 0)
      m_elevatorMotor.set(elevatorSpeed);

    if (seenSwitch) m_pivotMotor.set(m_pivotPID.calculate(Math.toRadians(m_pivotEncoder.getAbsolutePosition())) * ArmConstants.kMaxPivotSpeedPercent);
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

    // Don't adjust arm if no input because otherwise POV buttons won't work
    if (pivotSpeed == 0 && elevatorSpeed == 0) {
      return;
    }

    //Ensure elevator is above limit switch
    if (!seenSwitch && elevatorSpeed < 0) return;

    if (seenSwitch) m_pivotMotor.set(pivotSpeed);

    //Check limit switches
    if (m_minLimit.get()) m_elevatorMotor.set(ArmConstants.kMaxElevatorSpeedPercent);
    else if (m_maxLimit.get()) m_elevatorMotor.set(-ArmConstants.kMaxElevatorSpeedPercent);
    else m_elevatorMotor.set(elevatorSpeed);

    // Set new setpoint for arm angle if we are no longer moving the arm pivot
    if (pivotSpeed == 0) {
      m_pivotPID.setSetpoint(Math.toRadians(m_pivotEncoder.getAbsolutePosition()));
    }

    if (elevatorSpeed == 0)
      m_elevatorPID.setSetpoint(encoder2meters(m_elevatorMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition()));
  }

  //Because we can't stop both the arm and the pivot at the same time when using setArmSpeeds (see comment in setArmSpeeds) we need a sperate function
  public void stopElevator(){
    m_elevatorMotor.set(0);
  }


  public void stopPivot(){
    m_pivotMotor.set(0);
  }
}
