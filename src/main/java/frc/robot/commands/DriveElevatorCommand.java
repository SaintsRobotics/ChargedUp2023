// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class DriveElevatorCommand extends CommandBase {

  private final PIDController m_elevatorPIDController = new PIDController(
    ArmConstants.kPElevatorController, 0, 0);

  private ArmSubsystem m_armSubsystem;
  private double m_elevatorSetpoint;

  /** Creates a new DriveElevatorCommand. */
  public DriveElevatorCommand(ArmSubsystem armSubsystem, double setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_armSubsystem = armSubsystem;
    addRequirements(armSubsystem);
    m_elevatorSetpoint = setpoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevatorPIDController.setTolerance(0.1);
    m_elevatorPIDController.setSetpoint(m_elevatorSetpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_armSubsystem.setElevatorSpeed(m_elevatorPIDController.calculate(m_armSubsystem.getElevatorPosition(), m_elevatorSetpoint));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armSubsystem.setElevatorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_elevatorPIDController.atSetpoint();
  }
}
