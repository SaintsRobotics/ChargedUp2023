// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class PivotArmCommand extends CommandBase {

  private final PIDController m_pivotPIDController = new PIDController(
    ArmConstants.kPPivotController, 0, 0);

  private ArmSubsystem m_armSubsystem;
  private double m_pivotSetpoint;

  /** Creates a new DriveElevatorCommand. */
  public PivotArmCommand(ArmSubsystem armSubsystem, double setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_armSubsystem = armSubsystem;
    addRequirements(armSubsystem);
    m_pivotSetpoint = setpoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pivotPIDController.setTolerance(0.1);
    m_pivotPIDController.setSetpoint(m_pivotSetpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_armSubsystem.setPivotSpeed(m_pivotPIDController.calculate(m_armSubsystem.getPivotPosition(), m_pivotSetpoint));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armSubsystem.setPivotSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_pivotPIDController.atSetpoint();
  }
}

// 19 --> 26 (turn)