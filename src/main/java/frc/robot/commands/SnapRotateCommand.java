// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

/** Rotates the robot to the nearest 90 degree angle. */
public class SnapRotateCommand extends CommandBase {
    private final DriveSubsystem m_subsystem;
    private final PIDController m_PID = new PIDController(DriveConstants.kPSnapRotate, 0, 0);

    /**
     * Creates a new {@link SnapRotateCommand}.
     * 
     * @param subsystem The required subsystem.
     */
    public SnapRotateCommand(DriveSubsystem subsystem) {
        m_subsystem = subsystem;
        addRequirements(m_subsystem);

        m_PID.setTolerance(DriveConstants.kToleranceSnapRotate);
        m_PID.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        m_PID.setSetpoint(Math.round(m_subsystem.getPose().getRotation().getRadians() / (Math.PI / 2)) * (Math.PI / 2));
    }

    @Override
    public void execute() {
        m_subsystem.drive(
                0,
                0,
                m_PID.calculate(m_subsystem.getPose().getRotation().getRadians()),
                false);
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.drive(0, 0, 0, true);
    }

    @Override
    public boolean isFinished() {
        return m_PID.atSetpoint();
    }
}