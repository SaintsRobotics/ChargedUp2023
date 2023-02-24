package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class BalanceCommand extends CommandBase {

    private double m_tolerance = 3.5; // degrees
    private final PIDController m_balancePID = new PIDController(-0.025, 0, 0); //Previous -0.025

    private DriveSubsystem m_driveSubsystem;

    private double m_pitchAngle = 0;

    public BalanceCommand(DriveSubsystem driveSubsystem) {
        m_driveSubsystem = driveSubsystem;
        addRequirements(m_driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_balancePID.enableContinuousInput(-180, 180);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_pitchAngle = m_driveSubsystem.getGyroPitch();
        
        if (Math.abs(m_pitchAngle) > m_tolerance)
            m_driveSubsystem.drive(m_balancePID.calculate(m_pitchAngle, 0), 0, 0, false);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
