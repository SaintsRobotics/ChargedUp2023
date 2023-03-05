package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutonDriveCommand extends CommandBase {
    DriveSubsystem m_driveSubsystem;
  public AutonDriveCommand(DriveSubsystem drive) {
    m_driveSubsystem = drive;
    addRequirements(m_driveSubsystem);

  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveSubsystem.drive(-0.5, 0, 0, false);
  }
}
