// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

/** Follows a {@link Trajectory} created by PathWeaver. */
public class PathWeaverCommand extends CommandBase {
	private final DriveSubsystem m_subsystem;
	private final Boolean m_resetOdometry;

	// Defaults to an empty trajectory if PathWeaver file can not be found.
	private Trajectory m_trajectory = new Trajectory();
	private final SwerveControllerCommand m_command;

	/**
	 * Creates a new {@link PathWeaverCommand} that follows a path generated by
	 * PathWeaver.
	 * 
	 * @param subsystem      The required subsystem.
	 * @param trajectoryJSON The name of the json file containing the path. In the
	 *                       format "BlueHangar2Ball". Should be stored under
	 *                       deploy>paths.
	 * @param resetOdometry  Whether to reset odometry to the starting position of
	 *                       the path. Should be true if it is the first path in a
	 *                       series of paths.
	 */
	public PathWeaverCommand(DriveSubsystem subsystem, String trajectoryJSON, Boolean resetOdometry) {
		m_subsystem = subsystem;
		addRequirements(m_subsystem);

		m_resetOdometry = resetOdometry;

		try {
			m_trajectory = TrajectoryUtil.fromPathweaverJson(
					Filesystem.getDeployDirectory().toPath()
							.resolve("pathplanner/generatedJSON/" + trajectoryJSON + ".wpilib.json"));
		} catch (IOException ex) {
			DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
		}

		final ProfiledPIDController rotPID = new ProfiledPIDController(
				DriveConstants.kPRotController, DriveConstants.kIRotController, DriveConstants.kDRotController,
				new TrapezoidProfile.Constraints(0, 0));
		rotPID.enableContinuousInput(-Math.PI, Math.PI);

		// This doesn't require the subsystem because PathWeaverCommand requires it.
		m_command = new SwerveControllerCommand(
				m_trajectory,
				m_subsystem::getPose,
				DriveConstants.kDriveKinematics,
				new PIDController(0, 0, 0),
				new PIDController(0, 0, 0),
				rotPID,
				m_subsystem::setModuleStates);
	}

	@Override
	public void initialize() {
		if (m_resetOdometry) {
			m_subsystem.resetOdometry(m_trajectory.getInitialPose());
		}
		m_command.initialize();
	}

	@Override
	public void execute() {
		m_command.execute();
	}

	@Override
	public void end(boolean interrupted) {
		m_subsystem.drive(0, 0, 0, false);
		m_command.end(interrupted);
	}

	@Override
	public boolean isFinished() {
		return m_command.isFinished();
	}
}
