package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.BaseAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;

import frc.robot.commands.ArmCommand;
import frc.robot.commands.BalanceCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class AutoBuilder extends BaseAutoBuilder {
	private final DriveSubsystem m_driveSubsystem;
	private final ArmSubsystem m_armSubsystem;
	private final GrabberSubsystem m_grabberSubsystem;

	public class ScheduleDrive extends CommandBase {
		public final double vX;
		public final double vY;
		public final double vTheta;
		public boolean m_isFinished = false;

		public ScheduleDrive(DriveSubsystem subsystem, double deltaX, double deltaY, double deltaTheta, double time) {
			vX = deltaX / time;
			vY = deltaY / time;
			vTheta = deltaTheta / time;

			andThen(new ParallelDeadlineGroup(new WaitCommand(time),
					new RunCommand(() -> m_driveSubsystem.drive(vX, vY, vTheta, true), m_driveSubsystem)));

			m_isFinished = true;
		}

		@Override
		public void initialize() {
			super.initialize();
		}

		@Override
		public void execute() {
			super.execute();
		}

		@Override
		public void end(boolean interrupted) {
			super.end(interrupted);
		}

		@Override
		public boolean isFinished() {
			return m_isFinished;
		}
	}

	public AutoBuilder(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem,
			GrabberSubsystem grabberSubsystem) {
		super(driveSubsystem::getPose, driveSubsystem::resetOdometry, new HashMap<String, Command>(),
				DrivetrainType.HOLONOMIC,
				true);
				
		m_driveSubsystem = driveSubsystem;
		m_armSubsystem = armSubsystem;
		m_grabberSubsystem = grabberSubsystem;

		eventMap.put("DropHigh",
				new SequentialCommandGroup(
						new ArmCommand(m_armSubsystem, 38, 1.8),
						new ArmCommand(m_armSubsystem, 51.998, 1.99),
						new InstantCommand(m_grabberSubsystem::toggle, m_grabberSubsystem),
						new WaitCommand(0.5)));

		eventMap.put("RetractArm", new ArmCommand(m_armSubsystem, 34, ArmConstants.kElevatorMinPosition));

		eventMap.put("PickUp",
				new SequentialCommandGroup(
						new ArmCommand(m_armSubsystem, 85, 1.5),
						new InstantCommand(m_grabberSubsystem::toggle, m_grabberSubsystem),
						new WaitCommand(0.5)));

		eventMap.put("Balance", new BalanceCommand(m_driveSubsystem));
	}

	public CommandBase followPath(PathPlannerTrajectory trajectory) {
		SequentialCommandGroup driveCommand = new SequentialCommandGroup();
		Pose2d previousPose = trajectory.getInitialPose();
		double deltaX = 0;
		double deltaY = 0;
		double deltaTheta = 0;

		for (State state : trajectory.getStates()) {
			deltaX = state.poseMeters.getX() - previousPose.getX();
			deltaY = state.poseMeters.getY() - previousPose.getY();
			deltaTheta = state.poseMeters.getRotation().getRadians()
					- previousPose.getRotation().getRadians();

			driveCommand
					.addCommands(new ScheduleDrive(m_driveSubsystem, deltaX, deltaY, deltaTheta, state.timeSeconds));

			previousPose = state.poseMeters;
		}

		return driveCommand;
	}
}
