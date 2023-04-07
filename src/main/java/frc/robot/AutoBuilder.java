package frc.robot;

import java.util.HashMap;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.auto.BaseAutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutonConstants;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.BalanceCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class AutoBuilder extends BaseAutoBuilder {
	private final DriveSubsystem m_driveSubsystem;
	private final ArmSubsystem m_armSubsystem;
	private final GrabberSubsystem m_grabberSubsystem;

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
		PathPlannerTrajectory allianceTrajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(trajectory, DriverStation.getAlliance());

		return new CommandBase() {
			private final Timer m_timer = new Timer();
			private final PIDController m_xPID = new PIDController(-1, 0, 0);
			private final PIDController m_yPID = new PIDController(-1, 0, 0);
			private final PIDController m_thetaPID = new PIDController(Math.PI, 0, 0);

			@Override
			public void initialize() {
				m_xPID.reset();
				m_yPID.reset();
				m_thetaPID.reset();
				m_xPID.setTolerance(0.2);
				m_yPID.setTolerance(0.2);
				m_thetaPID.setTolerance(Math.PI/48);
				m_thetaPID.enableContinuousInput(-Math.PI, Math.PI);
				m_timer.restart();
			}

			@Override
			public void execute() {
				double now = m_timer.get();
				PathPlannerState current = (PathPlannerState)allianceTrajectory.sample(now);

				Pose2d actual = m_driveSubsystem.getPose();
				Pose2d target = current.poseMeters;
				
				double xDynamic = m_xPID.calculate(actual.getX(), target.getX());
				double xStatic = current.velocityMetersPerSecond * -target.getRotation().getCos();
				double vX = xDynamic + xStatic;

				double yDynamic = m_yPID.calculate(actual.getY(), target.getY());
				double yStatic = current.velocityMetersPerSecond * target.getRotation().getSin();
				double vY = yStatic + yDynamic;

				double thetaDynamic = m_thetaPID.calculate(actual.getRotation().getRadians(), -current.holonomicRotation.getRadians());
				double thetaStatic = -current.holonomicAngularVelocityRadPerSec;
				double vTheta = thetaDynamic + thetaStatic;

				MathUtil.clamp(vX, -AutonConstants.maxVelocity, AutonConstants.maxVelocity);
				MathUtil.clamp(vY, -AutonConstants.maxVelocity, AutonConstants.maxVelocity);
				MathUtil.clamp(vTheta, -AutonConstants.maxAngularVelocity, AutonConstants.maxAngularVelocity);

				SmartDashboard.putNumber("xDynamic", xDynamic);
				SmartDashboard.putNumber("yDynamic", yDynamic);
				SmartDashboard.putNumber("thetaDynamic", thetaDynamic);
				SmartDashboard.putNumber("xStatic", xStatic);
				SmartDashboard.putNumber("yStatic", yStatic);
				SmartDashboard.putNumber("thetaStatic", thetaStatic);
				
				m_driveSubsystem.drive(vX, vY, vTheta, true);
			}

			@Override
			public void end(boolean interrupted) {
				m_driveSubsystem.drive(0, 0, 0, false);
			}

			@Override
			public boolean isFinished() {
				return m_timer.hasElapsed(allianceTrajectory.getTotalTimeSeconds());
			}
		};
	}
}
