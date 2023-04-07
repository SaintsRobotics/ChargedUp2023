// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.LED.LEDBlinkCommand;
import frc.robot.commands.LED.LEDBlinkCommand.BlinkType;
import frc.robot.commands.LED.LEDCountdownCommand;
import frc.robot.commands.LED.LEDDefaultCommand;
import frc.robot.commands.LED.LEDRainbowCommand;
import frc.robot.commands.LED.LEDSwipeCommand;
import frc.robot.commands.LED.LEDSwipeCommand.SwipeType;
import frc.robot.commands.LED.LEDTipCommand;
import frc.robot.commands.SnapRotateCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.LEDSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    public final GrabberSubsystem grabberSubsystem = new GrabberSubsystem();
    private final LEDSubsystem m_LEDSubsystem = new LEDSubsystem();

    private final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
    private final XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    private boolean m_lockLED = false;

    private final HashMap<String, Command> m_eventMap = new HashMap<>();
    private final SwerveAutoBuilder m_autoBuilder = new SwerveAutoBuilder(
            m_robotDrive::getPose,
            m_robotDrive::resetOdometry,
            DriveConstants.kDriveKinematics,
            new PIDConstants(0, 0, 0),
            new PIDConstants(0, 0, 0),
            m_robotDrive::setModuleStates,
            m_eventMap,
            true,
            m_robotDrive);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        configureButtonBindings();

        m_armSubsystem.setDefaultCommand(
                new RunCommand(() -> m_armSubsystem.set(
                        MathUtil.applyDeadband(
                                -m_operatorController.getLeftY(),
                                OIConstants.kControllerDeadband),
                        MathUtil.applyDeadband(
                                -m_operatorController.getRightY(),
                                OIConstants.kControllerDeadband)),
                        m_armSubsystem));

        m_robotDrive.setDefaultCommand(

                new RunCommand(
                        () -> m_robotDrive.drive(
                                MathUtil.applyDeadband(
                                        -m_driverController.getLeftY(),
                                        OIConstants.kControllerDeadband)
                                        * DriveConstants.kMaxSpeedMetersPerSecond
                                        * (1 - m_driverController
                                                .getLeftTriggerAxis()
                                                * OIConstants.kSlowModeScalar)
                                        * 0.8,
                                MathUtil.applyDeadband(
                                        -m_driverController.getLeftX(),
                                        OIConstants.kControllerDeadband)
                                        * DriveConstants.kMaxSpeedMetersPerSecond
                                        * (1 - m_driverController
                                                .getLeftTriggerAxis()
                                                * OIConstants.kSlowModeScalar)
                                        * 0.8,
                                MathUtil.applyDeadband(
                                        -m_driverController.getRightX(),
                                        OIConstants.kControllerDeadband)
                                        * DriveConstants.kMaxAngularSpeedRadiansPerSecond
                                        * (1 - m_driverController
                                                .getLeftTriggerAxis()
                                                * OIConstants.kSlowModeScalar)
                                        / 2,
                                !m_driverController.getRightBumper()),
                        m_robotDrive));

        m_LEDSubsystem.setDefaultCommand(new LEDDefaultCommand(m_LEDSubsystem, () -> m_lockLED));

        m_chooser.addOption("Far", "Far");
        m_chooser.addOption("Charger", "Charger");
        m_chooser.addOption("Charger-comms", "Charger-comms");
        m_chooser.addOption("Near", "Near");
        m_chooser.addOption("FarTwoObject", "FarTwoObject");
        SmartDashboard.putData(m_chooser);

        m_eventMap.put("DropHigh",
                new SequentialCommandGroup(
                        new ArmCommand(m_armSubsystem, 38, 1.8),
                        new ArmCommand(m_armSubsystem, 51.998, 1.99),
                        new InstantCommand(grabberSubsystem::toggle, grabberSubsystem),
                        new WaitCommand(0.5)));

        m_eventMap.put("RetractArm", new ArmCommand(m_armSubsystem, 34, ArmConstants.kElevatorMinPosition));

        m_eventMap.put("PickUp",
                new SequentialCommandGroup(
                        new ArmCommand(m_armSubsystem, 85, 1.5),
                        new InstantCommand(grabberSubsystem::toggle, grabberSubsystem),
                        new WaitCommand(0.5)));

        m_eventMap.put("Balance", new BalanceCommand(m_robotDrive, m_LEDSubsystem));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of
     * its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or
     * {@link XboxController}), and then calling passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {

        // Driver Bindings
        new JoystickButton(m_driverController, Button.kStart.value)
                .onTrue(new InstantCommand(m_robotDrive::zeroHeading, m_robotDrive));

        new JoystickButton(m_driverController, Button.kY.value)
                .whileTrue(new BalanceCommand(m_robotDrive, m_LEDSubsystem));
        new JoystickButton(m_driverController, Button.kA.value)
                .onTrue(new SnapRotateCommand(m_robotDrive));

        // Operator Bindings
        new JoystickButton(m_operatorController, Button.kA.value)
                .onTrue(new InstantCommand(grabberSubsystem::toggle, grabberSubsystem));

        new POVButton(m_operatorController, 0).onTrue(new SequentialCommandGroup(
                new ArmCommand(m_armSubsystem, 38, 1.8),
                new ArmCommand(m_armSubsystem, 51.998, 1.99)));
        new POVButton(m_operatorController, 90).onTrue(new ArmCommand(m_armSubsystem, 43, 1.42));
        new POVButton(m_operatorController, 180).onTrue(new ArmCommand(m_armSubsystem, 49, 1.58));
        new POVButton(m_operatorController, 270)
                .onTrue(new ArmCommand(m_armSubsystem, 34, ArmConstants.kElevatorMinPosition));

        new JoystickButton(m_operatorController, Button.kLeftBumper.value)
                .whileTrue(new RunCommand(() -> m_LEDSubsystem.setCone(), m_LEDSubsystem))
                .onFalse(new SequentialCommandGroup(new InstantCommand(() -> {
                    m_lockLED = true;
                }, m_LEDSubsystem), new WaitCommand(10), new InstantCommand(() -> {
                    m_lockLED = false;
                }, m_LEDSubsystem)));
        new JoystickButton(m_operatorController, Button.kRightBumper.value)
                .whileTrue(new RunCommand(() -> m_LEDSubsystem.setCube(), m_LEDSubsystem))
                .onFalse(new SequentialCommandGroup(new InstantCommand(() -> {
                    m_lockLED = true;
                }, m_LEDSubsystem), new WaitCommand(10), new InstantCommand(() -> {
                    m_lockLED = false;
                }, m_LEDSubsystem)));

        new Trigger(() -> m_robotDrive.isTipped() && !m_lockLED)
                .whileTrue(new LEDTipCommand(m_LEDSubsystem, m_robotDrive.getGyro()));
        new Trigger(() -> DriverStation.getMatchTime() < 10 && DriverStation.isTeleop() && !m_lockLED && Robot.isReal()) // TODO:
                                                                                                                         // test
                                                                                                                         // this
                                                                                                                         // on
                                                                                                                         // real
                                                                                                                         // robot
                .whileTrue(new LEDCountdownCommand(m_LEDSubsystem));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        if (m_chooser.getSelected() == null) {
            return null;
        }

        String path = m_chooser.getSelected();

        return m_autoBuilder.fullAuto(
                PathPlanner.loadPathGroup(
                        path,
                        new PathConstraints(
                                AutonConstants.maxVelocity - (path.equals("Charger-comms") ? 0.75 : 0),
                                AutonConstants.maxAcceleration)));
    }

    /**
     * Returns a command that does LED startup effects.
     * 
     * @return Startup command.
     */
    public Command getStartupCommand() {
        if (m_lockLED)
            return new SequentialCommandGroup();
        return new SequentialCommandGroup(
                new LEDSwipeCommand(m_LEDSubsystem, SwipeType.kUp, 0, 0, 100, true),
                new LEDBlinkCommand(m_LEDSubsystem, BlinkType.kBlink, 0, 0, 0),
                new WaitCommand(5),
                new LEDRainbowCommand(m_LEDSubsystem));
    }

    /**
     * Returns RGB idle command.
     * 
     * @return RGB idle command.
     */
    public Command getIdleCommand() {
        return new LEDRainbowCommand(m_LEDSubsystem);
    }

    public void unlockLED() {
        m_lockLED = false;
    }

    public void setPivotMode(IdleMode mode) {
        m_armSubsystem.setPivotMode(mode);
    }

    public void setLEDCriticalRed(boolean disable) {
        if (disable) {
            m_lockLED = false;
            m_LEDSubsystem.setLED(0, 100, 0);
        } else {
            m_lockLED = true;
            m_LEDSubsystem.setLED(100, 0, 0);
        }
    }
}
