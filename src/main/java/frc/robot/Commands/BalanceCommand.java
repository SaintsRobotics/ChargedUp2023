package frc.robot.Commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;

public class BalanceCommand {
    
    private final AHRS m_balanceGyro = new AHRS();
    private final DriveSubsystem m_balanceDrive = new DriveSubsystem();
    private double m_balanceGyroAngle;
    private final double m_opt = 0;
    private double m_balanceDeadBand = 0.2;


    private final PIDController m_balancePID = new PIDController(0.1, 0, 0);

    public void periodic() {

    m_balanceGyroAngle = m_balanceGyro.getPitch();
    if (m_balanceGyroAngle < MathUtil.applyDeadband(m_opt, m_balanceDeadBand)) {

        m_balanceDrive.drive(m_balancePID.calculate(m_balanceGyroAngle), 0, 0, false);
    }

    if (m_balanceGyroAngle > MathUtil.applyDeadband(m_opt, m_balanceDeadBand)) {
        m_balanceDrive.drive(m_balancePID.calculate(m_balanceGyroAngle), 0, 0, false);

    }
}
}
