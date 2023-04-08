package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DuckConstants;

public class RubberDuck extends SubsystemBase {
    CANSparkMax m_duckMotor = new CANSparkMax(DuckConstants.kMotorPort, MotorType.kBrushless);
    Timer m_timer = new Timer();

    @Override
    public void periodic() {
        if (m_timer.hasElapsed(20)) { // Change direction every 20 seconds
            m_duckMotor.set(-m_duckMotor.get());
            m_timer.restart();
        }
    }

    public void init() {
        m_duckMotor.set(DuckConstants.kDuckSpeed);
        m_timer.restart();
    }

    public void end() {
        m_duckMotor.set(0);
        m_timer.reset();
        m_timer.stop();
    }
}
