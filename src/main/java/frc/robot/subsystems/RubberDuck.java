package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DuckConstants;

public class RubberDuck extends SubsystemBase {
    CANSparkMax m_duckMotor = new CANSparkMax(DuckConstants.kMotorPort, MotorType.kBrushless);

    public RubberDuck() {

    }

    @Override
    public void periodic() {
        m_duckMotor.set(DuckConstants.kDuckSpeed);
    }
}
