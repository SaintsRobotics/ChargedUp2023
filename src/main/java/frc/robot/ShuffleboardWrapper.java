package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class ShuffleboardWrapper {
  private static ShuffleboardWrapper m_default = new ShuffleboardWrapper("Default");
  
  /**
   * Gets the default instance
   * @return The default instance
   */
  public static ShuffleboardWrapper getDefault() {return m_default;}

  private final String m_name;

  /**
   * Creates a new Smartdashboard wrapper
   * @param name Tab name
   */
  public ShuffleboardWrapper(String name){
    m_name = name;
    Shuffleboard.getTab(m_name);
    Shuffleboard.getTab(m_name).getLayout("Match Info", BuiltInLayouts.kList).withPosition(0, 0).withSize(1, 3);
    Shuffleboard.getTab(m_name).getLayout("Match Info").addNumber("Time", () -> MathUtil.clamp(DriverStation.getMatchTime(), 0, 255));
    Shuffleboard.getTab(m_name).getLayout("Match Info").addBoolean("Enabled", () -> DriverStation.isEnabled());
    Shuffleboard.getTab(m_name).getLayout("Match Info").addBoolean("Driver Controller", () -> DriverStation.isJoystickConnected(0));
    Shuffleboard.getTab(m_name).getLayout("Match Info").addBoolean("Operator Controller", () -> DriverStation.isJoystickConnected(1));
  }
}
