package frc.robot.util;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Calculates acceleration for ChassisSpeeds using given acceleration constants.
 * This object maintains state and should be reused.
 */
public class AcceleratedChassisSpeeds {
  private SlewRateLimiter m_xTranslationRateLimiter;
  private SlewRateLimiter m_yTranslationRateLimiter;
  private SlewRateLimiter m_rotationRateLimiter;
  private ChassisSpeeds m_currentChassisSpeeds;

  /**
   * Create an accelerated ChassisSpeeds
   * @param translationMaxAcceleration The max acceleration for translation, in m/s^2
   * @param rotationMaxAcceleration The max acceleration for rotation, in radians/s^2
   */
  public AcceleratedChassisSpeeds(double translationMaxAcceleration, double rotationMaxAcceleration) {
    m_xTranslationRateLimiter = new SlewRateLimiter(translationMaxAcceleration);
    m_yTranslationRateLimiter = new SlewRateLimiter(translationMaxAcceleration);
    m_rotationRateLimiter = new SlewRateLimiter(rotationMaxAcceleration);
    m_currentChassisSpeeds = new ChassisSpeeds();
  }

  /**
   * Retrieves the current ChassisSpeeds
   * @return The current speeds
   */
  public ChassisSpeeds getCurrentSpeeds() {
    return m_currentChassisSpeeds;
  }

  /**
   * Calculates and saves the new ChassisSpeeds, given the acceleration
   * constants and the provided desired speeds.
   * @param desiredSpeeds The desired speeds
   * @return The new calculated speeds
   */
  public ChassisSpeeds calculateNewSpeeds(ChassisSpeeds desiredSpeeds) {
    m_currentChassisSpeeds = new ChassisSpeeds(
        m_xTranslationRateLimiter.calculate(desiredSpeeds.vxMetersPerSecond),
        m_yTranslationRateLimiter.calculate(desiredSpeeds.vyMetersPerSecond),
        m_rotationRateLimiter.calculate(desiredSpeeds.omegaRadiansPerSecond));
    return m_currentChassisSpeeds;
  }

  /**
   * Resets the speeds to 0
   */
  public void reset() {
    m_xTranslationRateLimiter.reset(0);
    m_yTranslationRateLimiter.reset(0);
    m_rotationRateLimiter.reset(0);
    m_currentChassisSpeeds = new ChassisSpeeds();
  }
}
