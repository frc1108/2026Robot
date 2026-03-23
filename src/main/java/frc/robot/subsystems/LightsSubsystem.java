package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LightsConstants;

public class LightsSubsystem extends SubsystemBase {
  private final Spark m_blinkin = new Spark(LightsConstants.kBlinkinPwmPort);
  private int m_modeIndex = 0;

  public LightsSubsystem() {
    applyCurrentMode();
  }

  public void cycleMode() {
    m_modeIndex = (m_modeIndex + 1) % LightsConstants.kBlinkinModeValues.length;
    applyCurrentMode();
  }

  private void applyCurrentMode() {
    m_blinkin.set(LightsConstants.kBlinkinModeValues[m_modeIndex]);
  }
}
