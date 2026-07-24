package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LightsConstants;

@Logged
public class LightsSubsystem extends SubsystemBase {
  private final Spark m_blinkin = new Spark(LightsConstants.kBlinkinPwmPort);
  private int m_modeIndex = 0;
  private DriverStation.Alliance m_lastAlliance = null;

  public LightsSubsystem() {
    applyCurrentMode();
  }

  @Override
  public void periodic() {
    DriverStation.getAlliance().ifPresent(alliance -> {
      if (alliance != m_lastAlliance) {
        m_lastAlliance = alliance;
        if (m_modeIndex == 0) {
          applyCurrentMode();
        }
      }
    });
  }

  public void cycleMode() {
    m_modeIndex = (m_modeIndex + 1) % LightsConstants.kBlinkinModeValues.length;
    applyCurrentMode();
  }

  private void applyCurrentMode() {
    if (m_modeIndex == 0) {
      m_blinkin.set(getAllianceModeValue());
      return;
    }
    m_blinkin.set(LightsConstants.kBlinkinModeValues[m_modeIndex]);
  }

  private double getAllianceModeValue() {
    return DriverStation.getAlliance()
        .map(alliance -> alliance == DriverStation.Alliance.Red
            ? LightsConstants.kSolidRed
            : LightsConstants.kSolidBlue)
        .orElse(LightsConstants.kSolidBlue);
  }
}
