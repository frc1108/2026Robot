// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

@Logged
public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX m_leftShooter;
  private final TalonFX m_rightShooter;
  private final VelocityVoltage m_leftVelocityRequest = new VelocityVoltage(0.0);
  private final VelocityVoltage m_rightVelocityRequest = new VelocityVoltage(0.0);
  private HoodSubsystem m_hood;
  private final InterpolatingDoubleTreeMap m_distanceToRpmMap = new InterpolatingDoubleTreeMap();
  private final int m_tablePairCount;
  private double m_filteredDistanceMeters = Double.NaN;
  private double m_lastAutoCommandRpm = Double.NaN;
  private double m_lastAutoUpdateTimestampSec = Double.NaN;
  @Logged private double autoTargetShooterRpm = 0.0;

  public ShooterSubsystem() {
    m_leftShooter = new TalonFX(ShooterConstants.kLeftShooterCanId, "rio");
    m_rightShooter = new TalonFX(ShooterConstants.kRightShooterCanId, "rio");
    configureVelocityControl();

    m_tablePairCount = Math.min(
        ShooterConstants.kShooterDistanceMeters.length,
        ShooterConstants.kShooterDistanceRpm.length);
    for (int i = 0; i < m_tablePairCount; i++) {
      m_distanceToRpmMap.put(
          ShooterConstants.kShooterDistanceMeters[i],
          ShooterConstants.kShooterDistanceRpm[i]);
    }
  }

  public void setHoodSubsystem(HoodSubsystem hood) {
    m_hood = hood;
  }

  private void configureVelocityControl() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = ShooterConstants.kShooterVelocityP;
    config.Slot0.kI = ShooterConstants.kShooterVelocityI;
    config.Slot0.kD = ShooterConstants.kShooterVelocityD;
    config.Slot0.kV = ShooterConstants.kShooterVelocityV;
    config.Slot0.kS = ShooterConstants.kShooterVelocityS;
    m_leftShooter.getConfigurator().apply(config);
    m_rightShooter.getConfigurator().apply(config);
  }

  private void setShooterVelocityRpm(double rpm) {
    double clampedRpm = MathUtil.clamp(rpm, -ShooterConstants.kShooterMaxRpm, ShooterConstants.kShooterMaxRpm);
    double rps = clampedRpm / 60.0;
    // Right wheel spins opposite direction.
    m_leftShooter.setControl(m_leftVelocityRequest.withVelocity(rps));
    m_rightShooter.setControl(m_rightVelocityRequest.withVelocity(-rps));
    autoTargetShooterRpm = clampedRpm;
  }

  public void stopShooter() {
    setShooterVelocityRpm(0.0);
  }

  public double getLeftShooterRpm() {
    return m_leftShooter.getVelocity().getValueAsDouble() * 60.0;
  }

  public double getRightShooterRpm() {
    return -m_rightShooter.getVelocity().getValueAsDouble() * 60.0;
  }

  public double getAverageShooterRpm() {
    return (getLeftShooterRpm() + getRightShooterRpm()) / 2.0;
  }

  private Command runShooterAtRpm(double rpm) {
    return startEnd(() -> setShooterVelocityRpm(rpm), this::stopShooter);
  }

  public Command shootCommand() {
    return runShooterAtRpm(ShooterConstants.kShooterFullRpm);
  }

  public Command slowShootCommand() {
    return runShooterAtRpm(ShooterConstants.kShooterSlowRpm);
  }

  public Command shootWithHoodCommand(double hoodAngleDegrees) {
    if (m_hood == null) {
      return shootCommand();
    }

    return Commands.parallel(
        shootCommand(),
        m_hood.setHoodAngleCommand(hoodAngleDegrees)
    );
  }

  public Command slowShootWithHoodCommand(double hoodAngleDegrees) {
    if (m_hood == null) {
      return slowShootCommand();
    }

    return Commands.parallel(
        slowShootCommand(),
        m_hood.setHoodAngleCommand(hoodAngleDegrees)
    );
  }

  public double getAutoShooterRpmForDistance(double distanceMeters) {
    if (m_tablePairCount == 0) {
      return ShooterConstants.kShooterFullRpm;
    }
    return m_distanceToRpmMap.get(distanceMeters);
  }

  public Command autoShootFromDistanceCommand(DoubleSupplier distanceMetersSupplier) {
    return this.run(() -> {
      double nowSec = Timer.getFPGATimestamp();
      if (!Double.isNaN(m_lastAutoUpdateTimestampSec)
          && (nowSec - m_lastAutoUpdateTimestampSec) < ShooterConstants.kAutoShooterUpdatePeriodSeconds) {
        return;
      }

      double dtSec = Double.isNaN(m_lastAutoUpdateTimestampSec)
          ? ShooterConstants.kAutoShooterUpdatePeriodSeconds
          : (nowSec - m_lastAutoUpdateTimestampSec);
      m_lastAutoUpdateTimestampSec = nowSec;

      double rawDistance = distanceMetersSupplier.getAsDouble();
      if (Double.isNaN(m_filteredDistanceMeters)) {
        m_filteredDistanceMeters = rawDistance;
      } else {
        double delta = rawDistance - m_filteredDistanceMeters;
        if (Math.abs(delta) > ShooterConstants.kAutoShooterDistanceDeadbandMeters) {
          m_filteredDistanceMeters += ShooterConstants.kAutoShooterDistanceFilterAlpha * delta;
        }
      }

      double desiredRpm = getAutoShooterRpmForDistance(m_filteredDistanceMeters);
      if (Double.isNaN(m_lastAutoCommandRpm)) {
        m_lastAutoCommandRpm = getAverageShooterRpm();
      }

      double maxStep = ShooterConstants.kAutoShooterRpmSlewRatePerSec * dtSec;
      double limitedRpm = m_lastAutoCommandRpm
          + MathUtil.clamp(desiredRpm - m_lastAutoCommandRpm, -maxStep, maxStep);

      if (Math.abs(limitedRpm - m_lastAutoCommandRpm) >= ShooterConstants.kAutoShooterMinCommandStepRpm) {
        setShooterVelocityRpm(limitedRpm);
        m_lastAutoCommandRpm = limitedRpm;
      }
    }).beforeStarting(() -> {
      m_filteredDistanceMeters = Double.NaN;
      m_lastAutoCommandRpm = Double.NaN;
      m_lastAutoUpdateTimestampSec = Double.NaN;
    }).finallyDo(() -> stopShooter());
  }
}
