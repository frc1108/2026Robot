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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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
  @Logged private double m_rawDistanceMeters = Double.NaN;
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

    Shuffleboard.getTab("Shooter").addNumber("Shooter RPM", this::getAverageShooterRpm);
    Shuffleboard.getTab("Shooter").addNumber("Shooter Distance Raw (m)", this::getRawDistanceMeters);
    Shuffleboard.getTab("Shooter").addNumber("Shooter Distance Filtered (m)", this::getFilteredDistanceMeters);
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

  public void setShooterTargetRpm(double rpm) {
    setShooterVelocityRpm(rpm);
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
  
  public double getRawDistanceMeters() {
    return m_rawDistanceMeters;
  }

  public double getFilteredDistanceMeters() {
    return m_filteredDistanceMeters;
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
    return interpolateShooterRpm(distanceMeters, ShooterConstants.kShooterDistanceMeters, ShooterConstants.kShooterDistanceRpm);
  }

  private double interpolateShooterRpm(double distanceMeters, double[] distances, double[] rpms) {
    int count = Math.min(distances.length, rpms.length);
    final double epsilon = 1e-9;

    if (count <= 0) {
      return ShooterConstants.kShooterFullRpm;
    }

    if (count == 1) {
      return rpms[0];
    }

    for (int i = 0; i < count; i++) {
      if (Math.abs(distanceMeters - distances[i]) < epsilon) {
        return rpms[i];
      }
    }

    if (distanceMeters <= distances[0]) {
      return rpms[0];
    }
    if (distanceMeters >= distances[count - 1]) {
      return rpms[count - 1];
    }

    int upper = 1;
    while (upper < count && distanceMeters > distances[upper]) {
      upper++;
    }
    int lower = Math.max(0, upper - 1);

    if (count < 3) {
      double t = (distanceMeters - distances[lower]) / (distances[upper] - distances[lower]);
      return rpms[lower] + t * (rpms[upper] - rpms[lower]);
    }

    int i0;
    int i1;
    int i2;
    if (lower == 0) {
      i0 = 0;
      i1 = 1;
      i2 = 2;
    } else if (upper == count - 1) {
      i0 = count - 3;
      i1 = count - 2;
      i2 = count - 1;
    } else {
      i0 = lower - 1;
      i1 = lower;
      i2 = upper;
    }

    double x0 = distances[i0];
    double x1 = distances[i1];
    double x2 = distances[i2];
    double y0 = rpms[i0];
    double y1 = rpms[i1];
    double y2 = rpms[i2];

    double x = distanceMeters;
    double l0 = ((x - x1) * (x - x2)) / ((x0 - x1) * (x0 - x2));
    double l1 = ((x - x0) * (x - x2)) / ((x1 - x0) * (x1 - x2));
    double l2 = ((x - x0) * (x - x1)) / ((x2 - x0) * (x2 - x1));
    return (y0 * l0) + (y1 * l1) + (y2 * l2);
  }

  public Command autoShootFromDistanceCommand(DoubleSupplier distanceMetersSupplier) {
    return autoShootFromDistanceCommand(
        distanceMetersSupplier,
        ShooterConstants.kShooterDistanceMeters,
        ShooterConstants.kShooterDistanceRpm);
  }

  public Command autoShootFromDistanceCommand(
      DoubleSupplier distanceMetersSupplier,
      double[] distanceTableMeters,
      double[] rpmTable) {
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
      m_rawDistanceMeters = rawDistance;
      // Keep the filter updated for telemetry, but use raw distance for exact interpolation.
      if (Double.isNaN(m_filteredDistanceMeters)) {
        m_filteredDistanceMeters = rawDistance;
      } else {
        double delta = rawDistance - m_filteredDistanceMeters;
        if (Math.abs(delta) > ShooterConstants.kAutoShooterDistanceDeadbandMeters) {
          m_filteredDistanceMeters += ShooterConstants.kAutoShooterDistanceFilterAlpha * delta;
        }
      }

      double desiredRpm = interpolateShooterRpm(rawDistance, distanceTableMeters, rpmTable);
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
