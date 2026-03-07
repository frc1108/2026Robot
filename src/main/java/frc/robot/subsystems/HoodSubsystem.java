package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

@Logged
public class HoodSubsystem extends SubsystemBase {
  private final TalonFXS m_hoodMotor;
  private final PositionVoltage m_hoodPositionRequest = new PositionVoltage(0.0);
  private final DutyCycleOut m_hoodDutyCycleRequest = new DutyCycleOut(0.0);
  private final VelocityVoltage m_hoodVelocityRequest = new VelocityVoltage(0.0).withSlot(1);
  private final InterpolatingDoubleTreeMap m_distanceToAngleMap = new InterpolatingDoubleTreeMap();
  private final int m_tablePairCount;
  private double m_filteredDistanceMeters = Double.NaN;
  private double m_lastAutoCommandAngleDegrees = Double.NaN;
  private double m_lastAutoUpdateTimestampSec = Double.NaN;
  private double m_autoZeroStartSec = Double.NaN;
  private double m_autoZeroOverCurrentStartSec = Double.NaN;
  private double m_lastAutoZeroCurrentPrintSec = Double.NaN;
  private boolean m_autoZeroComplete = false;
  private double m_lastCommandedPercent = 0.0;
  @Logged private double autoTargetAngleDegrees = 0.0;

  public HoodSubsystem() {
    m_hoodMotor = new TalonFXS(ShooterConstants.kHoodMotorCanId);
    configureHoodMotor();

    if (ShooterConstants.kAutoZeroHoodOnStartup) {
      zeroInternalEncoder(ShooterConstants.kHoodStartupZeroDegrees);
    }

    m_tablePairCount = Math.min(
        ShooterConstants.kHoodDistanceMeters.length,
        ShooterConstants.kHoodAngleDegrees.length);
    for (int i = 0; i < m_tablePairCount; i++) {
      m_distanceToAngleMap.put(ShooterConstants.kHoodDistanceMeters[i], ShooterConstants.kHoodAngleDegrees[i]);
    }
  }

  private void configureHoodMotor() {
    TalonFXSConfiguration config = new TalonFXSConfiguration();
    config.Slot0.kP = ShooterConstants.kHoodP;
    config.Slot0.kI = ShooterConstants.kHoodI;
    config.Slot0.kD = ShooterConstants.kHoodD;
    config.Slot1.kP = ShooterConstants.kHoodAutoZeroVelocityP;
    config.Slot1.kI = ShooterConstants.kHoodAutoZeroVelocityI;
    config.Slot1.kD = ShooterConstants.kHoodAutoZeroVelocityD;
    config.Slot1.kV = ShooterConstants.kHoodAutoZeroVelocityV;
    config.CurrentLimits.SupplyCurrentLimit = ShooterConstants.kHoodRunSupplyCurrentLimitAmps;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = ShooterConstants.kHoodRunStatorCurrentLimitAmps;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    // Match previous REV output range cap.
    config.Voltage.PeakForwardVoltage = 12.0 * ShooterConstants.kHoodMaxClosedLoopOutput;
    config.Voltage.PeakReverseVoltage = -12.0 * ShooterConstants.kHoodMaxClosedLoopOutput;
    m_hoodMotor.getConfigurator().apply(config);
    m_hoodMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  private void applyCurrentLimits(double supplyLimitAmps, double statorLimitAmps) {
    CurrentLimitsConfigs limits = new CurrentLimitsConfigs();
    limits.SupplyCurrentLimit = supplyLimitAmps;
    limits.SupplyCurrentLimitEnable = true;
    limits.StatorCurrentLimit = statorLimitAmps;
    limits.StatorCurrentLimitEnable = true;
    m_hoodMotor.getConfigurator().apply(limits);
  }

  private static double hoodDegreesToMotorRotations(double hoodDegrees) {
    return (hoodDegrees / 360.0) * ShooterConstants.kHoodGearRatio;
  }

  private static double motorRotationsToHoodDegrees(double motorRotations) {
    return (motorRotations / ShooterConstants.kHoodGearRatio) * 360.0;
  }

  public void zeroInternalEncoder(double knownAngleDegrees) {
    m_hoodMotor.setPosition(hoodDegreesToMotorRotations(knownAngleDegrees));
    autoTargetAngleDegrees = knownAngleDegrees;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hood/SupplyCurrentAmps", getHoodSupplyCurrentAmps());
    SmartDashboard.putNumber("Hood/StatorCurrentAmps", getHoodStatorCurrentAmps());
    SmartDashboard.putNumber("Hood/AngleDeg", getHoodAngle());
    SmartDashboard.putNumber("Hood/CommandedPercent", m_lastCommandedPercent);
    SmartDashboard.putBoolean("Hood/Connected", m_hoodMotor.isConnected());
  }

  public void setHoodAngle(double angleDegrees) {
    double clampedAngle = MathUtil.clamp(
        angleDegrees,
        ShooterConstants.kMinHoodAngleDegrees,
        ShooterConstants.kMaxHoodAngleDegrees);
    if (Math.abs(clampedAngle - autoTargetAngleDegrees) < ShooterConstants.kHoodCommandToleranceDegrees) {
      return;
    }
    m_hoodMotor.setControl(m_hoodPositionRequest.withPosition(hoodDegreesToMotorRotations(clampedAngle)));
    autoTargetAngleDegrees = clampedAngle;
    m_lastCommandedPercent = 0.0;
  }

  public double getHoodAngle() {
    return motorRotationsToHoodDegrees(m_hoodMotor.getPosition().getValueAsDouble());
  }

  public boolean isAtTarget(double targetAngle, double toleranceDegrees) {
    return Math.abs(getHoodAngle() - targetAngle) <= toleranceDegrees;
  }

  public void stop() {
    m_hoodMotor.stopMotor();
    m_lastCommandedPercent = 0.0;
  }

  public double getHoodSupplyCurrentAmps() {
    return m_hoodMotor.getSupplyCurrent().getValueAsDouble();
  }

  public double getHoodStatorCurrentAmps() {
    return m_hoodMotor.getStatorCurrent().getValueAsDouble();
  }

  public void setManualPercent(double percent) {
    double clamped = MathUtil.clamp(percent, -1.0, 1.0);
    m_hoodMotor.setControl(m_hoodDutyCycleRequest.withOutput(clamped));
    m_lastCommandedPercent = clamped;
  }

  private void setAutoZeroVelocityRps(double velocityRps) {
    m_hoodMotor.setControl(m_hoodVelocityRequest.withVelocity(velocityRps));
  }

  public Command setHoodAngleCommand(double angleDegrees) {
    return this.startEnd(() -> setHoodAngle(angleDegrees), this::stop);
  }

  public Command hoodUpCommand() {
    return this.startEnd(
        () -> {
          DriverStation.reportWarning("Hood up command active", false);
          setManualPercent(-Math.abs(ShooterConstants.kHoodManualPercentOutput));
        },
        this::stop);
  }

  public Command hoodDownCommand() {
    return this.startEnd(
        () -> {
          DriverStation.reportWarning("Hood down command active", false);
          setManualPercent(Math.abs(ShooterConstants.kHoodManualPercentOutput));
        },
        this::stop);
  }

  public Command autoZeroByCurrentCommand() {
    return new FunctionalCommand(
        () -> {
          m_autoZeroStartSec = Timer.getFPGATimestamp();
          m_autoZeroOverCurrentStartSec = Double.NaN;
          m_lastAutoZeroCurrentPrintSec = Double.NaN;
          m_autoZeroComplete = false;
          applyCurrentLimits(
              ShooterConstants.kHoodHomingSupplyCurrentLimitAmps,
              ShooterConstants.kHoodHomingStatorCurrentLimitAmps);
        },
        () -> {
          double nowSec = Timer.getFPGATimestamp();
          double homingDirection = Math.signum(ShooterConstants.kHoodAutoZeroPercentOutput);
          double homingVelocityRps =
              homingDirection * Math.abs(ShooterConstants.kHoodAutoZeroVelocityRps);
          setAutoZeroVelocityRps(homingVelocityRps);
          m_lastCommandedPercent = ShooterConstants.kHoodAutoZeroPercentOutput;

          double statorCurrentAmps = getHoodStatorCurrentAmps();
          SmartDashboard.putNumber("Hood/AutoZeroStatorCurrentAmps", statorCurrentAmps);

          if (Double.isNaN(m_lastAutoZeroCurrentPrintSec)
              || (nowSec - m_lastAutoZeroCurrentPrintSec) >= 0.20) {
            System.out.println(String.format("Hood auto-zero current: %.2f A", statorCurrentAmps));
            m_lastAutoZeroCurrentPrintSec = nowSec;
          }

          if (statorCurrentAmps >= ShooterConstants.kHoodAutoZeroCurrentThresholdAmps) {
            if (Double.isNaN(m_autoZeroOverCurrentStartSec)) {
              m_autoZeroOverCurrentStartSec = nowSec;
            }
          } else {
            m_autoZeroOverCurrentStartSec = Double.NaN;
          }

          boolean minRunSatisfied =
              (nowSec - m_autoZeroStartSec) >= ShooterConstants.kHoodAutoZeroMinRunSeconds;
          boolean overCurrentDebounced =
              !Double.isNaN(m_autoZeroOverCurrentStartSec)
                  && (nowSec - m_autoZeroOverCurrentStartSec) >= ShooterConstants.kHoodAutoZeroCurrentDebounceSeconds;
          m_autoZeroComplete = minRunSatisfied && overCurrentDebounced;
        },
        interrupted -> {
          stop();
          applyCurrentLimits(
              ShooterConstants.kHoodRunSupplyCurrentLimitAmps,
              ShooterConstants.kHoodRunStatorCurrentLimitAmps);
          if (!interrupted && m_autoZeroComplete) {
            zeroInternalEncoder(ShooterConstants.kHoodStartupZeroDegrees);
            DriverStation.reportWarning("Hood auto-zero complete", false);
          } else if (interrupted) {
            DriverStation.reportWarning("Hood auto-zero interrupted", false);
          }
        },
        () -> m_autoZeroComplete,
        this);
  }

  public double getAutoHoodAngleForDistance(double distanceMeters) {
    if (m_tablePairCount == 0) {
      return ShooterConstants.kMinHoodAngleDegrees;
    }
    return m_distanceToAngleMap.get(distanceMeters);
  }

  public Command autoHoodFromDistanceCommand(DoubleSupplier distanceMetersSupplier) {
    return this.run(() -> {
      double nowSec = Timer.getFPGATimestamp();
      if (!Double.isNaN(m_lastAutoUpdateTimestampSec)
          && (nowSec - m_lastAutoUpdateTimestampSec) < ShooterConstants.kAutoHoodUpdatePeriodSeconds) {
        return;
      }

      double dtSec = Double.isNaN(m_lastAutoUpdateTimestampSec)
          ? ShooterConstants.kAutoHoodUpdatePeriodSeconds
          : (nowSec - m_lastAutoUpdateTimestampSec);
      m_lastAutoUpdateTimestampSec = nowSec;

      double rawDistance = distanceMetersSupplier.getAsDouble();

      if (Double.isNaN(m_filteredDistanceMeters)) {
        m_filteredDistanceMeters = rawDistance;
      } else {
        double delta = rawDistance - m_filteredDistanceMeters;
        if (Math.abs(delta) > ShooterConstants.kAutoHoodDistanceDeadbandMeters) {
          m_filteredDistanceMeters += ShooterConstants.kAutoHoodDistanceFilterAlpha * delta;
        }
      }

      double desiredAngle = getAutoHoodAngleForDistance(m_filteredDistanceMeters);
      if (Double.isNaN(m_lastAutoCommandAngleDegrees)) {
        m_lastAutoCommandAngleDegrees = getHoodAngle();
      }

      double maxStep = ShooterConstants.kAutoHoodAngleSlewRateDegPerSec * dtSec;
      double limitedAngle = m_lastAutoCommandAngleDegrees
          + MathUtil.clamp(desiredAngle - m_lastAutoCommandAngleDegrees, -maxStep, maxStep);

      if (Math.abs(limitedAngle - m_lastAutoCommandAngleDegrees) >= ShooterConstants.kAutoHoodMinCommandStepDeg) {
        setHoodAngle(limitedAngle);
        m_lastAutoCommandAngleDegrees = limitedAngle;
      }
    }).beforeStarting(() -> {
      m_filteredDistanceMeters = Double.NaN;
      m_lastAutoCommandAngleDegrees = Double.NaN;
      m_lastAutoUpdateTimestampSec = Double.NaN;
    });
  }
}
