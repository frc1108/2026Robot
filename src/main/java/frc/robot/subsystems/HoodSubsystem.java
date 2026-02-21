package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

@Logged
public class HoodSubsystem extends SubsystemBase {
  private final SparkMax m_hoodMotor;
  private final RelativeEncoder m_hoodEncoder;
  private final SparkClosedLoopController m_hoodPID;
  private final InterpolatingDoubleTreeMap m_distanceToAngleMap = new InterpolatingDoubleTreeMap();
  private final int m_tablePairCount;
  private double m_filteredDistanceMeters = Double.NaN;
  private double m_lastAutoCommandAngleDegrees = Double.NaN;
  private double m_lastAutoUpdateTimestampSec = Double.NaN;
  @Logged private double autoTargetAngleDegrees = 0.0;

  public HoodSubsystem() {
    m_hoodMotor = new SparkMax(ShooterConstants.kHoodMotorCanId, MotorType.kBrushless);
    m_hoodMotor.configure(
        frc.robot.Configs.Shooter.hoodConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_hoodEncoder = m_hoodMotor.getEncoder();
    m_hoodPID = m_hoodMotor.getClosedLoopController();

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

  public void zeroInternalEncoder(double knownAngleDegrees) {
    m_hoodEncoder.setPosition(knownAngleDegrees);
    autoTargetAngleDegrees = knownAngleDegrees;
  }

  @Override
  public void periodic() {
  }

  public void setHoodAngle(double angleDegrees) {
    double clampedAngle = MathUtil.clamp(
        angleDegrees,
        ShooterConstants.kMinHoodAngleDegrees,
        ShooterConstants.kMaxHoodAngleDegrees);
    if (Math.abs(clampedAngle - autoTargetAngleDegrees) < ShooterConstants.kHoodCommandToleranceDegrees) {
      return;
    }
    m_hoodPID.setSetpoint(clampedAngle, ControlType.kPosition);
    autoTargetAngleDegrees = clampedAngle;
  }

  public double getHoodAngle() {
    return m_hoodEncoder.getPosition();
  }

  public boolean isAtTarget(double targetAngle, double toleranceDegrees) {
    return Math.abs(getHoodAngle() - targetAngle) <= toleranceDegrees;
  }

  public void stop() {
    m_hoodMotor.set(0.0);
  }

  public Command setHoodAngleCommand(double angleDegrees) {
    return this.startEnd(() -> setHoodAngle(angleDegrees), this::stop);
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
