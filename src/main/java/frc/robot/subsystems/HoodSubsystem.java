package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import java.util.function.DoubleSupplier;

@Logged
public class HoodSubsystem extends SubsystemBase {
  private final SparkMax m_hoodMotor;
  private final RelativeEncoder m_hoodEncoder;
  private final SparkClosedLoopController m_hoodPID;
  private final InterpolatingDoubleTreeMap m_distanceToAngleMap = new InterpolatingDoubleTreeMap();
  private final int m_tablePairCount;
  @Logged private double autoTargetAngleDegrees = 0.0;

  /** Creates a new HoodSubsystem. */
  public HoodSubsystem() {
    m_hoodMotor = new SparkMax(ShooterConstants.kHoodMotorCanId, MotorType.kBrushless);

  // Apply hood SparkMax configuration from central `Configs` and configure device
  m_hoodMotor.configure(frc.robot.Configs.Shooter.hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_hoodEncoder = m_hoodMotor.getEncoder();
    m_hoodPID = m_hoodMotor.getClosedLoopController();

    m_tablePairCount = Math.min(ShooterConstants.kHoodDistanceMeters.length, ShooterConstants.kHoodAngleDegrees.length);
    for (int i = 0; i < m_tablePairCount; i++) {
      m_distanceToAngleMap.put(ShooterConstants.kHoodDistanceMeters[i], ShooterConstants.kHoodAngleDegrees[i]);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Set the hood angle to a specific angle
   * @param angleDegrees Target angle in degrees [0, 45]
   */
  public void setHoodAngle(double angleDegrees) {
    double clampedAngle = MathUtil.clamp(angleDegrees, 
        ShooterConstants.kMinHoodAngleDegrees, 
        ShooterConstants.kMaxHoodAngleDegrees);
  m_hoodPID.setSetpoint(clampedAngle, ControlType.kPosition);
  autoTargetAngleDegrees = clampedAngle;
  }

  /**
   * Get the current hood angle
   * @return Hood angle in degrees
   */
  public double getHoodAngle() {
    return m_hoodEncoder.getPosition();
  }

  /**
   * Check if the hood is at the target angle
   * @param targetAngle Target angle in degrees
   * @param toleranceDegrees Tolerance in degrees
   * @return true if within tolerance
   */
  public boolean isAtTarget(double targetAngle, double toleranceDegrees) {
    double error = Math.abs(getHoodAngle() - targetAngle);
    return error <= toleranceDegrees;
  }

  /**
   * Stop the hood motor
   */
  public void stop() {
    m_hoodMotor.set(0.0);
  }

  /**
   * Command to set hood to a specific angle
   */
  public Command setHoodAngleCommand(double angleDegrees) {
    return this.startEnd(
        () -> setHoodAngle(angleDegrees),
        this::stop
    );
  }

  /**
   * Returns an interpolated hood angle for the requested distance.
   */
  public double getAutoHoodAngleForDistance(double distanceMeters) {
    if (m_tablePairCount == 0) {
      return ShooterConstants.kMinHoodAngleDegrees;
    }
    return m_distanceToAngleMap.get(distanceMeters);
  }

  /**
   * Continuously updates hood setpoint from a distance supplier.
   */
  public Command autoHoodFromDistanceCommand(DoubleSupplier distanceMetersSupplier) {
    return this.run(() -> {
      double distance = distanceMetersSupplier.getAsDouble();
      setHoodAngle(getAutoHoodAngleForDistance(distance));
    });
  }
}
