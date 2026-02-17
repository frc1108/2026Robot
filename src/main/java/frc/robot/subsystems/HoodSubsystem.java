package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

@Logged
public class HoodSubsystem extends SubsystemBase {
  private final CANSparkMax m_hoodMotor;
  private final RelativeEncoder m_hoodEncoder;
  private final SparkPIDController m_hoodPID;

  /** Creates a new HoodSubsystem. */
  public HoodSubsystem() {
    m_hoodMotor = new CANSparkMax(ShooterConstants.kHoodMotorCanId, MotorType.kBrushless);
    m_hoodMotor.restoreFactoryDefaults();
    m_hoodMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    
    m_hoodEncoder = m_hoodMotor.getEncoder();
    m_hoodEncoder.setPositionConversionFactor(360.0 / ShooterConstants.kHoodGearRatio); // Convert to degrees
    m_hoodEncoder.setVelocityConversionFactor(360.0 / (ShooterConstants.kHoodGearRatio * 60.0)); // Convert to degrees per second
    
    m_hoodPID = m_hoodMotor.getPIDController();
    m_hoodPID.setP(ShooterConstants.kHoodP);
    m_hoodPID.setI(ShooterConstants.kHoodI);
    m_hoodPID.setD(ShooterConstants.kHoodD);
    m_hoodPID.setOutputRange(-1.0, 1.0);
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
    m_hoodPID.setReference(clampedAngle, ControlType.kPosition);
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
}
