package frc.robot.commands;

import java.util.OptionalDouble;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * Drive while continuously aiming at the hopper using vision for rotation.
 * Left stick controls translation; rotation is controlled by PID to face the hopper.
 */
public class AimWhileDrivingCommand extends Command {
  private final VisionSubsystem m_vision;
  private final DriveSubsystem m_drive;
  private final CommandXboxController m_controller;
  private final PIDController m_rotationPID;

  public AimWhileDrivingCommand(VisionSubsystem vision, DriveSubsystem drive, CommandXboxController controller) {
    this.m_vision = vision;
    this.m_drive = drive;
    this.m_controller = controller;
    this.m_rotationPID = new PIDController(VisionConstants.kAimP, VisionConstants.kAimI, VisionConstants.kAimD);
    m_rotationPID.enableContinuousInput(-180, 180);
    m_rotationPID.setTolerance(VisionConstants.kAimingToleranceDegrees);
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    // No special init required
  }

  @Override
  public void execute() {
    // Read translation from left stick and apply deadband
    double xSpeed = -MathUtil.applyDeadband(m_controller.getLeftY(), OIConstants.kDriveDeadband);
    double ySpeed = -MathUtil.applyDeadband(m_controller.getLeftX(), OIConstants.kDriveDeadband);

    double rotationInputFallback = -MathUtil.applyDeadband(m_controller.getRightX(), OIConstants.kDriveDeadband);

    double rotation;
    double currentHeading = m_drive.getPose().getRotation().getDegrees();
    OptionalDouble targetHeading = m_vision.getHopperTargetHeadingDegrees(m_drive.getPose());
    if (targetHeading.isPresent()) {
      // Estimate field-relative translation velocity from current drive command.
      double vxField = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
      double vyField = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;

      // Shooter axis in field frame when at stationary target heading.
      double stationaryShooterAxisDeg =
          targetHeading.getAsDouble() + VisionConstants.kShooterYawOffsetDegrees;
      Rotation2d shooterAxis = Rotation2d.fromDegrees(stationaryShooterAxisDeg);

      // Lateral velocity component (left of shot axis positive).
      double vPerpMetersPerSec = (-shooterAxis.getSin() * vxField) + (shooterAxis.getCos() * vyField);

      // Ball inherits robot lateral velocity; aim against it.
      double leadDegrees = Math.toDegrees(Math.atan2(
          VisionConstants.kShotLeadGain * vPerpMetersPerSec,
          VisionConstants.kBallExitSpeedMetersPerSecond));
      leadDegrees = MathUtil.clamp(
          leadDegrees, -VisionConstants.kMaxShotLeadDegrees, VisionConstants.kMaxShotLeadDegrees);

      double compensatedTargetHeading = targetHeading.getAsDouble() - leadDegrees;
      compensatedTargetHeading = MathUtil.inputModulus(compensatedTargetHeading, -180.0, 180.0);

      rotation = m_rotationPID.calculate(currentHeading, compensatedTargetHeading);
      // Clamp to [-1,1]
      rotation = Math.max(-1.0, Math.min(1.0, rotation));
    } else {
      // Fallback only if tag ID is missing from field layout
      rotation = rotationInputFallback;
    }

    // Drive in field-relative mode so translation is still controlled by the driver
    m_drive.drive(xSpeed, ySpeed, rotation, true);
  }

  @Override
  public void end(boolean interrupted) {
    // Stop rotation when command ends
    m_drive.drive(0, 0, 0, false);
  }

  @Override
  public boolean isFinished() {
    // This is a whileTrue binding; return false so it only ends when released
    return false;
  }
}
