package frc.robot.commands;

import java.util.OptionalDouble;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

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
    m_rotationPID.reset();
  }

  @Override
  public void execute() {
    double xSpeed = -MathUtil.applyDeadband(m_controller.getLeftY(), OIConstants.kDriveDeadband);
    double ySpeed = -MathUtil.applyDeadband(m_controller.getLeftX(), OIConstants.kDriveDeadband);
    double rotationInputFallback = -MathUtil.applyDeadband(m_controller.getRightX(), OIConstants.kDriveDeadband);

    double currentHeading = m_drive.getPose().getRotation().getDegrees();
    OptionalDouble targetHeading = m_vision.getHopperTargetHeadingDegrees(m_drive.getPose());

    double rotation;
    if (targetHeading.isPresent()) {
      // Field-relative robot translation velocity from current stick command.
      double vxField = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
      double vyField = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;

      double stationaryShooterAxisDeg = targetHeading.getAsDouble() + VisionConstants.kShooterYawOffsetDegrees;
      Rotation2d shooterAxis = Rotation2d.fromDegrees(stationaryShooterAxisDeg);
      double vPerpMetersPerSec = (-shooterAxis.getSin() * vxField) + (shooterAxis.getCos() * vyField);

      // Ball inherits lateral robot velocity; lead against it.
      double leadDegrees = Math.toDegrees(Math.atan2(
          VisionConstants.kShotLeadGain * vPerpMetersPerSec,
          VisionConstants.kBallExitSpeedMetersPerSecond));
      leadDegrees = MathUtil.clamp(leadDegrees, -VisionConstants.kMaxShotLeadDegrees, VisionConstants.kMaxShotLeadDegrees);

      double compensatedTargetHeading = targetHeading.getAsDouble() - leadDegrees;
      compensatedTargetHeading = MathUtil.inputModulus(compensatedTargetHeading, -180.0, 180.0);
      rotation = m_rotationPID.calculate(currentHeading, compensatedTargetHeading);
      rotation *= VisionConstants.kAimRotationSign;
      rotation = MathUtil.clamp(rotation, -1.0, 1.0);
    } else {
      rotation = rotationInputFallback;
    }

    m_drive.drive(xSpeed, ySpeed, rotation, true);
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0, false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
