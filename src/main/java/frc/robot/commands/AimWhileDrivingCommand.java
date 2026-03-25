package frc.robot.commands;

import java.util.OptionalDouble;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
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
  private final boolean m_enableOrbitJiggle;
  private double m_startTimestampSec;

  public AimWhileDrivingCommand(VisionSubsystem vision, DriveSubsystem drive, CommandXboxController controller) {
    this(vision, drive, controller, false);
  }

  public AimWhileDrivingCommand(
      VisionSubsystem vision,
      DriveSubsystem drive,
      CommandXboxController controller,
      boolean enableOrbitJiggle) {
    this.m_vision = vision;
    this.m_drive = drive;
    this.m_controller = controller;
    this.m_enableOrbitJiggle = enableOrbitJiggle;
    this.m_rotationPID = new PIDController(VisionConstants.kAimP, VisionConstants.kAimI, VisionConstants.kAimD);
    m_rotationPID.enableContinuousInput(-180, 180);
    m_rotationPID.setTolerance(VisionConstants.kAimingToleranceDegrees);
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    m_rotationPID.reset();
    m_startTimestampSec = Timer.getFPGATimestamp();
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
      // Use measured chassis velocity (not stick commands) for lead compensation.
      ChassisSpeeds robotRelativeSpeeds = m_drive.getRobotRelativeSpeeds();
      Rotation2d robotHeading = m_drive.getPose().getRotation();
      double vxField =
          robotRelativeSpeeds.vxMetersPerSecond * robotHeading.getCos()
              - robotRelativeSpeeds.vyMetersPerSecond * robotHeading.getSin();
      double vyField =
          robotRelativeSpeeds.vxMetersPerSecond * robotHeading.getSin()
              + robotRelativeSpeeds.vyMetersPerSecond * robotHeading.getCos();

      double stationaryShooterAxisDeg =
          targetHeading.getAsDouble() + VisionConstants.getShooterAxisAngleDegrees();
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

    if (m_enableOrbitJiggle) {
      Translation2d jiggleVelocity = getOrbitJiggleVelocityField(m_drive.getPose(), nowSec());
      xSpeed += jiggleVelocity.getX() / DriveConstants.kMaxSpeedMetersPerSecond;
      ySpeed += jiggleVelocity.getY() / DriveConstants.kMaxSpeedMetersPerSecond;
    }

    xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
    ySpeed = MathUtil.clamp(ySpeed, -1.0, 1.0);
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

  private double nowSec() {
    return Timer.getFPGATimestamp();
  }

  private Translation2d getOrbitJiggleVelocityField(Pose2d robotPose, double nowSec) {
    if ((nowSec - m_startTimestampSec) < VisionConstants.kAimOrbitJiggleDelaySeconds) {
      return Translation2d.kZero;
    }

    var hopperPose = VisionSubsystem.getEstimatedHopperCenterPose();
    if (hopperPose.isEmpty()) {
      return Translation2d.kZero;
    }

    Translation2d robotToHopper = hopperPose.get().getTranslation().minus(robotPose.getTranslation());
    double distance = robotToHopper.getNorm();
    if (distance < 1e-6) {
      return Translation2d.kZero;
    }

    Translation2d tangent = new Translation2d(
        -robotToHopper.getY() / distance,
        robotToHopper.getX() / distance);
    double phaseSeconds = nowSec - m_startTimestampSec - VisionConstants.kAimOrbitJiggleDelaySeconds;
    double direction =
        ((int) Math.floor(phaseSeconds / VisionConstants.kAimOrbitJiggleHalfCycleSeconds)) % 2 == 0 ? 1.0 : -1.0;
    double jiggleSpeedMetersPerSecond =
        VisionConstants.kAimOrbitJiggleDistanceMeters / VisionConstants.kAimOrbitJiggleHalfCycleSeconds;
    return tangent.times(direction * jiggleSpeedMetersPerSecond);
  }
}
