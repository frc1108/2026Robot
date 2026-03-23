package frc.robot.commands;

import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AimAtFieldPoseWhileDrivingCommand extends Command {
  private final DriveSubsystem m_drive;
  private final CommandXboxController m_controller;
  private final Supplier<Optional<Pose2d>> m_targetPoseSupplier;
  private final PIDController m_rotationPID;

  public AimAtFieldPoseWhileDrivingCommand(
      DriveSubsystem drive,
      CommandXboxController controller,
      Supplier<Optional<Pose2d>> targetPoseSupplier) {
    m_drive = drive;
    m_controller = controller;
    m_targetPoseSupplier = targetPoseSupplier;
    m_rotationPID = new PIDController(VisionConstants.kAimP, VisionConstants.kAimI, VisionConstants.kAimD);
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

    Pose2d robotPose = m_drive.getPose();
    double currentHeading = robotPose.getRotation().getDegrees();
    Optional<Pose2d> targetPose = m_targetPoseSupplier.get();
    OptionalDouble targetHeading =
        targetPose.isPresent()
            ? VisionSubsystem.getTargetHeadingDegrees(robotPose, targetPose.get())
            : OptionalDouble.empty();

    double rotation;
    if (targetHeading.isPresent()) {
      ChassisSpeeds robotRelativeSpeeds = m_drive.getRobotRelativeSpeeds();
      Rotation2d robotHeading = robotPose.getRotation();
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

      double leadDegrees = Math.toDegrees(Math.atan2(
          VisionConstants.kShotLeadGain * vPerpMetersPerSec,
          VisionConstants.kBallExitSpeedMetersPerSecond));
      leadDegrees = MathUtil.clamp(
          leadDegrees,
          -VisionConstants.kMaxShotLeadDegrees,
          VisionConstants.kMaxShotLeadDegrees);

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
