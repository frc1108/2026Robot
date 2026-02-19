// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.BiConsumer;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

@Logged
public class VisionSubsystem extends SubsystemBase {
  private final List<PhotonCamera> m_photonCameras = new ArrayList<>();
  private final List<PhotonPoseEstimator> m_poseEstimators = new ArrayList<>();
  private final AprilTagFieldLayout m_fieldLayout;
  private final BiConsumer<Pose2d, Double> m_consumer;
  @NotLogged private final DriveSubsystem drive;
  private Pose3d estimated3dPose = new Pose3d();
  @Logged private Pose3d leftEstimated3dPose = new Pose3d();
  @Logged private Pose3d rightEstimated3dPose = new Pose3d();
  @Logged private String lastEstimatorCameraName = "";

  @Logged private double hopperYaw = 0.0;
  @Logged private double hopperPitch = 0.0;
  @Logged private double hopperDistance = 0.0;
  @Logged private double hopperAmbiguity = 1.0;
  @Logged private boolean hopperVisible = false;

  public VisionSubsystem(
      BiConsumer<Pose2d, Double> consumer,
      DriveSubsystem drive,
      String leftSideCamera,
      Transform3d cameraOffset) throws IOException {
    m_fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2026RebuiltWelded.m_resourceFile);
    m_consumer = consumer;
    this.drive = drive;

    m_photonCameras.add(new PhotonCamera(leftSideCamera));
    m_poseEstimators.add(new PhotonPoseEstimator(
        m_fieldLayout,
        PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        cameraOffset));
  }

  public VisionSubsystem(BiConsumer<Pose2d, Double> consumer, DriveSubsystem drive) throws IOException {
    m_fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2026RebuiltWelded.m_resourceFile);
    m_consumer = consumer;
    this.drive = drive;

    Transform3d leftOffset = new Transform3d(
        new Translation3d(
            VisionConstants.kLeftCameraOffsetX,
            VisionConstants.kLeftCameraOffsetY,
            VisionConstants.kLeftCameraOffsetZ),
        new Rotation3d(
            VisionConstants.kLeftCameraRotX,
            VisionConstants.kLeftCameraRotY,
            VisionConstants.kLeftCameraRotZ));
    m_photonCameras.add(new PhotonCamera(VisionConstants.kLeftCameraName));
    m_poseEstimators.add(new PhotonPoseEstimator(
        m_fieldLayout,
        PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        leftOffset));

    Transform3d rightOffset = new Transform3d(
        new Translation3d(
            VisionConstants.kRightCameraOffsetX,
            VisionConstants.kRightCameraOffsetY,
            VisionConstants.kRightCameraOffsetZ),
        new Rotation3d(
            VisionConstants.kRightCameraRotX,
            VisionConstants.kRightCameraRotY,
            VisionConstants.kRightCameraRotZ));
    m_photonCameras.add(new PhotonCamera(VisionConstants.kRightCameraName));
    m_poseEstimators.add(new PhotonPoseEstimator(
        m_fieldLayout,
        PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        rightOffset));
  }

  @Override
  public void periodic() {
    Pose2d fallbackPose = drive.getPose();
    // Use fused drive pose until we have at least one vision estimate.
    Pose2d robotPoseForFiltering =
        lastEstimatorCameraName.isEmpty() ? fallbackPose : estimated3dPose.toPose2d();

    PhotonTrackedTarget bestHopperTarget = null;
    double bestHopperAmbiguity = Double.MAX_VALUE;
    Pose2d bestHopperPoseForDistance = fallbackPose;

    for (int i = 0; i < m_photonCameras.size(); i++) {
      PhotonCamera cam = m_photonCameras.get(i);
      PhotonPoseEstimator est = m_poseEstimators.get(i);

      if (!cam.isConnected()) {
        continue;
      }

      var result = cam.getLatestResult();
      if (!result.hasTargets()) {
        continue;
      }

      List<PhotonTrackedTarget> badTargets = new ArrayList<>();
      for (PhotonTrackedTarget target : result.targets) {
        var tagPose = m_fieldLayout.getTagPose(target.getFiducialId());
        if (tagPose.isEmpty()) {
          badTargets.add(target);
          continue;
        }
        var distanceToTag = PhotonUtils.getDistanceToPose(robotPoseForFiltering, tagPose.get().toPose2d());
        if (target.getPoseAmbiguity() > VisionConstants.kMaxAmbiguity || distanceToTag > VisionConstants.kMaxDistanceMeters) {
          badTargets.add(target);
        }
      }
      result.targets.removeAll(badTargets);

      // Update this camera's pose estimator
      Optional<EstimatedRobotPose> poseResult = est.update(result);
      if (poseResult.isPresent()) {
        EstimatedRobotPose estimatedPose = poseResult.get();
        estimated3dPose = estimatedPose.estimatedPose;
        m_consumer.accept(estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds);
        String camName = cam.getName();
        lastEstimatorCameraName = camName;
        if (i == 0) {
          leftEstimated3dPose = estimatedPose.estimatedPose;
        } else if (i == 1) {
          rightEstimated3dPose = estimatedPose.estimatedPose;
        }
      }

      for (PhotonTrackedTarget target : result.targets) {
        if (target.getFiducialId() == VisionConstants.kHopperTagId) {
          if (target.getPoseAmbiguity() < bestHopperAmbiguity) {
            bestHopperAmbiguity = target.getPoseAmbiguity();
            bestHopperTarget = target;
            bestHopperPoseForDistance = estimated3dPose.toPose2d();
          }
        }
      }
    }

    if (bestHopperTarget != null) {
      hopperVisible = true;
      hopperYaw = bestHopperTarget.getYaw();
      hopperPitch = bestHopperTarget.getPitch();
      hopperAmbiguity = bestHopperTarget.getPoseAmbiguity();
      var tagPose = m_fieldLayout.getTagPose(bestHopperTarget.getFiducialId());
      if (tagPose.isPresent()) {
        hopperDistance = PhotonUtils.getDistanceToPose(bestHopperPoseForDistance, tagPose.get().toPose2d());
      }
    } else {
      hopperVisible = false;
      hopperYaw = 0.0;
      hopperPitch = 0.0;
      hopperDistance = 0.0;
      hopperAmbiguity = 1.0;
    }
  }

  public boolean canSeeHopper() {
    return hopperVisible;
  }

  public double getHopperYaw() {
    return hopperYaw;
  }

  public double getHopperPitch() {
    return hopperPitch;
  }

  public double getHopperDistance() {
    return hopperDistance;
  }

  public double getHopperAmbiguity() {
    return hopperAmbiguity;
  }

  public OptionalDouble getHopperTargetHeadingDegrees(Pose2d robotPose) {
    var hopperTagPose = m_fieldLayout.getTagPose(VisionConstants.kHopperTagId);
    if (hopperTagPose.isEmpty()) {
      return OptionalDouble.empty();
    }

    Pose2d hopperCenterPose = hopperTagPose.get().toPose2d().transformBy(
        new Transform2d(
            VisionConstants.kHopperCenterOffsetForwardMeters,
            VisionConstants.kHopperCenterOffsetLeftMeters,
            Rotation2d.kZero));

    Translation2d shooterOffsetRobot = new Translation2d(
        VisionConstants.kShooterOffsetForwardMeters,
        VisionConstants.kShooterOffsetLeftMeters);
    Translation2d shooterPositionField = robotPose.getTranslation().plus(
        shooterOffsetRobot.rotateBy(robotPose.getRotation()));

    double lineToHopperDeg = Math.toDegrees(Math.atan2(
        hopperCenterPose.getY() - shooterPositionField.getY(),
        hopperCenterPose.getX() - shooterPositionField.getX()));

    double shooterFacingDeg =
        robotPose.getRotation().getDegrees() + VisionConstants.kShooterYawOffsetDegrees;
    // Compute heading error in shooter-axis space, then convert back to robot heading.
    double shooterAimErrorDeg =
        MathUtil.inputModulus(lineToHopperDeg - shooterFacingDeg, -180.0, 180.0);

    double targetRobotHeadingDeg = MathUtil.inputModulus(
        robotPose.getRotation().getDegrees() + shooterAimErrorDeg, -180.0, 180.0);
    return OptionalDouble.of(targetRobotHeadingDeg);
  }

  public OptionalDouble getHopperCenterDistanceMeters(Pose2d robotPose) {
    var hopperTagPose = m_fieldLayout.getTagPose(VisionConstants.kHopperTagId);
    if (hopperTagPose.isEmpty()) {
      return OptionalDouble.empty();
    }

    Pose2d hopperCenterPose = hopperTagPose.get().toPose2d().transformBy(
        new Transform2d(
            VisionConstants.kHopperCenterOffsetForwardMeters,
            VisionConstants.kHopperCenterOffsetLeftMeters,
            Rotation2d.kZero));

    return OptionalDouble.of(robotPose.getTranslation().getDistance(hopperCenterPose.getTranslation()));
  }

  public Pose3d getEstimated3dPose() {
    return estimated3dPose;
  }

  public Pose3d getLeftEstimated3dPose() {
    return leftEstimated3dPose;
  }

  public Pose3d getRightEstimated3dPose() {
    return rightEstimated3dPose;
  }

  public String getLastEstimatorCameraName() {
    return lastEstimatorCameraName;
  }
}
