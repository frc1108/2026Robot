// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

@Logged
public class VisionSubsystem extends SubsystemBase {
  private final List<PhotonCamera> photonCameras = new ArrayList<>();
  private final List<PhotonPoseEstimator> poseEstimators = new ArrayList<>();
  private final AprilTagFieldLayout fieldLayout;
  private final BiConsumer<Pose2d, Double> consumer;
  @NotLogged private final DriveSubsystem drive;
  private Pose3d estimated3dPose;
  
  // Hopper targeting data for Advantage Scope logging
  @Logged private double hopperYaw = 0.0;
  @Logged private double hopperPitch = 0.0;
  @Logged private double hopperDistance = 0.0;
  @Logged private double hopperAmbiguity = 1.0;
  @Logged private boolean hopperVisible = false;

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem(BiConsumer<Pose2d, Double> consumer, DriveSubsystem drive, String LeftSideCamera,
    Transform3d cameraOffset) throws IOException {
  // Backwards-compatible constructor: create single camera/estimator lists
  photonCameras.add(new PhotonCamera(LeftSideCamera));
  System.out.println("[VisionSubsystem] Camera name: " + LeftSideCamera + ", cameraOffset=" + cameraOffset);

  fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2026RebuiltWelded.m_resourceFile);
  poseEstimators.add(new PhotonPoseEstimator(fieldLayout,
    PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    cameraOffset));
  estimated3dPose = new Pose3d();
  this.consumer = consumer;
  this.drive = drive;
  }

  /** Multi-camera constructor: initializes three cameras and pose estimators based on Constants. */
  public VisionSubsystem(BiConsumer<Pose2d, Double> consumer, DriveSubsystem drive) throws IOException {
  fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2026RebuiltWelded.m_resourceFile);
  this.consumer = consumer;
  this.drive = drive;
  estimated3dPose = new Pose3d();

  // Left camera
  Transform3d leftOffset = new Transform3d(
    new edu.wpi.first.math.geometry.Translation3d(VisionConstants.kLeftCameraOffsetX,
      VisionConstants.kLeftCameraOffsetY, VisionConstants.kLeftCameraOffsetZ),
    new edu.wpi.first.math.geometry.Rotation3d(VisionConstants.kLeftCameraRotX,
      VisionConstants.kLeftCameraRotY, VisionConstants.kLeftCameraRotZ));
  photonCameras.add(new PhotonCamera(VisionConstants.kLeftCameraName));
  poseEstimators.add(new PhotonPoseEstimator(fieldLayout,
    PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, leftOffset));
  System.out.println("[VisionSubsystem] Added camera: " + VisionConstants.kLeftCameraName + ", offset=" + leftOffset);

  // Right camera
  Transform3d rightOffset = new Transform3d(
    new edu.wpi.first.math.geometry.Translation3d(VisionConstants.kRightCameraOffsetX,
      VisionConstants.kRightCameraOffsetY, VisionConstants.kRightCameraOffsetZ),
    new edu.wpi.first.math.geometry.Rotation3d(VisionConstants.kRightCameraRotX,
      VisionConstants.kRightCameraRotY, VisionConstants.kRightCameraRotZ));
  photonCameras.add(new PhotonCamera(VisionConstants.kRightCameraName));
  poseEstimators.add(new PhotonPoseEstimator(fieldLayout,
    PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, rightOffset));
  System.out.println("[VisionSubsystem] Added camera: " + VisionConstants.kRightCameraName + ", offset=" + rightOffset);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // We'll iterate through all configured cameras, update their pose estimators,
    // and publish any valid estimates to the DriveSubsystem. We'll also aggregate
    // hopper (tag) detections across cameras and pick the best one.
    Pose2d fallbackPose = drive.getPose();
    Pose2d robotPoseForFiltering = (estimated3dPose != null) ? estimated3dPose.toPose2d() : fallbackPose;

    // Track best hopper detection across all cameras (choose by lowest ambiguity)
    PhotonTrackedTarget bestHopperTarget = null;
    double bestHopperAmbiguity = Double.MAX_VALUE;
    Pose2d bestHopperPoseForDistance = fallbackPose;

    for (int i = 0; i < photonCameras.size(); i++) {
      PhotonCamera cam = photonCameras.get(i);
      PhotonPoseEstimator est = poseEstimators.get(i);

      if (!cam.isConnected()) {
        continue;
      }

      var result = cam.getLatestResult();
      if (!result.hasTargets()) {
        continue;
      }

      // Filter targets by ambiguity and distance before updating estimator
      List<PhotonTrackedTarget> badTargets = new ArrayList<>();
      for (PhotonTrackedTarget target : result.targets) {
        var tagPose = fieldLayout.getTagPose(target.getFiducialId());
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
        // Publish to drive odometry (WPILib pose estimator will fuse it)
        consumer.accept(estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds);
      }

      // Check for hopper tag detections in this camera's result and keep the best one
      for (PhotonTrackedTarget target : result.targets) {
        if (target.getFiducialId() == VisionConstants.kHopperTagId) {
          if (target.getPoseAmbiguity() < bestHopperAmbiguity) {
            bestHopperAmbiguity = target.getPoseAmbiguity();
            bestHopperTarget = target;
            bestHopperPoseForDistance = (estimated3dPose != null) ? estimated3dPose.toPose2d() : fallbackPose;
          }
        }
      }
    }

    // Update hopper targeting data based on best found target (if any)
    if (bestHopperTarget != null) {
      hopperVisible = true;
      hopperYaw = bestHopperTarget.getYaw();
      hopperPitch = bestHopperTarget.getPitch();
      hopperAmbiguity = bestHopperTarget.getPoseAmbiguity();
      var tagPose = fieldLayout.getTagPose(bestHopperTarget.getFiducialId());
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

  /**
   * Update hopper targeting data from camera results
   */
  private void updateHopperTargeting(org.photonvision.targeting.PhotonPipelineResult result, Pose2d robotPose) {
    hopperVisible = false;
    hopperYaw = 0.0;
    hopperPitch = 0.0;
    hopperDistance = 0.0;
    hopperAmbiguity = 1.0;

    if (!result.hasTargets())
      return;

    // Find the hopper April tag
    for (PhotonTrackedTarget target : result.targets) {
      if (target.getFiducialId() == VisionConstants.kHopperTagId) {
        hopperVisible = true;
        hopperYaw = target.getYaw();
        hopperPitch = target.getPitch();
        hopperAmbiguity = target.getPoseAmbiguity();

        // Calculate distance from tag pose
        var tagPose = fieldLayout.getTagPose(target.getFiducialId());
        if (tagPose.isPresent()) {
          hopperDistance = PhotonUtils.getDistanceToPose(robotPose, tagPose.get().toPose2d());
        }
        return; // Only track the first hopper tag found
      }
    }
  }

  /**
   * Check if the camera can see the hopper April tag
   * @return true if hopper tag is visible
   */
  public boolean canSeeHopper() {
    return hopperVisible;
  }

  /**
   * Get the yaw (horizontal angle) to the hopper April tag
   * Positive yaw = target is to the right
   * Negative yaw = target is to the left
   * @return Yaw in degrees, or 0 if hopper not visible
   */
  public double getHopperYaw() {
    return hopperYaw;
  }

  /**
   * Get the pitch (vertical angle) to the hopper April tag
   * @return Pitch in degrees, or 0 if hopper not visible
   */
  public double getHopperPitch() {
    return hopperPitch;
  }

  /**
   * Get the distance to the hopper
   * @return Distance in meters
   */
  public double getHopperDistance() {
    return hopperDistance;
  }

  /**
   * Get the ambiguity of the hopper target (0 = perfect, 1 = ambiguous)
   * @return Ambiguity value
   */
  public double getHopperAmbiguity() {
    return hopperAmbiguity;
  }

  /**
   * Get the estimated 3D pose of the robot
   * @return Pose3d
   */
  public Pose3d getEstimated3dPose() {
    return estimated3dPose;
  }
}
