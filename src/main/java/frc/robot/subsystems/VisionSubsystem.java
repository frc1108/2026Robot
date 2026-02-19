// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private final PhotonCamera m_fuelCamera = new PhotonCamera(VisionConstants.kFuelCameraName);

  @Logged private double hopperYaw = 0.0;
  @Logged private double hopperPitch = 0.0;
  @Logged private double hopperDistance = 0.0;
  @Logged private double hopperAmbiguity = 1.0;
  @Logged private boolean hopperVisible = false;
  @Logged private boolean fuelVisible = false;
  @Logged private double fuelYaw = 0.0;
  @Logged private double fuelPitch = 0.0;
  @Logged private double fuelArea = 0.0;
  @Logged private int fuelTargetCount = 0;
  @Logged private int fuelSelectedClusterCount = 0;
  private final double[][] fuelHeatmap = new double[VisionConstants.kFuelHeatmapCellsX][VisionConstants.kFuelHeatmapCellsY];
  private final Field2d fuelHeatmapField = new Field2d();
  private double lastHeatmapTimestampSec = Double.NaN;

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
    setupFuelHeatmapWidget();
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

    // Front fuel camera (commented out for now).
    // Transform3d frontOffset = new Transform3d(
    //     new Translation3d(
    //         VisionConstants.kFrontFuelCameraOffsetX,
    //         VisionConstants.kFrontFuelCameraOffsetY,
    //         VisionConstants.kFrontFuelCameraOffsetZ),
    //     new Rotation3d(
    //         VisionConstants.kFrontFuelCameraRotX,
    //         VisionConstants.kFrontFuelCameraRotY,
    //         VisionConstants.kFrontFuelCameraRotZ));
    // m_photonCameras.add(new PhotonCamera(VisionConstants.kFrontFuelCameraName));
    // m_poseEstimators.add(new PhotonPoseEstimator(
    //     m_fieldLayout,
    //     PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    //     frontOffset));

    setupFuelHeatmapWidget();
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

    updateFuelTargeting();
  }

  private void updateFuelTargeting() {
    applyHeatmapDecay();
    fuelHeatmapField.setRobotPose(drive.getPose());

    if (!m_fuelCamera.isConnected()) {
      fuelVisible = false;
      fuelYaw = 0.0;
      fuelPitch = 0.0;
      fuelArea = 0.0;
      fuelTargetCount = 0;
      fuelSelectedClusterCount = 0;
      publishFuelHeatmapHotspots();
      return;
    }

    var result = m_fuelCamera.getLatestResult();
    if (!result.hasTargets()) {
      fuelVisible = false;
      fuelYaw = 0.0;
      fuelPitch = 0.0;
      fuelArea = 0.0;
      fuelTargetCount = 0;
      fuelSelectedClusterCount = 0;
      publishFuelHeatmapHotspots();
      return;
    }

    fuelTargetCount = result.targets.size();
    var selectedCluster = selectBestFuelCluster(result.targets);

    fuelVisible = true;
    fuelYaw = selectedCluster.meanYawDeg;
    fuelPitch = selectedCluster.meanPitchDeg;
    fuelArea = selectedCluster.totalArea;
    fuelSelectedClusterCount = selectedCluster.memberCount;

    updateFuelHeatmapFromTargets(result.targets);
    publishFuelHeatmapHotspots();
  }

  private void setupFuelHeatmapWidget() {
    Shuffleboard.getTab("Vision")
        .add("Fuel Heatmap", fuelHeatmapField)
        .withProperties(Map.of(
            "Field", VisionConstants.kFuelHeatmapFieldBackground,
            "Robot Width", 0.8,
            "Robot Length", 0.8))
        .withPosition(0, 0)
        .withSize(7, 4);
    SmartDashboard.putData("Fuel Heatmap", fuelHeatmapField);
  }

  private void applyHeatmapDecay() {
    double now = Timer.getFPGATimestamp();
    if (Double.isNaN(lastHeatmapTimestampSec)) {
      lastHeatmapTimestampSec = now;
      return;
    }

    double dt = now - lastHeatmapTimestampSec;
    lastHeatmapTimestampSec = now;
    if (dt <= 0.0) {
      return;
    }

    double decayScale = Math.max(0.0, 1.0 - (VisionConstants.kFuelHeatmapDecayPerSecond * dt));
    for (int x = 0; x < VisionConstants.kFuelHeatmapCellsX; x++) {
      for (int y = 0; y < VisionConstants.kFuelHeatmapCellsY; y++) {
        fuelHeatmap[x][y] *= decayScale;
      }
    }
  }

  private void updateFuelHeatmapFromTargets(List<PhotonTrackedTarget> targets) {
    Pose2d robotPose = drive.getPose();
    double fieldLength = m_fieldLayout.getFieldLength();
    double fieldWidth = m_fieldLayout.getFieldWidth();

    for (PhotonTrackedTarget target : targets) {
      double area = target.getArea();
      if (area <= 0.0) {
        continue;
      }

      double estimatedDistanceMeters = estimateFuelDistanceMetersFromArea(area);
      double headingDeg = robotPose.getRotation().getDegrees()
          + Math.toDegrees(VisionConstants.kFrontFuelCameraRotZ)
          + target.getYaw();
      double headingRad = Math.toRadians(headingDeg);
      double pointX = robotPose.getX() + estimatedDistanceMeters * Math.cos(headingRad);
      double pointY = robotPose.getY() + estimatedDistanceMeters * Math.sin(headingRad);

      if (pointX < 0.0 || pointX > fieldLength || pointY < 0.0 || pointY > fieldWidth) {
        continue;
      }

      int cellX = Math.min(
          VisionConstants.kFuelHeatmapCellsX - 1,
          Math.max(0, (int) ((pointX / fieldLength) * VisionConstants.kFuelHeatmapCellsX)));
      int cellY = Math.min(
          VisionConstants.kFuelHeatmapCellsY - 1,
          Math.max(0, (int) ((pointY / fieldWidth) * VisionConstants.kFuelHeatmapCellsY)));

      fuelHeatmap[cellX][cellY] += area;
    }
  }

  private double estimateFuelDistanceMetersFromArea(double area) {
    double estimate = VisionConstants.kFuelAreaToDistanceScale / Math.sqrt(area);
    return MathUtil.clamp(
        estimate,
        VisionConstants.kFuelMinEstimatedDistanceMeters,
        VisionConstants.kFuelMaxEstimatedDistanceMeters);
  }

  private void publishFuelHeatmapHotspots() {
    double fieldLength = m_fieldLayout.getFieldLength();
    double fieldWidth = m_fieldLayout.getFieldWidth();

    class Hotspot {
      final int x;
      final int y;
      final double value;

      Hotspot(int x, int y, double value) {
        this.x = x;
        this.y = y;
        this.value = value;
      }
    }

    List<Hotspot> top = new ArrayList<>();
    for (int x = 0; x < VisionConstants.kFuelHeatmapCellsX; x++) {
      for (int y = 0; y < VisionConstants.kFuelHeatmapCellsY; y++) {
        double value = fuelHeatmap[x][y];
        if (value < VisionConstants.kFuelHeatmapMinHotspotValue) {
          continue;
        }

        Hotspot candidate = new Hotspot(x, y, value);
        int insert = -1;
        for (int i = 0; i < top.size(); i++) {
          if (candidate.value > top.get(i).value) {
            insert = i;
            break;
          }
        }
        if (insert == -1) {
          if (top.size() < VisionConstants.kFuelHeatmapHotspotsToDisplay) {
            top.add(candidate);
          }
        } else {
          top.add(insert, candidate);
          if (top.size() > VisionConstants.kFuelHeatmapHotspotsToDisplay) {
            top.remove(top.size() - 1);
          }
        }
      }
    }

    for (int i = 0; i < VisionConstants.kFuelHeatmapHotspotsToDisplay; i++) {
      if (i < top.size()) {
        Hotspot hotspot = top.get(i);
        double xMeters = ((hotspot.x + 0.5) / VisionConstants.kFuelHeatmapCellsX) * fieldLength;
        double yMeters = ((hotspot.y + 0.5) / VisionConstants.kFuelHeatmapCellsY) * fieldWidth;
        fuelHeatmapField.getObject("FuelHotspot" + i).setPose(new Pose2d(xMeters, yMeters, Rotation2d.kZero));
      } else {
        fuelHeatmapField.getObject("FuelHotspot" + i).setPose(new Pose2d(-1.0, -1.0, Rotation2d.kZero));
      }
    }
  }

  private static final class FuelCluster {
    double meanYawDeg;
    double meanPitchDeg;
    double totalArea;
    int memberCount;
  }

  private FuelCluster selectBestFuelCluster(List<PhotonTrackedTarget> targets) {
    List<FuelCluster> clusters = new ArrayList<>();

    for (PhotonTrackedTarget target : targets) {
      double yaw = target.getYaw();
      double pitch = target.getPitch();
      double area = target.getArea();

      FuelCluster matched = null;
      for (FuelCluster cluster : clusters) {
        if (Math.abs(yaw - cluster.meanYawDeg) <= VisionConstants.kFuelClusterYawToleranceDegrees
            && Math.abs(pitch - cluster.meanPitchDeg) <= VisionConstants.kFuelClusterPitchToleranceDegrees) {
          matched = cluster;
          break;
        }
      }

      if (matched == null) {
        FuelCluster newCluster = new FuelCluster();
        newCluster.meanYawDeg = yaw;
        newCluster.meanPitchDeg = pitch;
        newCluster.totalArea = area;
        newCluster.memberCount = 1;
        clusters.add(newCluster);
      } else {
        int newCount = matched.memberCount + 1;
        matched.meanYawDeg = ((matched.meanYawDeg * matched.memberCount) + yaw) / newCount;
        matched.meanPitchDeg = ((matched.meanPitchDeg * matched.memberCount) + pitch) / newCount;
        matched.totalArea += area;
        matched.memberCount = newCount;
      }
    }

    FuelCluster best = clusters.get(0);
    for (int i = 1; i < clusters.size(); i++) {
      FuelCluster candidate = clusters.get(i);
      if (candidate.memberCount > best.memberCount
          || (candidate.memberCount == best.memberCount && candidate.totalArea > best.totalArea)) {
        best = candidate;
      }
    }
    return best;
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

  public boolean canSeeFuel() {
    return fuelVisible;
  }

  public double getFuelYaw() {
    return fuelYaw;
  }

  public double getFuelPitch() {
    return fuelPitch;
  }

  public double getFuelArea() {
    return fuelArea;
  }

  public int getFuelTargetCount() {
    return fuelTargetCount;
  }

  public int getFuelSelectedClusterCount() {
    return fuelSelectedClusterCount;
  }
}
