// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 11;
    public static final int kRearLeftDrivingCanId = 13;
    public static final int kFrontRightDrivingCanId = 15;
    public static final int kRearRightDrivingCanId = 17;

    public static final int kFrontLeftTurningCanId = 10;
    public static final int kRearLeftTurningCanId = 12;
    public static final int kFrontRightTurningCanId = 14;
    public static final int kRearRightTurningCanId = 16;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
  }
  public static final class IntakeConstants {
    public static final int frontIntakecanid = 21;
    public static final int backIntakecanid = 22;
    //public static final double frontIntakeSpeed = 0.95;
    //public static final double backIntakeSpeed = -0.95;
    //public static final double slowFrontIntakeSpeed = 0.25;
    //public static final double slowBackIntakeSpeed = -0.95;
    //public static final double reverseFrontIntakeSpeed = -0.95;
    //public static final double reverseBackIntakeSpeed = 0.95;
  public static final double intakeFrontVelocityRpm = 4000.0;
  public static final double slowIntakeFrontVelocityRpm = 1000.0;
  public static final double reverseIntakeFrontVelocityRpm = -4000.0;
    // Back-motor-specific RPM targets (allows tuning front/back separately)
    public static final double intakeBackVelocityRpm = -4000.0;
    public static final double slowIntakeBackVelocityRpm = -4000.0;
    public static final double reverseIntakeBackVelocityRpm = 4000.0;
  }
  public static final class TombConstants {
    // Assumed CAN IDs for Tomb motors. Change these if your hardware uses different IDs.
    public static final int frontTombCanId = 23;
    public static final int backTombCanId = 24;
    public static final int feederTombCanId = 25;
    public static final double backTombSpeed = 0.95;
    public static final double frontTombSpeed = -0.95;
    public static final double feederTombSpeed = -0.95;
  // Feeder velocity setpoint (RPM). Adjust as needed for your mechanism and tuning.
  public static final double feederTombVelocityRpm = -1500.0;
    public static final double reverseFrontTombSpeed = 0.95;
    public static final double reverseBackTombSpeed = -0.95;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class ShooterConstants {
    // Kraken X60 shooter motors
    public static final int kLeftShooterCanId = 31;
    public static final int kRightShooterCanId = 32;
    
    // Hood control - Neo 550
    public static final int kHoodMotorCanId = 33;
    public static final double kHoodGearRatio = 36.0; // Adjust based on your gearbox reduction
    public static final double kMinHoodAngleDegrees = 0.0;
    public static final double kMaxHoodAngleDegrees = 45.0;
    
    // PID constants for hood control
    public static final double kHoodP = 0.5;
    public static final double kHoodI = 0.0;
    public static final double kHoodD = 0.1;
    
    // Shooter speed presets
    public static final double kShooterFullSpeed = 0.90;
    public static final double kShooterSlowSpeed = 0.50;

    // Auto-hood interpolation table (distance meters -> hood angle degrees).
    // Replace these with measured shot data from your robot.
    public static final double[] kHoodDistanceMeters = {1.5, 2.5, 3.5, 4.5};
    public static final double[] kHoodAngleDegrees = {34.0, 28.0, 22.0, 17.0};
  }

  public static final class VisionConstants {
    // PhotonVision camera names (must match the name in PhotonVision UI)
  public static final String kLeftCameraName = "LeftSideCamera";
  public static final String kRightCameraName = "RightSideCamera";

    // April tag IDs
    public static final int kHopperTagId = 26; // Change to your hopper's April tag ID
    // Offset from hopper AprilTag to the actual hopper center, in the tag's frame.
    // +Forward is out from the tag face normal; +Left is tag-left.
    public static final double kHopperCenterOffsetForwardMeters = -0.305;
    public static final double kHopperCenterOffsetLeftMeters = 0.0;
    // Shooter mount geometry in robot frame.
    // +Forward is toward robot front bumper, +Left is toward robot left bumper.
    public static final double kShooterOffsetForwardMeters = -0.2;
    public static final double kShooterOffsetLeftMeters = 0.2;
    // Shooter bore direction relative to robot forward. Left-facing shooter is +90 deg.
    public static final double kShooterYawOffsetDegrees = 90.0;

    // Camera mounting position relative to robot center (meters and radians)
    // Left camera (these were the previous single-camera defaults)
    public static final double kLeftCameraOffsetX = 0.0;  // Forward/backward offset (meters)
    public static final double kLeftCameraOffsetY = 0.368;  // Left/right offset (meters)
    public static final double kLeftCameraOffsetZ = 0.711;  // Up/down offset (meters) - adjust to camera height
  // If the left camera is looking straight to the left and mounted level:
  public static final double kLeftCameraRotX = 0.0;     // Roll (radians)
  public static final double kLeftCameraRotY = 0.0;     // Pitch (radians)
  public static final double kLeftCameraRotZ = 1.5708;     // Yaw (radians) (+90°)

  // Right camera (mirror of left)
    public static final double kRightCameraOffsetX = 0.0;
    public static final double kRightCameraOffsetY = -0.368;
    public static final double kRightCameraOffsetZ = 0.711;
  // If the right camera is looking straight to the right and mounted level:
  public static final double kRightCameraRotX = 0.0;
  public static final double kRightCameraRotY = 0.0;
  public static final double kRightCameraRotZ = -1.5708; // (-90°)

    // Vision filtering constants
    public static final double kMaxDistanceMeters = 10.0;  // Max distance to consider targets
    public static final double kMaxAmbiguity = 0.35;       // Max pose ambiguity (0-1)
    
    // PID constants for aiming rotation
    public static final double kAimP = 0.02;
    public static final double kAimI = 0.0;
    public static final double kAimD = 0.001;
    
    // Tolerance for aiming (degrees)
    public static final double kAimingToleranceDegrees = 2.0;
    public static final double kAimingTimeoutSeconds = 2.0;
  }
}
