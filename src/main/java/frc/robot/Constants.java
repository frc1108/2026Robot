package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
  private Constants() {}

  public static final class DriveConstants {
    private DriveConstants() {}

    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI;

    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0.0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

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
    private ModuleConstants() {}

    public static final int kDrivingMotorPinionTeeth = 14;

    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60.0;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    public static final double kDrivingMotorReduction =
        (45.0 * 22.0) / (kDrivingMotorPinionTeeth * 15.0);
    public static final double kDriveWheelFreeSpeedRps =
        (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    private OIConstants() {}

    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class LightsConstants {
    private LightsConstants() {}

    public static final int kBlinkinPwmPort = 0;
    public static final double kSolidRed = 0.61;
    public static final double kSolidBlue = 0.87;

    // Mode 0 is alliance color. The rest are manual cycle options.
    public static final double[] kBlinkinModeValues = {
        0.0,
        kSolidRed,
        kSolidBlue
    };
  }

  public static final class IntakeConstants {
    private IntakeConstants() {}

    // Hardware
    public static final int frontIntakecanid = 21;
    public static final int backIntakecanid = 22;
    public static final int thirdIntakeCanId = 26;

    // Driver left trigger: normal intake
    public static final double intakeFrontVelocityRpm = 4000.0;
    public static final double intakeBackVelocityRpm = -4000.0;
    public static final double intakeThirdPercentOutput = -0.95;

    // Driver left bumper: slow intake
    public static final double slowIntakeFrontVelocityRpm = 2500.0;
    public static final double slowIntakeBackVelocityRpm = -2500.0;
    public static final double slowIntakeThirdPercentOutput = -0.60;

    // Driver POV down: reverse intake
    public static final double reverseIntakeFrontVelocityRpm = -4000.0;
    public static final double reverseIntakeBackVelocityRpm = 4000.0;
    public static final double reverseIntakeThirdPercentOutput = 0.95;
  }

  public static final class TombConstants {
    private TombConstants() {}

    // Hardware
    public static final int frontTombCanId = 23;
    public static final int backTombCanId = 24;
    public static final int feederTombCanId = 25;

    // Driver Y / tomb feed
    public static final double frontTombSpeed = -0.95;
    public static final double backTombSpeed = 0.95;
    public static final double feederTombSpeed = 0.95;
    public static final double feederTombVelocityRpm = -5000.0;
    // public static final double feederPulseForwardSeconds = 1.0;
    // public static final double feederPulseReverseSeconds = 0.05;

    // Reverse controls
    public static final double reverseFrontTombSpeed = 0.95;
    public static final double reverseBackTombSpeed = -0.95;
    public static final double reverseFeederTombVelocityRpm = 5000.0;
  }

  public static final class AutoConstants {
    private AutoConstants() {}

    public static final double kMaxSpeedMetersPerSecond = 3.0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3.0;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1.0;
    public static final double kPYController = 1.0;
    public static final double kPThetaController = 1.0;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond,
            kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    private NeoMotorConstants() {}

    public static final double kFreeSpeedRpm = 5676.0;
  }

  public static final class ShooterConstants {
    private ShooterConstants() {}

    // Hardware
    public static final int kLeftShooterCanId = 31;
    public static final int kRightShooterCanId = 32;
    public static final int kHoodMotorCanId = 33;
    public static final double kHoodGearRatio = 225.0;

    // Basic shooter targets
    public static final double kShooterFullRpm = 4200.0;
    public static final double kShooterSlowRpm = 2800.0;
    public static final double kShooterMaxRpm = 6000.0;

    // Shooter velocity loop
    public static final double kShooterVelocityP = 0.35;
    public static final double kShooterVelocityI = 0.0;
    public static final double kShooterVelocityD = 0.0;
    public static final double kShooterVelocityV = 0.16;
    public static final double kShooterVelocityS = 0.0;

    // Main hub / hopper distance-to-RPM table
    public static final double[] kShooterDistanceMeters = {1.5, 2.0, 2.5, 3.5, 4.0, 4.5,5.0};
    public static final double[] kShooterDistanceRpm = {1600.0, 1675.0, 1845.0, 2200.0, 2700.0, 3300.0, 3900.0};

    // Side-shot POV distance-to-RPM table
    public static final double[] kSideShotDistanceMeters = {2.0, 6.0, 10.0};
    public static final double[] kSideShotDistanceRpm = {2200.0, 3800.0, 4700.0};

    // Distance-based shooter smoothing
    public static final double kAutoShooterDistanceFilterAlpha = 0.01;
    public static final double kAutoShooterDistanceDeadbandMeters = 0.05;
    public static final double kAutoShooterRpmSlewRatePerSec = 10000.0;
    public static final double kAutoShooterMinCommandStepRpm = 1.0;
    public static final double kAutoShooterUpdatePeriodSeconds = 0.02;

    // Main hopper shot voltage compensation. This scales RPM by 12V / battery voltage.
    public static final double kShooterNominalVoltage = 12.0;
    public static final double kShooterVoltageCompMinVoltage = 8.0;

    // Hood mechanical limits
    public static final double kMinHoodAngleDegrees = 0.0;
    public static final double kMaxHoodAngleDegrees = 45.0;
    public static final double kHoodCommandToleranceDegrees = 1.5;

    // Hood closed-loop tuning
    public static final double kHoodP = 4.0;
    public static final double kHoodI = 0.0;
    public static final double kHoodD = 0.0;
    public static final double kHoodMaxClosedLoopOutput = 0.55;

    // Hood current limits
    public static final double kHoodRunSupplyCurrentLimitAmps = 5.0;
    public static final double kHoodRunStatorCurrentLimitAmps = 6.0;
    public static final double kHoodHomingSupplyCurrentLimitAmps = 1.25;
    public static final double kHoodHomingStatorCurrentLimitAmps = 3.0;

    // Hood auto-zero
    public static final boolean kAutoZeroHoodOnStartup = true;
    public static final double kHoodStartupZeroDegrees = 0.0;
    public static final double kHoodAutoZeroPercentOutput = -0.30;
    public static final double kHoodAutoZeroVelocityRps = 3.0;
    public static final double kHoodAutoZeroVelocityP = 0.25;
    public static final double kHoodAutoZeroVelocityI = 0.0;
    public static final double kHoodAutoZeroVelocityD = 0.0;
    public static final double kHoodAutoZeroVelocityV = 0.12;
    public static final double kHoodAutoZeroCurrentThresholdAmps = 2.25;
    public static final double kHoodAutoZeroCurrentDebounceSeconds = 0.15;
    public static final double kHoodAutoZeroMinRunSeconds = 0.20;

    // Manual hood control
    public static final double kHoodManualPercentOutput = 0.20;

    // Hood auto-angle table for main shot
    public static final double[] kHoodDistanceMeters = {1.5, 2.5, 3.86, 4.93};
    public static final double[] kHoodAngleDegrees = {0.0, 0.0, 0.0, 0.0};

    // Hood distance-based smoothing
    public static final double kAutoHoodDistanceFilterAlpha = 0.03;
    public static final double kAutoHoodDistanceDeadbandMeters = 0.10;
    public static final double kAutoHoodAngleSlewRateDegPerSec = 12.0;
    public static final double kAutoHoodMinCommandStepDeg = 0.2;
    public static final double kAutoHoodUpdatePeriodSeconds = 0.05;
  }

  public static final class VisionConstants {
    private VisionConstants() {}

    // PhotonVision camera names
    public static final String kLeftCameraName = "LeftSideCamera";
    public static final String kRightCameraName = "RightSideCamera";
    public static final String kFrontSideCameraName = "FrontSideCamera";
    public static final String kBackCameraName = "BackSideCamera";

    // Camera mount poses in robot frame
    public static final double kLeftCameraOffsetX = 0.159;
    public static final double kLeftCameraOffsetY = 0.374;
    public static final double kLeftCameraOffsetZ = 0.7112;
    public static final double kLeftCameraRotX = 0.0;
    public static final double kLeftCameraRotY = 0.0;
    public static final double kLeftCameraRotZ = 1.5708;

    public static final double kRightCameraOffsetX = 0.001;
    public static final double kRightCameraOffsetY = -0.374;
    public static final double kRightCameraOffsetZ = 0.7112;
    public static final double kRightCameraRotX = 0.0;
    public static final double kRightCameraRotY = 0.0;
    public static final double kRightCameraRotZ = -1.5708;

    public static final double kFrontSideCameraOffsetX = 0.5;
    public static final double kFrontSideCameraOffsetY = 0.0;
    public static final double kFrontSideCameraOffsetZ = 0.7112;
    public static final double kFrontSideCameraRotX = 0.0;
    public static final double kFrontSideCameraRotY = 0.0;
    public static final double kFrontSideCameraRotZ = 0.0;

    public static final double kBackCameraOffsetX = -0.057;
    public static final double kBackCameraOffsetY = 0.191;
    public static final double kBackCameraOffsetZ = 0.635;
    public static final double kBackCameraRotX = 0.0;
    public static final double kBackCameraRotY = 0.0;
    public static final double kBackCameraRotZ = 3.1415;

    // Vision pose filtering
    public static final double kMaxDistanceMeters = 100;
    public static final double kMaxAmbiguity = 0.35;
    public static final double kVisionStdDevCloseXY = 0.5;
    public static final double kVisionStdDevCloseTheta = 0.8;
    public static final double kVisionStdDevFarXY = 3.0;
    public static final double kVisionStdDevFarTheta = 6.0;

    // Main hopper / hub target definition
    public static final int kBlueHopperTagId = 26;
    public static final int kRedHopperTagId = 10;
    public static final double kHopperCenterOffsetForwardMeters = Units.inchesToMeters(-24.0);
    public static final double kHopperCenterOffsetLeftMeters = 0.0;

    // Driver POV left / right alternate shot targets
    public static final int kBlueLeftSideShotReferenceTagId = 26;
    public static final int kRedLeftSideShotReferenceTagId = 10;
    public static final int kBlueRightSideShotReferenceTagId = 26;
    public static final int kRedRightSideShotReferenceTagId = 10;
    public static final double kSideShotOffsetForwardMeters = Units.inchesToMeters(120.0);
    public static final double kLeftSideShotOffsetLeftMeters = Units.inchesToMeters(-100.0);
    public static final double kRightSideShotOffsetLeftMeters = Units.inchesToMeters(100.0);
    public static final double kSideShotHoodAngleDegrees = 42.5;

    // Auto-align reference point and shooter axis
    public static final double kAutoAlignCenterShiftForwardMeters = Units.inchesToMeters(-7.0);
    public static final double kAutoAlignCenterShiftLeftMeters = Units.inchesToMeters(7.0);
    public static final double kShooterAxisForwardMeters = 0.0;
    public static final double kShooterAxisLeftMeters = 1.0;

    public static double getShooterAxisAngleDegrees() {
      return Math.toDegrees(Math.atan2(kShooterAxisLeftMeters, kShooterAxisForwardMeters));
    }

    // Aim / auto-align PID
    public static final double kAimP = 0.03;
    public static final double kAimI = 0.0;
    public static final double kAimD = 0.002;
    public static final double kAimingToleranceDegrees = 0.0;
    public static final double kAimingTimeoutSeconds = 2.0;
    public static final double kAimRotationSign = 1.0;
    public static final double kAimOrbitJiggleDelaySeconds = 1.0;
    public static final double kAimOrbitJiggleDistanceMeters = Units.inchesToMeters(2
    );
    public static final double kAimOrbitJiggleHalfCycleSeconds = .2;

    // Moving-shot lead compensation
    public static final double kBallExitSpeedMetersPerSecond = 9.0;
    public static final double kShotLeadGain = 3.0;
    public static final double kMaxShotLeadDegrees = 30.0;

  }
}
