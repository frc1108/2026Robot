package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ModuleConstants;

public final class Configs {
    public static final class MAXSwerveModule {
        public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        static {
            // Use module constants to calculate conversion factors and feed forward gain.
            double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                    / ModuleConstants.kDrivingMotorReduction;
            double turningFactor = 2 * Math.PI;
            double nominalVoltage = 12.0;
            double drivingVelocityFeedForward = nominalVoltage / ModuleConstants.kDriveWheelFreeSpeedRps;

            drivingConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50);
            drivingConfig.encoder
                .positionConversionFactor(drivingFactor) // meters
                .velocityConversionFactor(drivingFactor / 60.0); // meters per second
            drivingConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // These are example gains you may need to them for your own robot!
                .pid(0.04, 0, 0)
                .outputRange(-1, 1)
                .feedForward.kV(drivingVelocityFeedForward);

            turningConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(20);

            turningConfig.absoluteEncoder
                // Invert the turning encoder, since the output shaft rotates in the opposite
                // direction of the steering motor in the MAXSwerve Module.
                .inverted(true)
                .positionConversionFactor(turningFactor) // radians
                .velocityConversionFactor(turningFactor / 60.0) // radians per second
                // This applies to REV Through Bore Encoder V2 (use REV_ThroughBoreEncoder for V1):
                .apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoderV2);
            turningConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(1, 0, 0)
                .outputRange(-1, 1)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0, turningFactor);
        }
    }
    public static final class Intake {
        public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();
        public static final SparkFlexConfig vortexIntakeConfig = new SparkFlexConfig();

        static{
            intakeConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(35)
                .voltageCompensation(10);
            vortexIntakeConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(80)
                .voltageCompensation(10);
        }
    }
    public static final class Feeder {
        public static final SparkFlexConfig feederConfig = new SparkFlexConfig();

        static{
            feederConfig
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(80)
                .voltageCompensation(10);
        }
    }
    public static final class Shooter {
        public static final SparkMaxConfig hoodConfig = new SparkMaxConfig();

        static {
            hoodConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(20)
                .voltageCompensation(10);
            hoodConfig.encoder
                .positionConversionFactor(360.0 / Constants.ShooterConstants.kHoodGearRatio)
                .velocityConversionFactor(360.0 / (Constants.ShooterConstants.kHoodGearRatio * 60.0));
            hoodConfig.closedLoop
                .pid(Constants.ShooterConstants.kHoodP, Constants.ShooterConstants.kHoodI, Constants.ShooterConstants.kHoodD)
                .outputRange(-Constants.ShooterConstants.kHoodMaxClosedLoopOutput,
                              Constants.ShooterConstants.kHoodMaxClosedLoopOutput);
        }
    }
}
