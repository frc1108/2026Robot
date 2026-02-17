package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
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
                    // These are example gains you may need to them for your own robot!
                    .pid(1, 0, 0)
                    .outputRange(-1, 1)
                    // Enable PID wrap around for the turning motor. This will allow the PID
                    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                    // to 10 degrees will go through 0 rather than the other direction which is a
                    // longer route.
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, turningFactor);
        }
    }
    public static final class Intake {
                public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();

        static{
        intakeConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(30).voltageCompensation(10);
    }}

    public static final class Feeder {
                public static final SparkMaxConfig feederConfig = new SparkMaxConfig();

        static{
        feederConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(30).voltageCompensation(10);
    }}

        public static final class Shooter {
                public static final SparkMaxConfig hoodConfig = new SparkMaxConfig();

                static {
                        // Configure hood encoder conversions (rotations -> degrees)
                        hoodConfig.encoder
                                .positionConversionFactor(360.0 / Constants.ShooterConstants.kHoodGearRatio)
                                .velocityConversionFactor(360.0 / (Constants.ShooterConstants.kHoodGearRatio * 60.0));

                        // Closed-loop PID for hood
                        hoodConfig.closedLoop
                                .pid(Constants.ShooterConstants.kHoodP, Constants.ShooterConstants.kHoodI, Constants.ShooterConstants.kHoodD)
                                .outputRange(-1.0, 1.0);

                        // Use brake idle for hood motor
                        hoodConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20).voltageCompensation(10);
                }
        }

}
