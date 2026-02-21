// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.TombConstants;

public class TombSubsystem extends SubsystemBase {
  final SparkMax m_frontTomb = new SparkMax(TombConstants.frontTombCanId, MotorType.kBrushless);
  final SparkMax m_backTomb = new SparkMax(TombConstants.backTombCanId, MotorType.kBrushless);
  final SparkMax m_feederTomb = new SparkMax(TombConstants.feederTombCanId, MotorType.kBrushless);
  private SparkClosedLoopController m_feederClosedLoopController;
  private RelativeEncoder m_feederEncoder;
  private int m_periodicCounter = 0;
  // By default use open-loop percent output for the feeder (maps RPM -> percent)
  // This avoids requiring PID tuning; closed-loop can be enabled later.
  private boolean m_useFeederClosedLoop = false;

  /** Creates a new TombSubsystem. */
  public TombSubsystem() {
    m_frontTomb.configure(
        Configs.Intake.intakeConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_feederTomb.configure(
        Configs.Feeder.feederConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  m_backTomb.configure(
    Configs.Intake.intakeConfig,
    ResetMode.kResetSafeParameters,
    PersistMode.kPersistParameters);
  // Initialize closed-loop controller for the feeder motor
  m_feederClosedLoopController = m_feederTomb.getClosedLoopController();
  m_feederEncoder = m_feederTomb.getEncoder();
  // Defensive: ensure IdleMode is Brake on each tomb controller
  m_frontTomb.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake), ResetMode.kResetSafeParameters,
    PersistMode.kPersistParameters);
  m_backTomb.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake), ResetMode.kResetSafeParameters,
    PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Periodically print feeder status (throttled)
    m_periodicCounter++;
    if (m_periodicCounter % 50 == 0) {
      double vel = Double.NaN;
      try {
        vel = m_feederEncoder.getVelocity();
      } catch (Exception e) {
        // ignore if encoder not available
      }
      System.out.println(String.format("Feeder status - encoderVel=%.2f rpm, closedLoop=%b", vel, m_useFeederClosedLoop));
    }
  }

  private void setFrontTombPower(double power) {
    
    m_frontTomb.set(power);
  }
  private void setBackTombPower(double power) {
    m_backTomb.set(power);

  }
    private void setFeederTombPower(double power) {
    m_feederTomb.set(power);

  }

  /** Set feeder target velocity in RPM using the SPARK MAX closed-loop controller. */
  public void setFeederVelocityRpm(double rpm) {
    if (m_useFeederClosedLoop && m_feederClosedLoopController != null) {
      m_feederClosedLoopController.setSetpoint(rpm, ControlType.kVelocity);
    } else {
      // Open-loop: map requested RPM to a percent of free speed of the motor
      double freeRpm = frc.robot.Constants.NeoMotorConstants.kFreeSpeedRpm;
      double percent = 0.0;
      if (rpm != 0.0 && freeRpm != 0.0) {
        percent = rpm / freeRpm;
        // clamp
        if (percent > 1.0) percent = 1.0;
        if (percent < -1.0) percent = -1.0;
      }
      m_feederTomb.set(percent);
    }
  }

  /** Enable or disable closed-loop feeder control at runtime. */
  public void enableFeederClosedLoop(boolean enabled) {
    m_useFeederClosedLoop = enabled;
    System.out.println("Feeder closed-loop set to " + enabled);
  }

  public Command tomb() {
    return this.startEnd(
        () -> {
          // Debug/logging and run feeder at configured velocity
          System.out.println("Tomb command START");
          this.setFrontTombPower(TombConstants.frontTombSpeed);
          this.setBackTombPower(TombConstants.backTombSpeed);
          // Run feeder in closed-loop velocity (if configured)
          this.setFeederVelocityRpm(TombConstants.feederTombVelocityRpm);

        },
        () -> {
          // Stop all tomb motors
          System.out.println("Tomb command END");
          this.setFrontTombPower(0.0);
          this.setBackTombPower(0.0);
          this.setFeederVelocityRpm(0.0);
        });
  }

    public Command reverseTomb() {
    return this.startEnd(
        () -> {
          this.setFrontTombPower(TombConstants.reverseFrontTombSpeed);
          this.setBackTombPower(TombConstants.reverseBackTombSpeed);
            
        },
        () -> {
          this.setFrontTombPower(0.0);
          this.setBackTombPower(0.0);
        });
  }

      public Command feederTomb() {
    return this.startEnd(
        () -> {
          this.setFeederTombPower(TombConstants.feederTombSpeed);
            
        },
        () -> {
          this.setFeederTombPower(0.0);
        });
  }

    /** Command that runs feeder at configured velocity (closed-loop) while held. */
    public Command feederVelocityCommand() {
      return this.startEnd(
          () -> this.setFeederVelocityRpm(TombConstants.feederTombVelocityRpm),
          () -> this.setFeederVelocityRpm(0.0));
    }

}
