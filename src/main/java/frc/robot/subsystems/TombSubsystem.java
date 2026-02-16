// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
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
  // Defensive: ensure IdleMode is Brake on each tomb controller
  m_frontTomb.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake), ResetMode.kResetSafeParameters,
    PersistMode.kPersistParameters);
  m_backTomb.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake), ResetMode.kResetSafeParameters,
    PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
    if (m_feederClosedLoopController != null) {
      m_feederClosedLoopController.setSetpoint(rpm, ControlType.kVelocity);
    } else {
      // Fallback: if closed-loop controller not available, use open-loop percent output
      m_feederTomb.set(Math.copySign(0.5, rpm));
    }
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
