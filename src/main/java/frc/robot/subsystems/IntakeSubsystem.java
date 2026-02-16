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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  final SparkMax m_frontIntake = new SparkMax(IntakeConstants.frontIntakecanid, MotorType.kBrushless);
  final SparkMax m_backIntake = new SparkMax(IntakeConstants.backIntakecanid, MotorType.kBrushless);
  private SparkClosedLoopController m_frontClosedLoopController;
  private SparkClosedLoopController m_backClosedLoopController;
  private RelativeEncoder m_frontEncoder;
  private RelativeEncoder m_backEncoder;
  private boolean m_useIntakeClosedLoop = false; // default to open-loop for safety
  private int m_periodicCounter = 0;
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    m_frontIntake.configure(
      Configs.Intake.intakeConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
      m_backIntake.configure(
        Configs.Intake.intakeConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
      // Defensive: explicitly ensure IdleMode is Brake on each controller
      m_frontIntake.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake), ResetMode.kResetSafeParameters,
          PersistMode.kPersistParameters);
      m_backIntake.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake), ResetMode.kResetSafeParameters,
          PersistMode.kPersistParameters);
    // Initialize closed-loop controllers and encoders for intake motors
    m_frontClosedLoopController = m_frontIntake.getClosedLoopController();
    m_backClosedLoopController = m_backIntake.getClosedLoopController();
    m_frontEncoder = m_frontIntake.getEncoder();
    m_backEncoder = m_backIntake.getEncoder();
      
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_periodicCounter++;
    if (m_periodicCounter % 100 == 0) {
      double fvel = Double.NaN;
      double bvel = Double.NaN;
      try {
        fvel = m_frontEncoder.getVelocity();
        bvel = m_backEncoder.getVelocity();
      } catch (Exception ignored) {
      }
      System.out.println(String.format("Intake status - frontVel=%.2f rpm backVel=%.2f rpm closedLoop=%b", fvel, bvel,
          m_useIntakeClosedLoop));
    }
  }
// Open-loop helpers removed; intake commands now use velocity APIs
  /** Set intake velocity in RPM for both motors (closed-loop when enabled). */
  public void setIntakeVelocityRpm(double rpm) {
    // Back-motor uses opposite sign by convention; map to front/back separately
    setIntakeVelocityRpm(rpm, -rpm);
  }

  /**
   * Set intake velocity in RPM for front and back motors separately.
   * This lets you tune front/back independently using constants in IntakeConstants.
   */
  public void setIntakeVelocityRpm(double frontRpm, double backRpm) {
    if (m_useIntakeClosedLoop && m_frontClosedLoopController != null && m_backClosedLoopController != null) {
      m_frontClosedLoopController.setSetpoint(frontRpm, ControlType.kVelocity);
      m_backClosedLoopController.setSetpoint(backRpm, ControlType.kVelocity);
    } else {
      // Open-loop mapping: convert desired RPM to percent of motor free speed
      double freeRpm = frc.robot.Constants.NeoMotorConstants.kFreeSpeedRpm;
      double fPercent = 0.0;
      double bPercent = 0.0;
      if (frontRpm != 0.0 && freeRpm != 0.0) {
        fPercent = frontRpm / freeRpm;
      }
      if (backRpm != 0.0 && freeRpm != 0.0) {
        bPercent = backRpm / freeRpm;
      }
      // clamp
      fPercent = Math.max(-1.0, Math.min(1.0, fPercent));
      bPercent = Math.max(-1.0, Math.min(1.0, bPercent));
      m_frontIntake.set(fPercent);
      m_backIntake.set(bPercent);
    }
  }

  /** Enable/disable intake closed-loop velocity control at runtime. */
  public void enableIntakeClosedLoop(boolean enabled) {
    m_useIntakeClosedLoop = enabled;
    System.out.println("Intake closed-loop set to " + enabled);
  }
public Command intake() {
  return this.startEnd(
    () -> {
      // Run intake at configured front/back velocities
  this.setIntakeVelocityRpm(IntakeConstants.intakeFrontVelocityRpm, IntakeConstants.intakeBackVelocityRpm);
    },
    () -> {
      this.setIntakeVelocityRpm(300.0, -300.0);
    });
}
  
public Command reverseIntake() {
  return this.startEnd(
    () -> {
    this.setIntakeVelocityRpm(IntakeConstants.reverseIntakeFrontVelocityRpm,
      IntakeConstants.reverseIntakeBackVelocityRpm);
    },
    () -> {
      this.setIntakeVelocityRpm(300.0, -300.0);
    });
}

public Command slowIntake() {
  return this.startEnd(
    () -> {
  this.setIntakeVelocityRpm(IntakeConstants.slowIntakeFrontVelocityRpm, IntakeConstants.slowIntakeBackVelocityRpm);
    },
    () -> {
      this.setIntakeVelocityRpm(300.0, -300.0);
    });
}

  /** Command that runs both intake motors at the configured intakeVelocityRpm. */
  public Command intakeVelocityCommand() {
  return this.startEnd(
    () -> this.setIntakeVelocityRpm(IntakeConstants.intakeFrontVelocityRpm, IntakeConstants.intakeBackVelocityRpm),
    () -> this.setIntakeVelocityRpm(300.0, -300.0));
  }

}
