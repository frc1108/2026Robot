// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
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

  /** Creates a new TombSubsystem. */
  public TombSubsystem() {
    m_frontTomb.configure(
        Configs.Intake.intakeConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_backTomb.configure(
        Configs.Intake.intakeConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
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

  public Command tomb() {
    return this.startEnd(
        () -> {
          this.setFrontTombPower(TombConstants.frontTombSpeed);
          this.setBackTombPower(TombConstants.backTombSpeed);
            
        },
        () -> {
          this.setFrontTombPower(0.0);
          this.setBackTombPower(0.0);
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
}
