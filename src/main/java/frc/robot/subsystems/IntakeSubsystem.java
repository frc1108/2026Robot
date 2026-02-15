// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.Power;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  final SparkMax m_frontIntake = new SparkMax(IntakeConstants.frontIntakecanid, MotorType.kBrushless);
  final SparkMax m_backIntake = new SparkMax(IntakeConstants.backIntakecanid, MotorType.kBrushless);
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
      
    
  }

  @Override
  public void periodic() {}
    // This method will be called once per scheduler run
private void setFrontIntakePower(double Power) {
m_frontIntake.set(Power);

}
private void setBackIntakePower(double Power) {
m_backIntake.set(Power);

}
public Command intake() {
  return this.startEnd(
    () -> {
      this.setFrontIntakePower(IntakeConstants.frontIntakeSpeed);
      this.setBackIntakePower(IntakeConstants.backIntakeSpeed);
    },
    () -> {
      this.setFrontIntakePower(0.1);
      this.setBackIntakePower(-0.1);
    });
}
  
public Command reverseIntake() {
  return this.startEnd(
    () -> {
      this.setFrontIntakePower(IntakeConstants.reverseFrontIntakeSpeed);
      this.setBackIntakePower(IntakeConstants.reverseBackIntakeSpeed);
    },
    () -> {
      this.setFrontIntakePower(0.1);
      this.setBackIntakePower(-0.1);
    });
}

}
