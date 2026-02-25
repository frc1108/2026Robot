// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

@Logged
public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX m_leftShooter;
  private final TalonFX m_rightShooter;
  private HoodSubsystem m_hood;

  public ShooterSubsystem() {
    m_leftShooter = new TalonFX(ShooterConstants.kLeftShooterCanId, "rio");
    m_rightShooter = new TalonFX(ShooterConstants.kRightShooterCanId, "rio");
  }

  public void setHoodSubsystem(HoodSubsystem hood) {
    m_hood = hood;
  }

  private void setShooterPower(double power) {
    // Right motor is mounted opposite the left, so it must be inverted.
    m_leftShooter.set(power);
    m_rightShooter.set(-power);
  }

  public void stopShooter() {
    setShooterPower(0.0);
  }

  public double getShooterPower() {
    return m_leftShooter.get();
  }

  private Command runShooterAt(double power) {
    return startEnd(() -> setShooterPower(power), this::stopShooter);
  }

  public Command shootCommand() {
    return runShooterAt(ShooterConstants.kShooterFullSpeed);
  }

  public Command slowShootCommand() {
    return runShooterAt(ShooterConstants.kShooterSlowSpeed);
  }

  public Command shootWithHoodCommand(double hoodAngleDegrees) {
    if (m_hood == null) {
      return shootCommand();
    }

    return Commands.parallel(
        shootCommand(),
        m_hood.setHoodAngleCommand(hoodAngleDegrees)
    );
  }

  public Command slowShootWithHoodCommand(double hoodAngleDegrees) {
    if (m_hood == null) {
      return slowShootCommand();
    }

    return Commands.parallel(
        slowShootCommand(),
        m_hood.setHoodAngleCommand(hoodAngleDegrees)
    );
  }
}
