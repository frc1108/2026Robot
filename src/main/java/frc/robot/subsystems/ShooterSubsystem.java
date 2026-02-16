// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
@Logged
public class ShooterSubsystem extends SubsystemBase {
  final TalonFX m_leftShooter = new TalonFX(31, "rio");
  final TalonFX m_rightShooter = new TalonFX(32, "rio");
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
private void setShooterPower(double power) {
  m_leftShooter.set(power);
  m_rightShooter.set(-power);
}

public Command shootCommand() {
return this.startEnd( () -> this.setShooterPower(0.10), () -> this.setShooterPower(0.0));
}

}
