// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.sensor.RomiGyro;
import frc.robot.subsystems.RomiDrivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;

/** An example command that uses an example subsystem. */
public class BlanchCode extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final RomiDrivetrain m_db;
  PIDController m_PIDController = new PIDController(0, 0, 0);
  private final RomiGyro m_gyro;
  private boolean onRamp = false;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public BlanchCode(RomiDrivetrain db, RomiGyro gyro) {
    m_db = db;
    m_gyro = gyro;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(db);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_gyro.reset();
    m_db.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    while (m_gyro.getAngleY() < 0) {
      m_db.arcadeDrive(0.3, 0);
    }

    if (m_gyro.getAngleY() > 0) {
      onRamp = true;
    }

    while (m_gyro.getAngleY() >= 0 && onRamp) {
      m_db.arcadeDrive(m_PIDController.calculate(m_gyro.getAngleY(), 0), 0);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_db.arcadeDrive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_gyro.getAngleY() == 0 && onRamp);
  }
}
