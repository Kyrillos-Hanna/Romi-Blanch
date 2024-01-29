// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.sensor.RomiGyro;
import frc.robot.subsystems.RomiDrivetrain;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class MoveForwardWithPID extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private RomiDrivetrain m_db;
  private double distance;
  private RomiGyro m_RomiGyro;
  private PIDController m_PID = new PIDController(0,0,0);

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveForwardWithPID(RomiDrivetrain db, double inches, RomiGyro gyroscope) {
    m_db = db;
    distance = inches;
    m_RomiGyro = gyroscope;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(db);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_db.resetEncoders();
    m_db.arcadeDrive(0,0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_db.arcadeDrive(0, m_PID.calculate(m_RomiGyro.getAngleX(), 0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_db.arcadeDrive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_db.getAverageDistanceInch() >= distance);
  }
}
