// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DefaultDrive extends CommandBase {

  private DriveSubsystem m_driveSubsystem;
  private DoubleSupplier m_forw;
  private DoubleSupplier m_strafe;
  private DoubleSupplier m_twist;

  public DefaultDrive(DriveSubsystem subsystem, DoubleSupplier forw, DoubleSupplier strafe, DoubleSupplier twist) {
    m_forw = forw;
    m_strafe = strafe;
    m_twist = twist;

    m_driveSubsystem = subsystem;
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveSubsystem.drive(m_strafe.getAsDouble(), m_forw.getAsDouble(), m_twist.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
