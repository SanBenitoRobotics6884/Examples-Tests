// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

  private WPI_TalonSRX m_lFront = new WPI_TalonSRX(Constants.DriveConstants.kFrontLeftMotorID);
  private WPI_TalonSRX m_lRear = new WPI_TalonSRX(Constants.DriveConstants.kRearLeftMotorID);
  private WPI_TalonSRX m_rFront = new WPI_TalonSRX(Constants.DriveConstants.kFrontRightMotorID);
  private WPI_TalonSRX m_rRear = new WPI_TalonSRX(Constants.DriveConstants.kRearRightMotorID);
  private MecanumDrive m_drive = new MecanumDrive(m_lFront, m_lRear, m_rFront, m_rRear);

  private double m_maxSpeed = Constants.DriveConstants.kMaxSpeedSlow;

  public DriveSubsystem() {
    m_lFront.configFactoryDefault();
    m_lRear.configFactoryDefault();
    m_rFront.configFactoryDefault();
    m_rRear.configFactoryDefault();
  }

  public void drive (double x, double y, double z) {
    m_drive.driveCartesian(y * m_maxSpeed, x * m_maxSpeed, z);
  }

  public void setMaxSpeed (double speed) {
    m_maxSpeed = speed;
  }

  @Override
  public void periodic() {}

}
