/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

  private static double kMaxSpeed = 0.4;
  private static double kMaxRotTarget = 100; //deg per second
  private static double kP = 0.01;

  private Joystick m_stick;
  private CANSparkMax m_lFront;
  private CANSparkMax m_lRear;
  private CANSparkMax m_rFront;
  private CANSparkMax m_rRear;
  private MecanumDrive m_drive;
  private ADIS16448_IMU m_gyro;

  @Override
  public void robotInit() {
    // initialize SPARK MAX
    m_lFront = new CANSparkMax(1, MotorType.kBrushless);
    m_lRear = new CANSparkMax(2, MotorType.kBrushless);
    m_rFront = new CANSparkMax(3, MotorType.kBrushless);
    m_rRear = new CANSparkMax(4, MotorType.kBrushless);
    
    m_lFront.restoreFactoryDefaults();
    m_lRear.restoreFactoryDefaults();
    m_rFront.restoreFactoryDefaults();
    m_rRear.restoreFactoryDefaults();

    m_rFront.setInverted(true);
    m_rRear.setInverted(true);

    m_stick = new Joystick(0);
    m_drive = new MecanumDrive(m_lFront, m_lRear, m_rFront, m_rRear);

    m_gyro = new ADIS16448_IMU(ADIS16448_IMU.IMUAxis.kZ, SPI.Port.kMXP, ADIS16448_IMU.CalibrationTime._1s);
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putData("Drive", m_drive);
    SmartDashboard.putData("Gyro", m_gyro);
    SmartDashboard.putNumber("Z Heading", m_gyro.getGyroAngleZ());
    SmartDashboard.putNumber("Z Rate", m_gyro.getGyroRateZ());
  }

  @Override
  public void teleopPeriodic() {

    double zRate = m_gyro.getGyroRateZ();
    double targetRate = m_stick.getRawAxis(4) * kMaxRotTarget;
    double error = targetRate - zRate;

    double forw = m_stick.getRawAxis(1) * kMaxSpeed;
    double strafe = m_stick.getRawAxis(0) * kMaxSpeed;
    double rot = -error * kP;
    m_drive.driveCartesian(forw, strafe, rot);
  }
}