/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

  private static double kMaxSpeed = 0.5;
  private static boolean isFieldCentric = false;

  private Joystick m_stick;
  private CANSparkMax m_lFront;
  private CANSparkMax m_lRear;
  private CANSparkMax m_rFront;
  private CANSparkMax m_rRear;
  private MecanumDrive m_drive;
  private ADXRS450_Gyro gyro;

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
    gyro = new ADXRS450_Gyro();
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putData("Drive", m_drive);
    SmartDashboard.putNumber("Gyro", gyro.getAngle());
  }

  @Override
  public void teleopPeriodic() {

    if (m_stick.getRawButton(5)) {
      isFieldCentric = false;
    } else if (m_stick.getRawButton(6)) {
      isFieldCentric = true;
    }

    if (m_stick.getRawButton(1)) {
      gyro.calibrate();
    }

    double forw = -m_stick.getRawAxis(1) * Math.abs(m_stick.getRawAxis(1)) * kMaxSpeed;
    double strafe = m_stick.getRawAxis(0) * Math.abs(m_stick.getRawAxis(0))  * kMaxSpeed;
    double rot = m_stick.getRawAxis(4) * Math.abs(m_stick.getRawAxis(4))  * kMaxSpeed;

    if (!isFieldCentric) {
      m_drive.driveCartesian(forw, strafe, rot);
    } else {
      m_drive.driveCartesian(forw, strafe, rot, gyro.getAngle());
    }
  }
}