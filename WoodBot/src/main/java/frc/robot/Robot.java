// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {
  
  Joystick m_joystick = new Joystick(0);
  WPI_VictorSPX m_LowerPulley = new WPI_VictorSPX(10);
  CANSparkMax m_IntakeMotor = new CANSparkMax(5, MotorType.kBrushless);
  WPI_TalonSRX m_LauncherMotor = new WPI_TalonSRX(3);
  
  static double kMaxSpeed = 1.0;
  static final double kPulleySpeed = 0.5;
  static final double kIntakeSpeed = 0.5;
  
  @Override
  public void robotInit() {
    m_LowerPulley.configFactoryDefault();
    m_IntakeMotor.restoreFactoryDefaults();
    m_LauncherMotor.configFactoryDefault();
  }

  
  @Override
  public void robotPeriodic() {}

  
  @Override
  public void autonomousInit() {
   
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
   
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if (m_joystick.getTrigger()) {
      m_LauncherMotor.set(kMaxSpeed);
    } else {
      m_LauncherMotor.set(0);
    }
    /*make constants for lower pulley speed and intake speed*/
    if (m_joystick.getRawButton(2)) {
      m_IntakeMotor.set(kIntakeSpeed);
    } else {
      m_IntakeMotor.set(0);
    }
    if (m_joystick.getRawButton(11)) {
      m_LowerPulley.set(kPulleySpeed);
    } else {
      m_LowerPulley.set(0);
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
