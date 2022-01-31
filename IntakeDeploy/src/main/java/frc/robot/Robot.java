// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

  private static int kMotorID = 3;
  private static double kSpeed = 0.2;
  private static double kSetpointRetract = -0.05;
  private static double kSetpointDeploy = 0.2;
  private static double setpoint = 0;
  private static boolean kInvert = true;

  private static double kP = 2;
  private static double kI = 0;
  private static double kD = 0;

  private Joystick m_controller = new Joystick(0);
  private WPI_TalonSRX m_motor = new WPI_TalonSRX(kMotorID);
  Encoder encoder = new Encoder(0, 1);
  PIDController pid = new PIDController(kP, kI, kD);

  

  @Override
  public void robotInit() {
    m_motor.configFactoryDefault();
    m_motor.setInverted(kInvert);
     // Also changes the units of getRate
  encoder.setDistancePerPulse(1./2048.);

  // Configures the encoder to consider itself stopped after .1 seconds
  encoder.setMaxPeriod(.1);

  // Configures the encoder to consider itself stopped when its rate is below 10
  encoder.setMinRate(2);

  // Reverses the direction of the encoder
  encoder.setReverseDirection(false);

  // Configures an encoder to average its period measurement over 5 samples
  // Can be between 1 and 127 samples
  encoder.setSamplesToAverage(5);
  }

  @Override
  public void robotPeriodic() {
  }

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
    if (m_controller.getRawButton(1)) {
      m_motor.set(pid.calculate(encoder.getDistance(), setpoint));
      //m_motor.set(m_controller.getY());
    } else {
      m_motor.set(0);
    }

    if (m_controller.getRawButton(7)) {
      setpoint = kSetpointRetract;
    } else if (m_controller.getRawButton(8)) {
      setpoint = kSetpointDeploy;
    }

    if (m_controller.getRawButton(2)) {
      encoder.reset();
    }

    SmartDashboard.putNumber("Encoder Position", encoder.getDistance());
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
}
