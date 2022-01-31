/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Robot extends TimedRobot {

  private static double kP = 0.001;
  private static double kI = 0;
  private static double kD = 0;

  private static double kSpeed = 1;
  private static double setpoint = 10;
  private static int deviceID = 5;
  private static double kCountsPerRev = 42;
  private static boolean kInverted = true;

  private double velocityRPM = 0;

  private Joystick m_stick;
  private CANSparkMax m_motor;
  private RelativeEncoder m_encoder;
  PIDController pid = new PIDController(kP, kI, kD);

  @Override
  public void robotInit() {
    // initialize SPARK MAX
    m_motor = new CANSparkMax(deviceID, MotorType.kBrushless);
    m_motor.restoreFactoryDefaults();

    m_encoder = m_motor.getEncoder();
    m_motor.setInverted(kInverted);

    m_stick = new Joystick(0);
  }

  @Override
  public void teleopPeriodic() {

    velocityRPM = -1 * m_encoder.getVelocity() / kCountsPerRev;
    if (m_stick.getTrigger()) m_motor.set(pid.calculate(m_encoder.getVelocity(), setpoint) * m_stick.getY() * 1);
    //if (m_stick.getTrigger()) m_motor.set(m_stick.getY() * kSpeed);

    SmartDashboard.putNumber("Encoder Position", m_encoder.getPosition());
    SmartDashboard.putNumber("Y", m_stick.getY());
    SmartDashboard.putNumber("Encoder Velocity", velocityRPM);
  }
}

//42