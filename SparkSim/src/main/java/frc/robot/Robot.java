// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  
  private Joystick m_joystick = new Joystick(0);
  public static final int kMotorId = 1;

  //Instantiate Spark and encoder like normal
  private CANSparkMax m_motor = new CANSparkMax(kMotorId, MotorType.kBrushless);
  private RelativeEncoder m_encoder;

  @Override
  public void robotInit() {
    //Configure devices like normal
    m_motor.restoreFactoryDefaults();
    m_encoder = m_motor.getEncoder();
  }

  @Override
  public void simulationInit() {
    //Add each spark max to REVPhysicsSim
    REVPhysicsSim.getInstance().addSparkMax(m_motor, DCMotor.getNEO(1));
  }

  @Override
  public void simulationPeriodic() {
    //Run simulation
    REVPhysicsSim.getInstance().run();
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    m_motor.setVoltage(m_joystick.getRawAxis(1));
    SmartDashboard.putNumber("Encoder", m_encoder.getVelocity());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}
