// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

  Compressor pcmCompressor;
  DoubleSolenoid exampleDoublePCM;
  Joystick m_joystick;

  static final boolean k_SOLENOID_CONNECTED = false;

  @Override
  public void robotInit() {
    pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
    m_joystick = new Joystick(0);

    if (k_SOLENOID_CONNECTED) {
      exampleDoublePCM = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);
    }
  }


  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {

    boolean pressureSwitch = pcmCompressor.getPressureSwitchValue();
    double current = pcmCompressor.getCurrent();

    SmartDashboard.putNumber("Compressor Current", current);
    SmartDashboard.putBoolean("Pressure Switch", pressureSwitch);

    if (k_SOLENOID_CONNECTED) {
      if (m_joystick.getRawButton(7)) exampleDoublePCM.set(DoubleSolenoid.Value.kForward);
      if (m_joystick.getRawButton(9)) exampleDoublePCM.set(DoubleSolenoid.Value.kOff);
      if (m_joystick.getRawButton(11)) exampleDoublePCM.set(DoubleSolenoid.Value.kReverse);
    }
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
