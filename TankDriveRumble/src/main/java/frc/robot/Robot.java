// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

  static final double kMaxSpeed = 0.7;
  static final double kSlowSpeed = 0.5;
  static final double kRumblePulseWidth = 0.3; // Duration of rumble pulse
  static final double kRumblePulseRate = 3;
  static final double kRumbleStrength = 1;
  static final double kTurnBonusScalar = 1.2;
  static final double kAccelerationRumbleThreshold = 8.5;
  static final double kCurrentRatioRumbleThreshold = 3;
  static final int kDrivePowerChannels[] = {0,1,14,15};

  double netAccelertion = 0;
  double speed = kSlowSpeed;

  WPI_TalonSRX frontLeft = new WPI_TalonSRX(1);
  WPI_TalonSRX frontRight = new WPI_TalonSRX(2);
  WPI_VictorSPX backLeft = new WPI_VictorSPX(10);
  WPI_VictorSPX backRight = new WPI_VictorSPX(20);

  PowerDistribution m_pdp = new PowerDistribution();
  ADIS16470_IMU m_gyro = new ADIS16470_IMU();
  DifferentialDrive m_Drive = new DifferentialDrive(frontLeft, frontRight);
  XboxController m_controller = new XboxController(0); 

  @Override
  public void robotInit() {
    frontLeft.configFactoryDefault();
    frontRight.configFactoryDefault();
    backLeft.configFactoryDefault();
    backRight.configFactoryDefault();
    frontRight.setInverted(true);
    backRight.follow(frontRight);
    backLeft.follow(frontLeft);
    backRight.setInverted(InvertType.FollowMaster);
    CameraServer.startAutomaticCapture(0);
  }


  @Override
  public void robotPeriodic() {

    // Variable max speed
    if(m_controller.getLeftBumper()){
      speed = kSlowSpeed;
    }
    if (m_controller.getRightBumper()){
      speed = kMaxSpeed;
    }

    // Calculate net acceleration (all directions) and subtract acc due to gravity
    netAccelertion = Math.sqrt(Math.pow(m_gyro.getAccelX(), 2) +
      Math.pow(m_gyro.getAccelY(), 2) + Math.pow(m_gyro.getAccelZ(), 2))-9.8;
    netAccelertion = Math.abs(netAccelertion);

    // Deadband net acceleration for random fluctuations
    if (netAccelertion < 0.05){
      netAccelertion = 0; 
    }

    // Calculate avg current from drive motors only
    double avgCurrent = ( m_pdp.getCurrent(kDrivePowerChannels[0]) +
                          m_pdp.getCurrent(kDrivePowerChannels[1]) +
                          m_pdp.getCurrent(kDrivePowerChannels[2]) +
                          m_pdp.getCurrent(kDrivePowerChannels[3])
                        ) / 4;
    double currentRatio = (avgCurrent / netAccelertion);

    // Pulse if robot is pushing against obstacle or other robot
    boolean isPulsing;
    if (currentRatio > kCurrentRatioRumbleThreshold) {
      isPulsing = true;
    } else {
      isPulsing = false;
    }

    //If pulsing, alternate between full rumble and no rumble
    if (isPulsing) {
      double pulseTime = (Timer.getFPGATimestamp() * kRumblePulseRate) % 1;

      if (pulseTime < kRumblePulseWidth) {
        m_controller.setRumble(RumbleType.kLeftRumble, kRumbleStrength);
      m_controller.setRumble(RumbleType.kRightRumble, kRumbleStrength);
      } else {
        m_controller.setRumble(RumbleType.kLeftRumble, -kRumbleStrength);
        m_controller.setRumble(RumbleType.kRightRumble, -kRumbleStrength);
      }
    //Otherwise rumble when past the acceleration threshold
    } else {
      if (netAccelertion > kAccelerationRumbleThreshold) {
        m_controller.setRumble(RumbleType.kLeftRumble, kRumbleStrength);
        m_controller.setRumble(RumbleType.kRightRumble, kRumbleStrength);
      } else {
        m_controller.setRumble(RumbleType.kLeftRumble, 0);
        m_controller.setRumble(RumbleType.kRightRumble, 0);
      }
    }

    SmartDashboard.putNumber("Acceleration", netAccelertion);
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
    m_Drive.arcadeDrive(m_controller.getLeftY() * speed, m_controller.getRightX() * speed * kTurnBonusScalar);
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
