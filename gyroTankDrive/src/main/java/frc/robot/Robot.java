// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.InvertType;
import edu.wpi.first.wpilibj.SPI;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  ADXRS450_Gyro gyro = new ADXRS450_Gyro();
  WPI_TalonSRX rightFront = new WPI_TalonSRX(2);
  WPI_VictorSPX rightFollower = new WPI_VictorSPX(20);
  WPI_TalonSRX leftFront = new WPI_TalonSRX(1);
  WPI_VictorSPX leftFollower = new WPI_VictorSPX(10);
  DifferentialDrive gyroTank = new DifferentialDrive(leftFront, rightFront);
  Joystick controller = new Joystick(0);
  static final double kMaxSpeed = 0.75; 
  static final double kMaxTurn = 360;
  double kP = 0.008;
  double kD = 0.00005;
  double kF = 0.1;
  PIDController pid = new PIDController(kP, 0, kD);



  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    rightFront.configFactoryDefault();
    leftFront.configFactoryDefault();
    rightFollower.configFactoryDefault();
    leftFollower.configFactoryDefault();
    rightFollower.follow(rightFront);
    leftFollower.follow(leftFront);
    rightFront.setInverted(true);
    rightFollower.setInverted(InvertType.FollowMaster);

    pid.disableContinuousInput();

    SmartDashboard.putNumber("KP", kP);
    SmartDashboard.putNumber("KD", kD);
    SmartDashboard.putNumber("KF", kF);
  }


  @Override
  public void robotPeriodic() {
    SmartDashboard.putData("Gyro", gyro);
    SmartDashboard.putNumber("Gyro Z", gyro.getRate());
    kP = SmartDashboard.getNumber("KP", 0);
    kD = SmartDashboard.getNumber("KD", 0);
    kF = SmartDashboard.getNumber("KF", 0);
    pid.setP(kP);
    pid.setD(kD);
  }

  @Override
  public void autonomousInit() {
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double target = controller.getX()*kMaxTurn;
    double forw = kMaxSpeed * controller.getRawAxis(1); /* positive is forward */
    double turnManual = kMaxSpeed * controller.getRawAxis(0); /* positive is right */
    double turnPID = pid.calculate(gyro.getRate(), target);
    if (turnPID > 0) {
      turnPID -= kF;
    } else if (turnPID < 0) {
      turnPID += kF;
    }

    if (Math.abs(forw) < 0.10) {
      forw = 0;
    }
    /*
    if (Math.abs(turn) < 0.10) {
        turn = 0;
    }
    */
    if (controller.getTrigger()) {
      gyroTank.arcadeDrive(forw, turnPID, false);
    } else {
      gyroTank.arcadeDrive(forw, turnManual, false);
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
