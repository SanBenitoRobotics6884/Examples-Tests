/**
 * Phoenix Software License Agreement
 *
 * Copyright (C) Cross The Road Electronics.  All rights
 * reserved.
 * 
 * Cross The Road Electronics (CTRE) licenses to you the right to 
 * use, publish, and distribute copies of CRF (Cross The Road) firmware files (*.crf) and 
 * Phoenix Software API Libraries ONLY when in use with CTR Electronics hardware products
 * as well as the FRC roboRIO when in use in FRC Competition.
 * 
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * CROSS THE ROAD ELECTRONICS BE LIABLE FOR ANY INCIDENTAL, SPECIAL, 
 * INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE
 */

/**
 * Description:
 * The MotionMagic example demonstrates the motion magic control mode.
 * Tested with Logitech F710 USB Gamepad inserted into Driver Station.
 * 
 * Be sure to select the correct feedback sensor using configSelectedFeedbackSensor() below.
 *
 * After deploying/debugging this to your RIO, first use the left Y-stick 
 * to throttle the Talon manually. This will confirm your hardware setup/sensors
 * and will allow you to take initial measurements.
 * 
 * Be sure to confirm that when the Talon is driving forward (green) the 
 * position sensor is moving in a positive direction. If this is not the 
 * cause, flip the boolean input to the setSensorPhase() call below.
 *
 * Ensure your feedback device is in-phase with the motor,
 * and you have followed the walk-through in the Talon SRX Software Reference Manual.
 * 
 * Controls:
 * Button 1(Button A): When held, put Talon in Motion Magic mode and allow Talon to drive [-10, 10] 
 * 	rotations.
 * Button 2(Button B): When pushed, the selected feedback sensor gets zero'd
 * POV 180(Dpad Down): When pushed, will decrement the smoothing of the motion magic down to 0
 * POV 0(Dpad Up): When pushed, will increment the smoothing of the motion magic up to 8
 * Left Joystick Y-Axis:
 * 	+ Percent Output: Throttle Talon SRX forward and reverse, use to confirm hardware setup.
 * Right Joystick Y-Axis:
 * 	+ Motion Magic: Servo Talon SRX forward and reverse, [-10, 10] rotations.
 * 
 * Gains for Motion Magic may need to be adjusted in java
 * 
 * Supported Version:
 * - Talon SRX: 4.00
 * - Victor SPX: 4.00
 * - Pigeon IMU: 4.00
 * - CANifier: 4.00
 */
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;

public class Robot extends TimedRobot {
	static WPI_TalonSRX m_motor = new WPI_TalonSRX(3);
	Joystick m_controller = new Joystick(0);

	public static final int kTimeoutMs = 30;
	public static final int kPIDLoopIdx = 0;
	public static final int kSlotIdx = 0;

	static int kCruiseVelocity = 3000;
	static int kCruiseAcceleration = 3000;
	static int kSmoothing = 0; //Motion Magic Smoothing. [0,8]

	static double kP = 0.2;
	static double kI = 0;
	static double kD = 0;
	static double kF = 0.2;
	static double kIZ = 0;
	static double kPeakOutput = 1;

	public void robotInit() {

		m_motor.configFactoryDefault();

		m_motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kPIDLoopIdx,
				kTimeoutMs);

		//Lower deadband
		m_motor.configNeutralDeadband(0.001, kTimeoutMs);

		/**
		 * Configure Talon SRX Output and Sensor direction accordingly Invert Motor to
		 * have green LEDs when driving Talon Forward / Requesting Postiive Output Phase
		 * sensor to have positive increment when driving Talon Forward (Green LED)
		 */
		m_motor.setSensorPhase(false);
		m_motor.setInverted(false);

		/* Set relevant frame periods to be at least as fast as periodic rate */
		m_motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);
		m_motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, kTimeoutMs);

		/* Set the peak and nominal outputs */
		m_motor.configNominalOutputForward(0, kTimeoutMs);
		m_motor.configNominalOutputReverse(0, kTimeoutMs);
		m_motor.configPeakOutputForward(1, kTimeoutMs);
		m_motor.configPeakOutputReverse(-1, kTimeoutMs);

		/* Set Motion Magic gains in slot0 - see documentation */
		m_motor.selectProfileSlot(kPIDLoopIdx, kSlotIdx);
		m_motor.config_kF(kSlotIdx, kF, kTimeoutMs);
		m_motor.config_kP(kSlotIdx, kP, kTimeoutMs);
		m_motor.config_kI(kSlotIdx, kI, kTimeoutMs);
		m_motor.config_kD(kSlotIdx, kD, kTimeoutMs);

		/* Set acceleration and vcruise velocity - see documentation */
		m_motor.configMotionCruiseVelocity(kCruiseVelocity, kTimeoutMs);
		m_motor.configMotionAcceleration(kCruiseAcceleration, kTimeoutMs);
		m_motor.configMotionSCurveStrength(kSmoothing);

		/* Zero the sensor once on robot boot up */
		//m_motor.setSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);

		int pulseWidthWithoutOverflows = m_motor.getSensorCollection().getPulseWidthPosition() & 0xFFF;
		m_motor.setSelectedSensorPosition(pulseWidthWithoutOverflows, kPIDLoopIdx, kTimeoutMs);

		SmartDashboard.putNumber("kP", kP);
		SmartDashboard.putNumber("kI", kI);
		SmartDashboard.putNumber("kD", kD);
		SmartDashboard.putNumber("kF", kF);
		SmartDashboard.putNumber("Cruise Velocity", kCruiseVelocity);
		SmartDashboard.putNumber("Cruise Acceleration", kCruiseAcceleration);
		SmartDashboard.putNumber("Smoothing", kSmoothing);

	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {

		updateGains();

		/* Get gamepad axis - forward stick is positive */
		double stickY = -1.0 * m_controller.getY(); 
		if (Math.abs(stickY) < 0.10) { stickY = 0; } /* deadband 10% */

		/**
		 * Perform Motion Magic when Button 1 is held, else run Percent Output, which can
		 * be used to confirm hardware setup.
		 */
		if (m_controller.getRawButton(1)) {
			/* Motion Magic */
      
			/* 4096 ticks/rev * 1/2 Rotation in either direction */
			double targetPos = stickY * 4096 * 0.5;
			m_motor.set(ControlMode.MotionMagic, targetPos);

		} else {
			/* Percent Output */
			m_motor.set(ControlMode.PercentOutput, stickY * 0.5);
		}
    SmartDashboard.putNumber("Motor Output", m_motor.getMotorOutputVoltage());
	}

	public static void updateGains() {
		double p = SmartDashboard.getNumber("kP", 0);
		double i = SmartDashboard.getNumber("kI", 0);
		double d = SmartDashboard.getNumber("kD", 0);
		double f = SmartDashboard.getNumber("kF", 0);
		int vel = (int)SmartDashboard.getNumber("Cruise Velocity", 0);
		int acc = (int)SmartDashboard.getNumber("Cruise Acceleration", 0);
		int smooth = (int)SmartDashboard.getNumber("Smoothing", 0);

		if (p != kP) { kP = p; m_motor.config_kP(kSlotIdx, p, kTimeoutMs); }
		if (i != kI) { kI = i; m_motor.config_kI(kSlotIdx, i, kTimeoutMs); }
		if (d != kD) { kD = d; m_motor.config_kD(kSlotIdx, d, kTimeoutMs); }
		if (f != kF) { kF = f; m_motor.config_kF(kSlotIdx, f, kTimeoutMs); }
		if (vel != kCruiseVelocity) { kCruiseVelocity = vel; m_motor.configMotionCruiseVelocity(vel, kTimeoutMs); }
		if (acc != kCruiseAcceleration) { kCruiseAcceleration = acc; m_motor.configMotionAcceleration(acc, kTimeoutMs); }
		if (smooth != kSmoothing) { kSmoothing = smooth; m_motor.configMotionSCurveStrength(smooth); }

	}

}