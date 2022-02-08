package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

  // initialize constants
  private static final double kRPMConversion = 9.5493; // Radians/sec to RPM
  private static final double kGearing = 10.71; // Gearbox gear ration 10.71:1
  private static final double kWheelRadius = 0.1524; // meters
  private static final double kMaxSpeedTranslational = 4; // m/s
  private static final double kMaxSpeedRot = 3; // rad/s
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput,
                maxRPM, maxVel, minVel, maxAcc, allowedErr;
  /*
  private double kPGyro;
  private double initialHeading;
  private double gyroError;
  */

  // Spark MAX Can id's
  private static final int kMotorID = 1;
  
  // Initialize Spark Maxs
  private CANSparkMax m_motor = new CANSparkMax(kMotorID, MotorType.kBrushless);
  // Initialize NEO encoders
  private RelativeEncoder m_encoder;

  // Initialize Onboard PID Controllers
  private SparkMaxPIDController m_pidController;

  // Initialize Translation2d objects for wheelbase
  private Translation2d m_frontLeftLocation;
  private Translation2d m_frontRightLocation;
  private Translation2d m_backLeftLocation;
  private Translation2d m_backRightLocation;

  // Initialize Target Wheel Speeds
  private double frontLeft;
  private double frontRight;
  private double backLeft;
  private double backRight;

  private Joystick m_joystick;

  /* private ADIS16448_IMU gyro = new ADIS16448_IMU(); */

  // Initialize classes for kinematics
  MecanumDriveKinematics m_kinematics;
  MecanumDriveWheelSpeeds wheelSpeeds;
  ChassisSpeeds speeds;

  @Override
  public void robotInit() {

    // Velocity Control Constants
    kP = 6e-3; 
    kI = 0;
    kD = 0; 
    kIz = 0; 
    kFF = 0.000015; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;

    // Smart Motion Constants
    maxVel = 2000; // RPM
    minVel = 0;
    maxAcc = 1500; // RPM/s
    allowedErr = 20;

    /* kPGyro = 0.05 */

    // Display constants on dashboard for live tuning
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Max Acceleration", maxAcc);
    SmartDashboard.putNumber("Allowed Closed Loop Error", allowedErr);
    
    m_joystick = new Joystick(0);

    /* gyro = new ADIS16448_IMU(); */

    m_motor.restoreFactoryDefaults();

    // Initialize encoders from NEOs connected to each spark
    m_encoder = m_motor.getEncoder();

    m_pidController = m_motor.getPIDController();

    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    int smartMotionSlot = 0;
    m_pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    m_pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    m_pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    m_pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    // Distane from each wheel to center
    m_frontLeftLocation = new Translation2d(0.381, 0.381);
    m_frontRightLocation = new Translation2d(0.381, -0.381);
    m_backLeftLocation = new Translation2d(-0.381, 0.381);
    m_backRightLocation = new Translation2d(-0.381, -0.381);

    // Kinematics handles drivetrain math
    m_kinematics = new MecanumDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
    );

  }

  @Override
  public void simulationInit() {
    REVPhysicsSim.getInstance().addSparkMax(m_motor, DCMotor.getNEO(1));
  }

  @Override
  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();
  }

  @Override
  public void robotPeriodic() {
    // Display velocity setpoints
    SmartDashboard.putNumber("LFSetpoint", frontLeft);
    SmartDashboard.putNumber("LRSetpoint", backLeft);
    SmartDashboard.putNumber("FRSetpoint", frontRight);
    SmartDashboard.putNumber("RRSetpoint", backRight);

    // Display motor speeds
    SmartDashboard.putNumber("Encoder RPM", m_encoder.getVelocity());
  }

  @Override
  public void autonomousInit() {
    /* initialHeading = gyro.getAngle(); */
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {

    // Get PID constants from dashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double maxA = SmartDashboard.getNumber("Max Acceleration", 0);
    double allE = SmartDashboard.getNumber("Allowed Closed Loop Error", 0);

    // Set PID controller constants to values obtained from user
    // if each values is modified by the user
    if((p != kP)) { m_pidController.setP(p); kP = p; }
    if((i != kI)) { m_pidController.setI(i); kI = i; }
    if((d != kD)) { m_pidController.setD(d); kD = d; }
    if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_pidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
    if((maxA != maxAcc)) { m_pidController.setSmartMotionMaxAccel(maxA,0); maxAcc = maxA; }
    if((allE != allowedErr)) { m_pidController.setSmartMotionAllowedClosedLoopError(allE,0); allowedErr = allE; }


    // Set target speeds for robot
    speeds = new ChassisSpeeds(
      m_joystick.getRawAxis(1) * kMaxSpeedTranslational, // Forward and backward
      m_joystick.getRawAxis(0) * kMaxSpeedTranslational, // Strafing
      m_joystick.getRawAxis(4) * kMaxSpeedRot // Rotation
    );

    /*
    error = initialHeading - gyro.getAngle();
    speeds = new ChassisSpeeds(
      m_joystick.getY() * kMaxSpeedTranslational,
      m_joystick.getX() * kMaxSpeedTranslational,
      kPGyro * error
    );
    */

    // Get individual wheel speeds from chassis speeds
    wheelSpeeds = m_kinematics.toWheelSpeeds(speeds);

    // Convert wheel speeds (m/s) to motor speeds (RPM)
    frontLeft = wheelSpeeds.frontLeftMetersPerSecond / kWheelRadius * kGearing * kRPMConversion;
    frontRight = wheelSpeeds.frontRightMetersPerSecond / kWheelRadius * kGearing * kRPMConversion;
    backLeft = wheelSpeeds.rearLeftMetersPerSecond / kWheelRadius * kGearing * kRPMConversion;
    backRight = wheelSpeeds.rearRightMetersPerSecond / kWheelRadius * kGearing * kRPMConversion;

    // Update velocity targets for onboard control loops
    m_pidController.setReference(frontLeft, CANSparkMax.ControlType.kSmartVelocity);

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

//Add Gyro & Field Centric
//Measure Wheelbase