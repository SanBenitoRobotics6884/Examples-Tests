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
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

  // initialize constants
  private static final boolean isOpenLoop = true; // Drive mode
  private static final boolean kRightSideInverted = true; // Invert right motors
  private static final double kRPMConversion = 9.5493; // Radians/sec to RPM
  private static final double kGearing = 10.71; // Gearbox gear ration 10.71:1
  private static final double kWheelRadius = 0.1524; // meters
  private static final double kSpeedScale = 0.6; // max speed. 0 to 1
  private static final double kStrafeScale = 0.6; // max strafe rate. 0 to 1
  private static final double kTwistScale = 0.6; // max twist rate. 0 to 1
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
  private static final int kLFrontID = 1;
  private static final int kLRearID = 10;
  private static final int kRFrontID = 2;
  private static final int kRRearID = 20;
  
  // Initialize Spark Maxs
  private CANSparkMax m_lFrontMotor = new CANSparkMax(kLFrontID, MotorType.kBrushless);
  private CANSparkMax m_lRearMotor = new CANSparkMax(kLRearID, MotorType.kBrushless);
  private CANSparkMax m_rFrontMotor = new CANSparkMax(kRFrontID, MotorType.kBrushless);
  private CANSparkMax m_rRearMotor = new CANSparkMax(kRRearID, MotorType.kBrushless);

  // Initialize NEO encoders
  private RelativeEncoder m_lFrontEncoder;
  private RelativeEncoder m_lRearEncoder;
  private RelativeEncoder m_rFrontEncoder;
  private RelativeEncoder m_rRearEncoder;

  // Initialize Onboard PID Controllers
  private SparkMaxPIDController m_lFrontPIDController;
  private SparkMaxPIDController m_lRearPIDController;
  private SparkMaxPIDController m_rFrontPIDController;
  private SparkMaxPIDController m_rRearPIDController;

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

  private MecanumDrive m_robotDrive;

  @Override
  public void robotInit() {

    // Velocity Control Constants
    kP = 6e-5; 
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

    m_lFrontMotor.restoreFactoryDefaults();
    m_lRearMotor.restoreFactoryDefaults();
    m_rFrontMotor.restoreFactoryDefaults();
    m_rRearMotor.restoreFactoryDefaults();

    m_rFrontMotor.setInverted(kRightSideInverted);
    m_rRearMotor.setInverted(kRightSideInverted);

    // Initialize encoders from NEOs connected to each spark
    m_lFrontEncoder = m_lFrontMotor.getEncoder();
    m_lRearEncoder = m_lRearMotor.getEncoder();
    m_rFrontEncoder = m_rFrontMotor.getEncoder();
    m_rRearEncoder = m_rRearMotor.getEncoder();

    m_lFrontPIDController = m_lFrontMotor.getPIDController();
    m_lRearPIDController = m_lRearMotor.getPIDController();
    m_rFrontPIDController = m_rFrontMotor.getPIDController();
    m_rRearPIDController = m_rRearMotor.getPIDController();

    m_lFrontPIDController.setP(kP);
    m_lFrontPIDController.setI(kI);
    m_lFrontPIDController.setD(kD);
    m_lFrontPIDController.setIZone(kIz);
    m_lFrontPIDController.setFF(kFF);
    m_lFrontPIDController.setOutputRange(kMinOutput, kMaxOutput);

    m_lRearPIDController.setP(kP);
    m_lRearPIDController.setI(kI);
    m_lRearPIDController.setD(kD);
    m_lRearPIDController.setIZone(kIz);
    m_lRearPIDController.setFF(kFF);
    m_lRearPIDController.setOutputRange(kMinOutput, kMaxOutput);

    m_rFrontPIDController.setP(kP);
    m_rFrontPIDController.setI(kI);
    m_rFrontPIDController.setD(kD);
    m_rFrontPIDController.setIZone(kIz);
    m_rFrontPIDController.setFF(kFF);
    m_rFrontPIDController.setOutputRange(kMinOutput, kMaxOutput);

    m_rRearPIDController.setP(kP);
    m_rRearPIDController.setI(kI);
    m_rRearPIDController.setD(kD);
    m_rRearPIDController.setIZone(kIz);
    m_rRearPIDController.setFF(kFF);
    m_rRearPIDController.setOutputRange(kMinOutput, kMaxOutput);

    int smartMotionSlot = 0;
    m_lFrontPIDController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    m_lFrontPIDController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    m_lFrontPIDController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    m_lFrontPIDController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    m_lRearPIDController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    m_lRearPIDController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    m_lRearPIDController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    m_lRearPIDController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    m_rFrontPIDController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    m_rFrontPIDController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    m_rFrontPIDController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    m_rFrontPIDController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    m_rRearPIDController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    m_rRearPIDController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    m_rRearPIDController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    m_rRearPIDController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);    

    // Distane from each wheel to center
    m_frontLeftLocation = new Translation2d(0.381, 0.381);
    m_frontRightLocation = new Translation2d(0.381, -0.381);
    m_backLeftLocation = new Translation2d(-0.381, 0.381);
    m_backRightLocation = new Translation2d(-0.381, -0.381);

    // Kinematics handles drivetrain math
    m_kinematics = new MecanumDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
    );

    m_robotDrive = new MecanumDrive(m_lFrontMotor, m_lRearMotor, m_rFrontMotor, m_rRearMotor);

  }

  @Override
  public void simulationInit() {
    REVPhysicsSim.getInstance().addSparkMax(m_lFrontMotor, DCMotor.getNEO(1));
    REVPhysicsSim.getInstance().addSparkMax(m_lRearMotor, DCMotor.getNEO(1));
    REVPhysicsSim.getInstance().addSparkMax(m_rFrontMotor, DCMotor.getNEO(1));
    REVPhysicsSim.getInstance().addSparkMax(m_rRearMotor, DCMotor.getNEO(1));
  }

  @Override
  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();
  }

  @Override
  public void robotPeriodic() {
    // Display velocity setpoints
    SmartDashboard.putNumber("frontLeft", frontLeft);
    SmartDashboard.putNumber("backLeft", backLeft);
    SmartDashboard.putNumber("frontRight", frontRight);
    SmartDashboard.putNumber("backRight", backRight);

    // Display motor speeds
    SmartDashboard.putNumber("L Front Encoder", m_lFrontEncoder.getVelocity());
    SmartDashboard.putNumber("L Rear Encoder", m_lRearEncoder.getVelocity());
    SmartDashboard.putNumber("R Front Encoder", m_rFrontEncoder.getVelocity());
    SmartDashboard.putNumber("R Rear Encoder", m_rRearEncoder.getVelocity());
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
    if((p != kP)) { m_lFrontPIDController.setP(p); kP = p; }
    if((i != kI)) { m_lFrontPIDController.setI(i); kI = i; }
    if((d != kD)) { m_lFrontPIDController.setD(d); kD = d; }
    if((iz != kIz)) { m_lFrontPIDController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_lFrontPIDController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_lFrontPIDController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
    if((maxA != maxAcc)) { m_lFrontPIDController.setSmartMotionMaxAccel(maxA,0); maxAcc = maxA; }
    if((allE != allowedErr)) { m_lFrontPIDController.setSmartMotionAllowedClosedLoopError(allE,0); allowedErr = allE; }

    // Set PID controller constants to values obtained from user
    // if each values is modified by the user
    if((p != kP)) { m_lRearPIDController.setP(p); kP = p; }
    if((i != kI)) { m_lRearPIDController.setI(i); kI = i; }
    if((d != kD)) { m_lRearPIDController.setD(d); kD = d; }
    if((iz != kIz)) { m_lRearPIDController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_lRearPIDController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_lRearPIDController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
    if((maxA != maxAcc)) { m_lRearPIDController.setSmartMotionMaxAccel(maxA,0); maxAcc = maxA; }
    if((allE != allowedErr)) { m_lRearPIDController.setSmartMotionAllowedClosedLoopError(allE,0); allowedErr = allE; }

    // Set PID controller constants to values obtained from user
    // if each values is modified by the user
    if((p != kP)) { m_rFrontPIDController.setP(p); kP = p; }
    if((i != kI)) { m_rFrontPIDController.setI(i); kI = i; }
    if((d != kD)) { m_rFrontPIDController.setD(d); kD = d; }
    if((iz != kIz)) { m_rFrontPIDController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_rFrontPIDController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_rFrontPIDController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
    if((maxA != maxAcc)) { m_rFrontPIDController.setSmartMotionMaxAccel(maxA,0); maxAcc = maxA; }
    if((allE != allowedErr)) { m_rFrontPIDController.setSmartMotionAllowedClosedLoopError(allE,0); allowedErr = allE; }

    // Set PID controller constants to values obtained from user
    // if each values is modified by the user
    if((p != kP)) { m_rRearPIDController.setP(p); kP = p; }
    if((i != kI)) { m_rRearPIDController.setI(i); kI = i; }
    if((d != kD)) { m_rRearPIDController.setD(d); kD = d; }
    if((iz != kIz)) { m_rRearPIDController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_rRearPIDController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_rRearPIDController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
    if((maxA != maxAcc)) { m_rRearPIDController.setSmartMotionMaxAccel(maxA,0); maxAcc = maxA; }
    if((allE != allowedErr)) { m_rRearPIDController.setSmartMotionAllowedClosedLoopError(allE,0); allowedErr = allE; }

    if (isOpenLoop) {
      // Drive using built in mecanum class. Does not use feedback
      m_robotDrive.driveCartesian(
        m_joystick.getX() * kSpeedScale,
        -m_joystick.getY() * kStrafeScale,
        m_joystick.getZ() * kTwistScale
      );
    } else {
      // Set target speeds for robot
      speeds = new ChassisSpeeds(
        m_joystick.getY() * kMaxSpeedTranslational, // Forward and backward
        m_joystick.getX() * kMaxSpeedTranslational, // Strafing
        m_joystick.getZ() * kMaxSpeedRot // Rotation
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
      m_lFrontPIDController.setReference(frontLeft, CANSparkMax.ControlType.kSmartVelocity);
      m_lRearPIDController.setReference(frontRight, CANSparkMax.ControlType.kSmartVelocity);
      m_rFrontPIDController.setReference(backLeft, CANSparkMax.ControlType.kSmartVelocity);
      m_rRearPIDController.setReference(backRight, CANSparkMax.ControlType.kSmartVelocity);

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

//Add Gyro & Field Centric
//Measure Wheelbase