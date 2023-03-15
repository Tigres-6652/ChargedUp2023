package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PUERTOSCAN;

public class DriveSubsystem extends SubsystemBase {

  WPI_TalonSRX MCI1ENC = new WPI_TalonSRX(PUERTOSCAN.PuertMotIzq1yEncoder);
  WPI_TalonSRX MCI2 = new WPI_TalonSRX(PUERTOSCAN.PuertMotIzq2);
  WPI_TalonSRX MCI3 = new WPI_TalonSRX(PUERTOSCAN.PuertMotIzq3);

  WPI_TalonSRX MCD4ENC = new WPI_TalonSRX(PUERTOSCAN.PuertMotDer1yEncoder);
  WPI_TalonSRX MCD5 = new WPI_TalonSRX(PUERTOSCAN.PuertMotDer2);
  WPI_TalonSRX MCD6 = new WPI_TalonSRX(PUERTOSCAN.PuertMotDer3);

  MotorControllerGroup motsizq = new MotorControllerGroup(MCI1ENC, MCI2, MCI3);
  MotorControllerGroup motsder = new MotorControllerGroup(MCD4ENC, MCD5, MCD6);

  DifferentialDrive chasis = new DifferentialDrive(motsizq, motsder);

  // LIMELIGHT //////
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");

  // control
  Joystick joystick = new Joystick(0);

  // path
  AHRS m_gyro = new AHRS(SPI.Port.kMXP); // navx
  DifferentialDriveOdometry m_odometry;

  @Override
  public void periodic() {

    m_odometry.update(
        m_gyro.getRotation2d(), getRightEncoderdistance(), getLeftEncoderdistance());

    SmartDashboard.putNumber("encizq", MCI1ENC.getSelectedSensorPosition() / 4096 / 9.01);
    SmartDashboard.putNumber("encizq", getLeftEncoderdistance());
    SmartDashboard.putNumber("encder", getRightEncoderdistance());
    SmartDashboard.putNumber("gyro", getHeading());
    SmartDashboard.putNumber("velocity", -MCD4ENC.getSelectedSensorVelocity() / 4096 / 9.01 * 10);

  }

  public DriveSubsystem() {
    resetTalons();
    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(getRotation2d(),
        getLeftEncoderdistance(), getRightEncoderdistance());

    motsizq.setInverted(false);
    motsder.setInverted(true);
    // configPIDDrivTr();

  }

  public void configPIDDrivTr() {

    MCI1ENC.configOpenloopRamp(0.05);
    MCD4ENC.configOpenloopRamp(0.05);

    /* Factory Default all hardware to prevent unexpected behaviour */
    MCI1ENC.configFactoryDefault();

    /* Config neutral deadband to be the smallest possible */
    MCI1ENC.configNeutralDeadband(0.001);

    /* Config sensor used for Primary PID [Velocity] */
    MCI1ENC.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative,
        Constants.KPIDchasis.kPIDLoopIdx,
        Constants.KPIDchasis.kTimeoutMs);

    /* Config the peak and nominal outputs */
    MCI1ENC.configNominalOutputForward(0, Constants.KPIDchasis.kTimeoutMs);
    MCI1ENC.configNominalOutputReverse(0, Constants.KPIDchasis.kTimeoutMs);
    MCI1ENC.configPeakOutputForward(1, Constants.KPIDchasis.kTimeoutMs);
    MCI1ENC.configPeakOutputReverse(-1, Constants.KPIDchasis.kTimeoutMs);

    /* Config the Velocity closed loop gains in slot0 */
    MCI1ENC.config_kF(Constants.KPIDchasis.kPIDLoopIdx, Constants.KPIDchasis.kGains_Velocit.kF,
        Constants.KPIDchasis.kTimeoutMs);
    MCI1ENC.config_kP(Constants.KPIDchasis.kPIDLoopIdx, Constants.KPIDchasis.kGains_Velocit.kP,
        Constants.KPIDchasis.kTimeoutMs);
    MCI1ENC.config_kI(Constants.KPIDchasis.kPIDLoopIdx, Constants.KPIDchasis.kGains_Velocit.kI,
        Constants.KPIDchasis.kTimeoutMs);
    MCI1ENC.config_kD(Constants.KPIDchasis.kPIDLoopIdx, Constants.KPIDchasis.kGains_Velocit.kD,
        Constants.KPIDchasis.kTimeoutMs);

    // MCI1ENC.configOpenloopRamp(2.0);

    // https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#
    /* Factory Default all hardware to prevent unexpected behaviour */
    MCD4ENC.configFactoryDefault();

    /* Config neutral deadband to be the smallest possible */
    MCD4ENC.configNeutralDeadband(0.001);

    /* Config sensor used for Primary PID [Velocity] */
    MCD4ENC.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative,
        Constants.KPIDchasis.kPIDLoopIdx,
        Constants.KPIDchasis.kTimeoutMs);

    /* Config the peak and nominal outputs */
    MCD4ENC.configNominalOutputForward(0, Constants.KPIDchasis.kTimeoutMs);
    MCD4ENC.configNominalOutputReverse(0, Constants.KPIDchasis.kTimeoutMs);
    MCD4ENC.configPeakOutputForward(1, Constants.KPIDchasis.kTimeoutMs);
    MCD4ENC.configPeakOutputReverse(-1, Constants.KPIDchasis.kTimeoutMs);

    /* Config the Velocity closed loop gains in slot0 */
    MCD4ENC.config_kF(Constants.KPIDchasis.kPIDLoopIdx, Constants.KPIDchasis.kGains_Velocit.kF,
        Constants.KPIDchasis.kTimeoutMs);
    MCD4ENC.config_kP(Constants.KPIDchasis.kPIDLoopIdx, Constants.KPIDchasis.kGains_Velocit.kP,
        Constants.KPIDchasis.kTimeoutMs);
    MCD4ENC.config_kI(Constants.KPIDchasis.kPIDLoopIdx, Constants.KPIDchasis.kGains_Velocit.kI,
        Constants.KPIDchasis.kTimeoutMs);
    MCD4ENC.config_kD(Constants.KPIDchasis.kPIDLoopIdx, Constants.KPIDchasis.kGains_Velocit.kD,
        Constants.KPIDchasis.kTimeoutMs);

    // https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#

  }

  public void CHASIS(double velocidad, double giro) {

    chasis.arcadeDrive(velocidad, -giro);

    double calculo_encizq;
    double calculo_encder;

    calculo_encizq = (-MCI1ENC.getSelectedSensorPosition() / 4096 / 2);
    calculo_encder = (MCD4ENC.getSelectedSensorPosition() / 4096 / 2);

    SmartDashboard.putNumber("encoderizquierdo", calculo_encizq);
    SmartDashboard.putNumber("encoderderecho", calculo_encder);

    SmartDashboard.putNumber("avgDist", (calculo_encder + calculo_encizq) / 2);
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(-MCI1ENC.getSelectedSensorVelocity() / 4096 / 9.01 * 10,
        MCD4ENC.getSelectedSensorVelocity() / 4096 / 9.01 * 10);
  }

  public void resetEncoders() {

    MCI1ENC.setSelectedSensorPosition(0);
    MCD4ENC.setSelectedSensorPosition(0);

  }

  public void resetTalons() {

    MCI1ENC.configFactoryDefault();
    MCI2.configFactoryDefault();
    MCI3.configFactoryDefault();

    MCD4ENC.configFactoryDefault();
    MCD5.configFactoryDefault();
    MCD6.configFactoryDefault();

  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    // m_odometry.resetPosition(pose, m_gyro.getRotation2d());
    m_odometry.resetPosition(getRotation2d(), getLeftEncoderdistance(), getRightEncoderdistance(), pose);
  }

  public void arcadeDrive(double fwd, double rot) {

    chasis.arcadeDrive(fwd, rot);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    MCI1ENC.setVoltage(leftVolts);
    MCD4ENC.setVoltage(rightVolts);
    chasis.feed();
  }

  public double getAverageEncoderDistance() {
    return (getLeftEncoderdistance() + getRightEncoderdistance()) / 2.0;
  }

  public double getLeftEncoderdistance() {
    return -MCI1ENC.getSelectedSensorPosition() / 4096 / 9.01;
  }

  public double getRightEncoderdistance() {
    return MCD4ENC.getSelectedSensorPosition() / 4096 / 9.01;
  }

  public void setMaxOutput(double maxOutput) {
    chasis.setMaxOutput(maxOutput);
  }

  public void zeroHeading() {
    m_gyro.reset();
  }

  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  public double getTurnRate() {
    return -m_gyro.getRate();
  }

  public Rotation2d getRotation2d() {

    return m_gyro.getRotation2d();
  }

}
