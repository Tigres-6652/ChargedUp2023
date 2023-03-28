package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
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
import frc.robot.Constants.AjustesMovimientoChasis;
import frc.robot.Constants.PUERTOSCAN;
import frc.robot.Constants.AjustesMovimientoChasis.autoapuntado;
import frc.robot.Constants.AjustesMovimientoChasis.val_Balanceo;

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

  double ajustdist;
  double ajutGi;

  // control
  Joystick joystick = new Joystick(0);

  double cambiovel1;
  double cambiovel2;

  // path
  AHRS m_gyro = new AHRS(SPI.Port.kMXP); // navx
  DifferentialDriveOdometry m_odometry;

  //Balanceo
  double velocidadbalanceo;

  @Override
  public void periodic() {

    m_odometry.update(
        m_gyro.getRotation2d(), getRightEncoderdistance(), getLeftEncoderdistance());

    /*SmartDashboard.putNumber("encizq", getLeftEncoderdistance());
    SmartDashboard.putNumber("encder", getRightEncoderdistance());
    SmartDashboard.putNumber("getTurnRate", getTurnRate());
    SmartDashboard.putNumber("getheading", getHeading());

    SmartDashboard.putNumber("velocidad", (MCI1ENC.getSelectedSensorVelocity() / 4096 * Math.PI * 6 * 10 * 2.54 / 100));*/

    SmartDashboard.putNumber("Angulo Balanceo", m_gyro.getRoll());

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

  public void CHASIS(double velocidad, double giro, boolean autoapuntado, boolean balanceo, boolean apuntadoagarrar) {

    double calculo_encizq;
    double calculo_encder;

    calculo_encizq = (-MCI1ENC.getSelectedSensorPosition() / 4096 * Math.PI * 6 * 2.54);
    calculo_encder = (-MCD4ENC.getSelectedSensorPosition() / 4096 * Math.PI * 6 * 2.54);

    /*
     * SmartDashboard.putNumber("encoderizquierdo", calculo_encizq);
     * SmartDashboard.putNumber("encoderderecho", calculo_encder);
     * 
     * SmartDashboard.putNumber("avgDist", (calculo_encder + calculo_encizq) / 2);
     */

    if (autoapuntado) {

      double x = tx.getDouble(0.0);

      double girokp=frc.robot.Constants.AjustesMovimientoChasis.autoapuntado.girokp;
      double GiroError= x;
      double ajustegiro=girokp*GiroError;




      // distancia
      double y = ty.getDouble(0.0);

      double distanciaKP = -AjustesMovimientoChasis.autoapuntado.distanciakp;
      double DistanciaError = y;
      double ajustedistancia = distanciaKP * DistanciaError;

      if (ajustedistancia > frc.robot.Constants.AjustesMovimientoChasis.autoapuntado.vel_max_distancias) {

        ajustdist = frc.robot.Constants.AjustesMovimientoChasis.autoapuntado.vel_max_distancias;

      } else if (ajustedistancia < frc.robot.Constants.AjustesMovimientoChasis.autoapuntado.vel_max_distancias && ajustedistancia > -frc.robot.Constants.AjustesMovimientoChasis.autoapuntado.vel_max_distancias) {

        ajustdist = ajustedistancia;

      } else if (ajustedistancia < -frc.robot.Constants.AjustesMovimientoChasis.autoapuntado.vel_max_distancias) {

        ajustdist = -frc.robot.Constants.AjustesMovimientoChasis.autoapuntado.vel_max_distancias;

      }

      if (ajustegiro > frc.robot.Constants.AjustesMovimientoChasis.autoapuntado.vel_max_giro) {

        ajutGi = frc.robot.Constants.AjustesMovimientoChasis.autoapuntado.vel_max_giro;

      } else if (ajustegiro < frc.robot.Constants.AjustesMovimientoChasis.autoapuntado.vel_max_giro && ajustegiro > -frc.robot.Constants.AjustesMovimientoChasis.autoapuntado.vel_max_giro) {

        ajutGi = ajustegiro;

      } else if (ajustegiro < -frc.robot.Constants.AjustesMovimientoChasis.autoapuntado.vel_max_giro) {

        ajutGi = -frc.robot.Constants.AjustesMovimientoChasis.autoapuntado.vel_max_giro;

      }

      chasis.arcadeDrive(ajustdist, -ajutGi);

    } else if (balanceo) {

     double angulo_equilibrio= m_gyro.getRoll();

     double vel=angulo_equilibrio*val_Balanceo.kp;

      if(vel>val_Balanceo.velocidadmaxima){
        velocidadbalanceo=val_Balanceo.velocidadmaxima;
      }else{
        velocidadbalanceo=vel;

      }
      if(vel<-val_Balanceo.velocidadmaxima){
        velocidadbalanceo=-val_Balanceo.velocidadmaxima;
      }else{
velocidadbalanceo=vel;
      }

chasis.arcadeDrive(velocidadbalanceo, 0);
    } else {

      if (apuntadoagarrar) {
        MCI1ENC.setNeutralMode(NeutralMode.Brake);
        MCI2.setNeutralMode(NeutralMode.Brake);
        MCI3.setNeutralMode(NeutralMode.Brake);
        MCD4ENC.setNeutralMode(NeutralMode.Brake);
        MCD5.setNeutralMode(NeutralMode.Brake);
        MCD6.setNeutralMode(NeutralMode.Brake);
        cambiovel1 = 0.9;
        cambiovel2 = 0.9;
      } else {
        MCI1ENC.setNeutralMode(NeutralMode.Coast);
        MCI2.setNeutralMode(NeutralMode.Coast);
        MCI3.setNeutralMode(NeutralMode.Coast);
        MCD4ENC.setNeutralMode(NeutralMode.Coast);
        MCD5.setNeutralMode(NeutralMode.Coast);
        MCD6.setNeutralMode(NeutralMode.Coast);
        cambiovel1 = .9;
        cambiovel2 = .9;
      }

      chasis.arcadeDrive(velocidad * cambiovel1, -giro * cambiovel2);

    }

  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        (MCI1ENC.getSelectedSensorVelocity() / 4096 * Math.PI * 6 * 10 * 2.54 / 100),
        (MCD4ENC.getSelectedSensorVelocity() / 4096 * Math.PI * 6 * 10 * 2.54 / 100));
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
    m_gyro.calibrate();

  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    // m_odometry.resetPosition(pose, m_gyro.getRotation2d());

    zeroHeading();
    m_odometry.resetPosition(getRotation2d(), getLeftEncoderdistance(), getRightEncoderdistance(), pose);
  }

  public void arcadeDrive(double fwd, double rot) {

    chasis.arcadeDrive(fwd, -rot);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    motsizq.setVoltage(leftVolts);
    motsder.setVoltage(rightVolts);
    chasis.feed();
  }

  public double getAverageEncoderDistance() {
    return (getLeftEncoderdistance() + getRightEncoderdistance()) / 2.0;
  }

  public double getLeftEncoderdistance() {
    return (MCI1ENC.getSelectedSensorPosition() / 4096 * Math.PI * 6 * 2.54) / 100;
  }

  public double getRightEncoderdistance() {
    return (MCD4ENC.getSelectedSensorPosition() / 4096 * Math.PI * 6 * 2.54) / 100;
  }

  public void setMaxOutput(double maxOutput) {
    chasis.setMaxOutput(maxOutput);
  }

  public void zeroHeading() {
    m_gyro.reset();
  }

  public double getHeading() {
    return -m_gyro.getRotation2d().getDegrees();
  }

  public double getTurnRate() {
    return m_gyro.getRate();
  }

  public Rotation2d getRotation2d() {

    return m_gyro.getRotation2d();
  }

}
