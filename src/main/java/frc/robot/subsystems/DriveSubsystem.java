package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
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
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AjustesMovimientoChasis;
import frc.robot.Constants.PUERTOSCAN;
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

  public static ADIS16470_IMU gyro = new ADIS16470_IMU();
  // ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  // Balanceo
  double velocidadbalanceo;

  // variable posicion sin mover
  double posicionfijaizq;
  double posicionfijader;

  @Override
  public void periodic() {

    m_odometry.update(
        getRotation2d()/* m_gyro.getRotation2d() */, getRightEncoderdistance(), getLeftEncoderdistance());

    SmartDashboard.putNumber("encizq", getLeftEncoderdistance());
    SmartDashboard.putNumber("encder", getRightEncoderdistance());

    /**
     * SmartDashboard.putNumber("getTurnRate", getTurnRate());
     * SmartDashboard.putNumber("getheading", getHeading());
     * 
     * SmartDashboard.putNumber("velocidad", (MCI1ENC.getSelectedSensorVelocity() /
     * 4096 * Math.PI * 6 * 10 * 2.54 / 100));
     */

    // SmartDashboard.putNumber("Angulo Balanceo", m_gyro.getRoll());

    /*
     * SmartDashboard.putNumber("gyro1", gyro.getAngle());
     * SmartDashboard.putNumber("gyro1vel", gyro.getRate());
     */

    // SmartDashboard.putNumber("gyro2", gyro.);

  }

  public DriveSubsystem() {
    // resetEncoders();

    m_odometry = new DifferentialDriveOdometry(getRotation2d(),
        getLeftEncoderdistance(), getRightEncoderdistance());

    configPIDDrivTr();

  }

  public void CHASIS(double velocidad, double giro, boolean autoapuntado, boolean balanceo, boolean apuntadoagarrar,
      boolean robotSinmovimiento) {
        SmartDashboard.putNumber("acel", MCI1ENC.getSelectedSensorVelocity() / 4096 * Math.PI * 6 * 10 * 2.54 / 100);
    
        /*
     * double calculo_encizq;
     * double calculo_encder;
     * 
     * calculo_encizq = (-MCI1ENC.getSelectedSensorPosition() / 4096 * Math.PI * 6 *
     * 2.54);
     * calculo_encder = (-MCD4ENC.getSelectedSensorPosition() / 4096 * Math.PI * 6 *
     * 2.54);
     */

    /*
     * SmartDashboard.putNumber("encoderizquierdo", calculo_encizq);
     * SmartDashboard.putNumber("encoderderecho", calculo_encder);
     * 
     * SmartDashboard.putNumber("avgDist", (calculo_encder + calculo_encizq) / 2);
     */

    if (autoapuntado) {

      double x = tx.getDouble(0.0);

      double girokp = frc.robot.Constants.AjustesMovimientoChasis.autoapuntado.girokp;
      double GiroError = x;
      double ajustegiro = girokp * GiroError;

      // distancia
      double y = ty.getDouble(0.0);

      double distanciaKP = -AjustesMovimientoChasis.autoapuntado.distanciakp;
      double DistanciaError = y;
      double ajustedistancia = distanciaKP * DistanciaError;

      if (ajustedistancia > frc.robot.Constants.AjustesMovimientoChasis.autoapuntado.vel_max_distancias) {

        ajustdist = frc.robot.Constants.AjustesMovimientoChasis.autoapuntado.vel_max_distancias;

      } else if (ajustedistancia < frc.robot.Constants.AjustesMovimientoChasis.autoapuntado.vel_max_distancias
          && ajustedistancia > -frc.robot.Constants.AjustesMovimientoChasis.autoapuntado.vel_max_distancias) {

        ajustdist = ajustedistancia;

      } else if (ajustedistancia < -frc.robot.Constants.AjustesMovimientoChasis.autoapuntado.vel_max_distancias) {

        ajustdist = -frc.robot.Constants.AjustesMovimientoChasis.autoapuntado.vel_max_distancias;

      }

      if (ajustegiro > frc.robot.Constants.AjustesMovimientoChasis.autoapuntado.vel_max_giro) {

        ajutGi = frc.robot.Constants.AjustesMovimientoChasis.autoapuntado.vel_max_giro;

      } else if (ajustegiro < frc.robot.Constants.AjustesMovimientoChasis.autoapuntado.vel_max_giro
          && ajustegiro > -frc.robot.Constants.AjustesMovimientoChasis.autoapuntado.vel_max_giro) {

        ajutGi = ajustegiro;

      } else if (ajustegiro < -frc.robot.Constants.AjustesMovimientoChasis.autoapuntado.vel_max_giro) {

        ajutGi = -frc.robot.Constants.AjustesMovimientoChasis.autoapuntado.vel_max_giro;

      }

      chasis.arcadeDrive(ajustdist, -ajutGi);

    } else if (balanceo) {
      double angulo_equilibrio = gyro.getXComplementaryAngle();

      double vel = angulo_equilibrio * val_Balanceo.kp;

      if (vel > val_Balanceo.velocidadmaxima) {
        velocidadbalanceo = val_Balanceo.velocidadmaxima;
      } else {
        velocidadbalanceo = vel;

      }
      if (vel < -val_Balanceo.velocidadmaxima) {
        velocidadbalanceo = -val_Balanceo.velocidadmaxima;
      } else {
        velocidadbalanceo = vel;
      }

      MCI1ENC.set(ControlMode.Velocity, velocidadbalanceo);
      MCD4ENC.set(ControlMode.Velocity, velocidadbalanceo);

      MCI2.follow(MCI1ENC);
      MCI3.follow(MCI1ENC);

      MCD5.follow(MCD4ENC);
      MCD6.follow(MCD4ENC);

    } else if (robotSinmovimiento) {

      MCI1ENC.set(ControlMode.Position, posicionfijaizq);
      MCD4ENC.set(ControlMode.Position, posicionfijader);

      MCI2.follow(MCI1ENC);
      MCI3.follow(MCI1ENC);

      MCD5.follow(MCD4ENC);
      MCD6.follow(MCD4ENC);

    } else {


      chasis.arcadeDrive(velocidad, giro);

      posicionfijaizq = MCI1ENC.getSelectedSensorPosition();
      posicionfijader = MCD4ENC.getSelectedSensorPosition();

    }

    SmartDashboard.putNumber("gyro1", gyro.getAngle());
    SmartDashboard.putNumber("gyro2", gyro.getXComplementaryAngle());
    SmartDashboard.putNumber("gyro3", gyro.getYComplementaryAngle());

    SmartDashboard.putNumber("posicionIzq", MCI1ENC.getSelectedSensorPosition());
    SmartDashboard.putNumber("posicionDer", MCD4ENC.getSelectedSensorPosition());

  }

  public void set_distance(double distancia) {

double pulses=DistanceToPulses(distancia);

    MCI1ENC.set(ControlMode.Position, pulses);
    MCD4ENC.set(ControlMode.Position, pulses);

    MCI2.follow(MCI1ENC);
    MCI3.follow(MCI1ENC);

    MCD5.follow(MCD4ENC);
    MCD6.follow(MCD4ENC);

  }

  public double DistanceToPulses(double distanceMTS) {

    double pulse = (((((distanceMTS * 100) / 2.54) / 6) / Math.PI) * 4096);

    return pulse;
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

  public void resetSensors() {

    MCI1ENC.setSelectedSensorPosition(0);
    MCD4ENC.setSelectedSensorPosition(0);

    // m_gyro.reset();

    gyro.reset();

  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(getRotation2d(), getLeftEncoderdistance(), getRightEncoderdistance(), pose);

  }

  public void arcadeDrive(double fwd, double rot) {

    chasis.arcadeDrive(fwd, -rot);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    motsizq.setVoltage(leftVolts);
    motsder.setVoltage(rightVolts);
    // chasis.feed();
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

    gyro.reset();
    // m_gyro.reset();
  }

  public double getHeading() {

    return 0;
    //return gyro.getAngle();
    // return -m_gyro.getRotation2d().getDegrees();
  }

  public double getTurnRate() {

    return 0;
    //return gyro.getRate();
    // return m_gyro.getRate();
  }

  public Rotation2d getRotation2d() {

    // return m_gyro.getRotation2d();

    return Rotation2d.fromDegrees(0);
    //return Rotation2d.fromDegrees(gyro.getRate());
  }

  public void configPIDDrivTr() {


    MCI1ENC.setInverted(false);
    MCI1ENC.setSensorPhase(false);

    MCI2.setInverted(false);
    MCI2.setSensorPhase(false);

    MCI3.setInverted(false);
    MCI3.setSensorPhase(false);

    MCD4ENC.setInverted(true);
    MCD4ENC.setSensorPhase(true);

    MCD5.setInverted(true);
    MCD5.setSensorPhase(true);

    MCD6.setInverted(true);
    MCD6.setSensorPhase(true);

    MCI2.follow(MCI1ENC);
    MCI3.follow(MCI1ENC);

    MCD5.follow(MCD4ENC);
    MCD6.follow(MCD4ENC);

  }

  public void set_ramp(double ramp){

MCI1ENC.configOpenloopRamp(ramp);
MCI2.configOpenloopRamp(ramp);
MCI3.configOpenloopRamp(ramp);

MCD4ENC.configOpenloopRamp(ramp);
MCD5.configOpenloopRamp(ramp);
MCD6.configOpenloopRamp(ramp);

  }

}
