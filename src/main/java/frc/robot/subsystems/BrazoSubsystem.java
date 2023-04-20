
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KPIDejeinferior;
import frc.robot.Constants.PUERTOSCAN;

public class BrazoSubsystem extends SubsystemBase {

  WPI_TalonSRX motejeabajo = new WPI_TalonSRX(PUERTOSCAN.PuertMotEjeinferior);
 // WPI_TalonFX motejearriba = new WPI_TalonFX(PUERTOSCAN.PuertMotEjeSuperior);

  CANSparkMax motejesuperioSparkMax= new CANSparkMax(PUERTOSCAN.PuertMotEjeSuperior, MotorType.kBrushless);

  ProfiledPIDController pidprofilejesup = new ProfiledPIDController(0.2, 0, 0.1, new TrapezoidProfile.Constraints(50,60));

  DigitalInput limitejeinferior = new DigitalInput(8);
  DigitalInput limitejesuperior = new DigitalInput(9);

  public BrazoSubsystem() {
  }

  @Override
  public void periodic() {

    SmartDashboard.putBoolean("limitarriba", !limitejesuperior.get());
    SmartDashboard.putBoolean("limitinferior", !limitejeinferior.get());
    SmartDashboard.putNumber("grados eje inferior", gradosEjeInferior());
    SmartDashboard.putNumber("grados eje superior", gradosEjeSuperior());
   //SmartDashboard.get("grados", motejesuperioSparkMax.getEncoder());
  }

  public void ejeinferior(double velinf) { // movimiento con control

    if (limitejeinferior.get() && gradosEjeInferior() < 85) {

      motejeabajo.set(-velinf);

    } else if (!limitejeinferior.get() && velinf < 0.001) {

      motejeabajo.set(-velinf);

    } else if (!limitejeinferior.get() && velinf > 0.001) {

      motejeabajo.set(0);

    } else if (gradosEjeInferior() > 80 && velinf < 0.001) {
      motejeabajo.set(0);

    } else if (gradosEjeInferior() > 80 && velinf > 0.001 && limitejeinferior.get()) {
      motejeabajo.set(-velinf);

    }
    if (!limitejeinferior.get()) {

      motejeabajo.setSelectedSensorPosition(0);

    }
  }


  public void ejesuperior(double velsup ) { // movimiento con control

    if (limitejesuperior.get() && gradosEjeSuperior() > -280) {

      motejesuperioSparkMax.set(-velsup);

    } else if (!limitejesuperior.get() && velsup < 0.001) {

      motejesuperioSparkMax.set(-velsup);

    } else if (!limitejesuperior.get() && velsup > 0.001) {

      motejesuperioSparkMax.set(0);

    } else if (gradosEjeSuperior() < -270 && velsup > 0.001 && limitejesuperior.get()) {
      motejesuperioSparkMax.set(-velsup);
    } else if (gradosEjeSuperior() < -270 && velsup < 0.001 && limitejesuperior.get()) {
      motejesuperioSparkMax.set(0);

    }

    if (!limitejesuperior.get()) {

      motejesuperioSparkMax.getEncoder().setPosition(0);
      
    }

  }

  public double gradosEjeSuperior() {

    double pulsosSensor = motejesuperioSparkMax.getEncoder().getPosition();

    double vuelta_eje_con_reduccion = ((pulsosSensor) / (100));

    double gradoseje = (vuelta_eje_con_reduccion / 3.285714) * (360) ;

    return gradoseje;
  } 

  public double gradosEjeInferior() {

    double pulsosSensor = motejeabajo.getSelectedSensorPosition();

    double vuelta_eje_con_reduccion = ((pulsosSensor) / (4096));

    double gradoseje = (vuelta_eje_con_reduccion / 4.5714) * (360);

    return gradoseje;

  }

  public double gradosEjeSuperiorApulsos(double gradosSup) {

    double pulsos = ((((((gradosSup) / 360) * 3.285714) * 100)));

    return pulsos;
  }

  public double gradosEjeInferiorApulsos(double gradosInf) {

    double pulsos = (((((gradosInf) / 360) * 4.5714) * 4096));

    return pulsos;
  }

  public void returnhome(boolean status) {

    if (status) {

      if (!limitejeinferior.get()) {

        motejeabajo.set(0);

      } else {

        motejeabajo.set(-0.8);

      }

      if (!limitejesuperior.get()) {

        motejesuperioSparkMax.set(0);
      } else {

        motejesuperioSparkMax.set(-0.6);

      }

    }

  }

  public void movimiento_brazo_angulo(double gradosinferior, double gradosuperior) {
    
    double posicion_inferior = gradosEjeInferiorApulsos(gradosinferior);
    double posicion_superior = gradosEjeSuperiorApulsos(gradosuperior);

    //double cuantotalta=(posicion_superior-gradosEjeSuperior());
    motejesuperioSparkMax.set(pidprofilejesup.calculate(gradosuperior, posicion_superior));
    motejeabajo.set(ControlMode.Position, posicion_inferior*2.5);

  }

  public void ResetEncoderLimit() {

    if (!limitejesuperior.get()) {

    //  motejearriba.setSelectedSensorPosition(0);
motejesuperioSparkMax.getEncoder().setPosition(0);

    }

    if (!limitejeinferior.get()) {

      motejeabajo.setSelectedSensorPosition(0);

    }
  }

  // Configuracion de motores para que no se muevan mucho al momento de tener el
  // robot encendido
  // *NO MOVERLE*
  public void config_motor_eje_inf() {

    motejeabajo.configFactoryDefault();

    motejeabajo.configNominalOutputForward(0, KPIDejeinferior.kTimeoutMs);
    motejeabajo.configNominalOutputReverse(0, KPIDejeinferior.kTimeoutMs);
    motejeabajo.configPeakOutputForward(1, KPIDejeinferior.kTimeoutMs);
    motejeabajo.configPeakOutputReverse(-1, KPIDejeinferior.kTimeoutMs);
    /* Config the Velocity closed loop gains in slot0 */
    motejeabajo.config_kF(KPIDejeinferior.kPIDLoopIdx, KPIDejeinferior.kF,
        KPIDejeinferior.kTimeoutMs);
    motejeabajo.config_kP(KPIDejeinferior.kPIDLoopIdx, KPIDejeinferior.KP,
        KPIDejeinferior.kTimeoutMs);
    motejeabajo.config_kI(KPIDejeinferior.kPIDLoopIdx, KPIDejeinferior.kI,
        KPIDejeinferior.kTimeoutMs);
    motejeabajo.config_kD(KPIDejeinferior.kPIDLoopIdx, KPIDejeinferior.kD,
        KPIDejeinferior.kTimeoutMs);

    motejeabajo.setInverted(true);
    motejeabajo.setSensorPhase(true);

    motejesuperioSparkMax.setInverted(true);
  }

  /*
   * Engranes Robot, conversion encorders:
   * 
   * Eje inferior:
   * 
   * 14 dientes en motor
   * 64 dientes
   * 
   * 
   * 
   * 4.5714
   * 
   * 
   * 
   * 
   * 
   * Eje superior:
   * 
   * 46 dientes
   * 14 dientes a motor
   * 
   * reduccion 100
   * 
   * 3.285714
   * 
   */

}



