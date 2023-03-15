
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PUERTOSCAN;

public class BrazoSubsystem extends SubsystemBase {

  WPI_TalonSRX motejeabajo=new WPI_TalonSRX(PUERTOSCAN.PuertMotEjeinferior);
  WPI_TalonFX motejearriba=new WPI_TalonFX(PUERTOSCAN.PuertMotEjeSuperior);

  public BrazoSubsystem() {}

  @Override
  public void periodic() {}


public void ejeinferior(double velocidadmotorinferior){
motejeabajo.set(velocidadmotorinferior);
}

public void ejesuperior(double velsup){
  motejearriba.set(velsup);


}


public void config_motor_eje_sup(){

//  motejearriba.set(TalonFXControlMode.Velocity,)

  /*MOTORCAPUCHA.configNominalOutputForward(0, Constants.KPIDCapucha.kTimeoutMs);
  MOTORCAPUCHA.configNominalOutputReverse(0, Constants.KPIDCapucha.kTimeoutMs);
  MOTORCAPUCHA.configPeakOutputForward(1, Constants.KPIDCapucha.kTimeoutMs);
  MOTORCAPUCHA.configPeakOutputReverse(-1, Constants.KPIDCapucha.kTimeoutMs);
  /* Config the Velocity closed loop gains in slot0 */
 /*  MOTORCAPUCHA.config_kF(Constants.KPIDCapucha.kPIDLoopIdx, Constants.KPIDCapucha.kGains_Velocit.kF,
      Constants.KPIDCapucha.kTimeoutMs);
  MOTORCAPUCHA.config_kP(Constants.KPIDCapucha.kPIDLoopIdx, Constants.KPIDCapucha.kGains_Velocit.kP,
      Constants.KPIDCapucha.kTimeoutMs);
  MOTORCAPUCHA.config_kI(Constants.KPIDCapucha.kPIDLoopIdx, Constants.KPIDCapucha.kGains_Velocit.kI,
      Constants.KPIDCapucha.kTimeoutMs);
  MOTORCAPUCHA.config_kD(Constants.KPIDCapucha.kPIDLoopIdx, Constants.KPIDCapucha.kGains_Velocit.kD,
      Constants.KPIDCapucha.kTimeoutMs);*/




  

}


public double gradosEjeSuperior(){

double pulsosSensor= motejearriba.getSelectedSensorPosition();


double vuelta_eje_con_reduccion=((pulsosSensor)/(4096)/(100));

double gradoseje=(vuelta_eje_con_reduccion/0.608695)*(360);

  return gradoseje;
}

public double gradosEjeInferior(){

  double pulsosSensor= motejeabajo.getSelectedSensorPosition();
  
  double vuelta_eje_con_reduccion=((pulsosSensor)/(4096)/(100));

  double gradoseje=(vuelta_eje_con_reduccion/4.5714)*(360);


  
  return gradoseje;

  
  }



public void config_motor_eje_inf(){


}


/*Engranes Robot, conversion encorders:

Eje inferior:

14 dientes en motor
64 dientes 

4.5714






Eje superior:

46 dientes a motor
28 dientes 

0.608695

 */

}
