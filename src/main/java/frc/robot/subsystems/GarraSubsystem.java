// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GarraSubsystem extends SubsystemBase {


  WPI_VictorSPX motor_garra=new WPI_VictorSPX(9);


  PowerDistribution pdp=new PowerDistribution(0, ModuleType.kCTRE);

  public GarraSubsystem() {
  }

  @Override
  public void periodic() {


  }

  public void stategarra(boolean lanzarcubo, boolean lanzarcono,boolean agarrar) {

SmartDashboard.putNumber("motor_garra",pdp.getCurrent(4));

    if (( lanzarcubo)) {

      motor_garra.set(-1);

    }else if (lanzarcono) {
      motor_garra.set(-0.2);

        
    }else if(agarrar){
      motor_garra.set(0.4);
    }else{

      motor_garra.set(0);

    }

  }

  public  void velocidadmotor(double velocidad){

    motor_garra.set(velocidad);

  }
}
