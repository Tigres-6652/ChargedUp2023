// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.sql.Time;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GarraSubsystem extends SubsystemBase {

  Solenoid pistongarra = new Solenoid(PneumaticsModuleType.CTREPCM, 0);

  I2C.Port iPort = I2C.Port.kOnboard;

  ColorSensorV3 sensotcolor = new ColorSensorV3(iPort);

  ColorMatch m_ColorMatcher = new ColorMatch();

  boolean conoengarra;

  boolean statepiston;
  boolean boton;
  boolean ejecucion = false;
  boolean cambio;
  double tiempo;
  boolean cumplimientotiempo;

  public GarraSubsystem() {
  }

  @Override
  public void periodic() {
    // Color detectColor=sensotcolor.getColor();
    // SmartDashboard.putNumber("sensorcolor", sensotcolor);

    SmartDashboard.putNumber("red", sensotcolor.getRed());
    SmartDashboard.putNumber("green", sensotcolor.getGreen());
    SmartDashboard.putNumber("blue", sensotcolor.getBlue());
    SmartDashboard.putBoolean("boton", boton);
    SmartDashboard.putBoolean("time", cumplimientotiempo);

    /*
     * if(sensotcolor.getRed()>250&&sensotcolor.getGreen()>400&&sensotcolor.getBlue(
     * )<200){
     * amarillo=true;
     * }else{
     * amarillo=false;
     * }
     */

    /*
     * if(sensotcolor.getRed()>250&&sensotcolor.getGreen()>400&&sensotcolor.getBlue(
     * )<200){
     * amarillo=true;
     * }else{
     * amarillo=false;
     * }
     */

    SmartDashboard.putBoolean("amarillo", conoengarra);
    SmartDashboard.putBoolean("piston", pistongarra.get());

    SmartDashboard.putNumber("proximidad", sensotcolor.getProximity());
    SmartDashboard.putBoolean("statepiston", statepiston);

  }

  public void pistongarrastate(boolean stateboton) {


    if (conoengarra || stateboton) {

      if (statepiston) {
      }
      statepiston = false;

    }

    if (stateboton && !statepiston) {
      statepiston = true;
      pistongarra.set(true);
      Timer.delay(0.45);

    }


    if (sensotcolor.getProximity() > 105) {

      
      conoengarra = true;

    } else {
      conoengarra = false;
    }

    pistongarra.set(statepiston);

  }
}
