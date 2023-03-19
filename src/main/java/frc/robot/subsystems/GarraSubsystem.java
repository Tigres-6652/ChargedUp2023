// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.posicionesbrazo.DistanciaCono;

public class GarraSubsystem extends SubsystemBase {

  Solenoid pistongarra = new Solenoid(PneumaticsModuleType.CTREPCM, 0);

  I2C.Port iPort = I2C.Port.kOnboard;

  ColorSensorV3 sensorcolor = new ColorSensorV3(iPort);

  boolean conoengarra;
  boolean statepiston;
  boolean boton;

  public GarraSubsystem() {
  }

  @Override
  public void periodic() {

    SmartDashboard.putBoolean("SensorProximidad", conoengarra);
    SmartDashboard.putNumber("Proximidad Cono", sensorcolor.getProximity());

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

    if (sensorcolor.getProximity() > DistanciaCono.DistanciaDeteccion) {
      conoengarra = true;
    } else {
      conoengarra = false;
    }
    pistongarra.set(statepiston);

  }
}
