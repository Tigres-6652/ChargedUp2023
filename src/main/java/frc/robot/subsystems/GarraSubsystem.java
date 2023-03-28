// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GarraSubsystem extends SubsystemBase {

  Solenoid pistongarra = new Solenoid(PneumaticsModuleType.CTREPCM, 0);

  

  
  boolean conoengarra;

  public GarraSubsystem() {
  }

  @Override
  public void periodic() {


  }

  public void pistongarrastate(boolean abre, boolean cierra) {

    if (( cierra)) {
      pistongarra.set(false);
      conoengarra = false;
    }

    if (abre ) {
      pistongarra.set(true);
      conoengarra = false;
    }

  }
}
