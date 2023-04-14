// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.Cmd;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class SetDistanceChasis extends CommandBase {


  private final DriveSubsystem driveSubsystem;
  private Supplier<Double> distance_mts;

  public SetDistanceChasis(DriveSubsystem driveSubsystem,Supplier<Double> distance_mts){
  
    this.distance_mts=distance_mts;
  this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
   


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

driveSubsystem.resetEncoders();

driveSubsystem.set_ramp(0.2);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

driveSubsystem.set_distance(distance_mts.get());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    driveSubsystem.CHASIS(0, 0, false, false, false, false);
    driveSubsystem.set_ramp(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
