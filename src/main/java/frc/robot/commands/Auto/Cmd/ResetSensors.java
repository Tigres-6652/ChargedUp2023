// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.Cmd;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ResetSensors extends CommandBase {

  private final DriveSubsystem driveSubsystem;

  public ResetSensors(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;

    addRequirements(driveSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    driveSubsystem.resetSensors();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
