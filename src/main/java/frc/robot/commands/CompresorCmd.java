
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CompresorSubsystem;

public class CompresorCmd extends CommandBase {

  private final CompresorSubsystem compresorSubsystem;

  private final boolean state;

  public CompresorCmd(CompresorSubsystem compresorSubsystem, boolean state) {

    this.state = state;
    this.compresorSubsystem = compresorSubsystem;
    addRequirements(compresorSubsystem);

  }

  @Override
  public void initialize() {
    compresorSubsystem.Compresor(false);

  }

  @Override
  public void execute() {

    compresorSubsystem.Compresor(state);

  }

  @Override
  public void end(boolean interrupted) {

    compresorSubsystem.Compresor(false);

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
