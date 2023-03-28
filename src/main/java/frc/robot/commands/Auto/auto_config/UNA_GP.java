// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.auto_config;

import com.fasterxml.jackson.annotation.JsonTypeInfo.None;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.BrazoCmd;
import frc.robot.commands.GarraCmd;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:

// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class UNA_GP extends SequentialCommandGroup {

  /** Creates a new UNA_GP. */
  public UNA_GP() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new BrazoCmd(RobotContainer.brazosubsystem, ()->0.0, ()->0.0, ()->false, ()->false, ()->false, ()->false, ()->true),
    new GarraCmd(RobotContainer.garraSusbsytem, ()->true, ()->false),new GarraCmd(RobotContainer.garraSusbsytem, ()->false, ()->true));

  //  (new BrazoCmd(RobotContainer.brazosubsystem, ()->0.0, ()->0.0, ()->false, ()->false, ()->false, ()->false, ()->true), Commands.sequence(new GarraCmd(RobotContainer.garraSusbsytem, ()->true, null)))
      //new SequentialCommandGroup( new BrazoCmd(RobotContainer.brazosubsystem, ()->0.0, ()->0.0, ()->false, ()->false, ()->false, ()->false, ()->true),new GarraCmd(RobotContainer.garraSusbsytem, ()->true, ()->false),new BrazoCmd(RobotContainer.brazosubsystem, ()->0.0, ()->0.0, ()->false, ()->true, ()->false, ()->false, ()->true),new GarraCmd(RobotContainer.garraSusbsytem, ()->true, ()->false))
  }
}

