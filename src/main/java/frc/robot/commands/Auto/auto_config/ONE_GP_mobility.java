// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.auto_config;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.BrazoCmd;
import frc.robot.commands.GarraCmd;
import frc.robot.commands.Auto.DrivetrainRamseteCommand;
import frc.robot.subsystems.GarraSubsystem;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ONE_GP_mobility extends SequentialCommandGroup {


  private static final Trajectory ONE_GP_individual = PathPlanner.loadPath("ONE_GP_individual",
      Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);


  public ONE_GP_mobility() {

    addCommands(

    /*      new ParallelDeadlineGroup(new BrazoCmd(RobotContainer.brazosubsystem, () -> 0.0, () -> 0.0, () -> false, () -> false,
            () -> false, () -> false, () -> true),new GarraCmd(RobotContainer.garraSusbsytem,()-> true, ()->false), new ,
        new ParallelDeadlineGroup(new BrazoCmd(RobotContainer.brazosubsystem, () -> 0.0, () -> 0.0, () -> false, () -> true,
            () -> false, () -> false, () -> false)),
        new ParallelDeadlineGroup(new GarraCmd(RobotContainer.garraSusbsytem, () -> true, () -> false), new WaitCommand(3)),

        new ParallelDeadlineGroup(
            new DrivetrainRamseteCommand(RobotContainer.driveSubsystem, ONE_GP_individual).robotRelative()));
             */
    );
  }

}
