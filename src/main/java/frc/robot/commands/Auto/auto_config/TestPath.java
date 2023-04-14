// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.auto_config;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.Auto.DrivetrainRamseteCommand;
import frc.robot.commands.Auto.Cmd.ResetSensors;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class TestPath extends SequentialCommandGroup {
  /** Creates a new TestPath. */

  private static final Trajectory TestPath = PathPlanner.loadPath("TestPath", AutoConstants.kMaxSpeedMetersPerSecond,
      AutoConstants.kMaxAccelerationMetersPerSecondSquared);

  public TestPath() {
    
    addCommands(
      new ParallelDeadlineGroup(new WaitCommand(0.1), new SequentialCommandGroup(new ResetSensors())),
        new ParallelDeadlineGroup(new DrivetrainRamseteCommand(RobotContainer.driveSubsystem, TestPath).robotRelative()));
                
  }

}
