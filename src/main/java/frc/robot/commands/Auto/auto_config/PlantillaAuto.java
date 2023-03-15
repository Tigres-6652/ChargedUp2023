
package frc.robot.commands.Auto.auto_config;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.Auto.DrivetrainRamseteCommand;


public class PlantillaAuto extends SequentialCommandGroup {
 
  private static final Trajectory Trajectory_test = PathPlanner.loadPath("testpath", Constants.AutoConstants.kMaxSpeedMetersPerSecond , Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
  
  public PlantillaAuto() {
    
    addCommands(

new ParallelDeadlineGroup(new DrivetrainRamseteCommand(RobotContainer.driveSubsystem, Trajectory_test) .robotRelative()) );
  }
}
