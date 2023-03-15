
package frc.robot.commands.Utils;

import java.io.IOException;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

public class TrajectoryLoader {

    private static final TrajectoryConfig MAX_SPEED_TRAJECTORY = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                    .setKinematics(DriveConstants.kDriveKinematics);

    private static final Trajectory DEFAULT_TRAJECTORY = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d()),
            List.of(),
            new Pose2d(1, 0, new Rotation2d()),
            new TrajectoryConfig(0.1, 0.1)
                    .setKinematics(DriveConstants.kDriveKinematics));

    public static Trajectory getTrajectory(String path) {
        try {
            return TrajectoryUtil.fromPathweaverJson(DriveConstants.DEPLOY_DIRECTORY.resolve(path));
        } catch (IOException e) {
            DriverStation.reportError("Error Opening (✿◠‿◠)  \"" + path + "\"  ! ʕ•́ᴥ•̀ʔっ", e.getStackTrace());

            System.err.println("Error Opening (っ＾▿＾)💨 \"" + path + "\"! ヽ(*・ω・)");
            System.out.println(e.getStackTrace());

            return DEFAULT_TRAJECTORY;
        }
    }

    public static Trajectory getTrajectory(String... paths) {
        Trajectory trajectory = getTrajectory(paths[0]);

        for (int i = 1; i < paths.length; ++i) {
            trajectory = trajectory.concatenate(getTrajectory(paths[i]));
        }

        return trajectory;
    }

    public static Trajectory getLine(double distance) {
        return TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d()),
                List.of(),
                new Pose2d(distance, 0, new Rotation2d()),
                MAX_SPEED_TRAJECTORY.setReversed(distance < 0));
    }

}
