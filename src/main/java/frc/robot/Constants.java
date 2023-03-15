package frc.robot;

import java.nio.file.Path;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.Filesystem;

public class Constants {

    public static final class PUERTOSCAN {

        public static int PuertMotDer1yEncoder = 1;
        public static int PuertMotDer2 = 2;
        public static int PuertMotDer3 = 3;

        public static int PuertMotIzq1yEncoder = 6;
        public static int PuertMotIzq2 = 5;
        public static int PuertMotIzq3 = 4;

        public static int PuertMotEjeSuperior=7;
        public static int PuertMotEjeinferior=1;




    }


    public static final class DriveConstants {


        public static final double ksVolts = 0.79733;
        public static final double kvVoltSecondsPerMeter = 3.0761;
        public static final double kaVoltSecondsSquaredPerMeter = 0.79086;

        public static final double kPDriveVel = 2.0033;

        public static final double kTrackwidthMeters = 0.69;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
                kTrackwidthMeters);

        public static Path DEPLOY_DIRECTORY = Filesystem.getDeployDirectory().toPath();

    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 1.5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1.5;
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

    }

    public static final class KPIDchasis {

        
        public static final int kSlotIdx = 0;
        public static final int kPIDLoopIdx = 0;
        public static final int kTimeoutMs = 30;
        public final static GainsDT kGains_Velocit = new GainsDT(2.0033, 0.000, 0.005, 1023.0 / 20660.0, 300,
                1.00);

    }


    public static final class KPIDejesuperior {

        public static final int kSlotIdx = 0;
        public static final int kPIDLoopIdx = 0;
        public static final int kTimeoutMs = 30;
        //public final static GainsCapucha kGains_Velocit = new GainsCapucha(0.25, 0.001, 20, 1023.0 / 7200.0, 300, 1.00);

        public final static double KP=0.003;
        public final static double kI=0;
        public final static double kD=0.005;
        public final static double kF=1023.0 / 20660.0;
        public final static int kIzone=300;
        public final static double kPeakOutput=1.00;

    }

    
    public static final class KPIDejeinferior {

        public static final int kSlotIdx = 0;
        public static final int kPIDLoopIdx = 0;
        public static final int kTimeoutMs = 30;
        //public final static GainsCapucha kGains_Velocit = new GainsCapucha(0.25, 0.001, 20, 1023.0 / 7200.0, 300, 1.00);

        public final static double KP=0.003;
        public final static double kI=0;
        public final static double kD=0.005;
        public final static double kF=1023.0 / 20660.0;
        public final static int kIzone=300;
        public final static double kPeakOutput=1.00;

    }


    

}
