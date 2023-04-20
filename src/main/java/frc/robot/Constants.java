package frc.robot;

import java.nio.file.Path;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.Filesystem;

public class Constants {

    public static final class posicionesbrazo {

        public static final class posicionConodoublesubstation { // teleop

            public static double ejesuperior = 83;
            public static double ejeinferior = 7;

        }

        public static final class posicionCuboDoubleSub {

            public static double ejesuperior = 80;
            public static double ejeinferior = 7;

        }

        public static final class posicioncubopiso {
            public static double ejesuperior = 145;
            public static double ejeinferior = 98;
        }

        public static final class posicionSafeArm { // teleop

            public static double ejesuperior = 270;
            public static double ejeinferior = 85;

        }

        public static final class posiciondejarcono {
            public static double ejesuperior = 190;
            public static double ejeinferior = 88;
        }

        public static final class posiciondejarCubo {
            public static double ejesuperior = 95;
            public static double ejeinferior = 30;
        }

        public static final class posiciondejarCuboArriba {
            public static double ejesuperior = 109;
            public static double ejeinferior = 13;
        }

        public static final class posiciondejarAbajo {
            public static double ejesuperior = 33;
            public static double ejeinferior = 0;
        }

        /*
         * public static final class topesvelocidad {
         * public static double Superior = 0.7;
         * public static double Inferior = 0.5;
         * 
         * }
         */
    }

    public static final class AjustesMovimientoChasis {

        public static final class val_Balanceo {
            public static double velocidadmaxima = 500;
            public static double kp = -50;
        }

        public static class autoapuntado { // limelight
            public static double girokp = 0.095;
            public static double distanciakp = -0.095;

            public static double vel_max_giro = 0.4;
            public static double vel_max_distancias = 0.5;
        }

    }

    // PUERTOS DE LOS MOTORES CAN
    public static final class PUERTOSCAN {
        public static int PuertMotDer1yEncoder = 1;
        public static int PuertMotDer2 = 2;
        public static int PuertMotDer3 = 3;
        public static int PuertMotIzq1yEncoder = 4;
        public static int PuertMotIzq2 = 5;
        public static int PuertMotIzq3 = 6;
        public static int   PuertMotEjeSuperior = 7;
        public static int PuertMotEjeinferior = 8;
    }

    // NO MOVERLE AL MENOS SI HICISTE EL TUNEO DE PID CON SYSTEM IDENTIFICATION TOOL
    public static final class DriveConstants {
        public static final double ksVolts = 1.1922;
        public static final double kvVoltSecondsPerMeter = 2.8442;
        public static final double kaVoltSecondsSquaredPerMeter = 0.82181;
        public static final double kPDriveVel = 1.8645;
        public static final double kTrackwidthMeters = 0.64;

        // With Navx
        /*
         * public static final double ksVolts = 1.279;
         * public static final double kvVoltSecondsPerMeter = 2.7816;
         * public static final double kaVoltSecondsSquaredPerMeter = 0.72925;
         * public static final double kPDriveVel = 0.80316;
         * public static final double kTrackwidthMeters = 0.64;
         */

        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
                kTrackwidthMeters);
        public static Path DEPLOY_DIRECTORY = Filesystem.getDeployDirectory().toPath();
    }

    // NO MOVERLE AL MENOS SI HICISTE EL TUNEO DE PID CON SYSTEM IDENTIFICATION TOOL
    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 4;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }

    // NO MOVERLE AL MENOS SI HICISTE EL TUNEO DE PID CON SYSTEM IDENTIFICATION TOOL
    public static final class KPIDchasis {
        public static final int kSlotIdx = 0;

        public static final int kPIDLoopIdx = 0;
        public static final int kTimeoutMs = 30;
        public final static GainsDT kGains_Velocit = new GainsDT(0.80316, 0.000, 0.005, 0, 300,
                1.00);
    }

    // NO MOVERLE A ESTOS VALORES
    public static final class KPIDejesuperior {
        public static final int kSlotIdx = 0;
        public static final int kPIDLoopIdx = 0;
        public static final int kTimeoutMs = 30;
        public final static double KP = 0.2;
        public final static double kI = 0;
        public final static double kD = 0.1;
        public final static double kF = 0;
        public final static int kIzone = 300;
        public final static double kPeakOutput = 1.0;
    }

    // NO MOVERLE A ESTOS VALORES
    public static final class KPIDejeinferior {
        public static final int kSlotIdx = 0;
        public static final int kPIDLoopIdx = 0;
        public static final int kTimeoutMs = 30;
        public final static double KP = 3.4;
        public final static double kI = 0;
        public final static double kD = 0.01;
        public final static double kF = 0;
        public final static int kIzone = 300;
        public final static double kPeakOutput = 1.0;
    }

}
