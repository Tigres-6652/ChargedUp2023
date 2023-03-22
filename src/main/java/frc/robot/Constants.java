package frc.robot;

import java.nio.file.Path;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.Filesystem;

public class Constants {

    public static final class posicionesbrazo {

        public static final class posiciondoublesubstation {

            public static double ejesuperior=-112;
            public static double ejeinferior=0;
        
        }

        public static final class posicioncentrodemasaabajoydentroframe {

            public static double ejesuperior=-35;
            public static double ejeinferior=0;

        }

        public static final class posiciondejarcono {
            public static double ejesuperior=-130;
            public static double ejeinferior=20;
        }

        public static final class topesvelocidad{
            public static double Superior=0.5;
            public static double Inferior=0.3;

        }
    }

public static final class AjustesMovimientoChasis{

public static final class val_Balanceo{
    public static double velocidadmaxima=0.5;
    public static double kp=0.05;
}

public static class autoapuntado{
public static double girokp=0.095;
public static double distanciakp=-0.095;
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
        public static int PuertMotEjeSuperior = 7;
        public static int PuertMotEjeinferior = 8;
    }

    // NO MOVERLE AL MENOS SI HICISTE EL TUNEO DE PID CON SYSTEM IDENTIFICATION TOOL
    public static final class DriveConstants {
        public static final double ksVolts = 1.279;
        public static final double kvVoltSecondsPerMeter = 2.7816;
        public static final double kaVoltSecondsSquaredPerMeter = 0.72925;
        public static final double kPDriveVel = 0.80316;
        public static final double kTrackwidthMeters = 0.64;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
                kTrackwidthMeters);
        public static Path DEPLOY_DIRECTORY = Filesystem.getDeployDirectory().toPath();
    }

    // NO MOVERLE AL MENOS SI HICISTE EL TUNEO DE PID CON SYSTEM IDENTIFICATION TOOL
    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 2;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }

    // NO MOVERLE AL MENOS SI HICISTE EL TUNEO DE PID CON SYSTEM IDENTIFICATION TOOL
    public static final class KPIDchasis {
        public static final int kSlotIdx = 0;
        
        public static final int kPIDLoopIdx = 0;
        public static final int kTimeoutMs = 30;
        public final static GainsDT kGains_Velocit = new GainsDT(2.0033, 0.000, 0.005, 1023.0 / 20660.0, 300,
                1.00);
    }

    // NO MOVERLE A ESTOS VALORES
    public static final class KPIDejesuperior {
        public static final int kSlotIdx = 0;
        public static final int kPIDLoopIdx = 0;
        public static final int kTimeoutMs = 30;
        public final static double KP = 0.003;
        public final static double kI = 0;
        public final static double kD = 0.005;
        public final static double kF = 1023.0 / 20660.0;
        public final static int kIzone = 300;
        public final static double kPeakOutput = 1.00;
    }

    // NO MOVERLE A ESTOS VALORES
    public static final class KPIDejeinferior {
        public static final int kSlotIdx = 0;
        public static final int kPIDLoopIdx = 0;
        public static final int kTimeoutMs = 30;
        public final static double KP = 0.003;
        public final static double kI = 0;
        public final static double kD = 0.005;
        public final static double kF = 1023.0 / 20660.0;
        public final static int kIzone = 300;
        public final static double kPeakOutput = 1.00;
    }

}
