package org.firstinspires.ftc.teamcode.drive.posePID2;
import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.usefuls.Math.M;

@Config
public class DTConstants {
    public static double xyP = 0.15, xyI = 0, xyD = .2;
    public static double rP = 0.5, rI = 0, rD = 0;
    public static double RBasePower = 0.05;
    public static double XYBasePower = 0.05;

    public static double maxAxialPower = 1;
    public static double maxAngularPower = 1;
    public static double allowedAxialError = 1;
    public static double allowedAngularError = M.toRadians(.5);
}
