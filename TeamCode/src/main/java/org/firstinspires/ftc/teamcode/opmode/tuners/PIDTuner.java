package org.firstinspires.ftc.teamcode.opmode.tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp
public class PIDTuner extends LinearOpMode {
    public static double setTarget;

    @Override
    public void runOpMode() throws InterruptedException {
        //initalize here
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();

        while(opModeIsActive()){



            //update here
            //add data here
            telemetry.update();
        }
    }
}
