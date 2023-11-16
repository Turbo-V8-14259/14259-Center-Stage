package org.firstinspires.ftc.teamcode.opmode.tests.servoTests;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.hardware.Deposit.DepoTurret;

@Config
@TeleOp
@Disabled

public class depoTurretTest extends LinearOpMode {
    public static double targetSet = 180;
    private DepoTurret turret;
    @Override
    public void runOpMode() throws InterruptedException {
        turret = new DepoTurret(hardwareMap.get(CRServo.class, "axon"), hardwareMap.get(AnalogInput.class, "input"));
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while(opModeIsActive()){
            turret.target = targetSet;
            turret.update();
            telemetry.addData("position", turret.updateAngle());
            telemetry.addData("target", targetSet);
            telemetry.update();
        }
    }

}
