package org.firstinspires.ftc.teamcode.opmode.tests.servoTests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.hardware.Deposit.DepoTurret;
import org.firstinspires.ftc.teamcode.hardware.Sensors.Imu;
import org.firstinspires.ftc.teamcode.usefuls.Gamepad.stickyGamepad;
@Config
@TeleOp
@Disabled
public class turretAngleLock extends LinearOpMode {

    DepoTurret turret;
    Imu imu;
    stickyGamepad gamepada;

    double IMUAngle = 0;

    double turretAngle = 0;

    public static double targetLockedAngle = 0;

    boolean a = false;
    boolean toggled = true;
    @Override
    public void runOpMode() throws InterruptedException {
        turret = new DepoTurret(hardwareMap.get(CRServo.class, "axon"), hardwareMap.get(AnalogInput.class, "input"));
        imu = new Imu(hardwareMap.get(IMU.class, "imu"));
        gamepada = new stickyGamepad(gamepad1);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        imu.init();
        waitForStart();
        while(opModeIsActive()){
            updateAll();
            if(gamepada.a && !a){ //locks on for the angles given at the moment
                turret.angleLockSet(targetLockedAngle, IMUAngle);
                turret.setState(DepoTurret.TurretState.ANGLE_LOCK);
                toggled = false;
                a = true;
            }else if(gamepada.a && a){ //goes to preset
                turret.setState(DepoTurret.TurretState.TELE_SCORING);
                toggled = true;
                a = false;
            }//toggle

            //It's a bit late at night, everything worked when I actually looked at the data but when I wasn't paying much attention it seemed like the target angle was drifting (preset) ?
            // todo: make sure im not high

            IMUAngle = imu.updateAngleWrapped();
            turretAngle = turret.updateAngle();
            telemetry.addData("toggled?", toggled);
            telemetry.addData("turret target", turret.target);
            telemetry.addData("turret angle", turretAngle);
            telemetry.addData("target locked angle", targetLockedAngle);
            telemetry.addData("IMU angle", IMUAngle);
        }
    }

    public void updateAll(){
        gamepada.update();
        telemetry.update();
        turret.update();
        imu.update();
    }
}
