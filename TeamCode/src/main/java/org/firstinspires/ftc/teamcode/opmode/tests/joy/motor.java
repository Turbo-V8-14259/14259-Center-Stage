package org.firstinspires.ftc.teamcode.opmode.tests.joy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.usefuls.Timer;


@TeleOp
public class motor extends LinearOpMode{

    int target=577;
    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime time = new ElapsedTime();
        DcMotor motor1 = hardwareMap.get(DcMotor.class,"motor1");
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double lastError = 0;
        double lastTime = 0;
        double currentError = 0;
        double currentTime = 0;
        waitForStart();

        while(opModeIsActive()){
            lastError = currentError;
            currentError = target - motor1.getCurrentPosition();
            double kp = currentError*0.1;
            lastTime = currentTime;
            currentTime = time.milliseconds();
            double kd = ((currentError-lastError)/(currentTime-lastTime))*0.5;
            double pout = kp+kd;
            motor1.setPower(pout);
            telemetry.addData("Encoder ", motor1.getCurrentPosition());
            telemetry.update();
        }
    }

}
