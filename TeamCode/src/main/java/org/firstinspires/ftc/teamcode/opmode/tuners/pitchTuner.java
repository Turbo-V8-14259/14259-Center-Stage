package org.firstinspires.ftc.teamcode.opmode.tuners;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.hardware.Deposit.DepoSlides;
import org.firstinspires.ftc.teamcode.hardware.Deposit.Pitch;
import org.firstinspires.ftc.teamcode.usefuls.Gamepad.stickyGamepad;
import org.firstinspires.ftc.teamcode.usefuls.Motor.DcMotorBetter;

@TeleOp
public class pitchTuner extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pitch pitch = new Pitch(new DcMotorBetter(hardwareMap.get(DcMotorEx.class, "pitch")));
        pitch.stopAndResetEncoder();
        stickyGamepad stickyPad = new stickyGamepad(gamepad1);
        waitForStart();
        while(opModeIsActive()){
            pitch.setPower(gamepad1.left_trigger - gamepad1.right_stick_x);
            telemetry.addData("Pitch encoder position ", pitch.pitchMotor.getCurrentPositionRAW());
            telemetry.update();
        }
    }
}
