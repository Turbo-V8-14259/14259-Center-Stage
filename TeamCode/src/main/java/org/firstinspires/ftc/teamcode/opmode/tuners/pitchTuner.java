package org.firstinspires.ftc.teamcode.opmode.tuners;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.hardware.Deposit.Pitch;
import org.firstinspires.ftc.teamcode.usefuls.Gamepad.stickyGamepad;
import org.firstinspires.ftc.teamcode.usefuls.Motor.DcMotorBetter;

@TeleOp
public class pitchTuner extends LinearOpMode {

    double pitchPower = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        Pitch pitch = new Pitch(new DcMotorBetter(hardwareMap.get(DcMotorEx.class, "Pitch")));
        pitch.stopAndResetEncoder();
        pitch.pitchMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while(opModeIsActive()){
            pitchPower = gamepad1.left_trigger - gamepad1.right_trigger;
            pitch.setPower(pitchPower);
            pitch.update();
            telemetry.addData("Pitch encoder position ", pitch.pitchMotor.getCurrentPositionRAW());
            telemetry.update();
        }
    }
}
