package org.firstinspires.ftc.teamcode.opmode.tests.servoTests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.Intake.Intake;
import org.firstinspires.ftc.teamcode.usefuls.Gamepad.stickyGamepad;
import org.firstinspires.ftc.teamcode.usefuls.Motor.ServoMotorBetter;


@TeleOp
public class intakeTest extends LinearOpMode {

    Intake intake;
    stickyGamepad gamepadOne;


    @Override
    public void runOpMode() throws InterruptedException {
        gamepadOne = new stickyGamepad(gamepad1);
        intake = new Intake(hardwareMap.get(DcMotorEx.class, "Intake"), new ServoMotorBetter(hardwareMap.get(Servo.class, "intakeArm")));
        intake.setState(Intake.IntakeState.INITIALIZE);
        intake.update();
        waitForStart();

        while(opModeIsActive()){
            if(gamepadOne.dpad_up){
                intake.setState(Intake.IntakeState.INCRIMENT_UP);
            }else if(gamepadOne.dpad_down) {
                intake.setState(Intake.IntakeState.INCRIMENT_DOWN);
            }
            intake.setPower(gamepad1.left_trigger - gamepad1.right_trigger);

            telemetry.addData("intake", intake.getState());
            telemetry.addData("intake power", intake.intakeMotor.getPower());
            telemetry.addData("intake position", intake.target);
            gamepadOne.update();
            telemetry.update();
            intake.update();
        }
    }
}
