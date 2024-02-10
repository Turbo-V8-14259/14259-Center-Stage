package org.firstinspires.ftc.teamcode.opmode.tests.motorTests.eachMotorIndividual;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.Intake.Intake;
import org.firstinspires.ftc.teamcode.hardware.Intake.LTIntake;
import org.firstinspires.ftc.teamcode.usefuls.Gamepad.stickyGamepad;
import org.firstinspires.ftc.teamcode.usefuls.Motor.ServoMotorBetter;


@TeleOp(name = "INTAKE TEST")
public class IntakeTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, "Intake");
        Servo intakeArm1 = hardwareMap.get(Servo.class, "intakeArm");
        Servo intakeArm2 = hardwareMap.get(Servo.class, "intakeArm2");
        LTIntake intake = new LTIntake(intakeMotor, new ServoMotorBetter(intakeArm2), new ServoMotorBetter(intakeArm1));
        stickyGamepad gamepad = new stickyGamepad(gamepad1);
//        intakeArm2.setPosition(1); // down position, 0.62 is up
//        intakeArm1.setPosition(0); //down position, 0.38 is up

        waitForStart();
        while(opModeIsActive()){
            intake.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
            if(gamepad.dpad_up){
                intake.setState(LTIntake.IntakeState.INITIALIZE);
            }else if(gamepad.dpad_down){
                intake.setState(LTIntake.IntakeState.INTAKE_TELE);
            }
            intake.update();
//            telemetry.addData("use the gamepad1 triggers to control the intake", intakeMotor.getPower());
//            telemetry.addData("intake arm 1 pos (exph port 2)", intakeArm1.getPosition());
//            telemetry.addData("intake arm 2 pos (exph port 3)", intakeArm2.getPosition());
            telemetry.update();
            gamepad.update();
        }
    }
}
