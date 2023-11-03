package org.firstinspires.ftc.teamcode.hardware.Deposit;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.usefuls.Math.M;
import org.firstinspires.ftc.teamcode.usefuls.Motor.DcMotorBetter;
import org.firstinspires.ftc.teamcode.usefuls.Motor.PID;
import org.firstinspires.ftc.teamcode.usefuls.Motor.ServoMotorBetter;

@Config
public class DepoArm {
    public enum DepoArmState {

    }

    private static final double LEFT_LOWER_BOUND = 0;
    private static final double LEFT_UPPER_BOUND = 1;
    private static final double RIGHT_LOWER_BOUND = 0;
    private static final double RIGHT_UPPER_BOUND = 1;

    private ServoMotorBetter leftArm;
    private ServoMotorBetter rightArm;
    
    public DepoArm(ServoMotorBetter leftArm, ServoMotorBetter rightArm) {
        this.leftArm = leftArm;
        this.leftArm.setLowerBound(DepoArm.LEFT_LOWER_BOUND);
        this.leftArm.setUpperBound(DepoArm.LEFT_UPPER_BOUND);

        this.rightArm = rightArm;
        this.rightArm.setLowerBound(DepoArm.RIGHT_LOWER_BOUND);
        this.rightArm.setUpperBound(DepoArm.RIGHT_UPPER_BOUND);
    }
}
