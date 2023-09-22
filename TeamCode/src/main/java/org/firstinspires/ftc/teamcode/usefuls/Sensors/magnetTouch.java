package org.firstinspires.ftc.teamcode.usefuls.Sensors;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class magnetTouch {
    private TouchSensor s;

    public magnetTouch(TouchSensor s) {
        this.s = s;
    }

    public boolean check() {
        boolean e = this.s.isPressed();
        return e;
    }

}
