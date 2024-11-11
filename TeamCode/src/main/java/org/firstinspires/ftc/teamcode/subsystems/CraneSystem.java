package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.hardware.RobotClaw;
import org.firstinspires.ftc.teamcode.hardware.RobotArm;

public class CraneSystem {

    private RobotClaw robotClaw;
    private RobotArm robotArm;

    public CraneSystem(RobotClaw robotClaw, RobotArm robotArm) {
        this.robotClaw = robotClaw;
        this.robotArm = robotArm;
    }

    public void dumpAtHighBasket(float triggerPressure) {
        robotArm.toHighBasket(triggerPressure);
        if (robotArm.isAtHighBasketPosition()) {
//            robotClaw.toDumpPosition();
        }
    }

    public void dumpAtLowBasket(float triggerPressure) {
        robotArm.toLowBasket(triggerPressure);
        if (robotArm.isAtLowBasketPosition()) {
//            robotClaw.toDumpPosition();
        }
    }

    public void backToBase(float triggerPressure) {
//        robotClaw.toIntakePosition();
//        if (robotClaw.isAtIntakePosition()) {
            robotArm.toBasePosition(triggerPressure);
//        }
    }

    public boolean isReadyToIntake() {
//        return robotClaw.isAtIntakePosition() && robotVerticalExtender.isAtBasePosition();
        return true;
    }

    public void stop() {
        robotArm.stop();
    }
public int getCurrentExtension(){
        return robotArm.getExtensionPosition();
}

    public int getCurrentRotation(){
        return robotArm.getRotationPosition();
    }

}
