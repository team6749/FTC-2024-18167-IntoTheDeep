package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.hardware.RobotClaw;
import org.firstinspires.ftc.teamcode.hardware.RobotVerticalExtender;

public class CraneSystem {

    private RobotClaw robotClaw;
    private RobotVerticalExtender robotVerticalExtender;

    public CraneSystem(RobotClaw robotClaw, RobotVerticalExtender robotVerticalExtender) {
        this.robotClaw = robotClaw;
        this.robotVerticalExtender = robotVerticalExtender;
    }

    public void dumpAtHighBasket(float triggerPressure) {
        robotVerticalExtender.toHighBasket(triggerPressure);
        if (robotVerticalExtender.isAtHighBasketPosition()) {
//            robotClaw.toDumpPosition();
        }
    }

    public void dumpAtLowBasket(float triggerPressure) {
        robotVerticalExtender.toLowBasket(triggerPressure);
        if (robotVerticalExtender.isAtLowBasketPosition()) {
//            robotClaw.toDumpPosition();
        }
    }

    public void backToBase(float triggerPressure) {
//        robotClaw.toIntakePosition();
//        if (robotClaw.isAtIntakePosition()) {
            robotVerticalExtender.toBasePosition(triggerPressure);
//        }
    }

    public boolean isReadyToIntake() {
//        return robotClaw.isAtIntakePosition() && robotVerticalExtender.isAtBasePosition();
        return true;
    }

    public void stop() {
        robotVerticalExtender.stop();
    }
}
