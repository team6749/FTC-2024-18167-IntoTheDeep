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

    public void dumpAtHighBasket() {
        robotVerticalExtender.toHighBasket();
        if (robotVerticalExtender.isAtHighBasketPosition()) {
            robotClaw.toDumpPosition();
        }
    }

    public void dumpAtLowBasket() {
        robotVerticalExtender.toLowBasket();
        if (robotVerticalExtender.isAtLowBasketPosition()) {
            robotClaw.toDumpPosition();
        }
    }

    public void backToBase() {
        robotClaw.toIntakePosition();
        if (robotClaw.isAtIntakePosition()) {
            robotVerticalExtender.toBasePosition();
        }
    }

    public boolean isReadyToIntake() {
        return robotClaw.isAtIntakePosition() && robotVerticalExtender.isAtBasePosition();
    }
}
