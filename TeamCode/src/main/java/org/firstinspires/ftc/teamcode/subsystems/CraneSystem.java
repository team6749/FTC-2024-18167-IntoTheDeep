package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.hardware.RobotBucket;
import org.firstinspires.ftc.teamcode.hardware.RobotVerticalExtender;

public class CraneSystem {

    private RobotBucket robotBucket;
    private RobotVerticalExtender robotVerticalExtender;

    CraneSystem(RobotBucket robotBucket, RobotVerticalExtender robotVerticalExtender) {
        this.robotBucket = robotBucket;
        this.robotVerticalExtender = robotVerticalExtender;
    }

    public void dumpAtHighBasket() {
        robotVerticalExtender.toHighBasket();
        if (robotVerticalExtender.isAtHighBasketPosition()) {
            robotBucket.toDumpPosition();
        }
    }

    public void dumpAtLowBasket() {
        robotVerticalExtender.toLowBasket();
        if (robotVerticalExtender.isAtLowBasketPosition()) {
            robotBucket.toDumpPosition();
        }
    }

    public void backToBase() {
        robotBucket.toIntakePosition();
        if (robotBucket.isAtIntakePosition()) {
            robotVerticalExtender.toBasePosition();
        }
    }

    public boolean isReadyToIntake() {
        return robotBucket.isAtIntakePosition() && robotVerticalExtender.isAtBasePosition();
    }
}
