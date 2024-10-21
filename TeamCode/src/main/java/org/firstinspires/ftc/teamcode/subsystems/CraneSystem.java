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

    }

    public void dumpAtLowBasket() {

    }

    public void moveToIntake() {

    }

    public boolean isReadyToIntake() {
        return robotBucket.isAtIntakePosition() && robotVerticalExtender.isAtBasePosition();
    }
}
