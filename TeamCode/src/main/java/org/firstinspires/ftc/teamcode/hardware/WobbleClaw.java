package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Reicher Robotics on 3/4/2018.
 */

public class WobbleClaw {
    // SERVO POSITIONS
    private final double OPEN = 0.0;
    private final double MID = 0.5;
    private final double CLOSE = 1.0;

    // SERVOS
    private int numServos = 1;
    private int WOBBLE_CLAW_SERVO = 0;

    public Servo[] servos = new Servo[numServos];

    public WobbleClaw(HardwareMap hwMap, String[] servoNames){
        for(int i = 0; i < numServos; i++){
            servos[i] = hwMap.servo.get(servoNames[i]);
        }
    }

    public void open() {
        servos[WOBBLE_CLAW_SERVO].setPosition(OPEN);
    }

    public void mid(){
        servos[WOBBLE_CLAW_SERVO].setPosition(MID);
    }

    public void close(){
        servos[WOBBLE_CLAW_SERVO].setPosition(CLOSE);
    }
}
/*
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class WobbleClaw implements ServoEx {

    Servo servo;
    double maxAngle, minAngle;
    final double maxPosition = 1;
    final double minPosition = 0;

    public WobbleClaw(HardwareMap hw, String servoName) {
        servo = hw.get(Servo.class, servoName);
        maxAngle = 180;
        minAngle = 0;
    }

    public WobbleClaw(HardwareMap hw, String servoName, double maxAngle, double minAngle) {
        servo = hw.get(Servo.class, servoName);
        this.maxAngle = maxAngle;
        this.minAngle = minAngle;
    }


    @Override
    public void rotateDegrees(double angle) {
        angle = getAngle() + angle;
        turnToAngle(angle);
    }

    @Override
    public void turnToAngle(double angle) {
        if(angle > maxAngle)
            angle = maxAngle;
        else if(angle < minAngle)
            angle = minAngle;

        setPosition((angle - minAngle) / (getAngleRange()));
    }

    @Override
    public void rotate(double position) {
        position = getPosition() + position;
        setPosition(position);
    }

    @Override
    public void setPosition(double position) {
        if(position > maxPosition)
            servo.setPosition(maxPosition);
        else if(position < minAngle)
            servo.setPosition(minPosition);
        else
            servo.setPosition(position);
    }

    @Override
    public void setRange(double min, double max) {
        this.minAngle = min;
        this.maxAngle = max;
    }

    @Override
    public void setInverted(boolean isInverted) {
        if(isInverted)
            servo.setDirection(Servo.Direction.REVERSE);
        else
            servo.setDirection(Servo.Direction.FORWARD);

    }

    @Override
    public boolean getInverted() {
        if(Servo.Direction.REVERSE == servo.getDirection())
            return true;
        else
            return false;
    }

    @Override
    public double getPosition() {
        return servo.getPosition();
    }

    @Override
    public double getAngle() {
        return getPosition() * getAngleRange() + minAngle;
    }

    public double getAngleRange() {
        return maxAngle - minAngle;
    }

    @Override
    public void disable() {
        servo.close();
    }

    @Override
    public String getDeviceType() {
        String port = Integer.toString(servo.getPortNumber());
        String controller = servo.getController().toString();
        return "SimpleServo: " + port + "; " + controller;
    }
}
 */