package org.firstinspires.ftc.teamcode.Thread;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.SharedFunctionality.RobotHardwareMap;


@SuppressWarnings({"unused", "WeakerAccess"})
public class Variables {

    private RobotHardwareMap robot = new RobotHardwareMap();

    private static final double     COUNTS_PER_MOTOR_REV    = 28 ;    // eg: TETRIX Motor Encoder
    private static final double     DRIVE_GEAR_REDUCTION    = 40;//3.7 ; //40    // This is < 1.0 if geared UP 40:1 reduce to 160 rpm
    private static final double     WHEEL_DIAMETER_MM   = 95 ;     // For figuring circumference
    public static final double     COUNTS_PER_MM         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_MM * Math.PI);

    private Orientation angles;
    private BNO055IMU imu;

    int count = 0;
    private double distance;
    private double stone2Distance;

    private int position;
    private boolean third;
    private boolean start = true;
    private double heading;
    private double roll;
    private double pitch;
    private double globalAngle;
    private Orientation lastAngles;
    private double avgEncoder;
    private boolean begin = true;
    private double beginAngle = 0;
    private boolean beginFlip;
    private boolean stopL;
    private boolean startL;
    private boolean strafeL;
    private boolean stopR;
    private boolean startR;
    private boolean strafeR;
    private boolean turning;
    private boolean turningA;
    private boolean turningB;
    private boolean turningX;
    private boolean turningY;
    private double SP;
    private double PV;
    private boolean reset = true;
    private double distanceL;
    private double distanceR;
    private double distanceB;
    private double distanceFM;
    private boolean opModeActive = true;

    public Variables(HardwareMap amasterConfig, Variables var){
        robot.init(amasterConfig);
        this.imu = robot.imu;
        Sensors sensors = new Sensors(var, robot);
        Thread Tsensors = new Thread(sensors);
        Tsensors.start();

    }


    public Orientation getLastAngles() {
        return lastAngles;
    }

    public void setLastAngles(Orientation lastAngles) {
        this.lastAngles = lastAngles;
    }

    public Orientation getAngles() {
        return angles;
    }

    public void setAngles(Orientation angles) {
        this.angles = angles;
    }

    public BNO055IMU getImu() {
        return imu;
    }

    public void setImu(BNO055IMU imu) {
        this.imu = imu;
    }

    public int getCount() {
        return count;
    }

    public void setCount(int count) {
        this.count = count;
    }

    public double getDistance() {
        return distance;
    }

    public void setDistance(double distance) {
        this.distance = distance;
    }

    public double getStone2Distance() {
        return stone2Distance;
    }

    public void setStone2Distance(double stone2Distance) {
        this.stone2Distance = stone2Distance;
    }

    public int getPosition() {
        return position;
    }

    public void setPosition(int position) {
        this.position = position;
    }

    public boolean isThird() {
        return third;
    }

    public void setThird(boolean third) {
        this.third = third;
    }

    public boolean isStart() {
        return start;
    }

    public void setStart(boolean start) {
        this.start = start;
    }

    public double getHeading() {
        return heading;
    }

    public void setHeading(double heading) {
        this.heading = heading;
    }

    public double getRoll() {
        return roll;
    }

    public void setRoll(double roll) {
        this.roll = roll;
    }

    public double getPitch() {
        return pitch;
    }

    public void setPitch(double pitch) {
        this.pitch = pitch;
    }

    public double getGlobalAngle() {
        return globalAngle;
    }

    public void setGlobalAngle(double globalAngle) {
        this.globalAngle = globalAngle;
    }

    public double getAvgEncoder() {
        return avgEncoder;
    }

    public void setAvgEncoder(double avgEncoder) {
        this.avgEncoder = avgEncoder;
    }

    public boolean isBegin() {
        return begin;
    }

    public void setBegin(boolean begin) {
        this.begin = begin;
    }

    public double getBeginAngle() {
        return beginAngle;
    }

    public void setBeginAngle(double beginAngle) {
        this.beginAngle = beginAngle;
    }

    public boolean isBeginFlip() {
        return beginFlip;
    }

    public void setBeginFlip(boolean beginFlip) {
        this.beginFlip = beginFlip;
    }

    public boolean isStopL() {
        return stopL;
    }

    public void setStopL(boolean stopL) {
        this.stopL = stopL;
    }

    public boolean isStartL() {
        return startL;
    }

    public void setStartL(boolean startL) {
        this.startL = startL;
    }

    public boolean isStrafeL() {
        return strafeL;
    }

    public void setStrafeL(boolean strafeL) {
        this.strafeL = strafeL;
    }

    public boolean isStopR() {
        return stopR;
    }

    public void setStopR(boolean stopR) {
        this.stopR = stopR;
    }

    public boolean isStartR() {
        return startR;
    }

    public void setStartR(boolean startR) {
        this.startR = startR;
    }

    public boolean isStrafeR() {
        return strafeR;
    }

    public void setStrafeR(boolean strafeR) {
        this.strafeR = strafeR;
    }

    public boolean isTurning() {
        return turning;
    }

    public void setTurning(boolean turning) {
        this.turning = turning;
    }

    public boolean isTurningA() {
        return turningA;
    }

    public void setTurningA(boolean turningA) {
        this.turningA = turningA;
    }

    public boolean isTurningB() {
        return turningB;
    }

    public void setTurningB(boolean turningB) {
        this.turningB = turningB;
    }

    public boolean isTurningX() {
        return turningX;
    }

    public void setTurningX(boolean turningX) {
        this.turningX = turningX;
    }

    public boolean isTurningY() {
        return turningY;
    }

    public void setTurningY(boolean turningY) {
        this.turningY = turningY;
    }

    public double getSP() {
        return SP;
    }

    public void setSP(double SP) {
        this.SP = SP;
    }

    public double getPV() {
        return PV;
    }

    public void setPV(double PV) {
        this.PV = PV;
    }

    public boolean isReset() {
        return reset;
    }

    public void setReset(boolean reset) {
        this.reset = reset;
    }

    public double getDistanceL() {
        return distanceL;
    }

    public void setDistanceL(double distanceL) {
        this.distanceL = distanceL;
    }

    public double getDistanceR() {
        return distanceR;
    }

    public void setDistanceR(double distanceR) {
        this.distanceR = distanceR;
    }

    public double getDistanceB() {
        return distanceB;
    }

    public void setDistanceB(double distanceB) {
        this.distanceB = distanceB;
    }

    public double getDistanceFM() {
        return distanceFM;
    }

    public void setDistanceFM(double distanceFM) {
        this.distanceFM = distanceFM;
    }

    public boolean isOpModeActive() {
        return opModeActive;
    }

    public void setOpModeActive(boolean opModeActive) {
        this.opModeActive = opModeActive;
    }

    public RobotHardwareMap getRobot() {
        return robot;
    }
}
