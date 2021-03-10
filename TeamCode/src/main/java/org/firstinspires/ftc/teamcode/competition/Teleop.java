package org.firstinspires.ftc.teamcode.competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.helperclasses.HelperMethods;
import org.firstinspires.ftc.teamcode.helperclasses.LQR;
import org.firstinspires.ftc.teamcode.helperclasses.ThreadPool;

import java.io.IOException;

import static java.lang.Math.cos;
import static java.lang.Math.sin;
import java.util.List;
import java.util.ArrayList;

@TeleOp(name = "Teleop", group = "DemoBot")
public class Teleop extends LinearOpMode
{
    private final double robotWidth = 15.375; //Width of actual robot
    private final double robotLength = 17.25; //Look above if you can't figure this out
    double OdomResetX;
    double OdomResetY;
    double OdomResetTheta;
    @Override
    public void runOpMode() throws InterruptedException
    {

        Hardware robot = new Hardware();
        robot.init(hardwareMap);
        final LQR lqr = new LQR(robot);
        VuforiaWebcamCameraResetOdometry vuWebcamReset = new VuforiaWebcamCameraResetOdometry();
        List<VuforiaTrackable> trythistho = new ArrayList<VuforiaTrackable>();
        trythistho = vuWebcamReset.initVuReset();

        /*
0 0 0 1 0 0
0 0 0 0 1 0
0 0 0 0 0 1
0 0 0 0 0 0
0 0 0 0 0 0
0 0 0 0 0 0

.7 0 0 0 0 0
0 1 0 0 0 0
0 0 205 0 0 0
0 0 0 .001 0 0
0 0 0 0 .001 0
0 0 0 0 0 .001

15 0 0 0
0 15 0 0
0 0 15 0
0 0 0 15
        *
        * */
        Thread t = new Thread()
        {

            @Override
            public void run()
            {

                try
                {
                    LQR.path = lqr.loadPath("/teleopFire.txt");
                } catch (IOException e)
                {
                    e.printStackTrace();
                } catch (ClassNotFoundException e)
                {
                    e.printStackTrace();
                }

            }

        };
        if (LQR.path == null)
            ThreadPool.pool.submit(t);
        waitForStart();
        robot.initServos();
        boolean a2Pressed = false;
        boolean a1Pressed = true;
        boolean y2Pressed = false;
        boolean b1Pressed = false;
        boolean x1Pressed = false;
        boolean upPressed = false;
        boolean downPressed = false;
        boolean autoAim = false;
        boolean slowDrive = false;
        boolean autoShoot = false;
        boolean autoShooting = false;
        boolean rBumper1Pressed = false;
        boolean leftWobbleDown = false;
        boolean leftClawOpen = false;
        boolean down2Pressed = false;
        boolean up2Pressed = false;
        boolean lBumper1Pressed = false;
        boolean noFire = false;
        boolean left2Pressed = false;
        double forward = 1;
        double angleSpeed = .45;
        double servoPosition = 1;
        double savedPosition = .42;

        double leftSub = 0;
        double rightSub = 0;
        double centerSub = 0;
        double driveSpeed = 1;
        boolean wobbleLock = false;
        boolean midLock = false;
        boolean mediumDrive = false;


        if (Hardware.fromAuto)
            robot.resetOdometry(Hardware.x + robotWidth/2, Hardware.y + robotLength/2, 0);
        else
            robot.resetOdometry(Hardware.x, Hardware.y, 0);
        while (opModeIsActive())
        {

            if(gamepad1.right_bumper)
                robot.resetOdometry(0,0,0);
            robot.updatePositionRoadRunner();
            double[] autoLaunch = robot.getFlyWheelAngle(Hardware.SelectedGoal.HIGHGOAL);
            //set drive speed
            if (slowDrive)
                driveSpeed = .3;
            else if(mediumDrive)
                driveSpeed=.6;
            else
                driveSpeed = 1;

            //automatically aim  at goal
            if (autoShoot)
            {

                double x = Hardware.x + 144;
                double y = Hardware.y + 36;
                double magnitude = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
                double xUnit = x / magnitude;
                double yUnit = y / magnitude;
                double xGoal = -144 + xUnit * 80;
                double yGoal = -36 + yUnit * 80;
                servoPosition = 1 + ((autoLaunch[ 0 ] * 180 / Math.PI - 3.1634748) / -68.3652519934);
                robot.setFlyWheelVelocity(autoLaunch[ 1 ] * 10.0267614 * 1.06);

                if (xGoal < -70)
                {
                    xGoal = -70;
                    if (robot.x < -33.47883)
                        yGoal = -66.3974;
                    else
                        yGoal = -5.6026;
                }


                double thetaGoal = Math.atan((36 - 5 + Hardware.y) / (144 - 5 + Hardware.x));
                if (thetaGoal < 0)
                    thetaGoal += 2 * Math.PI;
                double diff = thetaGoal - Hardware.theta;
                if (diff < 0)
                    diff += 2 * Math.PI;
                if (diff > Math.PI)
                    diff = Math.PI * 2 - diff;

                telemetry.addData("goals", xGoal + " " + yGoal + " " + diff + " " + thetaGoal);

                if (lqr.robotInCircle(xGoal, yGoal, 1) && HelperMethods.nearAngle(Hardware.theta, thetaGoal, .01))
                {

                    lqr.runLqrDrive(LQR.path, Hardware.x, Hardware.y, thetaGoal);
                    telemetry.addData("goal velocity", autoLaunch[ 1 ] * 10.0267614 * 1.05);
                    if (robot.flywheelMotorRight.getVelocity() >= autoLaunch[ 1 ] * 10.0267614 * 1.05 && !noFire)
                    {


                        if (!autoShooting)
                        {
                            robot.flickRing();
                            robot.queuedFlicks = 2;
                        }
                        autoShooting = true;


                    }
                } else if (robot.flywheelMotorRight.getVelocity() >= autoLaunch[ 1 ] * 10.0267614 * 1.05)
                {

                    noFire = false;

                } else
                {
                    lqr.runLqrDrive(LQR.path, xGoal, yGoal, thetaGoal);
                }

            } else if (autoAim)
            {

                byte sign = 1;
                double diff = Hardware.theta - Math.atan((36 + 9 + Hardware.y) / (144 + 9 + Hardware.x));
                if (diff < 0)
                    diff += 2 * Math.PI;
                if (diff > Math.PI)
                    sign = -1;
                if (diff > Math.PI)
                    diff = Math.PI * 2 - diff;
                if (Math.abs(diff) > .04 && Math.abs(diff) < .2)
                    diff = .2 * Math.abs(diff) / diff;
                else if (Math.abs(diff) < .04)
                    diff *= 4;
                robot.drive(forward * driveSpeed * gamepad1.left_stick_y, driveSpeed * gamepad1.left_stick_x, HelperMethods.clamp(-.9, sign * diff, .9));

            } else
                robot.drive(forward * driveSpeed * gamepad1.left_stick_y, driveSpeed * gamepad1.left_stick_x, driveSpeed * gamepad1.right_stick_x);

            telemetry.addData("x: ", Hardware.x);
            telemetry.addData("y: ", Hardware.y);
            telemetry.addData("theta: ", Hardware.theta);
            telemetry.addData("Red: ", robot.wobbleSensor.red());
            telemetry.addData("Auto Aim", autoAim);
            telemetry.addData("Slow Drive", slowDrive);
            telemetry.addData("Trackwidth", Hardware.trackwidth);
            telemetry.addData("Left Speed", robot.flywheelMotorLeft.getVelocity());
            telemetry.addData("Right Speed", robot.flywheelMotorRight.getVelocity());
            telemetry.addData("Angle Servo", servoPosition);
            telemetry.addData("Auto Angle", autoLaunch[ 0 ] * 180 / Math.PI);
            telemetry.addData("odom left", (robot.odom.getWheelPositions().get(1) - leftSub) * 2048 * 4 / 0.688975 / Math.PI / 2);
            telemetry.addData("odom right", (robot.odom.getWheelPositions().get(2) - rightSub) * 2048 * 4 / 0.688975 / Math.PI / 2);
            telemetry.addData("odom center", (robot.odom.getWheelPositions().get(0) - centerSub) * 2048 * 4 / 0.688975 / Math.PI / 2);
            telemetry.update();


            if (servoPosition<.43&&servoPosition>.45)
            {
                robot.greenLED.setState(true);

            }else
            {
                robot.greenLED.setState(false);

            }

            if (gamepad1.left_bumper)
            {

                leftSub = robot.odom.getWheelPositions().get(1);
                rightSub = robot.odom.getWheelPositions().get(2);
                centerSub = robot.odom.getWheelPositions().get(0);

            }

            if (gamepad2.dpad_up && !up2Pressed)
            {

                up2Pressed = true;
                leftWobbleDown = !leftWobbleDown;
                midLock = false;

            } else if (!gamepad2.dpad_up)
            {

                up2Pressed = false;

            }

            if (leftWobbleDown&&!midLock)
                robot.leftWobbleGoalDown();
            else if(!midLock)
            {
                robot.leftWobbleGoalUp();
                leftClawOpen = false;
            }
            if (gamepad2.dpad_down && !down2Pressed)
            {

                down2Pressed = true;
                leftClawOpen = !leftClawOpen;

            } else if (!gamepad2.dpad_down)
            {

                down2Pressed = false;

            }
            if (gamepad2.x && gamepad2.dpad_down)
            {

                leftClawOpen = true;
                wobbleLock = true;

            } else
            {

                wobbleLock = false;

            }
            if (robot.wobbleSensor.red() > 150 && !wobbleLock)
                leftClawOpen = false;


            if (leftWobbleDown)
                robot.leftWobbleGoalDown();
            else
                robot.leftWobbleGoalUp();

            if (gamepad1.right_bumper)
            {
                vuWebcamReset.findTargetsVuReset(trythistho);
                OdomResetX = -1*(72 + VuforiaWebcamCameraResetOdometry.VuOdomResetX);
                OdomResetY = -1*(72 + VuforiaWebcamCameraResetOdometry.VuOdomResetY);
                OdomResetTheta = VuforiaWebcamCameraResetOdometry.VuOdomResetTheta;
                robot.resetOdometry(OdomResetX, OdomResetY, OdomResetTheta);
            }

            if (leftClawOpen)
                robot.clawServoLeftOpen();
            else
                robot.clawServoLeftClose();


            if (gamepad1.b && !b1Pressed && !gamepad1.start)
            {

                mediumDrive=false;
                slowDrive = !slowDrive;
                b1Pressed = true;
            }
            if (!gamepad1.b)
            {

                b1Pressed = false;

            }
            if (gamepad1.a && !a1Pressed && !gamepad1.start)
            {
                slowDrive=false;
                mediumDrive=!mediumDrive;
                a1Pressed = true;
            }
            if (!gamepad1.a)
            {

                a1Pressed = false;

            }

            if (gamepad1.x && !x1Pressed)
            {

                x1Pressed = true;
                autoShoot = !autoShoot;
                if (!autoShoot)
                    autoShooting = false;
                noFire = true;

            } else if (!gamepad1.x)
            {
                x1Pressed = false;
            }
            if (gamepad1.left_bumper && !lBumper1Pressed)
            {
                x1Pressed = true;
                autoShoot = !autoShoot;
                if (!autoShoot)
                    autoShooting = false;
                noFire = false;

            } else if (!gamepad1.x)
            {
                lBumper1Pressed = false;
            }
            //flip direction of drive when dpad buttons are pressed
            if (gamepad1.dpad_up)
            {

                forward = 1;

            }
            if (gamepad1.dpad_down)
            {

                forward = -1;

            }

            //intake or outtake rings
            if (gamepad1.right_trigger < .01)
            {


                robot.setIntakePower(-gamepad1.left_trigger);

            } else
            {

                robot.setIntakePower(gamepad1.right_trigger);

            }


            //Setting power for intake to the right trigger


            //sets flywheel power to the left trigger
            if (!gamepad2.right_bumper && !autoShoot)
            {

                robot.setFlyWheelPower(gamepad2.right_trigger);

            }

            if (gamepad2.dpad_left && !left2Pressed)
            {

                midLock = !midLock;
                leftWobbleDown = true;
                left2Pressed = true;

            } else if (!gamepad2.dpad_left)
            {

                left2Pressed = false;

            }
            if (midLock)
            {

                robot.leftWobbleGoal.setPosition(.6);

            }

            if (gamepad2.dpad_right)
            {

                servoPosition = .695;

            }

            //makes the flywheel rotation servo move with b and x
            if (Math.abs(gamepad2.left_stick_y) > .03)
                servoPosition -= gamepad2.left_stick_y * angleSpeed / 30;

            if (gamepad2.left_bumper)
                servoPosition = 1 + ((autoLaunch[ 0 ] * 180 / Math.PI - 3.1634748) / -68.3652519934);
            if (gamepad2.right_bumper)
                robot.setFlyWheelVelocity(autoLaunch[ 1 ] * 10.0267614 * 1.06);
            if (servoPosition > .9)
                servoPosition = .9;
            else if (servoPosition < .17)
                servoPosition = .17;

            robot.flywheelRotateServoLeft.setPosition(servoPosition);


            //launch a single ring if a is pressed and the flywheels are moving
            if (gamepad2.a && !a2Pressed && (Math.abs(robot.flywheelMotorLeft.getVelocity()) > 20 || gamepad2.x))
            {
                robot.flickRing();
                a2Pressed = true;
            }
            if (!gamepad2.a)
            {

                a2Pressed = false;

            }

            //launch 3 rings if y is pressed and flywheels are moving
            if (gamepad2.y && !y2Pressed && (Math.abs(robot.flywheelMotorLeft.getVelocity()) > 20 || gamepad2.x))
            {
                robot.queuedFlicks = 2;
                robot.flickRing();
                y2Pressed = true;
            }
            if (!gamepad2.y)
            {

                y2Pressed = false;

            }
            if (gamepad2.left_trigger > .01)
            {

                savedPosition = servoPosition;

            }

            if (gamepad2.b)
            {


                servoPosition = savedPosition;

            }


        }
        ThreadPool.renewPool();

    }

    }
