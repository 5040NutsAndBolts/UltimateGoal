package org.firstinspires.ftc.teamcode.competition.Autonomous;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.competition.Hardware;
import org.firstinspires.ftc.teamcode.helperclasses.HelperMethods;
import org.firstinspires.ftc.teamcode.helperclasses.LQR;

import java.io.File;
import java.util.List;
import java.util.Scanner;

@Autonomous(name = "Auto", group = "Auto")
public class HighGoalAuto extends LinearOpMode
{

    @Override
    public void runOpMode() throws InterruptedException
    {

        final Hardware robot = new Hardware();
        robot.init(hardwareMap);
        robot.resetOdometry(0,0,0);
        Hardware.fromAuto=true;
        LQR lqr = new LQR(robot);
        /*
        *
0 0 0 1 0 0
0 0 0 0 1 0
0 0 0 0 0 1
0 0 0 0 0 0
0 0 0 0 0 0
0 0 0 0 0 0

1 0 0 0 0 0
0 1 0 0 0 0
0 0 12 0 0 0
0 0 0 .0005 0 0
0 0 0 0 .0005 0
0 0 0 0 0 .0005

15 0 0 0
0 15 0 0
0 0 15 0
0 0 0 15      *
        * */
        double[][][] path={{{}}};
                /*
        *
0 0 0 1 0 0
0 0 0 0 1 0
0 0 0 0 0 1
0 0 0 0 0 0
0 0 0 0 0 0
0 0 0 0 0 0

1 0 0 0 0 0
0 1 0 0 0 0
0 0 5 0 0 0
0 0 0 .0005 0 0
0 0 0 0 .0005 0
0 0 0 0 0 .0005

15 0 0 0
0 15 0 0
0 0 15 0
0 0 0 15      *
        * */
        double[][][] grabWobble2={{{}}};
        /*
0 0 0 1 0 0
0 0 0 0 1 0
0 0 0 0 0 1
0 0 0 0 0 0
0 0 0 0 0 0
0 0 0 0 0 0

.9 0 0 0 0 0
0 .9 0 0 0 0
0 0 2.5 0 0 0
0 0 0 .01 0 0
0 0 0 0 .01 0
0 0 0 0 0 .01

15 0 0 0
0 15 0 0
0 0 15 0
0 0 0 15
        * */
        double[][][] wobble={{{}}};

        /*
        *
0 0 0 1 0 0
0 0 0 0 1 0
0 0 0 0 0 1
0 0 0 0 0 0
0 0 0 0 0 0
0 0 0 0 0 0

.2 0 0 0 0 0
0 .5 0 0 0 0
0 0 15 0 0 0
0 0 0 .1 0 0
0 0 0 0 .1 0
0 0 0 0 0 .1

150 0 0 0
0 150 0 0
0 0 150 0
0 0 0 150
        * */
        double[][][] park = {{{}}};
/*
0 0 0 1 0 0
0 0 0 0 1 0
0 0 0 0 0 1
0 0 0 0 0 0
0 0 0 0 0 0
0 0 0 0 0 0

.3 0 0 0 0 0
0 .3 0 0 0 0
0 0 .45 0 0 0
0 0 0 .01 0 0
0 0 0 0 .01 0
0 0 0 0 0 .01

15 0 0 0
0 15 0 0
0 0 15 0
0 0 0 15
*/
        double[][][] wobble4={{{}}};
/*
0 0 0 1 0 0
0 0 0 0 1 0
0 0 0 0 0 1
0 0 0 0 0 0
0 0 0 0 0 0
0 0 0 0 0 0

1 0 0 0 0 0
0 .3 0 0 0 0
0 0 35 0 0 0
0 0 0 .0005 0 0
0 0 0 0 .0005 0
0 0 0 0 0 .0005

15 0 0 0
0 15 0 0
0 0 15 0
0 0 0 15
* */
        double[][][] shoot={{{}}};
/*
0 0 0 1 0 0
0 0 0 0 1 0
0 0 0 0 0 1
0 0 0 0 0 0
0 0 0 0 0 0
0 0 0 0 0 0

1 0 0 0 0 0
0 .4 0 0 0 0
0 0 12 0 0 0
0 0 0 .0005 0 0
0 0 0 0 .0005 0
0 0 0 0 0 .0005

15 0 0 0
0 15 0 0
0 0 15 0
0 0 0 15
* */
        double[][][] ontoWobble2 = {{{}}};
/*
0 0 0 1 0 0
0 0 0 0 1 0
0 0 0 0 0 1
0 0 0 0 0 0
0 0 0 0 0 0
0 0 0 0 0 0

1 0 0 0 0 0
0 .6 0 0 0 0
0 0 12 0 0 0
0 0 0 .0005 0 0
0 0 0 0 .0005 0
0 0 0 0 0 .0005

15 0 0 0
0 15 0 0
0 0 15 0
0 0 0 15
* */

        double[][][] beforeShoot = {{{}}};
        try
        {

            telemetry.addData("data","test");
            telemetry.update();
            //Gets LQR matrices file
            String content = new Scanner(new File(Environment.getExternalStorageDirectory() + "/lqrTestData.txt")).useDelimiter("\\Z").next();

            //split the file into individual matrices
            //▰▰▰▰▱▱▱ Loading bar scheme
            String[] data = content.split("\r\n\r\n");
            path = lqr.loadPath("/lqrTestData.txt");
            telemetry.addData("data","▰▱▱▱▱▱▱");
            telemetry.update();
            wobble = lqr.loadPath("/wobble.txt");
            telemetry.addData("data","▰▰▱▱▱▱▱");
            telemetry.update();
            wobble4 = lqr.loadPath("/wobble4.txt");
            telemetry.addData("data","▰▰▰▱▱▱▱");
            telemetry.update();
            grabWobble2=lqr.loadPath("/wobbleGrab2.txt");
            telemetry.addData("data","▰▰▰▰▱▱▱");
            telemetry.update();
            ontoWobble2=lqr.loadPath("/ontoWobble2.txt");
            telemetry.addData("data","▰▰▰▰▰▱▱");
            telemetry.update();
            shoot=lqr.loadPath("/shoot.txt");
            telemetry.addData("data","▰▰▰▰▰▰▱");
            telemetry.update();
            beforeShoot=lqr.loadPath("/beforeShoot.txt");

            telemetry.addData("test",content.substring(0,60)+"\n\n\n\n\n\n"+data[0]+"\n\n\n"+path[0][0][0]);



        } catch (Exception e)
        {
            telemetry.addData("error: ",e.toString());

        }
        Vuforia vuforia = new Vuforia();
        vuforia.initVuforia(robot);
        vuforia.initTfod(robot);
        vuforia.tfod.activate();
        robot.leftWobbleGoalUp();
        robot.clawServoLeftClose();
        //telemetry.addData("test","test");
        //telemetry.update();

        Thread t = new Thread()
        {

            @Override
            public void run()
            {

                while(opModeIsActive())
                    robot.updatePositionRoadRunner();

            }

        };

        int k = 0;
        String rings = "";
        int auto = 0;

        while(!isStarted())
        {

            //Recognize stack height
            List<Recognition> thrit = vuforia.tfod.getRecognitions();
            if(!thrit.isEmpty())
                rings = thrit.get(0).getLabel();
            else
                rings = "";
            if(rings.equals("Single"))
            {
                telemetry.addData("Auto", 1);
                auto=1;
            }
            else if(rings.equals("Quad"))
            {
                telemetry.addData("Auto", 4);
                auto=4;
            }
            else
            {
                telemetry.addData("Auto", 0);
                auto=0;
            }
            telemetry.addData("Label",rings);
            k++;
            telemetry.update();

        }



        waitForStart();


        t.start();

        ElapsedTime e = new ElapsedTime();
        double wobbleX,  wobbleY, wobbleRadius, timeOut;

        if(auto==0)
        {
            wobbleX=70.75;
            wobbleY=-3.25;
            wobbleRadius=1.2;
            timeOut=4;


        }else if(auto==4)
        {

            wobbleX=116.2;
            wobbleY=-3.75;
            wobbleRadius=2.5;
            timeOut=7;

        }
        else
        {

            wobbleX=97;
            wobbleY=21;
            wobbleRadius=1.2;
            timeOut=7;


        }
        e.startTime();
        if(auto==1)
        {

            while(opModeIsActive()&&!lqr.robotInCircle(56,-5,2.5))
            {

                lqr.runLqrDrive(wobble, 56, -5, 0);
                telemetry.addData("x: ", Hardware.x);
                telemetry.addData("y: ", Hardware.y);
                telemetry.addData("theta: ", Hardware.theta);
                telemetry.addData("Goals", wobbleX + " " + wobbleY + " " + wobbleRadius);
                telemetry.update();
                if (robot.x < -40)
                    robot.leftWobbleGoalMid();
            }

        }
        else if(auto==4)
            {

                while(opModeIsActive()&&!lqr.robotInCircle(36,-4,2.5)&&e.seconds()<timeOut)
                {

                    lqr.runLqrDrive(wobble, 36, -4, 0);
                    telemetry.addData("x: ", Hardware.x);
                    telemetry.addData("y: ", Hardware.y);
                    telemetry.addData("theta: ", Hardware.theta);
                    telemetry.addData("Goals", wobbleX + " " + wobbleY + " " + wobbleRadius);
                    telemetry.update();

                }

            }
        //place first wobble
        if(auto==4)
        {

            while(opModeIsActive()&&!lqr.robotInCircle(wobbleX,wobbleY,wobbleRadius)&&robot.x<60&&e.seconds()<timeOut)
            {

                lqr.runLqrDrive(wobble,wobbleX,wobbleY,0);
                telemetry.addData("x: ", Hardware.x);
                telemetry.addData("y: ", Hardware.y);
                telemetry.addData("theta: ", Hardware.theta);
                telemetry.addData("Goals",wobbleX+" "+wobbleY+" "+wobbleRadius);
                telemetry.update();
                if(robot.x>40)
                    robot.leftWobbleGoalMid();

            }
            while(opModeIsActive()&&!lqr.robotInCircle(wobbleX,wobbleY,wobbleRadius)&&e.seconds()<timeOut)
            {

                lqr.runLqrDrive(wobble4,wobbleX,wobbleY,0);
                telemetry.addData("x: ", Hardware.x);
                telemetry.addData("y: ", Hardware.y);
                telemetry.addData("theta: ", Hardware.theta);
                telemetry.addData("Goals",wobbleX+" "+wobbleY+" "+wobbleRadius);
                telemetry.update();
                if(robot.x>40)
                    robot.leftWobbleGoalMid();

            }

        }
        else
            while(opModeIsActive()&&!lqr.robotInCircle(wobbleX,wobbleY,wobbleRadius)&&e.seconds()<timeOut)
            {

            lqr.runLqrDrive(wobble,wobbleX,wobbleY,0);
            telemetry.addData("x: ", Hardware.x);
            telemetry.addData("y: ", Hardware.y);
            telemetry.addData("theta: ", Hardware.theta);
            telemetry.addData("Goals",wobbleX+" "+wobbleY+" "+wobbleRadius);
            telemetry.update();
            if(robot.x>40)
                robot.leftWobbleGoalMid();
            if(robot.x>wobbleX-30&&auto!=1)
                lqr.setPowerMultiply(.4);
            

            }
        robot.hardBrakeMotors();
        robot.leftWobbleGoalDown();
        e.reset();
        e.startTime();
        while(e.seconds()<.9);
        robot.clawServoLeftOpen();
        while(e.seconds()<1);
        lqr.setPowerMultiply(1);
        while(opModeIsActive()&&!lqr.robotInCircle(wobbleX+5,robot.y,3))
        {

            lqr.runLqrDrive(wobble,wobbleX+5,robot.y,0);
            telemetry.addData("x: ", Hardware.x);
            telemetry.addData("y: ", Hardware.y);
            telemetry.addData("theta: ", Hardware.theta);
            telemetry.update();

        }
        e.reset();
        double shootingShift=0;
        if(auto==0)
            shootingShift=-.75;
        //drive to point to shoot into the high goal
        if(auto==1)
        {

            while(opModeIsActive()&&!lqr.robotInCircle(59,13+shootingShift,3.5))
            {

                robot.setFlyWheelVelocity(2360);
                lqr.runLqrDrive(beforeShoot,59,13+shootingShift,0);
                robot.flywheelRotateServoLeft.setPosition(.4);
                telemetry.addData("x: ", Hardware.x);
                telemetry.addData("y: ", Hardware.y);
                telemetry.addData("theta: ", Hardware.theta);
                telemetry.update();

            }

        }
        else
        {
            while(opModeIsActive()&&!lqr.robotInCircle(59,13+shootingShift,3.5))
            {

                robot.setFlyWheelVelocity(2360);
                lqr.runLqrDrive(beforeShoot,59,13+shootingShift,0);
                robot.flywheelRotateServoLeft.setPosition(.4);
                telemetry.addData("x: ", Hardware.x);
                telemetry.addData("y: ", Hardware.y);
                telemetry.addData("theta: ", Hardware.theta);
                telemetry.update();

            }
        }
        while(opModeIsActive()&&e.seconds()<5&&(!lqr.robotInCircle(59,13+shootingShift,1.2)||!HelperMethods.nearAngle(Hardware.theta,0,.05)))
        {

            robot.setFlyWheelVelocity(2360);
            lqr.runLqrDrive(shoot,59,13+shootingShift,0);
            robot.flywheelRotateServoLeft.setPosition(.4);
            telemetry.addData("x: ", Hardware.x);
            telemetry.addData("y: ", Hardware.y);
            telemetry.addData("theta: ", Hardware.theta);
            telemetry.update();

        }

        e.startTime();
        robot.hardBrakeMotors();

        e.reset();
        e.startTime();
        //shoot 3 rings
        while(e.seconds()<1.3&&opModeIsActive()){robot.setFlyWheelVelocity(2360);telemetry.addData("vel",robot.flywheelMotorLeft.getVelocity());}

        for(int i = 0; i<4; i++)
        {
            robot.flickRing();
            e.reset();
            e.startTime();
            while(e.seconds()<.5&&opModeIsActive())
            {
                robot.setFlyWheelVelocity(2360);
                robot.flywheelRotateServoLeft.setPosition(.4);
                telemetry.addData("vel",robot.flywheelMotorLeft.getVelocity());
                telemetry.addData("x: ", Hardware.x);
                telemetry.addData("y: ", Hardware.y);
                telemetry.addData("theta: ", Hardware.theta);
                telemetry.update();
            }
        }
        e.reset();
        e.startTime();
        while(e.seconds()<.5&&opModeIsActive()){robot.setFlyWheelVelocity(2360);}
        robot.setFlyWheelPower(0);
        e.reset();
        e.startTime();
        robot.flywheelRotateServoLeft.setPosition(.7433);

        //intake ring from stack
        if(auto==4||auto==1)
        {

            double timeAdd=0;
            if(auto==4)
                timeAdd=3;
            while(e.seconds()<1.3+timeAdd)
            {

                double rotate;
                if((int)(e.seconds()*5)%2==0)
                    rotate=.1;
                else
                    rotate=-.1;

                robot.setIntakePower(1);
                double strafeAdd = 0;
                double secondForwardAdd=0;
                double forwardAdd=0;
                if(auto==4)
                {
                    strafeAdd = -.175;
                    forwardAdd=.165;
                    secondForwardAdd=.085;
                }
                if(e.seconds()>.7)
                    robot.drive(-.2+secondForwardAdd,0,rotate);
                else
                    robot.drive(-.185+forwardAdd,-.1+strafeAdd,0);

            }

            e.reset();
            while(e.seconds()<.3)
            {

                robot.drive(.5,.5,0);

            }

        }

        e.reset();
        //Go away from rings and in the direction of the wobble goal
        while(opModeIsActive()&&!lqr.robotInCircle(27,43,15))
                {

                    lqr.runLqrDrive(grabWobble2,27,43,0);
                    telemetry.addData("x: ", Hardware.x);
                    telemetry.addData("y: ", Hardware.y);
                    telemetry.addData("theta: ", Hardware.theta);
                    telemetry.update();
                    if(e.seconds()>.1)
                    {

                        robot.leftWobbleGoalDown();

                    }

                }
       /* if(auto==4)
        {
            //Move towards wobble of the x
            while (opModeIsActive() && !lqr.robotInCircle(25, 35, .55))
            {

                lqr.runLqrDrive(path, 25, 35, 0);
                telemetry.addData("x: ", Hardware.x);
                telemetry.addData("y: ", Hardware.y);
                telemetry.addData("theta: ", Hardware.theta);
                telemetry.update();


            }

            robot.hardBrakeMotors();
            e.reset();
            while (e.seconds() < .1) ;
            //Strafe to approach wobble
            while (opModeIsActive() && !lqr.robotInCircle(25, 32.75, 1))
            {

                lqr.runLqrDrive(ontoWobble2, 25, 32.75, 0);
                telemetry.addData("x: ", Hardware.x);
                telemetry.addData("y: ", Hardware.y);
                telemetry.addData("theta: ", Hardware.theta);
                telemetry.update();


            }
            robot.hardBrakeMotors();
            //move to point and grab second wobble
            e.reset();
            lqr.setPowerMultiply(.1);
            while (opModeIsActive() && robot.wobbleSensor.red() < 135 && e.seconds() < 2.1)
            {

                lqr.runLqrDrive(ontoWobble2, 25, 11, 0);
                telemetry.addData("x: ", Hardware.x);
                telemetry.addData("y: ", Hardware.y);
                telemetry.addData("theta: ", Hardware.theta);
                telemetry.update();

            }
        }
        else if(auto==1)
        {
            //Move towards wobble of the x
            while (opModeIsActive() && !lqr.robotInCircle(23.75, 40, .55))
            {

                lqr.runLqrDrive(path, 23.75, 40, 0);
                telemetry.addData("x: ", Hardware.x);
                telemetry.addData("y: ", Hardware.y);
                telemetry.addData("theta: ", Hardware.theta);
                telemetry.update();


            }

            robot.hardBrakeMotors();
            e.reset();
            while (e.seconds() < .1) ;
            //Strafe to approach wobble
            while (opModeIsActive() && !lqr.robotInCircle(23.75, 34, 1))
            {

                lqr.runLqrDrive(ontoWobble2, 23.75, 34, 0);
                telemetry.addData("x: ", Hardware.x);
                telemetry.addData("y: ", Hardware.y);
                telemetry.addData("theta: ", Hardware.theta);
                telemetry.update();


            }
            robot.hardBrakeMotors();
            //move to point and grab second wobble
            e.reset();
            lqr.setPowerMultiply(.13);
            while (opModeIsActive() && robot.wobbleSensor.red() < 135 && e.seconds() < 2.1)
            {

                lqr.runLqrDrive(ontoWobble2, 23.75, 6, 0);
                telemetry.addData("x: ", Hardware.x);
                telemetry.addData("y: ", Hardware.y);
                telemetry.addData("theta: ", Hardware.theta);
                telemetry.update();

            }
        }*/
        //else
        //{
        double shiftY=2;
        if(auto==1)
            shiftY=3;
        if(auto==4)
            shiftY=1;
            //Move towards wobble of the x
        e.reset();
        while (e.seconds()<2&&opModeIsActive() && !lqr.robotInCircle(30, 28.5+shiftY, .55))
        {

                lqr.runLqrDrive(path, 30, 28.5+shiftY, 0);
                telemetry.addData("x: ", Hardware.x);
                telemetry.addData("y: ", Hardware.y);
                telemetry.addData("theta: ", Hardware.theta);
                telemetry.update();


            }

            robot.hardBrakeMotors();
            e.reset();
            while (e.seconds() < .1) ;
            //Strafe to approach wobble
            while (e.seconds()<2&&opModeIsActive() && !lqr.robotInCircle(24, 28.5+shiftY, 1))
            {

                lqr.runLqrDrive(ontoWobble2, 24, 28.5+shiftY, 0);
                telemetry.addData("x: ", Hardware.x);
                telemetry.addData("y: ", Hardware.y);
                telemetry.addData("theta: ", Hardware.theta);
                telemetry.update();


            }
            robot.hardBrakeMotors();
            //move to point and grab second wobble
            e.reset();
            double powerAdd=0;
            if(auto==4)
                powerAdd=.05;
            lqr.setPowerMultiply(.24+powerAdd);
            while (opModeIsActive() && robot.wobbleSensor.red() < 100 && e.seconds() < 2.1)
            {

                lqr.runLqrDrive(ontoWobble2, 6, 28.5+shiftY, 0);
                telemetry.addData("x: ", Hardware.x);
                telemetry.addData("y: ", Hardware.y);
                telemetry.addData("theta: ", Hardware.theta);
                telemetry.update();

            }
        //}
        lqr.setPowerMultiply(1);
        robot.hardBrakeMotors();
        e.reset();
        while(e.seconds()<.1);
        robot.clawServoLeftClose();

        e.reset();
        while(e.seconds()<1.5);
        robot.leftWobbleGoalMid();
        e.reset();
        while(e.seconds()<.1);
        while(opModeIsActive()&&!lqr.robotInCircle(41.1,33.5,3))
        {

            lqr.runLqrDrive(ontoWobble2,41.1,33.5,0);
            telemetry.addData("x: ", Hardware.x);
            telemetry.addData("y: ", Hardware.y);
            telemetry.addData("theta: ", Hardware.theta);
            telemetry.update();


        }

        if(auto==4)
        {
            while (opModeIsActive() && !lqr.robotInCircle(115, 0.25, 2))
            {

                lqr.runLqrDrive(ontoWobble2, 115, -0.5, 0);
                telemetry.addData("x: ", Hardware.x);
                telemetry.addData("y: ", Hardware.y);
                telemetry.addData("theta: ", Hardware.theta);
                telemetry.update();


            }
            robot.hardBrakeMotors();
            robot.leftWobbleGoalDown();
            e.reset();
            while (e.seconds() < .7) ;
            robot.clawServoLeftOpen();
            e.reset();
            while (e.seconds() < .2) ;
            while (opModeIsActive() && !lqr.robotInCircle(Hardware.x, 12, 2))
            {

                lqr.runLqrDrive(ontoWobble2, Hardware.x, 12, 0);
                telemetry.addData("x: ", Hardware.x);
                telemetry.addData("y: ", Hardware.y);
                telemetry.addData("theta: ", Hardware.theta);
                telemetry.update();


            }
        }
        else if(auto==1)
        {

            while(opModeIsActive()&&!lqr.robotInCircle(90,30,2))
            {

                lqr.runLqrDrive(ontoWobble2,90,30,0);
                telemetry.addData("x: ", Hardware.x);
                telemetry.addData("y: ", Hardware.y);
                telemetry.addData("theta: ", Hardware.theta);
                telemetry.update();


            }
            robot.hardBrakeMotors();
            robot.leftWobbleGoalDown();
            e.reset();
            while(e.seconds()<.7);
            robot.clawServoLeftOpen();
            e.reset();
            while(e.seconds()<.2);
            while(opModeIsActive()&&!lqr.robotInCircle(93,33,2))
            {

                lqr.runLqrDrive(ontoWobble2,93,33,0);
                telemetry.addData("x: ", Hardware.x);
                telemetry.addData("y: ", Hardware.y);
                telemetry.addData("theta: ", Hardware.theta);
                telemetry.update();


            }
            robot.leftWobbleGoalUp();

        }
        else
            {
                while(opModeIsActive()&&!lqr.robotInCircle(71,3,2))
                {

                    lqr.runLqrDrive(ontoWobble2,71,3,0);
                    telemetry.addData("x: ", Hardware.x);
                    telemetry.addData("y: ", Hardware.y);
                    telemetry.addData("theta: ", Hardware.theta);
                    telemetry.update();


                }
                robot.hardBrakeMotors();
                robot.leftWobbleGoalDown();
                e.reset();
                while(e.seconds()<.7);
                robot.clawServoLeftOpen();
                e.reset();
                while(e.seconds()<.2);
                while(opModeIsActive()&&!lqr.robotInCircle(Hardware.x,12,-2))
                {

                    lqr.runLqrDrive(ontoWobble2,Hardware.x,12,0);
                    telemetry.addData("x: ", Hardware.x);
                    telemetry.addData("y: ", Hardware.y);
                    telemetry.addData("theta: ", Hardware.theta);
                    telemetry.update();


                }
            }
        robot.setIntakePower(0);

        while(opModeIsActive()&&!lqr.robotInCircle(59.5,5,3.5))
        {

                robot.setFlyWheelVelocity(2360);
                lqr.runLqrDrive(beforeShoot,59.5,5,0);
                robot.flywheelRotateServoLeft.setPosition(.4);
                telemetry.addData("x: ", Hardware.x);
                telemetry.addData("y: ", Hardware.y);
                telemetry.addData("theta: ", Hardware.theta);
                telemetry.update();

        }
        e.reset();
        while(opModeIsActive()&&e.seconds()<3&&(!lqr.robotInCircle(59.5,13,1.2)||!HelperMethods.nearAngle(Hardware.theta,.05,.02)))
        {

            robot.setFlyWheelVelocity(2360);
            lqr.runLqrDrive(shoot,59.5,13,0.02);
            robot.flywheelRotateServoLeft.setPosition(.4);
            telemetry.addData("x: ", Hardware.x);
            telemetry.addData("y: ", Hardware.y);
            telemetry.addData("theta: ", Hardware.theta);
            telemetry.update();

        }

        e.startTime();
        robot.hardBrakeMotors();

        e.reset();
        e.startTime();


        while(e.seconds()<.7&&opModeIsActive()){robot.setFlyWheelVelocity(2360);}

        for(int i = 0; i<4; i++)
        {

            if(opModeIsActive())
                robot.flickRing();
            e.reset();
            e.startTime();
            while(e.seconds()<.2&&opModeIsActive())
            {
                robot.setFlyWheelVelocity(2360);
                robot.flywheelRotateServoLeft.setPosition(.4);
                telemetry.addData("vel",robot.flywheelMotorLeft.getVelocity());
                telemetry.addData("x: ", Hardware.x);
                telemetry.addData("y: ", Hardware.y);
                telemetry.addData("theta: ", Hardware.theta);
                telemetry.update();
            }
            if(!opModeIsActive())
                return;
        }
        e.reset();
        e.startTime();
        while(e.seconds()<.1&&opModeIsActive()){robot.setFlyWheelVelocity(2360);}
        robot.setFlyWheelPower(0);
        e.reset();
        e.startTime();
        robot.flywheelRotateServoLeft.setPosition(.7433);



        while(opModeIsActive()&&!lqr.robotInCircle(71,8,2))
        {

            lqr.runLqrDrive(ontoWobble2,71,8,0);
            telemetry.addData("x: ", Hardware.x);
            telemetry.addData("y: ", Hardware.y);
            telemetry.addData("theta: ", Hardware.theta);
            telemetry.update();
            if(!opModeIsActive())
            {

                robot.hardBrakeMotors();

            }


        }
        robot.hardBrakeMotors();



        /*while(!lqr.robotInCircle(24,48,.5))
        {

            robot.updatePositionRoadRunner();
            lqr.runLqrDrive(path,24,48,0);

        }
        while(!lqr.robotInCircle(0,24,.5))
        {

            robot.updatePositionRoadRunner();
            lqr.runLqrDrive(path,0,48,0);

        }*/

    }

}
