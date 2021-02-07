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
            String[] data = content.split("\r\n\r\n");
            path = lqr.loadPath("/lqrTestData.txt");

            wobble = lqr.loadPath("/wobble.txt");
            wobble4 = lqr.loadPath("/wobble4.txt");
            grabWobble2=lqr.loadPath("/wobbleGrab2.txt");
            ontoWobble2=lqr.loadPath("/ontoWobble2.txt");
            park = lqr.loadPath("/park.txt");
            shoot=lqr.loadPath("/shoot.txt");
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
        double wobbleX,  wobbleY, wobbleRadius;

        if(auto==0)
        {
            wobbleX=-71.5;
            wobbleY=3.25;
            wobbleRadius=1.2;


        }else if(auto==4)
        {

            wobbleX=-121;
            wobbleY=6;
            wobbleRadius=2.5;

        }
        else
        {

            wobbleX=-100;
            wobbleY=-17;
            wobbleRadius=1.2;


        }

        if(auto==1)
        {

            while(opModeIsActive()&&!lqr.robotInCircle(-56,5,2.5))
            {

                lqr.runLqrDrive(wobble, -56, 5, 0);
                telemetry.addData("x: ", Hardware.x);
                telemetry.addData("y: ", Hardware.y);
                telemetry.addData("theta: ", Hardware.theta);
                telemetry.addData("Goals", wobbleX + " " + wobbleY + " " + wobbleRadius);
                telemetry.update();
                if (robot.x < -40)
                    robot.leftWobbleGoal.setPosition(.5);
            }

        }
        else if(auto==4)
            {

                while(opModeIsActive()&&!lqr.robotInCircle(-36,4,2.5))
                {

                    lqr.runLqrDrive(wobble, -36, 4, 0);
                    telemetry.addData("x: ", Hardware.x);
                    telemetry.addData("y: ", Hardware.y);
                    telemetry.addData("theta: ", Hardware.theta);
                    telemetry.addData("Goals", wobbleX + " " + wobbleY + " " + wobbleRadius);
                    telemetry.update();

                }

            }
        if(auto==4)
        {

            while(opModeIsActive()&&!lqr.robotInCircle(wobbleX,wobbleY,wobbleRadius)&&robot.x>-60)
            {

                lqr.runLqrDrive(wobble,wobbleX,wobbleY,0);
                telemetry.addData("x: ", Hardware.x);
                telemetry.addData("y: ", Hardware.y);
                telemetry.addData("theta: ", Hardware.theta);
                telemetry.addData("Goals",wobbleX+" "+wobbleY+" "+wobbleRadius);
                telemetry.update();
                if(robot.x<-40)
                    robot.leftWobbleGoal.setPosition(.5);

            }
            while(opModeIsActive()&&!lqr.robotInCircle(wobbleX,wobbleY,wobbleRadius))
            {

                lqr.runLqrDrive(wobble4,wobbleX,wobbleY,0);
                telemetry.addData("x: ", Hardware.x);
                telemetry.addData("y: ", Hardware.y);
                telemetry.addData("theta: ", Hardware.theta);
                telemetry.addData("Goals",wobbleX+" "+wobbleY+" "+wobbleRadius);
                telemetry.update();
                if(robot.x<-40)
                    robot.leftWobbleGoal.setPosition(.5);

            }

        }
        else
            while(opModeIsActive()&&!lqr.robotInCircle(wobbleX,wobbleY,wobbleRadius))
            {

            lqr.runLqrDrive(wobble,wobbleX,wobbleY,0);
            telemetry.addData("x: ", Hardware.x);
            telemetry.addData("y: ", Hardware.y);
            telemetry.addData("theta: ", Hardware.theta);
            telemetry.addData("Goals",wobbleX+" "+wobbleY+" "+wobbleRadius);
            telemetry.update();
            if(robot.x<-40)
                robot.leftWobbleGoal.setPosition(.5);

            }
        robot.hardBrakeMotors();
        robot.leftWobbleGoalDown();
        e.reset();
        e.startTime();
        while(e.seconds()<.6);
        robot.clawServoLeftOpen();
        while(e.seconds()<1.5);
        while(opModeIsActive()&&!lqr.robotInCircle(robot.x,wobbleY-5,2))
        {

            lqr.runLqrDrive(wobble,robot.x,wobbleY-5,0);
            telemetry.addData("x: ", Hardware.x);
            telemetry.addData("y: ", Hardware.y);
            telemetry.addData("theta: ", Hardware.theta);
            telemetry.update();

        }
        //drive to point to shoot into the high goal
        while(opModeIsActive()&&!lqr.robotInCircle(-59,-3,3.5))
        {

            robot.setFlyWheelVelocity(2400);
            lqr.runLqrDrive(beforeShoot,-59,-3,0);
            robot.flywheelRotateServoLeft.setPosition(.345);
            telemetry.addData("x: ", Hardware.x);
            telemetry.addData("y: ", Hardware.y);
            telemetry.addData("theta: ", Hardware.theta);
            telemetry.update();

        }
        while(opModeIsActive()&&!lqr.robotInCircle(-59,-4,1.2)||!HelperMethods.nearAngle(Hardware.theta,0,.025))
        {

            robot.setFlyWheelVelocity(2400);
            lqr.runLqrDrive(shoot,-59,-4,0);
            robot.flywheelRotateServoLeft.setPosition(.345);
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
        while(e.seconds()<.8&&opModeIsActive()){robot.setFlyWheelVelocity(2400);}

        for(int i = 0; i<4; i++)
        {
            robot.flickRing();
            e.reset();
            e.startTime();
            while(e.seconds()<.5&&opModeIsActive())
            {
                robot.setFlyWheelVelocity(2400);
                robot.flywheelRotateServoLeft.setPosition(.345);
                telemetry.addData("vel",robot.flywheelMotorLeft.getVelocity());
                telemetry.addData("x: ", Hardware.x);
                telemetry.addData("y: ", Hardware.y);
                telemetry.addData("theta: ", Hardware.theta);
                telemetry.update();
            }
        }
        e.reset();
        e.startTime();
        while(e.seconds()<.5&&opModeIsActive()){robot.setFlyWheelVelocity(2400);}
        robot.setFlyWheelPower(0);
        e.reset();
        e.startTime();
        robot.flywheelRotateServoLeft.setPosition(.7);

        //intake ring from stack
        if(auto==4||auto==1)
        {

            while(e.seconds()<1.3)
            {

                double rotate;
                if((int)(e.seconds()*5)%2==0)
                    rotate=.2;
                else
                    rotate=-.2;

                robot.setIntakePower(1);
                if(e.seconds()>.7)
                    robot.drive(-.075,0,rotate);
                else
                    robot.drive(-.15,.37,0);

            }

            e.reset();
            while(e.seconds()<.3)
            {

                robot.drive(.4,0,0);

            }

        }

        e.reset();
        while(opModeIsActive()&&!lqr.robotInCircle(-27,-60,21))
                {

                    lqr.runLqrDrive(grabWobble2,-27,-62,0);
                    telemetry.addData("x: ", Hardware.x);
                    telemetry.addData("y: ", Hardware.y);
                    telemetry.addData("theta: ", Hardware.theta);
                    telemetry.update();
                    if(e.seconds()>.1)
                    {

                        robot.leftWobbleGoalDown();

                    }

                }
        while(opModeIsActive()&&!lqr.robotInCircle(-25,-40,.45))
        {

            lqr.runLqrDrive(path,-25,-40,0);
            telemetry.addData("x: ", Hardware.x);
            telemetry.addData("y: ", Hardware.y);
            telemetry.addData("theta: ", Hardware.theta);
            telemetry.update();


        }

        robot.hardBrakeMotors();
        e.reset();
        while(e.seconds()<.2);

        while(opModeIsActive()&&!lqr.robotInCircle(-25,-34.5,1))
        {

            lqr.runLqrDrive(ontoWobble2,-25,-34.5,0);
            telemetry.addData("x: ", Hardware.x);
            telemetry.addData("y: ", Hardware.y);
            telemetry.addData("theta: ", Hardware.theta);
            telemetry.update();


        }
        robot.hardBrakeMotors();
        //move to point and grab second wobble
        while(opModeIsActive()&&!lqr.robotInCircle(-25,-33,1))
        {

            lqr.runLqrDrive(ontoWobble2,-25,-33,0);
            telemetry.addData("x: ", Hardware.x);
            telemetry.addData("y: ", Hardware.y);
            telemetry.addData("theta: ", Hardware.theta);
            telemetry.update();

        }
        robot.hardBrakeMotors();
        e.reset();
        while(e.seconds()<.1);
        robot.clawServoLeftClose();

        e.reset();
        while(e.seconds()<2);
        robot.leftWobbleGoal.setPosition(.5);
        e.reset();
        while(e.seconds()<.1);
        while(opModeIsActive()&&!lqr.robotInCircle(-41.1,-33.5,1))
        {

            lqr.runLqrDrive(ontoWobble2,-41.1,-33.5,0);
            telemetry.addData("x: ", Hardware.x);
            telemetry.addData("y: ", Hardware.y);
            telemetry.addData("theta: ", Hardware.theta);
            telemetry.update();


        }
        robot.setIntakePower(0);
        if(auto==4)
        {
            while (opModeIsActive() && !lqr.robotInCircle(-117, -3, 2))
            {

                lqr.runLqrDrive(ontoWobble2, -117, -3, 0);
                telemetry.addData("x: ", Hardware.x);
                telemetry.addData("y: ", Hardware.y);
                telemetry.addData("theta: ", Hardware.theta);
                telemetry.update();


            }
            robot.leftWobbleGoalDown();
            e.reset();
            while (e.seconds() < .5) ;
            robot.clawServoLeftOpen();
            e.reset();
            while (e.seconds() < .2) ;
            while (opModeIsActive() && !lqr.robotInCircle(Hardware.x, -12, 2))
            {

                lqr.runLqrDrive(ontoWobble2, Hardware.x, -12, 0);
                telemetry.addData("x: ", Hardware.x);
                telemetry.addData("y: ", Hardware.y);
                telemetry.addData("theta: ", Hardware.theta);
                telemetry.update();


            }
        }
        else if(auto==1)
        {

            while(opModeIsActive()&&!lqr.robotInCircle(-97,-20,2))
            {

                lqr.runLqrDrive(ontoWobble2,-97,-20,0);
                telemetry.addData("x: ", Hardware.x);
                telemetry.addData("y: ", Hardware.y);
                telemetry.addData("theta: ", Hardware.theta);
                telemetry.update();


            }
            robot.leftWobbleGoalDown();
            e.reset();
            while(e.seconds()<.5);
            robot.clawServoLeftOpen();
            e.reset();
            while(e.seconds()<.2);
            while(opModeIsActive()&&!lqr.robotInCircle(Hardware.x,-27,2))
            {

                lqr.runLqrDrive(ontoWobble2,Hardware.x,-27,0);
                telemetry.addData("x: ", Hardware.x);
                telemetry.addData("y: ", Hardware.y);
                telemetry.addData("theta: ", Hardware.theta);
                telemetry.update();


            }

        }
        else
            {
                while(opModeIsActive()&&!lqr.robotInCircle(-70.5,-3,2))
                {

                    lqr.runLqrDrive(ontoWobble2,-70.5,-3,0);
                    telemetry.addData("x: ", Hardware.x);
                    telemetry.addData("y: ", Hardware.y);
                    telemetry.addData("theta: ", Hardware.theta);
                    telemetry.update();


                }
                robot.leftWobbleGoalDown();
                e.reset();
                while(e.seconds()<.5);
                robot.clawServoLeftOpen();
                e.reset();
                while(e.seconds()<.2);
                while(opModeIsActive()&&!lqr.robotInCircle(Hardware.x,-12,2))
                {

                    lqr.runLqrDrive(ontoWobble2,Hardware.x,-12,0);
                    telemetry.addData("x: ", Hardware.x);
                    telemetry.addData("y: ", Hardware.y);
                    telemetry.addData("theta: ", Hardware.theta);
                    telemetry.update();


                }
            }

        if(auto==1||auto==4)
        {

            while(opModeIsActive()&&!lqr.robotInCircle(-58.5,-5,3.5))
            {

                robot.setFlyWheelVelocity(2400);
                lqr.runLqrDrive(beforeShoot,-58.5,-5,0);
                robot.flywheelRotateServoLeft.setPosition(.345);
                telemetry.addData("x: ", Hardware.x);
                telemetry.addData("y: ", Hardware.y);
                telemetry.addData("theta: ", Hardware.theta);
                telemetry.update();

            }
            while(opModeIsActive()&&!lqr.robotInCircle(-58.5,-5,1.2)||!HelperMethods.nearAngle(Hardware.theta,0,.025))
            {

                robot.setFlyWheelVelocity(2400);
                lqr.runLqrDrive(shoot,-58.5,-5,0);
                robot.flywheelRotateServoLeft.setPosition(.345);
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
            while(e.seconds()<1&&opModeIsActive()){robot.setFlyWheelVelocity(2400);}

            for(int i = 0; i<3; i++)
            {
                robot.flickRing();
                e.reset();
                e.startTime();
                while(e.seconds()<.05&&opModeIsActive())
                {
                    robot.setFlyWheelVelocity(2400);
                    robot.flywheelRotateServoLeft.setPosition(.345);
                    telemetry.addData("vel",robot.flywheelMotorLeft.getVelocity());
                    telemetry.addData("x: ", Hardware.x);
                    telemetry.addData("y: ", Hardware.y);
                    telemetry.addData("theta: ", Hardware.theta);
                    telemetry.update();
                }
            }
            e.reset();
            e.startTime();
            while(e.seconds()<.5&&opModeIsActive()){robot.setFlyWheelVelocity(2400);}
            robot.setFlyWheelPower(0);
            e.reset();
            e.startTime();
            robot.flywheelRotateServoLeft.setPosition(.69);

        }

        while(opModeIsActive()&&!lqr.robotInCircle(-70,-3,2))
        {

            lqr.runLqrDrive(ontoWobble2,-70,-3,0);
            telemetry.addData("x: ", Hardware.x);
            telemetry.addData("y: ", Hardware.y);
            telemetry.addData("theta: ", Hardware.theta);
            telemetry.update();


        }



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
