#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpProjectionDisplay.h>
#include <visp3/robot/vpSimulatorCamera.h>
#include <visp3/robot/vpWireFrameSimulator.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>

void display_trajectory(const vpImage<unsigned char> &I, std::vector<vpPoint> &point, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam);


void display_trajectory(const vpImage<unsigned char> &I, std::vector<vpPoint> &point, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam)
{
    static std::vector<vpImagePoint> traj[4];
    vpImagePoint cog;
    for (unsigned int i = 0; i<4; i++)
    {
        point[i].project(cMo);
        vpMeterPixelConversion::convertPoint(cam,point[i].get_x(),point[i].get_y(), cog);
        traj[i].push_back(cog);
    }
    for(unsigned int i = 0 ; i < 4 ; i++)
        for(unsigned int j =1 ; j < traj[i].size(); j++)
        {
            vpDisplay::displayLine(I, traj[i][j-1], traj[i][j],vpColor::green);

        }
}

int main()
{
    try
    {
        // declaring the intial and final points
        vpHomogeneousMatrix cdMo(0, 0, 0.75, 0, 0, 0);
        vpHomogeneousMatrix cMo(0.15, -0.1, 1., vpMath::rad(10), vpMath::rad(-10), vpMath::rad(50));

        //setting the four corners of the object in world frame
        std::vector<vpPoint> point(4);
        point[0].setWorldCoordinates(-0.1, -0.1, 0);
        point[1].setWorldCoordinates(0.1, -0.1, 0);
        point[2].setWorldCoordinates(0.1, 0.1, 0);
        point[3].setWorldCoordinates(-0.1, 0.1, 0);

        //setting up the visual servo task as eyeinhand with gain = 0.5
        vpServo task;
        task.setServo(vpServo::EYEINHAND_CAMERA);
        task.setInteractionMatrixType(vpServo::CURRENT);
        task.setLambda(0.5);

        //current point feature p[i] and desired point feature pd[i]
        vpFeaturePoint p[4], pd[4];
        for (unsigned int i = 0; i < 4; i++)
        {
          point[i].track(cdMo);
          vpFeatureBuilder::create(pd[i], point[i]);
          point[i].track(cMo);
          vpFeatureBuilder::create(p[i], point[i]);
          task.addFeature(p[i], pd[i]);   //points added to the tracking visual sevo task
        }
        //writing matrices for two homogeneous transformation wMc & wMo position of camera and positon of the object in the world
        vpHomogeneousMatrix wMc, wMo;

        //create instance of free flying camera and sampling time as 0.040, this will be used to find the next position of the camera
        vpSimulatorCamera robot;
        robot.setSamplingTime(0.040);
        robot.getPosition(wMc);  // position of camera in world frame
        wMo = wMc * cMo;     //obtain position of object in world frame by multiplying position of camera in world frame and position of object in camera frame

        // now we'll create two windows to simulate internal and external views of the camera
        vpImage<unsigned char> Iint(480,640,255);
        vpImage<unsigned char> Iext(480,640,255);

        //if you have x11 or gdi to display frames since im having vpDisplayX i'm using that
        vpDisplayX displayInt(Iint, 0, 0, "Internal view");
        vpDisplayX displayEnt(Iext, 670, 0, "External view");

        //defining camera papameters
        vpCameraParameters cam(840, 840, Iint.getWidth()/2, Iint.getHeight()/2);
        vpHomogeneousMatrix cextMo(0,0,3,0,0,0);   //position of the external camera that will observe the evolution of the simulated camera during the servo

        //parameters for the wireframe simulator
        vpWireFrameSimulator sim;
        sim.initScene(vpWireFrameSimulator::PLATE, vpWireFrameSimulator::D_STANDARD);   // defining the shapes of the object in the scene and d_standard indicates that the object displayed at desired position is also a plate
        sim.setCameraPositionRelObj(cMo);    //the servo loop updates the wireframe simulator with new camera position
        sim.setDesiredCameraPosition(cdMo);
        sim.setExternalCameraPosition(cextMo);
        sim.setExternalCameraParameters(cam);
        sim.setInternalCameraParameters(cam);

        while(1)
        {
            robot.getPosition(wMc);   //as there is motion of the robot so the camera coordinates in world frame changes
            //When a velocity is applied to our free flying camera, the position of the camera frame wMc will evolve wrt the world frame. From this position we compute the position of object in the new camera frame.
            cMo = wMc.inverse() * wMo;  //obtaining the position of the object in camera frame
            for(unsigned int i = 0; i<4; i++)
            {
                point[i].track(cMo);
                vpFeatureBuilder::create(p[i],point[i]);  //The current visual features are then updated by projecting the 3D points in the image-plane associated to the new camera location cMo.
            }
            vpColVector v = task.computeControlLaw();   //the velocity skew $ {\bf v}_c $ is computed.
            robot.setVelocity(vpRobot::CAMERA_FRAME,v);   // 6-dimension velocity vector is then applied to the camera.

            sim.setCameraPositionRelObj(cMo);

            vpDisplay::display(Iint);
            vpDisplay::display(Iext);

            sim.getInternalImage(Iint);
            sim.getExternalImage(Iext);

            display_trajectory(Iint, point, cMo, cam);
            vpDisplay::flush(Iint);
            vpDisplay::flush(Iext);

            if (vpDisplay::getClick(Iint, false))
                    break;
                  vpTime::wait(1000 * robot.getSamplingTime());
        }
        task.kill();
    }
    catch (const vpException &e)
    {
        std::cout << "Catch an exception: " << e << std::endl;
      }

}
