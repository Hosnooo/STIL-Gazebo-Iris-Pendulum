#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>
#include <fstream>
#include <iostream>
#include <StabController.h>
#include <TracController.h>
#include <rtwtypes.h>
#include <iomanip> // For table formatting
#include <filesystem> // For file deletion

namespace gazebo
{
class StabControllerPlugin : public ModelPlugin
{
private:
    physics::ModelPtr model;
    physics::LinkPtr baseLink, pendulum, load;
    physics::JointPtr pendulumJoint, loadJoint;
    event::ConnectionPtr updateConnection;
    std::ofstream logFile;
    bool isFileInitialized = false;
    double Param[4] = {1.5, 0.2, 1, 9.8};
    double Kv15[15] = {2.2667,  3.2469,  3.2469,    3.0876,    8.5803,    8.5803,   -0.0224 ,   9.8503,      9.8503,    0 , 5.4498,  5.4498,     0,   -0.0316, -0.0316};
    double Kv12[12] = { 2.2361, 3.1623, 3.1623, 3.0777, 8.4827, 8.4827, 0.0, 9.7962, 9.7962, 0.0, 5.4399, 5.4399};
    double u[3] = {};
    double err[3] = {};
    int stage = 0;

    double holdTime = 10.0; // Delay in seconds before activating the Mission

    //Simulation time
    double currentSimTime;
    double timeSinceLastRequest;
    double lastRequestSimTime;
    double logCount = 0;

    double Setpoint[3] = {};

    //Relative position of the pendulum (Lx, Ly, Lz)
    double Lx;
    double Ly;
    double Lz;
    double L; // Pendulum length

    //Pendulum angles (alpha, beta)
    double beta;
    double alpha;

    //Pendulum angular derivatives
    double gamma_beta;
    double gamma_alpha;

    // Quadrotor orientation in roll, pitch, yaw
    double roll;
    double pitch;
    double yaw;

    //State packets
    double dv[10] = {};
    double currLoc[3] = {};

    double getDistance(double Setpoint[3], double currLoc[3]) {
        double distanceSquared = std::pow((currLoc[0] - Setpoint[0]), 2) 
                               + std::pow((currLoc[1] + Setpoint[1]), 2) 
                               + std::pow((currLoc[2] + Setpoint[2] - 0.95), 2);
        return std::sqrt(distanceSquared);
    }

    void IntrgralStabController(const double x[10], const double Kv[15],
                           const double param[4], const double setpoint[3],
                           double u[3], double err_int[3], double dt = 0.001){
                    
        err_int[0] += (setpoint[0] - x[0])*dt;
        err_int[1] += (setpoint[1] - x[1])*dt;
        err_int[2] += (setpoint[2] - x[2])*dt;

        //set a bound for the error integration
        for(int i=0; i<3; i++){
            if(err_int[i]>20) err_int[i] = 20;
            else if (err_int[i]<-20) err_int[i] = -20;
        }       
        
        double x_ext[13]={};

        for(int i=0; i<10; i++){
            x_ext[i] = x[i];
        }

        for(int i=10; i<13; i++){
            x_ext[i] = err_int[i-10];
        }
    
        StabController(x_ext, Kv, param, setpoint, u);
    }

public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/) override
    {
        model = _model;

        // Get links
        baseLink = model->GetLink("base_link");
        pendulum = model->GetLink("pendulum");
        load = model->GetLink("load");

        // Get joints
        pendulumJoint = model->GetJoint("pendulum_joint");
        loadJoint = model->GetJoint("load_joint");

        if (!baseLink || !pendulum || !load || !pendulumJoint || !loadJoint)
        {
            gzerr << "Required links or joints not found in the model!\n";
            return;
        }

        // Delete the existing log file if it exists
        const std::string fileName = "log.csv";
        if (std::filesystem::exists(fileName))
        {
            std::filesystem::remove(fileName);
        }

        // Create and open CSV file
        logFile.open("log.txt", std::ios::out);
        if (!logFile.is_open())
        {
            gzerr << "Unable to open or create CSV file for logging!\n";
            return;
        }

        // Write header to CSV if not initialized
        if (!isFileInitialized)
        {
            logFile << "Time,PosX,PosY,PosZ,Roll,Pitch,Yaw,VelX,VelY,VelZ,"
                    << "Alpha,Beta,GammaAlpha,GammaBeta,ForceX,ForceY,ForceZ\n";
            isFileInitialized = true;
        }

        #include <iostream> // Required for std::cout

        for (int i = 0; i < 12; i++) {
            std::cout << "Kv[" << i << "]: " << Kv12[i] << std::endl;
        }

        std::cout << "Quad Mass: " << Param[0] << std::endl;
        std::cout << "Pend Mass: " << Param[1] << std::endl;
        std::cout << "Pend Length: " << Param[2] << std::endl;
        std::cout << "Gravity: " << Param[3] << std::endl;

        // Connect to the update event
        updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&StabControllerPlugin::OnUpdate, this));
    }

    void OnUpdate()
    {
        // Retrieve base link (quadrotor) states
        ignition::math::Vector3d quadPosition = baseLink->WorldPose().Pos();
        ignition::math::Vector3d quadVelocity = baseLink->WorldLinearVel();
        ignition::math::Quaterniond quadOrientation = baseLink->WorldPose().Rot();

        // Retrieve load states
        ignition::math::Vector3d loadPosition = load->WorldPose().Pos();
        ignition::math::Vector3d loadVelocity = load->WorldLinearVel();

        // Calculate relative position of the pendulum (Lx, Ly, Lz)
        Lx = loadPosition.X() - quadPosition.X();
        Ly = loadPosition.Y() - quadPosition.Y();
        Lz = loadPosition.Z() - quadPosition.Z();

        // Calculate pendulum angles (alpha, beta)
        L = sqrt((Lx * Lx) + (Ly * Ly) + (Lz * Lz)); // Pendulum length
        beta = asin(Lx/L);
        alpha = asin(-Ly/(L*cos(beta)));

        // Retrieve pendulum joint angular velocities
        gamma_beta = (loadVelocity.X() - quadVelocity.X()) / cos(beta);
        gamma_alpha = ((-loadVelocity.Y()) - (-quadVelocity.Y()) - sin(alpha) * sin(beta) * gamma_beta) /
                            (-cos(alpha) * cos(beta));

        // Retrieve quadrotor orientation in roll, pitch, yaw
        roll = quadOrientation.Roll();
        pitch = quadOrientation.Pitch();
        yaw = quadOrientation.Yaw();

        // Controller logic
        dv[0] = quadPosition.X();
        dv[1] = -quadPosition.Y();
        dv[2] = -quadPosition.Z();
        dv[3] = beta;
        dv[4] = alpha;
        dv[5] = quadVelocity.X();
        dv[6] = -quadVelocity.Y();
        dv[7] = -quadVelocity.Z();
        dv[8] = gamma_alpha;
        dv[9] = gamma_beta;

        currLoc[0] = quadPosition.X();
        currLoc[1] = quadPosition.Y();
        currLoc[2] = quadPosition.Z();

        currentSimTime = model->GetWorld()->SimTime().Double();
        timeSinceLastRequest = currentSimTime - lastRequestSimTime;

        switch (stage)
        {
        case 0:
            Setpoint[0] = 0; Setpoint[1] = 0; Setpoint[2] = -1.5; // Frame is defined: x (+), y (-), z (-) 
            // std::cout << " Still Hovering " << std::endl;
            if (currentSimTime >= holdTime){
                stage = 1;
                holdTime = 0;
            }
            break;
        case 1:
            Setpoint[0] = 1; Setpoint[1] = -1; Setpoint[2] = -2;
            break;
        case 2:
            Setpoint[0] = -2; Setpoint[1] = -1; Setpoint[2] = -3;
            break;
        case 3:
            Setpoint[0] = 1; Setpoint[1] = 2; Setpoint[2] = -2;
            break;
        case 5:
            Setpoint[0] = 0; Setpoint[1] = 0; Setpoint[2] = -1.5;
            break;            
        default:
            break;
        }

        if (stage > 0 && getDistance(Setpoint, currLoc) < 0.2 && holdTime < 3.0)
            holdTime += timeSinceLastRequest;
            
        if (stage > 0 && getDistance(Setpoint, currLoc) < 0.2 && holdTime >= 3.0){
            holdTime = 0;
            std::cout << "Stage: " << stage << std::endl;
            stage += 1;
        }

        if (stage != 4){
            StabController(dv, Kv12, Param, Setpoint, u);
            // IntrgralStabController(dv, Kv15, Param, Setpoint, u, err, timeSinceLastRequest);
        }
        

        if (stage == 4){
            holdTime += timeSinceLastRequest;
            if ( holdTime > 32.0){
                std::cout << "Tracking Done! Landing .." << std::endl;
                stage += 1;
                holdTime = 0;
            }
            else{
                TracController(dv, Kv12, Param, holdTime, u);
                //std::cout << "Still Tracking" << std::endl;
            }           
        }


        ignition::math::Vector3d controlForce(u[0], -u[1], -u[2]);
        this->baseLink->AddForce(controlForce);

        // Update Request Time
        lastRequestSimTime = currentSimTime;

        // Write log as a table to a file each (0.01 secs)
        logCount += timeSinceLastRequest;

        if (logCount > 0.1){
            logCount = 0;
            if (logFile.is_open())
            {
                logFile << std::fixed << std::setprecision(4);
                logFile << "+----------------------------------------------+\n";
                logFile << "| Time: " << currentSimTime << " s\n";
                logFile << "+----------------------------------------------+\n";
                logFile << "| Dsrd Position (X, Y, Z): (" << Setpoint[0] << ", " << Setpoint[1] << ", " << Setpoint[2] << ")\n";
                logFile << "| Quad Position (X, Y, Z): (" << quadPosition.X() << ", " << -quadPosition.Y() << ", " << -quadPosition.Z() + 0.95 << ")\n";
                logFile << "| Orientation (Roll, Pitch, Yaw): (" << roll << ", " << pitch << ", " << yaw << ")\n";
                logFile << "| Velocity (Vx, Vy, Vz): (" << quadVelocity.X() << ", " << quadVelocity.Y() << ", " << quadVelocity.Z() << ")\n";
                logFile << "| Pendulum Angles (Alpha, Beta): (" << alpha << ", " << beta << ")\n";
                logFile << "| Gamma (GammaAlpha, GammaBeta): (" << gamma_alpha << ", " << gamma_beta << ")\n";
                logFile << "| Control Forces (Fx, Fy, Fz): (" << u[0] << ", " << u[1] << ", " << u[2] << ")\n";
                logFile << "+----------------------------------------------+\n\n";
            }
        }
    }

    ~StabControllerPlugin()
    {
        if (logFile.is_open())
        {
            logFile.close();
        }
    }
};

// Register the plugin with Gazebo
GZ_REGISTER_MODEL_PLUGIN(StabControllerPlugin)
} // namespace gazebo