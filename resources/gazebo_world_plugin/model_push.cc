#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <ctime>

namespace gazebo
{
  class ModelPush : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      start = std::clock();
      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ModelPush::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      // Apply a small linear velocity to the model.

      // gazebo::math::Pose pose = this->model->GetWorldPose();
      // std::cout << pose << std::endl << std::endl;
      // //std::cout << pose << std::endl;
      //
      // pose += gazebo::math::Pose((-0.0, 0.0),
      //                                (-0.0, 0.0),
      //                                (-0.0, 0.0),
      //                                (-0.0, 0.0),
      //                                (-0.001, 0.001),
      //                                (-0.0, 0.0));
     // this->model->SetWorldPose(pose);
      duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
      if(duration > 0){
        this->model->SetLinearVel(ignition::math::Vector3d(0, 3, 0));
        //std::cout << "seconds:" << duration << std::endl;
      }
      if(duration > 10){
        this->model->SetLinearVel(ignition::math::Vector3d(3, 0, 0));

        // pose = gazebo::math::Pose((-0.0, 0.01),
        //                            (-0.01, 0.01),
        //                            (-0.01, 0.01),
        //                            (-0.01, 0.01),
        //                            (-0.01, 0.01),
        //                            (-0.01, 0.01));
        // this->model->SetWorldPose(pose);
        // std::cout << "seconds:" << duration << std::endl;
      }
      if(duration > 20){
        this->model->SetLinearVel(ignition::math::Vector3d(0, -3, 0));
        // std::cout << "seconds:" << duration << std::endl;
      }
      if(duration > 30){
        this->model->SetLinearVel(ignition::math::Vector3d(-3, 0, 0));
        // std::cout << "seconds:" << duration << std::endl;
        // this->model->SetWorldPose(pose);
      }
      if(duration > 40){
        //this->model->SetLinearVel(ignition::math::Vector3d(0, 0, 0));
        // std::cout << "seconds:" << duration << std::endl;
        start = std::clock(); // reset start timer
      }

      //
    }

    private: int cnt = 0;
    std::clock_t start;
    double duration;
    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
