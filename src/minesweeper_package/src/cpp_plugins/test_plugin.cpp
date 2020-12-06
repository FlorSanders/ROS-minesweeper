#include <stdio.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

using namespace std;

Class TestPlugin1() : public ModelPlugin() {

public: TestPlugin1() {}

public : virtual void Load(physics::ModelPtr _model,  sdf::ElementPter _sdf) {
     std::cerr "\n test plugin included to model [" << _model->GetName()n << "]\n";
}


};

GZ_REGISTER_MODEL_PLUGIN(TestPlugin1)
