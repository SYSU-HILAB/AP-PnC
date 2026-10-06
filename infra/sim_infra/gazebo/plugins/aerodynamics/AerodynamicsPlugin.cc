/**
 * @Author: Erchao Rong
 * @Date:   2023-04-24 16:20:46
 * @Last Modified by:   Erchao Rong
 * @Last Modified time: 2024-12-29 15:36:06
 */
/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <algorithm>
#include <gazebo_plugins/AerodynamicsPlugin.hh>
#include <gz/common/Profiler.hh>
#include <gz/msgs.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/ExternalWorldWrenchCmd.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport.hh>
#include <gz/transport/Node.hh>
#include <map>
#include <sdf/Element.hh>
#include <string>
#include <vector>

using namespace gz;
using namespace sim;
using namespace systems;

class gz::sim::systems::AerodynamicsPluginPrivate
{
 private:
  // Model instance created via the shared factory (canonical names:
  // lyu | phi | ma | advanced | none).
  std::shared_ptr<aerodynamics::AerodynamicsInterface> aero;
  std::string                                          aeroModel;
  // Initialize the system
 public:
  void Load(const EntityComponentManager &_ecm, const sdf::ElementPtr &_sdf);

  /// \brief Compute lift and drag forces and update the corresponding
  /// components
  /// \param[in] _ecm Immutable reference to the EntityComponentManager
  void Update(EntityComponentManager &_ecm);

  /// \brief Resolve the aerodynamics model from <aero_model> (deprecated
  /// integer fallback: <aero_id>) and create it via aerodynamics::make_aero
  /// \return true if successful, false otherwise
  bool parseAerodynamicsModel(const sdf::ElementPtr &_sdf);

  /// \brief Model interface
  Model model{kNullEntity};

  /// \brief angle of attach when airfoil stalls  Reserved
  double alphaStall = GZ_PI_2;

  /// \brief air density
  /// at 25 deg C it's about 1.1839 kg/m^3
  /// At 20 °C and 101.325 kPa, dry air has a density of 1.2041 kg/m3.
  double rho = 1.2041;

  /// \brief effective planeform surface area
  double area = 1.0;

  /// \brief center of pressure in link local coordinates with respect to the
  /// link's center of mass
  gz::math::Vector3d cp = math::Vector3d::Zero;

  /// \brief Link entity targeted this plugin.
  Entity linkEntity;

  /// \brief Set during Load to true if the configuration for the system is
  /// valid and the post-update can run
  bool validConfig{false};

  /// \brief Copy of the sdf configuration used for this plugin
  sdf::ElementPtr sdfConfig;

  /// \brief Initialization flag
  bool initialized{false};

  /// \brief Topic name for publishing aerodynamic force
  std::string _topic_force;

  /// \brief Topic name for publishing aerodynamic moment
  std::string _topic_moment;

  /// \brief Publisher for aerodynamic force
  gz::transport::Node::Publisher _force_pub;

  /// \brief Publisher for aerodynamic moment
  gz::transport::Node::Publisher _moment_pub;

  /// \brief Publisher for linear velocity
  gz::transport::Node::Publisher _lin_vel_pub;

  /// \brief Publisher for angular velocity
  gz::transport::Node::Publisher _ang_vel_pub;

  /// \brief Transport node for publishing
  gz::transport::Node _node;

  /// \brief Flag indicating if topics have been advertised
  bool advertised{false};

  /// \brief Flag to omit publishing moment
  bool ifOmitMoment{false};

  /// \brief Flag to omit publishing force
  bool ifOmitForce{false};
};

bool AerodynamicsPluginPrivate::parseAerodynamicsModel(
    const sdf::ElementPtr &_sdf)
{
  std::string model = _sdf->Get<std::string>("aero_model", "").first;

  // Deprecated integer selector (pre-unification): 0=phi 1=ma 2=lyu 3=advanced,
  // any other value meant "none".
  if (model.empty() && _sdf->HasElement("aero_id"))
  {
    static const std::map<int, std::string> legacy_ids{
        {0, "phi"}, {1, "ma"}, {2, "lyu"}, {3, "advanced"}};
    const int  id = _sdf->Get<int>("aero_id", -1).first;
    const auto it = legacy_ids.find(id);
    if (it != legacy_ids.end())
    {
      model = it->second;
      gzwarn << "<aero_id> is deprecated; use <aero_model>" << model
             << "</aero_model> instead" << std::endl;
    }
    else
    {
      model = "none";
      gzwarn << "Invalid <aero_id> " << id
             << "; falling back to 'none'. Use <aero_model> (lyu | phi | ma | "
                "advanced | none) instead."
             << std::endl;
    }
  }

  if (model.empty())
  {
    model = "phi";
    gzwarn << "No <aero_model> specified; defaulting to '" << model
           << "'. Set <aero_model> (lyu | phi | ma | advanced | none) "
              "explicitly."
           << std::endl;
  }

  try
  {
    this->aero      = aerodynamics::make_aero(model);
    this->aeroModel = model;
  }
  catch (const std::exception &e)
  {
    gzerr << "Failed to create aerodynamics model '" << model
          << "': " << e.what() << std::endl;
    this->validConfig = false;
    return false;
  }
  return true;
}
//////////////////////////////////////////////////
void AerodynamicsPluginPrivate::Load(const EntityComponentManager &_ecm,
                                     const sdf::ElementPtr        &_sdf)
{
  // this->alphaStall = _sdf->Get<double>("alpha_stall",
  // this->alphaStall).first;

  this->rho  = _sdf->Get<double>("air_density", this->rho).first;
  this->area = _sdf->Get<double>("area", this->area).first;

  this->cp = _sdf->Get<math::Vector3d>("cp", this->cp).first;
  if (!this->parseAerodynamicsModel(_sdf))
  {
    return;  // validConfig already false; keep the plugin inert
  }
  if (_sdf->HasElement("link_name"))
  {
    sdf::ElementPtr elem     = _sdf->GetElement("link_name");
    auto            linkName = elem->Get<std::string>();
    auto            entities =
        entitiesFromScopedName(linkName, _ecm, this->model.Entity());

    if (entities.empty())
    {
      gzerr << "Link with name[" << linkName << "] not found. "
            << "The AerodynamicsPlugin will not generate forces\n";
      this->validConfig = false;
      return;
    }
    else if (entities.size() > 1)
    {
      gzerr << "Multiple link entities with name[" << linkName << "] found. "
            << "Please use unique name.\n";
    }

    this->linkEntity = *entities.begin();
    if (!_ecm.EntityHasComponentType(this->linkEntity,
                                     components::Link::typeId))
    {
      this->linkEntity = kNullEntity;
      gzerr << "Entity with name[" << linkName << "] is not a link\n";
      this->validConfig = false;
      return;
    }
  }
  else
  {
    gzerr << "The AerodynamicsPlugin system requires the 'link_name' "
             "parameter\n";
    this->validConfig = false;
    return;
  }

  // If we reached here, we have a valid configuration
  this->validConfig = true;

  // if(_sdf->HasElement(""))
  if (advertised)
  {
  }
  else
  {
    auto force   = _sdf->Get<std::string>("topic_force");
    auto moment  = _sdf->Get<std::string>("topic_moment");
    ifOmitMoment = _sdf->Get<bool>("if_omit_moment");
    ifOmitForce  = _sdf->Get<bool>("if_omit_force");
    if (force.size() == 0 && moment.size() == 0)
      gzerr << "The AerodynamicsPlugin system requires the 'topic_force' "
               "'topic_moment' parameter\n";

    // this->_topic_force = force;
    // this->_topic_moment = moment;

    _force_pub   = _node.Advertise<gz::msgs::Vector3d>("/" + model.Name(_ecm) +
                                                       "/" + force);
    _moment_pub  = _node.Advertise<gz::msgs::Vector3d>("/" + model.Name(_ecm) +
                                                       "/" + moment);
    _lin_vel_pub = _node.Advertise<gz::msgs::Vector3d>("/" + model.Name(_ecm) +
                                                       "/" + "lin_vel");
    _ang_vel_pub = _node.Advertise<gz::msgs::Vector3d>("/" + model.Name(_ecm) +
                                                       "/" + "ang_vel");

    advertised = true;
  }
}

//////////////////////////////////////////////////
AerodynamicsPlugin::AerodynamicsPlugin()
    : System(), dataPtr(std::make_unique<AerodynamicsPluginPrivate>())
{
}

//////////////////////////////////////////////////
void AerodynamicsPluginPrivate::Update(EntityComponentManager &_ecm)
{
  GZ_PROFILE("AerodynamicsPluginPrivate::Update");
  // get linear velocity at cp in world frame
  const auto worldLinVel =
      _ecm.Component<components::WorldLinearVelocity>(this->linkEntity);
  const auto worldAngVel =
      _ecm.Component<components::WorldAngularVelocity>(this->linkEntity);
  const auto worldPose =
      _ecm.Component<components::WorldPose>(this->linkEntity);

  if (!worldLinVel || !worldAngVel || !worldPose)
    return;

  const auto &pose    = worldPose->Data();
  const auto  cpWorld = pose.Rot().RotateVector(this->cp);
  const auto  vel = worldLinVel->Data() + worldAngVel->Data().Cross(cpWorld);

  if (vel.Length() <= 0.01)
    const auto velI = math::Vector3d::Zero;
  // else
  //   const auto velI = vel.Normalized();

  auto velB_FLU = pose.Rot().RotateVectorReverse(vel);

  // TODO wrapper to use
  auto Q_frd2flu = math::Quaternion<double>(0, 1.0, 0, 0);

  auto velB = Q_frd2flu.RotateVectorReverse(velB_FLU);
  // velB.Eog
  double alpha = 0;
  double beta  = 0;

  Eigen::Vector3d velB_eigen(velB.X(), velB.Y(), velB.Z());
  Eigen::Vector3d fB_eigen(0, 0, 0);
  Eigen::Vector3d mB_eigen(0, 0, 0);
  this->aero->getAeroWrench(velB_eigen, fB_eigen, mB_eigen, alpha, beta);

  // spanwiseI used to be momentDirection
  // math::Vector3d moment = cm * q * this->area * spanwiseI;
  // force and torque about cg in world frame
  math::Vector3d forceB(fB_eigen[0], fB_eigen[1], fB_eigen[2]);
  math::Vector3d torqueB(mB_eigen[0], mB_eigen[1], mB_eigen[2]);
  if (this->ifOmitForce)
    forceB = math::Vector3d::Zero;
  else
    forceB = Q_frd2flu.RotateVector(forceB) * this->area / 0.24;

  if (this->ifOmitMoment)
    torqueB = math::Vector3d::Zero;
  else
    torqueB = Q_frd2flu.RotateVector(torqueB) * this->area / 0.24;  // no change

  // Correct for nan or inf
  forceB.Correct();
  // this->cp.Correct();
  torqueB.Correct();
  torqueB.X() = 0;
  // torqueB.Y() = 0;
  torqueB.Z() = 0;
  // We want to apply the force at cp. The old AerodynamicsPlugin plugin did
  // the following:
  //     this->link->AddForceAtRelativePosition(force, this->cp);
  // The documentation of AddForceAtRelativePosition says:
  //> Add a force (in world frame coordinates) to the body at a
  //> position relative to the center of mass which is expressed in the
  //> link's own frame of reference.
  // But it appears that 'cp' is specified in the link frame so it probably
  // should have been
  //     this->link->AddForceAtRelativePosition(
  //         force, this->cp - this->link->GetInertial()->CoG());
  //
  // \todo(addisu) Create a convenient API for applying forces at offset
  // positions
  auto forceI  = pose.Rot().RotateVector(forceB);
  auto torqueI = pose.Rot().RotateVector(torqueB);
  // torqueI.Z() = 0;
  // torqueI.X() = 0;
  // const auto totalTorque = torque + cpWorld.Cross(force);
  Link link(this->linkEntity);

  link.AddWorldWrench(_ecm, forceI, torqueI);

  // gz::msgs::Actuators msg;
  // msg.set

  gz::msgs::Vector3d force_msg;
  gz::msgs::Vector3d torque_msg;
  force_msg.set_x(forceB.X());
  force_msg.set_y(forceB.Y());
  force_msg.set_z(forceB.Z());
  torque_msg.set_x(torqueB.X());
  torque_msg.set_y(torqueB.Y());
  torque_msg.set_z(torqueB.Z());

  // gz::msgs::Ve
  auto vel_msg = gz::msgs::Convert(worldLinVel->Data());
  auto ang_msg = gz::msgs::Convert(worldAngVel->Data());

  // testing velB
  //  force_msg.set_x(velB.X());
  //  force_msg.set_y(velB.Y());
  //  force_msg.set_z(velB.Z());

  _force_pub.Publish(force_msg);
  _moment_pub.Publish(torque_msg);
  _lin_vel_pub.Publish(vel_msg);
  _ang_vel_pub.Publish(ang_msg);
  // force_msg.set
  // gz::msgs::Wrench wrenchB;
  // wrenchB.set_allocated_force(&forceB);
  // wrenchB.set_allocated_torque(&torqueB);
  // Debug
  // auto linkName =
  // _ecm.Component<components::Name>(this->linkEntity)->Data(); gzdbg <<
  // "=============================\n"; gzdbg << "Link: [" << linkName << "]
  // pose: [" << pose
  //        << "] dynamic pressure: [" << q << "]\n";
  // gzdbg << "spd: [" << vel.Length() << "] vel: [" << vel << "]\n";
  // gzdbg << "LD plane spd: [" << velInLDPlane.Length() << "] vel : ["
  //        << velInLDPlane << "]\n";
  // gzdbg << "forward (inertial): " << forwardI << "\n";
  // gzdbg << "upward (inertial): " << upwardI << "\n";
  // gzdbg << "q: " << q << "\n";
  // gzdbg << "cl: " << cl << "\n";
  // gzdbg << "lift dir (inertial): " << liftI << "\n";
  // gzdbg << "Span direction (normal to LD plane): " << spanwiseI << "\n";
  // gzdbg << "sweep: " << sweep << "\n";
  // gzdbg << "alpha: " << alpha << "\n";
  // gzdbg << "lift: " << lift << "\n";
  // gzdbg << "drag: " << drag << " cd: " << cd << " cda: "
  //        << this->cda << "\n";
  // gzdbg << "moment: " << moment << "\n";
  // gzdbg << "velB: " << velB << "\n";
  // gzdbg << "force: " << forceB << "\n";
  // gzdbg << "torque: " << torqueB << "\n";
  // gzdbg << "totalTorque: " << totalTorque << "\n";
}

//////////////////////////////////////////////////
void AerodynamicsPlugin::Configure(
    const Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm, EventManager &)
{
  this->dataPtr->model = Model(_entity);
  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "The AerodynamicsPlugin system should be attached to a model "
             "entity. "
          << "Failed to initialize." << std::endl;
    return;
  }
  this->dataPtr->sdfConfig = _sdf->Clone();
}

//////////////////////////////////////////////////
void AerodynamicsPlugin::PreUpdate(const UpdateInfo       &_info,
                                   EntityComponentManager &_ecm)
{
  GZ_PROFILE("AerodynamicsPlugin::PreUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
           << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
           << "s]. System may not work properly." << std::endl;
  }

  if (!this->dataPtr->initialized)
  {
    // We call Load here instead of Configure because we can't be guaranteed
    // that all entities have been created when Configure is called
    this->dataPtr->Load(_ecm, this->dataPtr->sdfConfig);
    this->dataPtr->initialized = true;

    if (this->dataPtr->validConfig)
    {
      Link link(this->dataPtr->linkEntity);
      link.EnableVelocityChecks(_ecm, true);
    }
  }

  if (_info.paused)
    return;

  // This is not an "else" because "initialized" can be set in the if block
  // above
  if (this->dataPtr->initialized && this->dataPtr->validConfig)
  {
    this->dataPtr->Update(_ecm);
  }
}

GZ_ADD_PLUGIN(AerodynamicsPlugin, System, AerodynamicsPlugin::ISystemConfigure,
              AerodynamicsPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(AerodynamicsPlugin, "gz::sim::systems::AerodynamicsPlugin")

// TODO(CH3): Deprecated, remove on version 8
GZ_ADD_PLUGIN_ALIAS(AerodynamicsPlugin,
                    "ignition::gazebo::systems::AerodynamicsPlugin")
