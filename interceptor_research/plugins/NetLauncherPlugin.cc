#include "NetLauncherPlugin.hh"

#include <atomic>
#include <chrono>
#include <string>

#include <gz/common/Console.hh>
#include <gz/common/Profiler.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/double.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/Pose.hh>

using namespace avci;
using namespace gz;
using namespace sim;

//////////////////////////////////////////////////
class NetLauncherPlugin::Impl
{
  public: void OnFire(const msgs::Double &_msg);

  /// \brief Dunyada <model>::<link> arar.
  public: static Entity FindLink(EntityComponentManager &_ecm,
              const std::string &_modelName, const std::string &_linkName);

  public: std::string muzzleLinkName{"muzzle_link"};
  public: std::string netModelName{"net_cone"};
  public: std::string netLinkName{"net_link"};
  public: std::string fireTopic;
  public: double defaultSpeed{20.0};
  public: math::Vector3d launchAxis{1.0, 0.0, 0.0};

  public: Entity modelEntity{kNullEntity};
  public: Entity muzzleLink{kNullEntity};
  public: Entity netLink{kNullEntity};
  public: Entity jointEntity{kNullEntity};

  public: transport::Node node;
  public: std::atomic<bool> fireRequested{false};
  public: std::atomic<double> requestedSpeed{0.0};

  public: bool attached{false};
  public: bool fired{false};
  /// \brief Ayirma yapildiktan sonra hizin verilecegi adim.
  public: bool pendingVelocity{false};
};

//////////////////////////////////////////////////
void NetLauncherPlugin::Impl::OnFire(const msgs::Double &_msg)
{
  this->requestedSpeed = _msg.data();
  this->fireRequested = true;
}

//////////////////////////////////////////////////
Entity NetLauncherPlugin::Impl::FindLink(EntityComponentManager &_ecm,
    const std::string &_modelName, const std::string &_linkName)
{
  Entity found{kNullEntity};
  _ecm.Each<components::Link, components::Name, components::ParentEntity>(
      [&](const Entity &_e, const components::Link *, const components::Name *_name,
          const components::ParentEntity *_parent) -> bool
      {
        if (_name->Data() != _linkName)
          return true;
        auto pn = _ecm.Component<components::Name>(_parent->Data());
        if (pn && pn->Data() == _modelName)
        {
          found = _e;
          return false;
        }
        return true;
      });
  return found;
}

//////////////////////////////////////////////////
NetLauncherPlugin::NetLauncherPlugin()
  : impl(std::make_unique<NetLauncherPlugin::Impl>())
{
}

//////////////////////////////////////////////////
NetLauncherPlugin::~NetLauncherPlugin() = default;

//////////////////////////////////////////////////
void NetLauncherPlugin::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm, EventManager &)
{
  this->impl->modelEntity = _entity;

  if (_sdf->HasElement("muzzle_link"))
    this->impl->muzzleLinkName = _sdf->Get<std::string>("muzzle_link");
  if (_sdf->HasElement("net_model"))
    this->impl->netModelName = _sdf->Get<std::string>("net_model");
  if (_sdf->HasElement("net_link"))
    this->impl->netLinkName = _sdf->Get<std::string>("net_link");
  if (_sdf->HasElement("muzzle_speed"))
    this->impl->defaultSpeed = _sdf->Get<double>("muzzle_speed");
  if (_sdf->HasElement("launch_axis"))
    this->impl->launchAxis = _sdf->Get<math::Vector3d>("launch_axis");

  const auto modelName = _ecm.Component<components::Name>(_entity)->Data();
  this->impl->fireTopic = _sdf->HasElement("fire_topic")
      ? _sdf->Get<std::string>("fire_topic")
      : "/" + modelName + "/net/fire";

  if (!this->impl->node.Subscribe(this->impl->fireTopic,
                                  &NetLauncherPlugin::Impl::OnFire,
                                  this->impl.get()))
  {
    gzerr << "[NetLauncher] Atesleme topic'ine abone olunamadi: "
          << this->impl->fireTopic << "\n";
    return;
  }

  gzmsg << "[NetLauncher] hazir | namlu=" << this->impl->muzzleLinkName
        << " | ag=" << this->impl->netModelName << "::" << this->impl->netLinkName
        << " | ates=" << this->impl->fireTopic
        << " | cikis hizi=" << this->impl->defaultSpeed << " m/s\n";
}

//////////////////////////////////////////////////
void NetLauncherPlugin::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("NetLauncherPlugin::PreUpdate");
  if (_info.paused)
    return;

  // 1) Ilk adim(lar)da agi namluya kilitle
  if (!this->impl->attached && !this->impl->fired)
  {
    Model model(this->impl->modelEntity);
    this->impl->muzzleLink = model.LinkByName(_ecm, this->impl->muzzleLinkName);
    this->impl->netLink = Impl::FindLink(_ecm, this->impl->netModelName,
                                         this->impl->netLinkName);
    if (this->impl->muzzleLink == kNullEntity || this->impl->netLink == kNullEntity)
      return;  // henuz dunyaya girmemisler

    this->impl->jointEntity = _ecm.CreateEntity();
    _ecm.CreateComponent(this->impl->jointEntity,
        components::DetachableJoint(
            {this->impl->muzzleLink, this->impl->netLink, "fixed"}));
    this->impl->attached = true;

    Link netLink(this->impl->netLink);
    netLink.EnableVelocityChecks(_ecm, true);
    // WorldPose bileseni yoksa yaratilsin (atis yonu hesabi icin sart)
    if (!_ecm.Component<components::WorldPose>(this->impl->netLink))
      _ecm.CreateComponent(this->impl->netLink, components::WorldPose());
    if (!_ecm.Component<components::WorldPose>(this->impl->muzzleLink))
      _ecm.CreateComponent(this->impl->muzzleLink, components::WorldPose());

    gzmsg << "[NetLauncher] ag namluya kilitlendi\n";
    return;
  }

  // 3) Ayirmanin ISLENDIGI adimin ARDINDAN hizi ver.
  // Ayni adimda hem joint'i silip hem hiz vermek ise yaramiyor: joint hala
  // etkinken verilen hiz fizik cozucude yutuluyor.
  if (this->impl->pendingVelocity)
  {
    this->impl->pendingVelocity = false;

    double speed = this->impl->requestedSpeed.load();
    if (speed <= 0.0)
      speed = this->impl->defaultSpeed;

    Link muzzle(this->impl->muzzleLink);
    Link netLink(this->impl->netLink);

    // 1) Namlunun DUNYA cercevesindeki yonelimi -> atis yonu
    math::Vector3d dirWorld{1, 0, 0};
    if (auto mp = muzzle.WorldPose(_ecm))
      dirWorld = mp->Rot().RotateVector(this->impl->launchAxis);
    dirWorld.Normalize();

    // 2) Cikis hizini TEK ADIMLIK kuvvet impulsu ile ver.
    //
    // Link::SetLinearVelocity denendi ve ISE YARAMADI: link seviyesindeki hiz
    // komutunu DART sessizce yok sayiyor, ag hic kimildamadi (menzil 0.019 m,
    // uc kosumda da ayni). Model seviyesinde calisiyor ama ag ayri bir model
    // ve govdesi tek link.
    //
    // Impuls yontemi eskiden guvenilmezdi ama sebebi impulsun kendisi degil,
    // ayirma ile impulsun AYRI SURECLERDEN gelmesiydi. Burada ikisi de
    // eklentinin icinde ve adim adim siralanmis durumda.
    //   F = m * v / dt   ->  bir adim uygulaninca  dv = F*dt/m = v
    double mass = 0.30;  // net_cone varsayilani
    if (auto inertial = _ecm.Component<components::Inertial>(this->impl->netLink))
      mass = inertial->Data().MassMatrix().Mass();

    const double dt = std::chrono::duration<double>(_info.dt).count();
    if (dt <= 0.0)
    {
      this->impl->pendingVelocity = true;  // bir sonraki adimda tekrar dene
      return;
    }

    const math::Vector3d force = dirWorld * (mass * speed / dt);
    netLink.AddWorldForce(_ecm, force);

    gzmsg << "[NetLauncher] ATES | hiz=" << speed << " m/s | kutle=" << mass
          << " kg | dt=" << dt << " s | kuvvet=" << force.Length() << " N"
          << " | yon=(" << dirWorld.X() << ", " << dirWorld.Y() << ", "
          << dirWorld.Z() << ")\n";
    return;
  }

  // 2) Atesleme istegi: baglantiyi kopar
  if (this->impl->fireRequested.exchange(false) && !this->impl->fired)
  {
    if (this->impl->jointEntity != kNullEntity)
      _ecm.RequestRemoveEntity(this->impl->jointEntity);
    this->impl->fired = true;
    this->impl->pendingVelocity = true;
  }
}

GZ_ADD_PLUGIN(avci::NetLauncherPlugin,
              gz::sim::System,
              avci::NetLauncherPlugin::ISystemConfigure,
              avci::NetLauncherPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(avci::NetLauncherPlugin, "avci::NetLauncherPlugin")
