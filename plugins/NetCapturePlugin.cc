#include "NetCapturePlugin.hh"

#include <mutex>
#include <set>
#include <string>
#include <vector>

#include <gz/common/Profiler.hh>
#include <gz/msgs/contacts.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/common/Console.hh>

using namespace avci;
using namespace gz;
using namespace sim;

//////////////////////////////////////////////////
class NetCapturePlugin::Impl
{
  /// \brief Temas mesaji geldiginde cagrilir (transport is parcaciginda).
  public: void OnContact(const msgs::Contacts &_msg);

  /// \brief Dunyada verilen adi tasiyan modelin link'ini bulur.
  public: Entity FindLink(EntityComponentManager &_ecm,
              const std::string &_modelName, const std::string &_linkName) const;

  /// \brief "model::link" biciminden model adini ayirir.
  public: static std::string ModelOf(const std::string &_scoped);

  public: std::string netModelName{"net_cone"};
  public: std::string netLinkName{"net_link"};
  public: std::string captureTopic{"/net/captured"};
  public: std::set<std::string> targetModels;

  /// \brief Yakalama icin gereken en dusuk ag hizi [m/s].
  /// Ag namluda dururken hedefe surtunse bile kilitlenmesin diye.
  public: double minSpeed{1.0};

  public: transport::Node node;
  public: transport::Node::Publisher capturePub;

  public: std::mutex mutex;
  /// \brief Temas eden, hedef listesindeki model adlari.
  public: std::set<std::string> pendingTargets;

  public: bool captured{false};
  /// \brief Hiz bileseni bir kez etkinlestirildi mi.
  public: bool velocityEnabled{false};
  /// \brief Teshis: hic temas mesaji geldi mi.
  public: bool sawAnyContact{false};
  /// \brief Teshis: gorulen benzersiz collision adlari.
  public: std::set<std::string> seenNames;
  /// \brief <debug>true</debug> ile temas ayrintilari basilir.
  public: bool debug{false};
  public: Entity detachableJointEntity{kNullEntity};
  public: Entity worldEntity{kNullEntity};
};

//////////////////////////////////////////////////
std::string NetCapturePlugin::Impl::ModelOf(const std::string &_scoped)
{
  // Temas mesajlarindaki collision adi "model::link::collision" biciminde
  auto pos = _scoped.find("::");
  return (pos == std::string::npos) ? _scoped : _scoped.substr(0, pos);
}

//////////////////////////////////////////////////
void NetCapturePlugin::Impl::OnContact(const msgs::Contacts &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  if (this->debug && !this->sawAnyContact)
  {
    this->sawAnyContact = true;
    gzmsg << "[NetCapture] ilk temas mesaji alindi (" << _msg.contact_size()
          << " temas)\n";
  }
  for (const auto &contact : _msg.contact())
  {
    for (const auto &name : {contact.collision1().name(), contact.collision2().name()})
    {
      const auto model = ModelOf(name);
      if (this->debug && this->seenNames.insert(name).second)
        gzmsg << "[NetCapture] gorulen temas ismi: " << name << "\n";
      if (model != this->netModelName && this->targetModels.count(model) > 0)
      {
        if (this->pendingTargets.insert(model).second)
          gzmsg << "[NetCapture] hedef temasi kaydedildi: " << model << "\n";
      }
    }
  }
}

//////////////////////////////////////////////////
Entity NetCapturePlugin::Impl::FindLink(EntityComponentManager &_ecm,
    const std::string &_modelName, const std::string &_linkName) const
{
  Entity found{kNullEntity};
  _ecm.Each<components::Link, components::Name, components::ParentEntity>(
      [&](const Entity &_entity, const components::Link *,
          const components::Name *_name, const components::ParentEntity *_parent) -> bool
      {
        if (_name->Data() != _linkName)
          return true;
        auto parentName = _ecm.Component<components::Name>(_parent->Data());
        if (parentName && parentName->Data() == _modelName)
        {
          found = _entity;
          return false;  // aramayi bitir
        }
        return true;
      });
  return found;
}

//////////////////////////////////////////////////
NetCapturePlugin::NetCapturePlugin()
  : impl(std::make_unique<NetCapturePlugin::Impl>())
{
}

//////////////////////////////////////////////////
NetCapturePlugin::~NetCapturePlugin() = default;

//////////////////////////////////////////////////
void NetCapturePlugin::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm, EventManager &)
{
  this->impl->worldEntity = _entity;

  if (_sdf->HasElement("net_model"))
    this->impl->netModelName = _sdf->Get<std::string>("net_model");
  if (_sdf->HasElement("net_link"))
    this->impl->netLinkName = _sdf->Get<std::string>("net_link");
  if (_sdf->HasElement("capture_topic"))
    this->impl->captureTopic = _sdf->Get<std::string>("capture_topic");
  if (_sdf->HasElement("min_speed"))
    this->impl->minSpeed = _sdf->Get<double>("min_speed");
  if (_sdf->HasElement("debug"))
    this->impl->debug = _sdf->Get<bool>("debug");

  // Birden fazla <target_model> olabilir
  for (auto elem = _sdf->FindElement("target_model"); elem;
       elem = elem->GetNextElement("target_model"))
  {
    this->impl->targetModels.insert(elem->Get<std::string>());
  }
  if (this->impl->targetModels.empty())
  {
    gzwarn << "[NetCapture] Hic <target_model> verilmedi; hicbir sey yakalanmaz.\n";
  }

  // Temas topic'i: verilmediyse standart sensor yolundan turetilir
  std::string contactTopic;
  if (_sdf->HasElement("contact_topic"))
  {
    contactTopic = _sdf->Get<std::string>("contact_topic");
  }
  else
  {
    const auto worldName = _ecm.Component<components::Name>(_entity)->Data();
    contactTopic = "/world/" + worldName + "/model/" + this->impl->netModelName +
                   "/link/" + this->impl->netLinkName +
                   "/sensor/net_contact/contact";
  }

  if (!this->impl->node.Subscribe(contactTopic, &NetCapturePlugin::Impl::OnContact,
                                  this->impl.get()))
  {
    gzerr << "[NetCapture] Temas topic'ine abone olunamadi: " << contactTopic << "\n";
    return;
  }

  this->impl->capturePub =
      this->impl->node.Advertise<msgs::StringMsg>(this->impl->captureTopic);

  gzmsg << "[NetCapture] hazir | ag=" << this->impl->netModelName
        << "::" << this->impl->netLinkName
        << " | temas=" << contactTopic
        << " | hedef sayisi=" << this->impl->targetModels.size() << "\n";
}

//////////////////////////////////////////////////
void NetCapturePlugin::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("NetCapturePlugin::PreUpdate");

  if (_info.paused || this->impl->captured)
    return;

  auto netLink = this->impl->FindLink(_ecm, this->impl->netModelName,
                                      this->impl->netLinkName);
  if (netLink == kNullEntity)
    return;  // ag henuz dunyaya girmemis olabilir

  // Hiz bileseni ilk karede olusmuyor; bir kez acip sonraki karelerde okuyoruz.
  Link link(netLink);
  if (!this->impl->velocityEnabled)
  {
    link.EnableVelocityChecks(_ecm, true);
    this->impl->velocityEnabled = true;
    return;  // bu karede hiz henuz gecerli degil
  }

  std::string target;
  {
    std::lock_guard<std::mutex> lock(this->impl->mutex);
    if (this->impl->pendingTargets.empty())
      return;
    target = *this->impl->pendingTargets.begin();
    // DIKKAT: burada TEMIZLEMIYORUZ. Hiz kapisi bu kareyi reddederse temas
    // bilgisi kaybolmasin - ilk denemede tam bu yuzden hic yakalama olmadi
    // (temas geliyordu, hiz 0 okunuyordu, kayit siliniyordu).
  }

  // Ag yeterince hizli mi? (namluda dururken kazara kilitlenmeyi onler)
  if (auto vel = link.WorldLinearVelocity(_ecm))
  {
    if (vel->Length() < this->impl->minSpeed)
      return;
  }
  else
  {
    gzwarn << "[NetCapture] ag hizi okunamadi, yakalama ertelendi\n";
    return;
  }

  // Hedefin ILK link'ini bul (hedef modelin adi biliniyor, link adi bilinmiyor)
  Entity targetLink{kNullEntity};
  _ecm.Each<components::Link, components::Name, components::ParentEntity>(
      [&](const Entity &_e, const components::Link *, const components::Name *,
          const components::ParentEntity *_parent) -> bool
      {
        auto parentName = _ecm.Component<components::Name>(_parent->Data());
        if (parentName && parentName->Data() == target)
        {
          targetLink = _e;
          return false;
        }
        return true;
      });

  if (targetLink == kNullEntity)
  {
    gzerr << "[NetCapture] Hedef link'i bulunamadi: " << target << "\n";
    return;
  }

  // Kilitle: calisma aninda sabit (fixed) DetachableJoint yarat.
  // Desen: ardupilot_gazebo/src/ParachutePlugin.cc
  this->impl->detachableJointEntity = _ecm.CreateEntity();
  _ecm.CreateComponent(this->impl->detachableJointEntity,
      components::DetachableJoint({netLink, targetLink, "fixed"}));

  this->impl->captured = true;
  {
    std::lock_guard<std::mutex> lock(this->impl->mutex);
    this->impl->pendingTargets.clear();
  }

  msgs::StringMsg msg;
  msg.set_data(target);
  this->impl->capturePub.Publish(msg);

  gzmsg << "[NetCapture] YAKALANDI: '" << target << "' aga kilitlendi (t="
        << std::chrono::duration<double>(_info.simTime).count() << " s)\n";
}

GZ_ADD_PLUGIN(avci::NetCapturePlugin,
              gz::sim::System,
              avci::NetCapturePlugin::ISystemConfigure,
              avci::NetCapturePlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(avci::NetCapturePlugin, "avci::NetCapturePlugin")
