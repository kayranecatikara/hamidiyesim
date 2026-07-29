#ifndef NET_CAPTURE_PLUGIN_HH_
#define NET_CAPTURE_PLUGIN_HH_

#include <memory>

#include <gz/sim/System.hh>

namespace avci
{
/// \brief Atilan agin temas ettigi hedefi kendine kilitleyen eklenti.
///
/// Gazebo Harmonic'te hazir "birlestirme" sistemi yok:
/// gz-sim-detachable-joint-system yalnizca KOPARABILIYOR, calisma aninda
/// yeni bir baglanti KURAMIYOR. CTU MRS'in link_attacher.cpp'si tam bunu
/// yapiyor ama Gazebo Classic + ROS1 icin yazilmis.
///
/// Bu eklenti, ArduPilot'un ParachutePlugin.cc'sindeki deseni
/// (_ecm.CreateEntity() + components::DetachableJoint) ters yonde kullaniyor:
/// parasut komutla AYRILIRKEN, ag komutla degil TEMASLA BAGLANIYOR.
///
/// SDF parametreleri:
/// \code{.xml}
/// <plugin filename="NetCapturePlugin" name="avci::NetCapturePlugin">
///   <net_model>net_cone</net_model>       <!-- agin model adi -->
///   <net_link>net_link</net_link>         <!-- yakalayan link -->
///   <contact_topic>...</contact_topic>    <!-- opsiyonel, otomatik turetilir -->
///   <target_model>target_box</target_model>   <!-- birden fazla olabilir -->
///   <capture_topic>/net/captured</capture_topic>
///   <min_speed>0.0</min_speed>            <!-- bu hizin altinda yakalama yok -->
/// </plugin>
/// \endcode
class NetCapturePlugin
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPreUpdate
{
  public: NetCapturePlugin();
  public: ~NetCapturePlugin() override;

  public: void Configure(const gz::sim::Entity &_entity,
              const std::shared_ptr<const sdf::Element> &_sdf,
              gz::sim::EntityComponentManager &_ecm,
              gz::sim::EventManager &_eventMgr) override;

  public: void PreUpdate(const gz::sim::UpdateInfo &_info,
              gz::sim::EntityComponentManager &_ecm) override;

  private: class Impl;
  private: std::unique_ptr<Impl> impl;
};
}  // namespace avci

#endif  // NET_CAPTURE_PLUGIN_HH_
