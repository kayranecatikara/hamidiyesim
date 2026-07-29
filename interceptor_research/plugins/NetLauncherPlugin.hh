#ifndef NET_LAUNCHER_PLUGIN_HH_
#define NET_LAUNCHER_PLUGIN_HH_

#include <memory>

#include <gz/sim/System.hh>

namespace avci
{
/// \brief Agi namluda tutar ve tek komutla atar.
///
/// NEDEN AYRI EKLENTI (hazir sistemler yetmedi):
/// Once ag, gz-sim-detachable-joint-system ile namluya baglanip
/// gz-sim-apply-link-wrench-system ile itiliyordu. Iki AYRI topic'e iki AYRI
/// disaridan mesaj gitmesi gerekiyordu ve arasindaki gecikme kontrol
/// edilemiyordu: impuls ag hala askidayken gelirse yutuluyor (menzil 2 m),
/// biraz gec gelirse ag once dusuyordu. Ayni parametrelerle olculen menzil
/// kosumlar arasi 2 m ile 108 m arasinda oynadi.
///
/// Bu eklenti ayirma + hiz vermeyi AYNI PreUpdate adiminda yapar; yaris yok.
/// Ayrica impuls (F*dt) yerine dogrudan Link::SetLinearVelocity kullanir:
/// cikis hizi tam istenen deger olur, fizik adimina bagimli degildir.
///
/// SDF parametreleri (interceptor modelinin icine konur):
/// \code{.xml}
/// <plugin filename="NetLauncherPlugin" name="avci::NetLauncherPlugin">
///   <muzzle_link>muzzle_link</muzzle_link>
///   <net_model>net_cone</net_model>
///   <net_link>net_link</net_link>
///   <fire_topic>/avci_net_interceptor/net/fire</fire_topic>
///   <muzzle_speed>20.0</muzzle_speed>      <!-- varsayilan, mesajla ezilebilir -->
///   <launch_axis>1 0 0</launch_axis>       <!-- namlu cerceve ekseni -->
/// </plugin>
/// \endcode
///
/// Atesleme:  gz topic -t <fire_topic> -m gz.msgs.Double -p 'data: 20'
///            (data <= 0 ise <muzzle_speed> kullanilir)
class NetLauncherPlugin
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPreUpdate
{
  public: NetLauncherPlugin();
  public: ~NetLauncherPlugin() override;

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

#endif  // NET_LAUNCHER_PLUGIN_HH_
