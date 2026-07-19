#ifndef SJTU_DRONE_DESCRIPTION__PLUGIN_DRONE_H_
#define SJTU_DRONE_DESCRIPTION__PLUGIN_DRONE_H_

#include <memory>
#include <string>

#include <gz/sim/System.hh>

namespace sjtu_drone
{

// Forward declare
class DroneSimpleControllerPrivate;

class DroneSimpleController
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:
  DroneSimpleController();
  ~DroneSimpleController() override;

  void Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &_eventMgr) override;

  void PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm) override;

private:
  std::unique_ptr<DroneSimpleControllerPrivate> impl_;
};

}  // namespace sjtu_drone

#endif  // SJTU_DRONE_DESCRIPTION__PLUGIN_DRONE_H_
