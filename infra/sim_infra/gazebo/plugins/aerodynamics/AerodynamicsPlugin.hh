/**
 * @Author: Erchao Rong
 * @Date:   2023-04-24 18:14:54
 * @Last Modified by:   Erchao Rong
 * @Last Modified time: 2024-12-29 11:53:12
 */

/**
 * S : Area of the wing
 * rho : the air density
 * link_name : what link the aero force added on
 * **/

#ifndef GZ_SIM_SYSTEMS_LIFT_DRAG_HH_
#define GZ_SIM_SYSTEMS_LIFT_DRAG_HH_

#include <aerodynamics/aero_factory.hpp>
#include <aerodynamics/aero_interface.hpp>
#include <gz/sim/System.hh>
#include <memory>
#include <string>

#include "gz/transport.hh"
namespace gz
{
    namespace sim
    {
        // Inline bracket to help doxygen filtering.
        inline namespace GZ_SIM_VERSION_NAMESPACE
        {
            namespace systems
            {
                constexpr double air_density_in_model = 1.184;
                constexpr double area_in_model        = 0.1955;
                // Forward declaration
                class AerodynamicsPluginPrivate;

                /// \brief The AerodynamicsPlugin system computes lift and drag
                /// forces enabling simulation of aerodynamic robots.
                ///
                class AerodynamicsPlugin : public System,
                                           public ISystemConfigure,
                                           public ISystemPreUpdate
                {
                    /// \brief Constructor
                   public:
                    AerodynamicsPlugin();

                    /// \brief Destructor
                   public:
                    ~AerodynamicsPlugin() override = default;

                    // Documentation inherited
                   public:
                    void Configure(
                        const Entity                              &_entity,
                        const std::shared_ptr<const sdf::Element> &_sdf,
                        EntityComponentManager                    &_ecm,
                        EventManager &_eventMgr) override;

                    /// Documentation inherited
                   public:
                    void PreUpdate(const UpdateInfo       &_info,
                                   EntityComponentManager &_ecm) final;

                    /// \brief Private data pointer
                   private:
                    std::unique_ptr<AerodynamicsPluginPrivate> dataPtr;
                };
            }  // namespace systems
        }      // namespace GZ_SIM_VERSION_NAMESPACE
    }          // namespace sim
}  // namespace gz

#endif
