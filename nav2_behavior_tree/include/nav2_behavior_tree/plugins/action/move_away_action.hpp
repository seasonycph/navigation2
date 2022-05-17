#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__MOVE_AWAY_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__MOVE_AWAY_ACTION_HPP_

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "msg_srvs_pkg/action/sb_recovery.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A nav2_behavior_tree::BtActionNode class that wraps msg_srvs_pkg::action::SbRecovery
 */
class SBRecoveryAction : public BtActionNode<msg_srvs_pkg::action::SbRecovery>
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::SBRecoveryAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  SBRecoveryAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Function to perform some user-defined operation on tick
   */
  void on_tick() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<int>("max_time", 1, "Max time")
      });
  }
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__MOVE_AWAY_ACTION_HPP_
