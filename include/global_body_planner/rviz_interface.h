#ifndef RVIZ_INTERFACE_H
#define RVIZ_INTERFACE_H

#include <ros/ros.h>
#include <global_body_planner/BodyPlan.h>

//! A class for interfacing between RViz and planning topics.
/*!
   RVizInterface is a container for all of the logic utilized in the template node.
   The implementation must provide a clean and high level interface to the core algorithm
*/
class RVizInterface {
public:
	/**
	 * @brief Constructor for RVizInterface Class
	 * @param[in] nh ROS NodeHandle to publish and subscribe from
	 * @return Constructed object of type RVizInterface
	 */
	RVizInterface(ros::NodeHandle nh);

	/**
	 * @brief Calls ros spinOnce and pubs data at set frequency
	 */
	void spin();

private:
	/**
     * @brief Callback function to handle new body plan data
     * @param[in] Body plan message contining interpolated output of body planner
     */
    void bodyPlanCallback(const global_body_planner::BodyPlan::ConstPtr& msg);

    /**
     * @brief Callback function to handle new body plan discrete state data
     * @param[in] Body plan message contining discrete output of body planner
     */
    void discreteBodyPlanCallback(const global_body_planner::BodyPlan::ConstPtr& msg);

	/// ROS subscriber for the body plan
	ros::Subscriber body_plan_sub_;

	/// ROS subscriber for the body plan
	ros::Subscriber discrete_body_plan_sub_;

	/// ROS Publisher for the interpolated body plan vizualization
	ros::Publisher body_plan_viz_pub_;

	/// ROS Publisher for the discrete body plan vizualization
	ros::Publisher discrete_body_plan_viz_pub_;

	/// Nodehandle to pub to and sub from
	ros::NodeHandle nh_;

	/// Update rate for sending and receiving data, unused since pubs are called in callbacks
	double update_rate_;

	/// Handle for the map frame
    std::string map_frame_;
};

#endif // RVIZ_INTERFACE_H
