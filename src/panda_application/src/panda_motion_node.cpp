#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/MoveAction.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
class PandaMotionNode
{
public:
    PandaMotionNode()
        : nh_(),
          move_group_("panda_arm")

    {
        move_group_.setPlanningTime(5.0);
        move_group_.setMaxVelocityScalingFactor(0.15);
        move_group_.setMaxAccelerationScalingFactor(0.10);

        start_service_ = nh_.advertiseService(
            "/panda_task/start",
            &PandaMotionNode::startTask,
            this
        );
        target_sub_ = nh_.subscribe(
            "/panda_vision/target_pose",
            1,
            &PandaMotionNode::targetCallback,
            this
        );

        move_service_ = nh_.advertiseService(
            "/panda_task/move_to_target",
            &PandaMotionNode::moveToTarget,
            this
        );

        open_gripper_service_ = nh_.advertiseService(
            "/panda_task/open_gripper",
            &PandaMotionNode::openGripper,
            this
        );

        close_gripper_service_ = nh_.advertiseService(
            "/panda_task/close_gripper",
            &PandaMotionNode::closeGripper,
            this
        );
        pick_demo_service_ = nh_.advertiseService(
            "/panda_task/pick_demo",
            &PandaMotionNode::pickDemo,
            this
        );
        plan_approach_service_ = nh_.advertiseService(
            "/panda_task/plan_vision_approach",
            &PandaMotionNode::planVisionApproach,
            this
        );
        move_vision_approach_service_ = nh_.advertiseService(
            "/panda_task/move_vision_approach",
            &PandaMotionNode::moveVisionApproach,
            this
        );
        plan_grasp_service_ = nh_.advertiseService(
            "/panda_task/plan_vision_grasp",
            &PandaMotionNode::planVisionGrasp,
            this
        );
        move_grasp_service_ = nh_.advertiseService(
            "/panda_task/move_vision_grasp",
            &PandaMotionNode::moveVisionGrasp,
            this
        );
        close_and_lift_service_ = nh_.advertiseService(
            "/panda_task/close_and_lift",
            &PandaMotionNode::closeAndLift,
            this
        );
        lift_after_grasp_service_ = nh_.advertiseService(
            "/panda_task/lift_after_grasp",
            &PandaMotionNode::liftAfterGrasp,
            this
        );
        ROS_INFO("Panda motion node ready.");
        ROS_INFO("Vision target topic: /panda_vision/target_pose");
        ROS_INFO("Move service: /panda_task/move_to_target");
    }

private:
    ros::NodeHandle nh_;
    moveit::planning_interface::MoveGroupInterface move_group_;
    ros::ServiceServer start_service_;
    ros::ServiceServer move_service_;
    ros::Subscriber target_sub_;
    typedef actionlib::SimpleActionClient<
    franka_gripper::MoveAction
    > MoveClient;

    typedef actionlib::SimpleActionClient<
    franka_gripper::GraspAction
    > GraspClient;

    MoveClient move_client_{
    "/franka_gripper/move", true
    };

    GraspClient grasp_client_{
    "/franka_gripper/grasp", true
    };

    ros::ServiceServer open_gripper_service_;
    ros::ServiceServer close_gripper_service_;
    ros::ServiceServer pick_demo_service_;
    ros::ServiceServer plan_approach_service_;
    ros::ServiceServer move_vision_approach_service_;
    ros::ServiceServer plan_grasp_service_;
    ros::ServiceServer move_grasp_service_;
    ros::ServiceServer close_and_lift_service_;
    ros::ServiceServer lift_after_grasp_service_;
    geometry_msgs::Pose target_pose_;
    bool has_target_{false};
    bool first_target_received_{false};
    bool grasp_confirmed_{false};

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_{tf_buffer_};

    void targetCallback(
        const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        if (msg->header.frame_id.empty())
        {
            ROS_WARN("Rejected target: frame_id is empty.");
            return;
        }

        geometry_msgs::PoseStamped target_in_base;

        try
        {
            tf_buffer_.transform(
                *msg,
                target_in_base,
                "panda_link0",
                ros::Duration(1.0)
            );
        }
        catch (const tf2::TransformException& ex)
        {
            ROS_WARN_STREAM_THROTTLE(
                5.0,
                "Target transform failed: " << ex.what()
            );
            return;
        }

        const auto& q = target_in_base.pose.orientation;

        const double norm =
            q.x * q.x + q.y * q.y +
            q.z * q.z + q.w * q.w;

        if (norm < 1e-8)
        {
            ROS_WARN("Rejected target: invalid quaternion.");
            return;
        }

        target_pose_ = target_in_base.pose;
        has_target_ = true;

        if (!first_target_received_)
        {
            ROS_INFO_STREAM(
                "First vision target received in panda_link0: x="
                << target_pose_.position.x
                << ", y=" << target_pose_.position.y
                << ", z=" << target_pose_.position.z
            );
            first_target_received_ = true;
        }
        else
        {
            ROS_DEBUG_STREAM_THROTTLE(
                2.0,
                "Vision target in panda_link0: x="
                << target_pose_.position.x
                << ", y=" << target_pose_.position.y
                << ", z=" << target_pose_.position.z
            );
        }
    }
    bool moveVisionApproach(
        std_srvs::Trigger::Request&,
        std_srvs::Trigger::Response& response)
    {
        if (!has_target_)
        {
            response.success = false;
            response.message = "No vision target received.";
            return true;
        }

        // 当前视觉目标是方块顶部中心。
        // 先移动到顶部上方 12 cm，不执行下降抓取。
        geometry_msgs::Pose approach_pose = target_pose_;
        approach_pose.position.z += 0.145;

        // 设置向下的末端姿态。
        approach_pose.orientation.x = 1.0;
        approach_pose.orientation.y = 0.0;
        approach_pose.orientation.z = 0.0;
        approach_pose.orientation.w = 0.0;

        move_group_.setPoseReferenceFrame("panda_link0");
        move_group_.clearPoseTargets();

        // 明确把 panda_hand_tcp 作为目标链接。
        if (!move_group_.setPoseTarget(
                approach_pose, "panda_hand_tcp"))
        {
            response.success = false;
            response.message = "Unable to set TCP target.";
            return true;
        }

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto result = move_group_.plan(plan);

        move_group_.clearPoseTargets();

        if (result != moveit::core::MoveItErrorCode::SUCCESS)
        {
            response.success = false;
            response.message =
                "Approach planning failed. No motion executed.";
            ROS_ERROR_STREAM(response.message);
            return true;
        }

        // 只有规划成功后才执行轨迹。
        result = move_group_.execute(plan);

        response.success =
            (result == moveit::core::MoveItErrorCode::SUCCESS);

        response.message = response.success
            ? "Reached vision approach pose."
            : "Approach trajectory execution failed.";

        ROS_INFO_STREAM(response.message);
        return true;
    }
    bool closeAndLift(
        std_srvs::Trigger::Request&,
        std_srvs::Trigger::Response& response)
    {
        if (!has_target_)
        {
            response.success = false;
            response.message = "No vision target received.";
            return true;
        }

        // 1. 闭合夹爪。
        std_srvs::Trigger::Response gripper_response;
        commandGrasp(gripper_response);

        if (!gripper_response.success)
        {
            response.success = false;
            response.message =
                "Gripper close failed: " + gripper_response.message;
            ROS_ERROR_STREAM(response.message);
            return true;
        }

        // 2. 抬升回到目标上方。
        // target_pose_ 是方块顶部中心，接近点在其上方 12 cm。
        geometry_msgs::Pose lift_pose = target_pose_;
        lift_pose.position.z += 0.145;

        lift_pose.orientation.x = 1.0;
        lift_pose.orientation.y = 0.0;
        lift_pose.orientation.z = 0.0;
        lift_pose.orientation.w = 0.0;

        move_group_.setPoseReferenceFrame("panda_link0");
        move_group_.clearPoseTargets();

        if (!move_group_.setPoseTarget(
                lift_pose, "panda_hand_tcp"))
        {
            response.success = false;
            response.message = "Unable to set TCP lift target.";
            return true;
        }

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto result = move_group_.plan(plan);

        move_group_.clearPoseTargets();

        if (result != moveit::core::MoveItErrorCode::SUCCESS)
        {
            response.success = false;
            response.message =
                "Lift planning failed after gripper closure.";
            ROS_ERROR_STREAM(response.message);
            return true;
        }

        // 3. 只有规划成功后才执行抬升轨迹。
        result = move_group_.execute(plan);

        response.success =
            (result == moveit::core::MoveItErrorCode::SUCCESS);

        response.message = response.success
            ? "Gripper close command succeeded and lift completed."
            : "Gripper closed, but lift execution failed.";

        ROS_INFO_STREAM(response.message);
        return true;
    }
    bool liftAfterGrasp(
        std_srvs::Trigger::Request&,
        std_srvs::Trigger::Response& response)
    {
        if (!grasp_confirmed_)
        {
            response.success = false;
            response.message = "No confirmed grasp. Call close_gripper first.";
            ROS_ERROR_STREAM(response.message);
            return true;
        }

        if (!has_target_)
        {
            response.success = false;
            response.message = "No vision target received.";
            return true;
        }

        move_group_.setPoseReferenceFrame("panda_link0");

        geometry_msgs::PoseStamped current_tcp =
            move_group_.getCurrentPose("panda_hand_tcp");

        if (current_tcp.header.frame_id.empty())
        {
            response.success = false;
            response.message = "Failed to get current TCP pose.";
            ROS_ERROR_STREAM(response.message);
            return true;
        }

        ROS_INFO_STREAM(
            "Current TCP before lift: x="
            << current_tcp.pose.position.x
            << ", y=" << current_tcp.pose.position.y
            << ", z=" << current_tcp.pose.position.z
        );

        ROS_INFO_STREAM(
            "Current TCP quaternion: x="
            << current_tcp.pose.orientation.x
            << ", y=" << current_tcp.pose.orientation.y
            << ", z=" << current_tcp.pose.orientation.z
            << ", w=" << current_tcp.pose.orientation.w
        );

        geometry_msgs::Pose lift_pose = current_tcp.pose;
        lift_pose.position.z += 0.10;

        ROS_INFO_STREAM(
            "Lift target: x="
            << lift_pose.position.x
            << ", y=" << lift_pose.position.y
            << ", z=" << lift_pose.position.z
        );

        move_group_.clearPoseTargets();

        if (!move_group_.setPoseTarget(
                lift_pose, "panda_hand_tcp"))
        {
            response.success = false;
            response.message = "Unable to set TCP lift target.";
            return true;
        }

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto result = move_group_.plan(plan);

        move_group_.clearPoseTargets();

        if (result != moveit::core::MoveItErrorCode::SUCCESS)
        {
            response.success = false;
            response.message = "Lift planning failed.";
            ROS_ERROR_STREAM(response.message);
            return true;
        }

        ROS_INFO("Lift planning succeeded.");

        result = move_group_.execute(plan);

        response.success =
            (result == moveit::core::MoveItErrorCode::SUCCESS);

        response.message = response.success
            ? "Lift after grasp succeeded."
            : "Lift execution failed.";

        ROS_INFO_STREAM(response.message);
        return true;
    }
    bool moveVisionGrasp(
        std_srvs::Trigger::Request&,
        std_srvs::Trigger::Response& response)
    {
        if (!has_target_)
        {
            response.success = false;
            response.message = "No vision target received.";
            return true;
        }

        // 视觉目标是方块顶部中心。
        // 方块边长 5 cm，先将 TCP 目标设在方块中心高度。
        geometry_msgs::Pose grasp_pose = target_pose_;


        // 末端朝下。
        grasp_pose.orientation.x = 1.0;
        grasp_pose.orientation.y = 0.0;
        grasp_pose.orientation.z = 0.0;
        grasp_pose.orientation.w = 0.0;

        move_group_.setPoseReferenceFrame("panda_link0");
        move_group_.clearPoseTargets();

        // 使用真实的夹爪 TCP，而不是默认的 panda_link8。
        if (!move_group_.setPoseTarget(
                grasp_pose, "panda_hand_tcp"))
        {
            response.success = false;
            response.message = "Unable to set TCP grasp target.";
            return true;
        }

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto result = move_group_.plan(plan);

        move_group_.clearPoseTargets();

        if (result != moveit::core::MoveItErrorCode::SUCCESS)
        {
            response.success = false;
            response.message =
                "Grasp descent planning failed. No motion executed.";
            ROS_ERROR_STREAM(response.message);
            return true;
        }

        // 规划成功后才执行下降轨迹。
        result = move_group_.execute(plan);

        response.success =
            (result == moveit::core::MoveItErrorCode::SUCCESS);

        response.message = response.success
            ? "Reached grasp pose. Gripper was not commanded."
            : "Grasp descent execution failed.";

        ROS_INFO_STREAM(response.message);
        return true;
    }
    bool planVisionGrasp(
        std_srvs::Trigger::Request&,
        std_srvs::Trigger::Response& response)
    {
        if (!has_target_)
        {
            response.success = false;
            response.message = "No vision target received.";
            return true;
        }

        // 视觉目标当前表示方块顶部中心。
        // 方块边长为 0.05 m，暂时以中心高度作为抓取点。
        geometry_msgs::Pose grasp_pose = target_pose_;


        // 末端朝下。
        grasp_pose.orientation.x = 1.0;
        grasp_pose.orientation.y = 0.0;
        grasp_pose.orientation.z = 0.0;
        grasp_pose.orientation.w = 0.0;

        move_group_.setPoseReferenceFrame("panda_link0");
        move_group_.clearPoseTargets();

        // 以工具中心点 TCP 作为规划目标。
        if (!move_group_.setPoseTarget(
                grasp_pose, "panda_hand_tcp"))
        {
            response.success = false;
            response.message = "Unable to set TCP grasp target.";
            return true;
        }

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto result = move_group_.plan(plan);

        move_group_.clearPoseTargets();

        response.success =
            (result == moveit::core::MoveItErrorCode::SUCCESS);

        response.message = response.success
            ? "Grasp pose planning succeeded. No motion executed."
            : "Grasp pose planning failed. No motion executed.";

        ROS_INFO_STREAM(response.message);
        return true;
    }
    bool startTask(std_srvs::Trigger::Request&,
                   std_srvs::Trigger::Response& response)
    {
        std::vector<double> joints = move_group_.getCurrentJointValues();

        if (joints.size() != 7)
        {
            response.success = false;
            response.message =
                "Expected 7 Panda arm joints; received " +
                std::to_string(joints.size());
            return true;
        }

        joints[0] += 0.10;
        move_group_.setJointValueTarget(joints);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto result = move_group_.plan(plan);

        if (result != moveit::core::MoveItErrorCode::SUCCESS)
        {
            response.success = false;
            response.message = "Planning failed.";
            return true;
        }

        result = move_group_.execute(plan);
        response.success =
            (result == moveit::core::MoveItErrorCode::SUCCESS);
        response.message = response.success
            ? "Panda motion completed."
            : "Trajectory execution failed.";

        return true;
    }
    bool planVisionApproach(
        std_srvs::Trigger::Request&,
        std_srvs::Trigger::Response& response)
    {
        if (!has_target_)
        {
            response.success = false;
            response.message =
                "No vision target received.";
            return true;
        }

        // 当前视觉目标表示方块顶部中心。
        // 将 TCP 目标设在顶部上方 12 cm。
        geometry_msgs::Pose approach_pose = target_pose_;
        approach_pose.position.z += 0.12;

        // 末端朝下。
        approach_pose.orientation.x = 1.0;
        approach_pose.orientation.y = 0.0;
        approach_pose.orientation.z = 0.0;
        approach_pose.orientation.w = 0.0;

        move_group_.setPoseReferenceFrame("panda_link0");
        move_group_.clearPoseTargets();

        // 明确使用 TCP，而不是默认的 panda_link8。
        if (!move_group_.setPoseTarget(
                approach_pose, "panda_hand_tcp"))
        {
            response.success = false;
            response.message =
                "Unable to set panda_hand_tcp target.";
            return true;
        }

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto result = move_group_.plan(plan);

        move_group_.clearPoseTargets();

        response.success =
            (result == moveit::core::MoveItErrorCode::SUCCESS);

        response.message = response.success
            ? "Approach plan succeeded. Trajectory was not executed."
            : "Approach planning failed. No motion was executed.";

        ROS_INFO_STREAM(response.message);
        return true;
    }
    bool commandOpenGripper(std_srvs::Trigger::Response& response)
    {
        if (!move_client_.waitForServer(ros::Duration(3.0)))
        {
            response.success = false;
            response.message = "Move action server unavailable.";
            ROS_ERROR_STREAM(response.message);
            return true;
        }

        franka_gripper::MoveGoal goal;
        goal.width = 0.08;
        goal.speed = 0.05;

        move_client_.sendGoal(goal);

        if (!move_client_.waitForResult(ros::Duration(10.0)))
        {
            move_client_.cancelGoal();
            response.success = false;
            response.message = "Move action timed out.";
            ROS_ERROR_STREAM(response.message);
            return true;
        }

        const actionlib::SimpleClientGoalState state =
            move_client_.getState();

        response.success =
            (state == actionlib::SimpleClientGoalState::SUCCEEDED);

        response.message = response.success
            ? "Gripper open succeeded."
            : "Gripper open failed: " + state.toString();

        if (response.success)
        {
            grasp_confirmed_ = false;
        }

        ROS_INFO_STREAM(response.message);
        return true;
    }

    bool commandGrasp(std_srvs::Trigger::Response& response)
    {
        if (!grasp_client_.waitForServer(ros::Duration(3.0)))
        {
            response.success = false;
            response.message = "Grasp action server unavailable.";
            ROS_ERROR_STREAM(response.message);
            return true;
        }

        franka_gripper::GraspGoal goal;
        goal.width = 0.05;
        goal.epsilon.inner = 0.005;
        goal.epsilon.outer = 0.005;
        goal.speed = 0.05;
        goal.force = 10.0;

        grasp_client_.sendGoal(goal);

        if (!grasp_client_.waitForResult(ros::Duration(10.0)))
        {
            grasp_client_.cancelGoal();
            response.success = false;
            response.message = "Grasp action timed out.";
            ROS_ERROR_STREAM(response.message);
            return true;
        }

        const actionlib::SimpleClientGoalState state =
            grasp_client_.getState();

        response.success =
            (state == actionlib::SimpleClientGoalState::SUCCEEDED);

        response.message = response.success
            ? "Grasp succeeded."
            : "Grasp failed: " + state.toString();

        if (response.success)
        {
            grasp_confirmed_ = true;
        }

        ROS_INFO_STREAM(response.message);
        return true;
    }

    bool pickDemo(
        std_srvs::Trigger::Request&,
        std_srvs::Trigger::Response& response)
    {
        if (!has_target_)
        {
            response.success = false;
            response.message =
                "No target received. Publish /panda_task/target_pose first.";
            return true;
        }

        // 使用最近一次收到的目标位姿。
        const geometry_msgs::Pose grasp_pose = target_pose_;

        // 抓取前先移动到目标上方 10 cm。
        geometry_msgs::Pose approach_pose = grasp_pose;
        approach_pose.position.z += 0.10;

        std_srvs::Trigger::Response gripper_response;
        std::string error;

        // 1. 打开夹爪。
        commandOpenGripper(gripper_response);

        if (!gripper_response.success)
        {
            response.success = false;
            response.message =
                "Failed to open gripper: " + gripper_response.message;
            return true;
        }

        // 2. 移动到目标上方。
        if (!moveToPose(approach_pose, error))
        {
            response.success = false;
            response.message = "Approach failed: " + error;
            return true;
        }

        // 3. 下降到抓取目标。
        if (!moveToPose(grasp_pose, error))
        {
            response.success = false;
            response.message = "Descent failed: " + error;

            // 尝试退回目标上方。
            std::string recovery_error;
            moveToPose(approach_pose, recovery_error);
            return true;
        }

        // 4. 闭合夹爪。
        commandGrasp(gripper_response);

        if (!gripper_response.success)
        {
            response.success = false;
            response.message =
                "Failed to close gripper: " + gripper_response.message;

            // 尝试抬离目标。
            std::string recovery_error;
            moveToPose(approach_pose, recovery_error);
            return true;
        }

        // 5. 抬起机械臂。
        if (!moveToPose(approach_pose, error))
        {
            response.success = false;
            response.message =
                "Gripper closed, but lift failed: " + error;
            return true;
        }

        response.success = true;
        response.message = "Pick sequence completed.";
        ROS_INFO_STREAM(response.message);

        return true;
    }
    bool openGripper(
        std_srvs::Trigger::Request&,
        std_srvs::Trigger::Response& response)
    {
        return commandOpenGripper(response);
    }
    bool moveToPose(
        const geometry_msgs::Pose& pose,
        std::string& error)
    {
        move_group_.setPoseReferenceFrame("panda_link0");
        move_group_.setPoseTarget(pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto result = move_group_.plan(plan);

        if (result != moveit::core::MoveItErrorCode::SUCCESS)
        {
            move_group_.clearPoseTargets();
            error = "Motion planning failed.";
            ROS_ERROR_STREAM(error);
            return false;
        }

        result = move_group_.execute(plan);
        move_group_.clearPoseTargets();

        if (result != moveit::core::MoveItErrorCode::SUCCESS)
        {
            error = "Trajectory execution failed.";
            ROS_ERROR_STREAM(error);
            return false;
        }

        return true;
    }
    bool closeGripper(
        std_srvs::Trigger::Request&,
        std_srvs::Trigger::Response& response)
    {
        return commandGrasp(response);
    }
    bool moveToTarget(std_srvs::Trigger::Request&,
                      std_srvs::Trigger::Response& response)
    {
        if (!has_target_)
        {
            response.success = false;
            response.message =
                "No target pose received on /panda_task/target_pose.";
            return true;
        }

        // Use the Panda base frame for the target.
        move_group_.setPoseReferenceFrame("panda_link0");
        move_group_.setPoseTarget(target_pose_);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto result = move_group_.plan(plan);

        if (result != moveit::core::MoveItErrorCode::SUCCESS)
        {
            move_group_.clearPoseTargets();
            response.success = false;
            response.message = "Target pose planning failed.";
            return true;
        }

        result = move_group_.execute(plan);
        move_group_.clearPoseTargets();

        response.success =
            (result == moveit::core::MoveItErrorCode::SUCCESS);
        response.message = response.success
            ? "Target pose reached."
            : "Target trajectory execution failed.";

        return true;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "panda_motion_node");

    ros::AsyncSpinner spinner(2);
    spinner.start();

    PandaMotionNode node;
    ros::waitForShutdown();
    return 0;
}
