#include "task_demo.h"

MotionManager::MotionManager(ros::NodeHandle &n):node(n){
    
    ar_pose = node.subcribe("ar_pose_marker", 1, ar_marker_cb);
    agv_target_pub = node.advertise<std_msgs::String>("/set_goal", 1);
    get_agv_res = node.subcribe("/move_base/result", 1, get_agv_result);
};

double * task_demo::update_robot_state(){
    robot_state = robot.receiveRobotState();
    current_pose.pos = robot_state.toolTobase_pos_m;
    end_pose.pos = robot_state.toolTobase_pos_m;
    return robot_state.toolTobase_pos_m;
};

bool task_demo::robot_move_to_pose(int pose_num){
    q_init = robot.receiveRobotState().q;
    switch(pose_num):
        case 0:
            MOVEJ(0.2,q_init,robot_start_pose,robot);
            break;
        case 1:
            MOVEJ(0.2,q_init,identify_ar_pose,robot);
            break;
        case 2:
           MOVEJ(0.2,q_init,identify_ar_pose,robot);
           break;
        case 3:
            MOVEJ(0.2,q_init,identify_ar_pose,robot);
            break;
        case 4:
            MOVEJ(0.2,q_init,identify_ar_pose,robot);
            break;
        case 5:
            MOVEJ(0.2,q_init,pubsh_pose,robot);
            break;
        default:
            cout<<"pose_num error!"<<endl;
            break;
};

bool task_demo::robot_roation_move(double roation_z){

    Eigen::Affine3d initial_transform;
    Eigen::Affine3d rot_change;
    Eigen::Affine3d cur_transform;

    std::array<double, 16> init_position;
    
    init_position = update_robot_state();

    initial_transform = Eigen::Matrix4d::Map(init_position.data()).transpose();
    
    rot_change.linear() << Eigen::AngleAxisd(roation_z,Eigen::Vector3d::UnitZ()).toRotationMatrix()*
    Eigen::AngleAxisd(0.0,Eigen::Vector3d::UnitY()).toRotationMatrix()*
    Eigen::AngleAxisd(0.0,Eigen::Vector3d::UnitX()).toRotationMatrix();

    cur_transform.linear()<<initial_transform.linear()*rot_change.linear();
    cur_transform.translation() = initial_transform.translation();
    std::array<double, 16> new_pose;
    Eigen::Map<Eigen::Matrix4d>(&new_pose[0], 4, 4) = cur_transform.matrix().transpose();

    end_pose.pos = new_pose;
    std::cout<<current_pose.pos<<std::endl;
    std::cout<<"---------------"<<std::endl;
    std::cout<<end_pose.pos<<std::endl;
    //end_pose.pos[3] += 0.02;
    MOVEL(0.2, current_pose, end_pose,robot);

    return true;
};

bool task_demo::robot_pan_move(float x = 0.0, float y = 0.0, float z = 0.0){
    
    update_robot_state();
    if(x != 0.0 || y != 0.0 || z != 0.0){
        end_pose.pos[3] += x;
        end_pose.pos[7] += y;
        end_pose.pos[11] += z;
        MOVEL(0.2, current_pose, end_pose,robot);
        return true;
    }else{
        cout<<"end_pose same as current_pose!!"<<endl;
    }
    return false;
};

bool task_demo::robot_move_to_target(float x, float y, float z, double roation_z){
    
    update_robot_state();
    std::cout<<current_pose.pos<<std::endl;
       
    end_pose.pos[7] += ((0.114021+ar_traget_pose.pose.position.x) - 0.06); //-0.114021
    end_pose.pos[3] += ((ar_traget_pose.pose.position.y + 0.038596*cos(robot_angle)) - 0.035*cos(robot_angle));
    MOVEL(0.2, current_pose, end_pose,robot);
    sleep(1.0);

    robot_roation_move(roation_z);
    sleep(1.0);

    update_robot_state();
    if(ar_traget_pose.pose.position.z!=0.0){
    	end_pose.pos[11] -= (ar_traget_pose.pose.position.z-0.120-0.008);
    }else{
    	end_pose.pos[11] -= 0.15;
    }

    MOVEL(0.2, current_pose, end_pose,robot);
    sleep(2.0);

    update_robot_state();
    end_pose.pos[11] += 0.10;
    MOVEL(0.2, current_pose, end_pose,robot);

    return true;
};

void task_demo::agv_move_to_next_target(int point_num){
    if(move_point[point_num]){
        target_msg.data = move_point(point_num)
        agv_target_pub.publish(target_msg);
    }else{
        cout<< "point num error !"<<endl;
    }
};

void task_demo::ar_marker_cb(ar_track_alvar_msgs::AlvarMarkers req){
    int current_id = -1;
    if(!req.markers.empty()){
        for(int i=0;i<req.markers.size();i++){
            if(req.markers[i].id == ar_marker_id){
                current_id = i;
            }
        }
    }
    float wx = req.markers[id].pose.pose.orientation.x;
    float wy = req.markers[id].pose.pose.orientation.y;
    float wz = req.markers[id].pose.pose.orientation.z;
    float ww = req.markers[id].pose.pose.orientation.w;

    float x = req.markers[id].pose.pose.position.x;
    float y = req.markers[id].pose.pose.position.y;
    float z = req.markers[id].pose.pose.position.z;

    ar_traget_pose.pose.position=req.markers[id].pose.pose.position;
    ar_traget_pose.pose.orientation=req.markers[id].pose.pose.orientation;
};


void task_demo::get_agv_result(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg){
    
    switch(msg->status.status){
        case 0:
            cout<<"target waiting to be processed!"<<endl;
            break;
        case 1:
            cout<<"target processing!"<<endl;
            agv_move = true;
            agv_arrive = false;
            break;
        case 2:
            cout<<"target got canceled!"<<endl;
            break;
        case 3:
            cout<<"agv move to target success!"<<endl;
            agv_move = false;
            agv_arrive = true;
            break;
        case 4:
            cout<<"agv move to target fail!"<<endl;
            agv_move = false;
            agv_arrive = false;
            break;
        case 5:
            cout<<"target be rejected!"<<endl;
            break;
        case 6:
            cout<<"target unfinish!"<<endl;
            break;
        case 7:
            cout<<"target revoke!"<<endl;
            break;
        case 8:
            cout<<"target revoke success!"<<endl;
            break;
        default:
            cout<<"unknow status!"<<endl;
            break;
    }
};