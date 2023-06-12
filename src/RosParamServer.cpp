#include "removert/RosParamServer.h"


RosParamServer::RosParamServer()
: nh(nh_super), ROSimg_transporter_(nh)
{
    nh.param<bool>("removert/isScanFileKITTIFormat", isScanFileKITTIFormat_, true);

    // for visualization 
 
    scan_rimg_msg_publisher_ = ROSimg_transporter_.advertise("/scan_rimg_single", 10);
    map_rimg_msg_publisher_ = ROSimg_transporter_.advertise("/map_rimg_single", 10);
    diff_rimg_msg_publisher_ = ROSimg_transporter_.advertise("/diff_rimg_single", 10);
    map_rimg_ptidx_msg_publisher_ = ROSimg_transporter_.advertise("/map_rimg_ptidx_single", 10);

    nh.param<float>("removert/rimg_color_min", rimg_color_min_, 0.0);
    nh.param<float>("removert/rimg_color_max", rimg_color_max_, 10.0);
    kRangeColorAxis = std::pair<float, float> {rimg_color_min_, rimg_color_max_}; // meter
    kRangeColorAxisForDiff = std::pair<float, float>{0.0, 0.5}; // meter 

    // fov 
    nh.param<float>("removert/sequence_vfov", kVFOV, 50.0);
    nh.param<float>("removert/sequence_hfov", kHFOV, 360.0);
    kFOV = std::pair<float, float>(kVFOV, kHFOV);

    // resolution 
    nh.param<std::vector<float>>("removert/remove_resolution_list", remove_resolution_list_, std::vector<float>());
    nh.param<std::vector<float>>("removert/revert_resolution_list", revert_resolution_list_, std::vector<float>());

    // sequcne system info 
    nh.param<std::vector<double>>("removert/ExtrinsicLiDARtoPoseBase", kVecExtrinsicLiDARtoPoseBase, std::vector<double>());
    kSE3MatExtrinsicLiDARtoPoseBase = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(kVecExtrinsicLiDARtoPoseBase.data(), 4, 4);
    kSE3MatExtrinsicPoseBasetoLiDAR = kSE3MatExtrinsicLiDARtoPoseBase.inverse();

    // parsing file paths 
    nh.param<std::string>("removert/sequence_scan_dir", sequence_scan_dir_, "/use/your/directory/having/*.bin/pcd");
    for(auto& _entry : fs::directory_iterator(sequence_scan_dir_)) {
        sequence_scan_names_.emplace_back(_entry.path().filename());
        sequence_scan_paths_.emplace_back(_entry.path());
    }
    if (isScanFileKITTIFormat_) {
        std::sort(sequence_scan_names_.begin(), sequence_scan_names_.end());
         std::sort(sequence_scan_paths_.begin(), sequence_scan_paths_.end());
    }
    else {
        std::sort(sequence_scan_names_.begin(), sequence_scan_names_.end(), [](string& s1, string& s2) {
        string _s1 = s1.substr(0, s1.size() - 4);
        string _s2 = s2.substr(0, s2.size() - 4);
        return std::stoi(_s1) < std::stoi(_s2);
         });

        std::sort(sequence_scan_paths_.begin(), sequence_scan_paths_.end(), [](string& s1, string& s2) {
        string _s1;
        for (int i = s1.size()-1; i >=0; --i) {
            if(s1[i] == '/')
                break;
            _s1 = s1[i] + _s1;
        }
        _s1 = _s1.substr(0, _s1.size() - 4);
        string _s2;
        for (int i = s2.size()-1; i >=0; --i) {
            if(s2[i] == '/')
                break;
            _s2 = s2[i] + _s2;
        }
        _s2 = _s2.substr(0, _s2.size() - 4);
        return std::stoi(_s1) < std::stoi(_s2);
        });
    }

    num_total_scans_of_sequence_ = sequence_scan_paths_.size();
    ROS_INFO_STREAM("\033[1;32m Total : " << num_total_scans_of_sequence_ << " scans in the directory.\033[0m");

    // point cloud pre-processing
    nh.param<float>("removert/downsample_voxel_size", kDownsampleVoxelSize, 0.05);

    // parsing pose info
    nh.param<std::string>("removert/sequence_pose_path", sequence_pose_path_, "/use/your/path/having/pose.txt");
    std::ifstream pose_file_handle (sequence_pose_path_);
    int num_poses {0};
    std::string strOneLine;

    if (isScanFileKITTIFormat_) {
        while (getline(pose_file_handle, strOneLine)) 
        {
        // str to vec
        std::vector<double> ith_pose_vec = splitPoseLine(strOneLine, ' ');
        if(ith_pose_vec.size() == 12) {
            ith_pose_vec.emplace_back(double(0.0)); 
            ith_pose_vec.emplace_back(double(0.0)); 
            ith_pose_vec.emplace_back(double(0.0)); 
            ith_pose_vec.emplace_back(double(1.0));
        }
        // vec to eig
        Eigen::Matrix4d ith_pose = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(ith_pose_vec.data(), 4, 4);
        Eigen::Matrix4d ith_pose_inverse = ith_pose.inverse();
        // save (move)
        // cout << "Pose of scan: " << sequence_scan_names_.at(num_poses) << endl;
        // cout << ith_pose << endl;
        sequence_scan_poses_.emplace_back(ith_pose);
        sequence_scan_inverse_poses_.emplace_back(ith_pose_inverse);
        num_poses++;
        }
    }
    else {
        while (getline(pose_file_handle, strOneLine)) 
            {
                // str to vec
                std::vector<double> ith_pose_vec = splitPoseLine(strOneLine, ' ');
                if(ith_pose_vec.size() == 12) {
                    ith_pose_vec.emplace_back(double(0.0)); 
                    ith_pose_vec.emplace_back(double(0.0)); 
                    ith_pose_vec.emplace_back(double(0.0)); 
                    ith_pose_vec.emplace_back(double(1.0));
                }
                std::stringstream ss(strOneLine);
                vector<string> _res;
                string item;
                while(std::getline(ss, item, ' '))  {
                    _res.push_back(item);
                }
                vector<double> _pose = {stod(_res[2]), stod(_res[3]), stod(_res[4]), stod(_res[5]), stod(_res[6]), stod(_res[7]), stod(_res[8])};
                Eigen::Quaterniond _q(_pose[6], _pose[3], _pose[4], _pose[5]);
                Eigen::Vector3d _trans(_pose[0], _pose[1], _pose[2]);
                Eigen::Isometry3d _isom = Eigen::Isometry3d::Identity();
                _isom.rotate(_q);
                _isom.pretranslate(_trans);       
                // vec to eig
                Eigen::Matrix4d ith_pose = _isom.matrix();
                Eigen::Matrix4d ith_pose_inverse = ith_pose.inverse();
                // save (move)
                // cout << "Pose of scan: " << sequence_scan_names_.at(num_poses) << endl;
                // cout << ith_pose << endl;
                sequence_scan_poses_.emplace_back(ith_pose);
                sequence_scan_inverse_poses_.emplace_back(ith_pose_inverse);
                num_poses++;
            }
    }

    // check the number of scans and the number of poses are equivalent
    assert(sequence_scan_paths_.size() == sequence_scan_poses_.size());

    // target scan index range (used in Removert.cpp)
    nh.param<int>("removert/start_idx", start_idx_, 1);
    nh.param<int>("removert/end_idx", end_idx_, 100);
    nh.param<bool>("removert/use_batch_removal", use_batch_removal, false);
    nh.param<int>("removert/batch_size", batch_size, 100);
    nh.param<bool>("removert/visualizationFlag", visualizationFlag, true);

    nh.param<bool>("removert/use_keyframe_gap", use_keyframe_gap_, true);
    nh.param<bool>("removert/use_keyframe_meter", use_keyframe_meter_, false);
    nh.param<int>("removert/keyframe_gap", keyframe_gap_, 10);
    nh.param<float>("removert/keyframe_meter", keyframe_gap_meter_, 2.0);

    // faster
    nh.param<int>("removert/num_omp_cores", kNumOmpCores, 4);

    // save info
    nh.param<bool>("removert/saveMapPCD", kFlagSaveMapPointcloud, false);
    nh.param<bool>("removert/saveCleanScansPCD", kFlagSaveCleanScans, false);
    nh.param<std::string>("removert/save_pcd_directory", save_pcd_directory_, "/");


    usleep(100);
} // ctor RosParamServer

