/*  Copyright (C) 2015 Alessandro Tondo
 *  email: tondo.codes+ros <at> gmail.com
 *
 *  This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public 
 *  License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any 
 *  later version.
 *  This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied 
 *  warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more 
 *  details.
 *  You should have received a copy of the GNU General Public License along with this program.
 *  If not, see <http://www.gnu.org/licenses/>.
 */

#include "phasespace_acquisition/phasespace_core.h"

#include <sstream>

PhasespaceCore::PhasespaceCore(std::string mode) {
  // handles server private parameters (private names are protected from accidental name collisions)
  private_node_handle_ = new ros::NodeHandle("~");

  // publisher/subscriber info (from ROS params if specified)
  private_node_handle_->param("topic_name", topic_name_, std::string(DEFAULT_TOPIC_NAME));
  private_node_handle_->param("topic_queue_length", topic_queue_length_, DEFAULT_TOPIC_QUEUE_LENGTH);
  private_node_handle_->param("verbose_mode", verbose_mode_, false);

  if (mode == "talker") {
    // PhaseSpace configuration dependent ROS params
    private_node_handle_->param("server_ip", server_ip_, std::string(DEFAULT_SERVER_IP));
    private_node_handle_->param("init_marker_count", init_marker_count_, DEFAULT_MARKER_COUNT);
    private_node_handle_->param("init_flags", init_flags_, DEFAULT_INIT_FLAGS);
    private_node_handle_->param("init_camera_count", init_camera_count_, 4);

    private_node_handle_->param("acquire_camera_poses", acquire_camera_poses_, false);
    acquire_rigid_bodies_ = private_node_handle_->hasParam("rigid_body_files");

    // initializes the communication with PhaseSpace server
    tracker_ = 0;
    markers_ = new OWLMarker[init_marker_count_];
    cameras_ = new OWLCamera[init_camera_count_];
    initializeCommunication();

    publisher_ = node_handle_.advertise<phasespace_acquisition::PhaseSpaceMarkerArray>(topic_name_, topic_queue_length_);
    ROS_INFO_STREAM("[PhaseSpace] Node is publishing messages... (<ctrl+c> to terminate)");
  }
  else if (mode == "listener") {
    private_node_handle_->param("log_file_name", log_file_name_, std::string(DEFAULT_LOG_FILE_TERMINAL_NAME));
    private_node_handle_->param("log_file_name_old", log_file_name_old_, std::string(DEFAULT_LOG_FILE_TERMINAL_NAME_OLD));

    // if 'log_multi_files_' is enabled, the user has to handle the logging process inside the main code using methods
    // provided by this class and the 'Subject()' one. Otherwise any linstener node will build just two log files for
    // the entire acquisition (same data but with a different way of formatting it)
    private_node_handle_->param("log_multi_files", log_multi_files_, DEFAULT_LOG_MULTI_FILES);
    private_node_handle_->param("log_files_base_path", log_file_base_path_, std::string(DEFAULT_LOG_FILE_BASE_PATH));

    if (!log_multi_files_) {
      // stores in 'date_time_' the current time converted into a handful form (date/time format YYYYMMDD_HHMMSS)
      std::time_t raw_time;
      char buffer[16];
      std::time(&raw_time);
      std::strftime(buffer, 16, "%G%m%d_%H%M%S", std::localtime(&raw_time));
      date_time_ = buffer;

      // creates folder if it doesn't exist
      std::string command = "mkdir -p " + log_file_base_path_;
      system(command.c_str());

      // opens the log files
      log_file_.open(log_file_base_path_ + date_time_ + log_file_name_ + ".dat");
      log_file_old_.open(log_file_base_path_ + date_time_ + log_file_name_old_ + ".dat");
    }

    // the 'log_files_map_' is initialized in both cases
    log_files_map_.insert(std::make_pair(log_file_name_, &log_file_));
    log_files_map_.insert(std::make_pair(log_file_name_old_, &log_file_old_));

    subscriber_ = node_handle_.subscribe(topic_name_, topic_queue_length_, &PhasespaceCore::messageCallback, this);
    ROS_INFO_STREAM("[PhaseSpace] Node is retrieving messages... (<ctrl+c> to terminate)");
  }
  else {
    // string 'mode' is not either 'talker' or 'listener' (unexpectedly)
    ROS_FATAL_STREAM("[PhaseSpace] Undefined string passed to the constructor");
    throw excp_;
  }

  // statistics variables initialization
  markers_count_ = new std::vector<int> (init_marker_count_, 0);
  partial_markers_count_ = new std::vector<int> (init_marker_count_, 0);
  num_data_retrieved_ = 0;
  partial_num_data_retrieved_ = 0;
  num_log_files_ = 0;
  start_time_ = ros::Time::now();
}

PhasespaceCore::~PhasespaceCore() {
  ros::Time elapsed_time = ros::Time::now();

  std::cout << "\n\n";

  // closes log files
  if (!log_multi_files_) {
    std::string complete_path;
    std::ifstream last_file;

    complete_path = log_file_base_path_ + date_time_ + log_file_name_ + ".dat";
    if (log_file_.is_open()) {
      num_log_files_++;
      std::cout << "[ INFO] [" << elapsed_time << "]: [PhaseSpace] Log file generated: " << complete_path << '\n';
      log_file_.close();
    }
    // checks for emptiness (in case removes the file)
    last_file.open(complete_path);
    if (last_file.peek() == std::ifstream::traits_type::eof() && std::remove(complete_path.c_str()) == 0) {
      std::cout << "[ INFO] [" << elapsed_time
                << "]: [PhaseSpace] Log file was empty, thus has been removed: " << complete_path << '\n';
    }

    complete_path = log_file_base_path_ + date_time_ + log_file_name_old_ + ".dat";
    if (log_file_old_.is_open()) {
      num_log_files_++;
      std::cout << "[ INFO] [" << elapsed_time << "]: [PhaseSpace] Log file generated: " << complete_path << '\n';
      log_file_old_.close();
    }
    // checks for emptiness
    last_file.open(complete_path);
    if (last_file.peek() == std::ifstream::traits_type::eof() && std::remove(complete_path.c_str()) == 0) {
      std::cout << "[ INFO] [" << elapsed_time
                << "]: [PhaseSpace] Log file was empty, thus has been removed: " << complete_path << '\n';
    }
  }

  // prints statistics (can't use rosconsole macros due to ros::shutdown() callback)
  std::cout << "[ INFO] [" << elapsed_time << "]: [PhaseSpace] Final statistics..." << '\n';
  int marker_id = 0;
  for (auto const& occurrences : *markers_count_) {
    if (occurrences > 0) {
      int perc = 100*occurrences/num_data_retrieved_;
      std::cout << "       + Marker[" << marker_id << "]: " << occurrences << " occurrences (" << perc << " %)" << '\n';
    }
    marker_id++;
  }
  std::cout << "       + Total elapsed time: " << elapsed_time - start_time_ << '\n';
  std::cout << "       + Total data acquired: " << num_data_retrieved_ << '\n';
  std::cout << "       + Total data rate: " << (double)num_data_retrieved_ / (elapsed_time-start_time_).toSec() << '\n';
  std::cout << "       + Total log files created: " << num_log_files_ << std::endl;

  // performs system cleanup and client connection termination
  owlDone();
  delete private_node_handle_;
  delete markers_;
  delete markers_count_;
  delete cameras_;
}

std::map<std::string, std::ofstream*> PhasespaceCore::getLogFilesMap() {
  return log_files_map_;
}

bool PhasespaceCore::getStatus() {
  return node_handle_.ok();
}

void PhasespaceCore::initializeRigidBodies(const std::vector<std::string>& filenames) {
  tracker_++;
  for (std::vector<std::string>::const_iterator it = filenames.begin(); it != filenames.end(); ++it) {
    std::ifstream file(*it);

    if (!file.is_open()) {
      ROS_ERROR_STREAM("[PhaseSpace] Could not open file: " << *it);
      throw excp_;
    }

    // set rigid body tracker
    owlTrackeri(tracker_, OWL_CREATE, OWL_RIGID_TRACKER);

    std::string line;
    // set markers for rigid body
    int i = 0;
    while (std::getline(file, line)) {
      // format is: "led_id, x y z"
      std::cout << line << std::endl;
      std::size_t comma = line.find_first_of(",");
      if (comma == std::string::npos) {
        ROS_FATAL_STREAM("Problem parsing file " << *it);
        ROS_FATAL_STREAM("Line '" << line << "' does NOT contain comma separator.");
        throw excp_;
      }
      int marker_id = boost::lexical_cast<int>(line.substr(0, comma));
      float position[3];
      std::stringstream ss(line.substr(comma+1));
      ss >> position[0] >> position[1] >> position[2];

      owlMarkeri(MARKER(tracker_, marker_id), OWL_SET_LED, marker_id);
      owlMarkerfv(MARKER(tracker_, marker_id), OWL_SET_POSITION, position);

      std::cout << "Marker " << marker_id << " with " << position[0] << " " << position[1] << " " << position[2] << std::endl;
      i++;
    }
    
    ROS_INFO_STREAM("[PhaseSpace] Enabling rigid tracker for: " << *it);
    owlTracker(tracker_, OWL_ENABLE);
    
    // flush requests and check for errors
    if(!owlGetStatus()) {
      ROS_FATAL_STREAM("[PhaseSpace] Error in rigid tracker setup: " << owlGetError());
      return;
    }
  }
}

void PhasespaceCore::initializeCommunication() {
  ROS_INFO_STREAM("[PhaseSpace] Initalizing the PhaseSpace...");

  // opens a socket and configures the communication channels to pass data between the OWL server and client; this
  // function will block until there is a connection or an error; returns the passed flags if OK
  if (owlInit(server_ip_.c_str(), init_flags_) < 0) {
    ROS_FATAL_STREAM("[PhaseSpace] Can't initialize the communication with OWL server: " << owlGetError());
    throw excp_;
  }
  
  // initializes a point tracker:
  //  - OWL_CREATE: tells the system to create a tracker
  //  - OWL_POINT_TRACKER: specifies the creation of a point tracker
  owlTrackeri(tracker_, OWL_CREATE, OWL_POINT_TRACKER);

  // creates the marker structures, each with the proper id:
  //  - MARKER Macro: builds a marker id out of a tracker id and marker index
  //  - OWL_SET_LED: the following 'i' is an integer representing the LED ID of the marker
  for (int i=0; i<init_marker_count_; i++) {
    //if (i != 7 && i != 10 && i != 11)
    owlMarkeri(MARKER(tracker_, i), OWL_SET_LED, i);
  }

  
  // checking the status will block until all commands are processed and any errors are sent to the client
  if (!owlGetStatus()) {
    ROS_FATAL_STREAM("[PhaseSpace] Initialization generic error: " << owlGetError());
    throw excp_;
  }
  
  if (acquire_rigid_bodies_) {
    XmlRpc::XmlRpcValue list_of_files;
    if (private_node_handle_->getParam("rigid_body_files", list_of_files)) {
        ROS_ASSERT(list_of_files.getType() == XmlRpc::XmlRpcValue::TypeArray);
        
        std::vector<std::string> filenames;
        
        for (int32_t i = 0; i < list_of_files.size(); ++i) {
            ROS_ASSERT(list_of_files[i].getType() == XmlRpc::XmlRpcValue::TypeString);
            filenames.push_back(static_cast<std::string>(list_of_files[i]));
        }
        
        initializeRigidBodies(filenames);
        //initializeRigidBodies({"/home/clemens/clemens_sandbox/phasespace_acquisition/rigid_bodies/testobject.rb"});
    }
  }
  
  // enables the tracker (it has to be disabled when the markers have to be added)
  owlTracker(0, OWL_ENABLE);

  // report all values in meters rather than millimeters
  owlScale(0.001);
  
  // sets frequency with default maximum value (OWL_MAX_FREQUENCY = 480 Hz):
  //  - OWL_FREQUENCY: specifies the rate at which the server streams data
  owlSetFloat(OWL_FREQUENCY, OWL_MAX_FREQUENCY);
  // enables the streaming of data
  owlSetInteger(OWL_STREAMING, OWL_ENABLE);
}

void PhasespaceCore::messageCallback(const phasespace_acquisition::PhaseSpaceMarkerArray::ConstPtr &msg) {
  // counts how many markers are visible at the moment
  int* markers_count = markers_count_->data();
  for (auto const& marker : msg->data) {
    markers_count[marker.id]++;  // if received, it has 'cond' value > 0 for sure
  }
  if (verbose_mode_) {
    printPhaseSpaceMarkerArray(msg->data);  // debug info
  }

  // please, make sure that a node does not use both this function and 'readAndPublish()' one
  num_data_retrieved_++;
  partial_num_data_retrieved_++;

  // stores all marker positions in log files ('log_file_old_' only use a different format)
  for (auto const& marker : msg->data) {
    log_file_ << std::fixed
              << msg->acquisition_time << ";"
              << marker.id << ";"
              << marker.cond << ";"
              << marker.point.x << ";"
              << marker.point.y << ";"
              << marker.point.z << std::endl;

    log_file_old_ << std::fixed
                  << marker.id << " "
                  << marker.cond << " "
                  << marker.point.x << " "
                  << marker.point.y << " "
                  << marker.point.z << std::endl;
  }
  log_file_old_ << msg->acquisition_time << " " << msg->num_acquired_makers << " 0 0 0" << std::endl << std::endl;
}

void PhasespaceCore::printPhaseSpaceMarkerArray(const std::vector<phasespace_acquisition::PhaseSpaceMarker> &data) {
  for (auto const& marker : data) {
    ROS_DEBUG_STREAM("[PhaseSpace] marker[" << marker.id << "] = ["
                     << marker.point.x << "  "
                     << marker.point.y << "  "
                     << marker.point.z << "]");
  }

  if (!data.empty()) {
    ROS_DEBUG_STREAM("");  // adds one extra line to space the output
  }
}

void PhasespaceCore::printStats() {
  ROS_INFO_STREAM("[PhaseSpace] Partial statistics...");
  int marker_id = 0;
  for (auto const& occurrences : *partial_markers_count_) {
    if (occurrences > 0) {
      int perc = 100*occurrences/partial_num_data_retrieved_;
      std::cout << "       + Marker[" << marker_id << "]: " << occurrences << " occurrences (" << perc << " %)" << '\n';
    }
    marker_id++;
  }
  std::cout << "       + Data acquired: " << partial_num_data_retrieved_ << std::endl;

  // resets values
  partial_num_data_retrieved_ = 0;
  partial_markers_count_->clear();
}

void PhasespaceCore::publishMessage(ros::Time current_time, int num_visible_markers, int num_markers) {
  // 'markers' message contains a vector of 'marker' messages
  phasespace_acquisition::PhaseSpaceMarker marker;
  phasespace_acquisition::PhaseSpaceMarkerArray markers;
  
  markers.acquisition_time = current_time - start_time_;
  markers.num_acquired_makers = num_visible_markers;
  markers.header.stamp = ros::Time::now();
  markers.header.frame_id = "/phasespace_world";
  
  for (int i=0; i<num_markers; i++) {
    if(markers_[i].cond > 0) {
      marker.id = INDEX(markers_[i].id);
      marker.cond = markers_[i].cond;
      marker.point.x = markers_[i].x;
      marker.point.y = markers_[i].y;
      marker.point.z = markers_[i].z;
      
      markers.data.push_back(marker);
    }
  }

  if (verbose_mode_){
    printPhaseSpaceMarkerArray(markers.data);  // debug info
  }

  publisher_.publish(markers);
}

void PhasespaceCore::readAndPublish() {
  // get the rigid body
  int num_rigids = 0;
  if (acquire_rigid_bodies_) {
    num_rigids = owlGetRigids(&rigid_, 1);
  }
  
  // queries the server for new markers data and returns the number of current active markers (0 means old data)
  int num_markers = owlGetMarkers(markers_, init_marker_count_);
  if (owlGetError() != OWL_NO_ERROR) {
    ROS_FATAL_STREAM("[PhaseSpace] Error while reading markers' positions: " << owlGetError());
    throw excp_;
  }

  // waits for available data
  if (num_markers == 0) {
    return;
  }

  ros::Time current_time = ros::Time::now();

  // please, make sure that a node does not use both this function and 'gloveMessageCallback(&msg)' one
  num_data_retrieved_++;
  partial_num_data_retrieved_++;

  // counts how many markers are visible at the moment
  int* markers_count = markers_count_->data();
  int num_visible_markers = 0;
  for (int i=0; i<num_markers; i++) {
    if (markers_[i].cond > 0) {
      markers_count[i]++;
      num_visible_markers++;
    }
  }

  publishMessage(current_time, num_visible_markers, num_markers);
  
  if (num_rigids > 0) {
    if (rigid_.cond > 0) {
      std::cout << num_rigids << " " << rigid_.cond << std::endl;
      //tf::Transform transform(tf::Quaternion(rigid_[i].pose[4], rigid_[i].pose[5], rigid_[i].pose[6], rigid_[i].pose[3]), tf::Vector3(rigid_[i].pose[0], rigid_[i].pose[1], rigid_[i].pose[2]));
      tf::Transform transform(tf::Quaternion(rigid_.pose[4], rigid_.pose[5], rigid_.pose[6], rigid_.pose[3]), tf::Vector3(rigid_.pose[0], rigid_.pose[1], rigid_.pose[2]));
      tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "phasespace_world", std::string("rigid_body")));
    }
  }

  if (acquire_camera_poses_) {
    int num_cameras = owlGetCameras(cameras_, init_camera_count_);
    if (owlGetError() != OWL_NO_ERROR) {
      ROS_FATAL_STREAM("[PhaseSpace] Error while reading cameras' positions: " << owlGetError());
      throw excp_;
    }

    if (num_cameras == 0) {
      return;
    }
    
    // publish camera poses
    for (int i=0; i<num_cameras; i++) {
      if (cameras_[i].cond > 0) {
        tf::Transform transform(tf::Quaternion(cameras_[i].pose[4], cameras_[i].pose[5], cameras_[i].pose[6], cameras_[i].pose[3]), tf::Vector3(cameras_[i].pose[0], cameras_[i].pose[1], cameras_[i].pose[2]));
        tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "phasespace_world", std::string("phasespace_camera_") + boost::lexical_cast<std::string>(cameras_[i].id)));
      }
    }
  }
}

void PhasespaceCore::updateNumLogFiles(int num_to_add) {
  if (std::abs(num_to_add) > log_files_map_.size()) {
    ROS_ERROR_STREAM("[PhaseSpace] Number of log files to be added (" << num_to_add <<
                     ") exceed the number of actual log files (" << log_files_map_.size() << ")");
    return;
  }
  num_log_files_ += num_to_add;
}

