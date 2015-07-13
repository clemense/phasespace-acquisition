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

#ifndef GUARD_PHASESPACE_CORE_H
#define GUARD_PHASESPACE_CORE_H

// Standard libraries
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <exception>
#include <vector>
// ROS libraries
#include <ros/ros.h>
#include <ros/time.h>
// Auto-generated from msg/ directory libraries
#include "phasespace_acquisition/PhaseSpaceMarker.h"
#include "phasespace_acquisition/PhaseSpaceMarkerArray.h"
// PhaseSpace API library
#include "owl.h"

// license info to be displayed at the beginning
#define LICENSE_INFO "\n*\n* Copyright (C) 2015 Alessandro Tondo\n* This program comes with ABSOLUTELY NO WARRANTY.\n* This is free software, and you are welcome to\n* redistribute it under GNU GPL v3.0 conditions.\n* (for details see <http://www.gnu.org/licenses/>).\n*\n\n"
// default values for ROS params (if not specified by the user)
#define DEFAULT_TOPIC_NAME "phasespace_topic"
#define DEFAULT_TOPIC_QUEUE_LENGTH 1
#define DEFAULT_MARKER_COUNT 72
#define DEFAULT_INIT_FLAGS 0
#define DEFAULT_SERVER_IP "192.168.1.230"
#define DEFAULT_LOG_FILE_BASE_PATH "logs/"
#define DEFAULT_LOG_FILE_TERMINAL_NAME "_phasespace"
#define DEFAULT_LOG_FILE_TERMINAL_NAME_OLD "_phasespace_old"
#define DEFAULT_LOG_MULTI_FILES false

struct PhasespaceCoreException : std::exception {
  const char* what() const noexcept {return "PhasespaceCore exception";}
};

/*  This class purpose is to provide a ROS interface which lets to retrieve data (marker positions) from the
 *  PhaseSpace Motion Capture system and publish it on a topic using 'phasespace_acquisition::PhaseSpaceMarkerArray'
 *  messages. This data can be collected in log files if necessary.
 *  The class provides several parameters which can be set by the user at runtime to handle several configuration
 *  (e.g., different server ip or maximum number of active markers).
 *
 *  If you edit this class, please try to follow these C++ style guidelines: http://wiki.ros.org/CppStyleGuide.
 *
 *  ROS params:
 *    + topic_name
 *    + topic_queue_length
 *    + verbose_mode
 *    + server_ip
 *    + init_marker_count
 *    + init_flags
 *    + log_file_base_path
 *    + log_file_name
 *    + log_file_name_old
 *    + log_multi_files
 */
class PhasespaceCore {
 public:
  /*  The constructor retrieves the class parameters if specified by the user (default values otherwise) and can be
   *  used to create two distinct purpose object:
   *    + talker: initializes the 'publisher_' to publish 'phasespace_acquisition::PhaseSpaceMarkerArray' messages on
   *      the 'topic_name_' topic (which is provided as a ROS param) and call the 'initializeCommunication()' method to
   *      start the communication with the PhaseSpace Motion Capture system.
   *    + listener: initializes the 'subscriber_' with the 'PhasespaceCore::messageCallback()' method to retrieves
   *      messages on the 'topic_name_' topic (which is provided as a ROS param). Also, initializes the log file
   *      structures where will be stored retrieved message data.
   *  In both cases some statistics variables are initialized.
   *  The destructor prints on screen the statistics.
   *
   *  Parameters:
   *    + mode: specifies if the new istance has to be a publisher node ("talker") or a subscriber one ("listener").
   *  Exceptions:
   *    + directly if mode != "talker" || mode != "listener".
   *    + through 'initializeCommunication()' function.
   *  Other methods called:
   *    + initializeCommunication()
   *  Main private variables modified:
   *    + publisher_ (iff mode == "talker")
   *    + subscriber_ (iff mode == "listener")
   */
  PhasespaceCore(std::string mode);
  ~PhasespaceCore();

  /*  Retrieves data from the PhaseSpace Motion Capture system, checks for errors and if it has changed from the last
   *  acquisition calls the publish method to push a message on the 'toipic_name_' topic (which is set in the
   *  constructor through the relative ROS param 'topic_name').
   *  It must be called only by the 'talker' in its main inside the acquisition loop.
   *
   *  Other methods called:
   *    + publishMessage(...)
   *  Main private variables modified:
   *    + num_data_retrieved_
   *    + partial_num_data_retrieved_
   */
  void readAndPublish();

  /*  Simple method to retrieve 'log_files_map_' (private variable). Only the 'listener' can use this method.
   *
   *  Return value:
   *    + log_files_map_, each field is made up by the log file terminal name and its std::ofstream pointer.
   */
  std::map<std::string, std::ofstream*> getLogFilesMap();

  /*  Simple method to check the ROS status.
   *
   *  Return value:
   *    + true, if ROS is still running.
   */
  bool getStatus();

  /*  Simple method to print on screen (using ROS debug console level) the statistics of the acquired markers. This
   *  method is implemented for a listener node which needs to know if the markers are retrieved correctly at a certain
   *  time. Everytime the method is called, it resets the statistics variables.
   *
   *  Main private variables modified:
   *    + partial_markers_count_
   *    + partial_num_data_retrieved_
   */
  void printStats();

  /*  Updates the number of log files already generated in a multi logs environment (it must be used iff
   *  'log_multi_files_' is set to true in the constructor through the relative ROS param 'log_multi_files').
   *  Only the 'listener' can use this method.
   *
   *  Parameters:
   *    + num_to_add: number of new files to be added to the existing one.
   *  Main private variables modified:
   *    + num_log_files_
   */
  void updateNumLogFiles(int num_to_add);

 private:
  ros::NodeHandle *private_node_handle_;
  ros::NodeHandle node_handle_;
  ros::Publisher publisher_;
  ros::Subscriber subscriber_;

  // PhaseSpace topic variables
  std::string topic_name_;
  int topic_queue_length_;

  // PhaseSpace configuration and reading variables
  std::string server_ip_;
  int init_marker_count_;
  int init_flags_;
  int tracker_;
  OWLMarker *markers_;

  // statistics variables
  ros::Time start_time_;
  int num_data_retrieved_;
  int partial_num_data_retrieved_;
  std::vector<int> *markers_count_;  // occurrences per marker
  std::vector<int> *partial_markers_count_;  // occurrences per marker
  int num_log_files_;

  // log files
  std::string log_file_name_;
  std::string log_file_name_old_;
  std::ofstream log_file_;
  std::ofstream log_file_old_;
  bool log_multi_files_;
  std::string date_time_;
  std::string log_file_base_path_;  // used only if !log_multi_files_ (handled automatically otherwise)
  std::map<std::string, std::ofstream*> log_files_map_;  // terminal file name with the proper std::ofstream pointer

  // other stuff
  bool verbose_mode_;
  PhasespaceCoreException excp_;


  /*  Initializes the communication with the PhaseSpace server throught its API functions. It creates the point traker
   *  and all the active marker structures, based on the settings provided as ROS param (retrieved in the constructor)
   *  that can be specified by the user at runtime. Check the ROS param list above. Finally it enables the data stream.
   *
   *  Exceptions:
   *    + directly if 'owlInit(...)' function fails.
   *    + directly if 'owlGetStatus()' function fails.
   *  Main private variables modified:
   *    + tracker_
   */
  void initializeCommunication();

  /*  Retrieves data from the messages in the subscriber topic queue (when there are available) and fills the log
   *  files with it. Also, calls the 'printPhaseSpaceMarkerArray()' method if 'verbose_mode_' is true.
   *
   *  Parameters:
   *    + msg: points to the last 'phasespace_acquisition::PhaseSpaceMarkerArray' message of the subscriber topic queue.
   *  Other methods called:
   *    + printPhaseSpaceMarkerArray(const std::vector <phasespace_acquisition::PhaseSpaceMarker> &data)
   *  Main private variables modified:
   *    + num_data_retrieved_
   *    + partial_num_data_retrieved_
   */
  void messageCallback(const phasespace_acquisition::PhaseSpaceMarkerArray::ConstPtr &msg);

  /*  Simple method to print on screen (using ROS debug console level) the marker position values of the given
   *  'phasespace_acquisition::PhaseSpaceMarker' vector ('data'). Usually the whole set of visible active markers is
   *  furnished to this method.
   *
   *  Parameters:
   *    + data: marker position values grouped per Marker in a std::vector structure.
   */
  void printPhaseSpaceMarkerArray(const std::vector <phasespace_acquisition::PhaseSpaceMarker> &data);

  /*  Generates a 'phasespace_acquisition::PhaseSpaceMarkerArray' message filled with the data stored in 'markers_'
   *  (previously retrieved from the PhaseSpace server) and publishes it through the 'publisher_' (on the 'topic_name_'
   *  topic provided as ROS param).
   *
   *  Other methods called:
   *    + printPhaseSpaceMarkerArray(const std::vector <phasespace_acquisition::PhaseSpaceMarker> &data)
   */
  void publishMessage(ros::Time current_time, int num_visible_markers, int num_markers);
};

#endif
