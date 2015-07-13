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

int main(int argc, char **argv) {
  std::cout << LICENSE_INFO << std::flush;

  ros::init(argc, argv, "phasespace_talker");
  PhasespaceCore *talker;
  try {
    talker = new PhasespaceCore("talker");
  } catch (const PhasespaceCoreException& e) {
    ROS_FATAL_STREAM(e.what() << ": failure in 'constructor' function");
    ros::shutdown();
    std::exit(EXIT_FAILURE);
  }

  while (talker->getStatus()) {
    talker->readAndPublish();
    ros::spinOnce();
  }

  delete talker;
  return 0;
}
