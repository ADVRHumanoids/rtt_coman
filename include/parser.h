/*
 * Copyright (C) 2016 Cogimon
 * Author: Enrico Mingo Hoffman
 * email:  enrico.mingo@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU Lesser General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#ifndef _RTT_COMAN_PARSER_H__
#define _RTT_COMAN_PARSER_H__

#include <control_modes.h>
#include <tinyxml.h>
#include <srdfdom_advr/model.h>
#include <urdf/model.h>
#include <algorithm>

namespace cogimon {

struct parsed_words{
        static constexpr const char* robot_tag = "robot";
        static constexpr const char* rtt_gazebo_tag = "rtt-gazebo";
        static constexpr const char* controller_tag = "controller";
        static constexpr const char* reference_attribute = "reference";
        static constexpr const char* type_attribute = "type";
};

struct gains{
    typedef std::string kinematic_chain;

    std::map<kinematic_chain, std::vector<std::string>> map_controllers;

    gains()
    {

    }

    gains(const gains& _gains)
    {
        map_controllers = _gains.map_controllers;
    }
};

class gain_parser{
public:
    gain_parser();

    bool initFile(const std::string& filename);
    void printGains();

    gains Gains;
private:
    boost::shared_ptr<TiXmlDocument> _doc;



};

}

#endif
