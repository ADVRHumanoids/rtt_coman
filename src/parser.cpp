#include <parser.h>

using namespace cogimon;

gain_parser::gain_parser()
{

}

bool gain_parser::initFile(const std::string &filename)
{
    _doc.reset(new TiXmlDocument(filename));

    if(!_doc->LoadFile())
        return false;

    TiXmlHandle hDoc(_doc.get());
    TiXmlElement *pRoot, *pParm;
    TiXmlElement *ppParam, *pppParam;
    pRoot = _doc->FirstChildElement(cogimon::parsed_words::robot_tag);

    if(!pRoot)
        return false;

    pParm = pRoot->FirstChildElement(cogimon::parsed_words::rtt_gazebo_tag);
    while(pParm)
    {
         std::string kin_chain_name = pParm->Attribute(cogimon::parsed_words::reference_attribute);

         std::vector<std::string> controllers;

         ppParam = pParm->FirstChildElement(cogimon::parsed_words::controller_tag);
         while(ppParam)
         {
            std::string controller_type = ppParam->Attribute(cogimon::parsed_words::type_attribute);
            controllers.push_back(controller_type);



            ppParam = ppParam->NextSiblingElement(cogimon::parsed_words::controller_tag);
         }

         if(controllers.size() > 0)
            Gains.map_controllers[kin_chain_name] = controllers;

         pParm = pParm->NextSiblingElement(cogimon::parsed_words::rtt_gazebo_tag);
    }

    return true;
}

void gain_parser::printGains()
{
    std::map<cogimon::gains::kinematic_chain, std::vector<std::string>>::iterator map_controllers_it;

    for(map_controllers_it = Gains.map_controllers.begin();
        map_controllers_it != Gains.map_controllers.end(); map_controllers_it++)
    {
        std::string kin_chain_name = map_controllers_it->first;
        std::vector<std::string> controllers = map_controllers_it->second;

        std::cout<<kin_chain_name<<": [  ";
        for(unsigned int i = 0; i < controllers.size(); ++i)
            std::cout<<controllers[i]<<"  ";
        std::cout<<"]"<<std::endl;
    }
}

