#ifndef YAMLHELP_H
#define YAMLHELP_H

//#include "yaml-cpp/yaml.h"
#include "define_type.h"

class YamlHelp
{
public:
    YamlHelp();

//    ErrInfo GetYaml(const std::string& file, std::map<std::string, std::string>& key_vals);

    ErrInfo GetYamlFromCV(const std::string& file, ConfigInfo& info);
};

#endif // YAMLHELP_H
