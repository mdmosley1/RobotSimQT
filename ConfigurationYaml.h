#ifndef CONFIGURATIONYAML_H
#define CONFIGURATIONYAML_H

#include <string>
#include "yamlConfig.h" // YAMLConfig

class ConfigurationYaml
{
public:
    /**
       @brief Constructor will use default configuration by default, verbosity at verbose 1
       @param _verbosity (const YAMLConfig::Verbosity &) verbosity level of YAMLConfig
    */
    ConfigurationYaml(const YAMLConfig::Verbosity & _verbosity = YAMLConfig::VERBOSE_1);

    /**
       @brief Init reads the yaml file and assigns the values to the corresponding configuration variables
       @param _fileName (const std::string &) pass-by-reference because using a const
       @return int:
       \n 0 if succesful
       \n negative if error
       \n positive if warning
    */
    int Init(const std::string & _fileName = std::string());
    
    YAMLConfig::ReturnCode GetVar(std::string _varName, int& _var);
    YAMLConfig::ReturnCode GetVar(std::string _varName, bool& _var);
    YAMLConfig::ReturnCode GetVar(std::string _varName, double& _var);
    YAMLConfig::ReturnCode GetVar(std::string _varName, std::string& _var);

    template<typename T>
    bool SetParam(std::string _varName, T& _var);

    double x_bound_max_ = 1000;
    double y_bound_max_ = 1000;
    double x_bound_min_ = 0;
    double y_bound_min_ = 0;

private:
    YAMLConfig::Config configuration_; /**< yaml config interface */
}; // end class ConfigurationYaml : public Configuration



#endif /* CONFIGURATIONYAML_H */
