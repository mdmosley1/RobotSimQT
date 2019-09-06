#include "ConfigurationYaml.h"
#include "logging.h"

#include <iostream>

using namespace std;

ConfigurationYaml::ConfigurationYaml(const YAMLConfig::Verbosity & _verbosity)
{ 
  configuration_.SetVerbosity(_verbosity);
} // end ConfigurationYaml()

    YAMLConfig::ReturnCode ConfigurationYaml::GetVar(std::string _varName, int& _var)
{
    return configuration_.GetInt(_varName, &_var);
}

YAMLConfig::ReturnCode ConfigurationYaml::GetVar(std::string _varName, double& _var)
{
    return configuration_.GetDouble(_varName, &_var);
}

YAMLConfig::ReturnCode ConfigurationYaml::GetVar(std::string _varName, bool& _var)
{
    return configuration_.GetBool(_varName, &_var);
}

YAMLConfig::ReturnCode ConfigurationYaml::GetVar(std::string _varName, std::string& _var)
{
    return configuration_.GetString(_varName, &_var);
}        

template<typename T>
bool ConfigurationYaml::SetParam(std::string _varName, T& _var)
{
    //YAMLConfig::ReturnCode yamlReturn = configuration_.GetInt(_varName, &_var);

    YAMLConfig::ReturnCode yamlReturn = GetVar(_varName, _var);

    if (yamlReturn != YAMLConfig::ReturnCode::RET_SUCCESS)
    {
        WARN("ConfigurationYaml: could not load " << _varName
                 << ", leaving as default (" << _var << ")");
        return false;
    }
    cout << _varName << " = " << _var << "\n";
    return true;
}    

int ConfigurationYaml::Init(const std::string & _fileName)
{
  YAMLConfig::ReturnCode loadStatus = configuration_.LoadFile(_fileName);
  int retVal=0;

  YAMLConfig::ReturnCode yamlReturn;
  if (loadStatus == YAMLConfig::RET_SUCCESS)
  {
      retVal = SetParam("x_bound_max", x_bound_max_) ? retVal : 1;
      retVal = SetParam("y_bound_max", y_bound_max_) ? retVal : 1;
      retVal = SetParam("x_bound_min", x_bound_min_) ? retVal : 1;
      retVal = SetParam("y_bound_min", y_bound_min_) ? retVal : 1;
  }
}
