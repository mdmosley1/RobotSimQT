/**
 * @file yamlConfig.h
 * @author Zach La Celle
 *
 * This library is a wrapper for yaml-cpp, and is used
 * to read in yaml-based config files.
 *
 * The eventual goal is to have this replace the
 * proprietary ConfigFile format we developed in
 * utils.
 *
 * @todo Add support for sequences (arrays)
 * @todo Add support for maps
 *
 * The YAML specification can be found here:
 * http://www.yaml.org/refcard.html
 */

#pragma once

#include <string>
#include <vector>
#include <list>
#include <map>
#include <yaml-cpp/yaml.h>

#define PRINT(message, verbosity) Print(message, verbosity, __FUNCTION__, __LINE__);
#define PRINTMORE(message, exception, verbosity) Print(message, exception, verbosity, __FUNCTION__, __LINE__);

namespace YAML
{
class Exception;
class Node;
}

namespace YAMLConfig
{

enum ReturnCode
{
  RET_UNKNOWN = -1,
  RET_SUCCESS = 0,
  RET_FAILURE = 1,
  RET_MISSING = 2,
  RET_BAD_TYPE = 3,
  RET_NOT_UPDATED = 4,
  RET_UPDATED = 5
};

enum Verbosity
{
  QUIET     = 0,
  ERROR     = 1,
  WARNING   = 2,
  INFO      = 3,
  VERBOSE_1 = 4,
  VERBOSE_2 = 5,
  VERBOSE_3 = 6,
  ALL       = 7
};

template<typename T> static int As(const YAML::Node& _node, T* _value)
{
  int retVal = 0;
  if( _value == NULL )
  {
    retVal = -1;
  } //end: if( value == NULL )
  else
  {
    try
    {
      T tempVal = _node.as<T>();
      *_value = tempVal;
    }
    catch(YAML::Exception exception)
    {
      retVal = -1;
    }
  } //end: else
  return retVal;
} //end: As()

class Config
{
public:
  Config();
  ~Config();
  
  //Helper functions
  void SetVerbosity(YAMLConfig::Verbosity _verbosity);
  ReturnCode Update();

  //Setup functions
  ReturnCode LoadFile(std::string _filename);

  //Config functions
  YAMLConfig::ReturnCode GetBool(const std::string _name, bool* _value);
  YAMLConfig::ReturnCode GetInt(const std::string _name, int* _value);
  YAMLConfig::ReturnCode GetUnsignedInt(const std::string _name, unsigned int* _value);
  YAMLConfig::ReturnCode GetDouble(const std::string _name, double* _value);
  YAMLConfig::ReturnCode GetString(const std::string _name, std::string* _value);
  YAMLConfig::ReturnCode GetMap(const std::string _name,
                                std::map<std::string, std::string>* _value);
  template<typename T> YAMLConfig::ReturnCode GetSequence(const std::string _name, std::vector<T>* _value);

  //Helper Function for Custom YAML parsing by parent.
  //Useful when we have a dynamic set of sequences/maps which aren't well known
  //to YAML itself: for instance:
  //
  // test:
  //   node1:
  //     string: "asdf"
  //     value: 1.0
  //   node2:
  //     string: "wasd"
  //     value: 1.5
  YAMLConfig::ReturnCode GetNode(const std::string _name, YAML::Node* _node);

  //If you're using node walking, and need to attempt to cast things to a type,
  //you can use this templated "As()" that will save you from having to handle
  //try/catch exceptions
  template<typename T> int As(const YAML::Node& _node, T* _value);

  // Helper function that calls the appropriate Get*() function based on type
  YAMLConfig::ReturnCode ReadConfigVar(std::string _varName, std::string _varType, void* _varAddress);
  
private:
  std::string ComposePreamble(int _verbosity,
                              std::string _functionName,
                              int _line);
  void Print(std::string _message,
             YAML::Exception& _exception,
             int _verbosity,
             std::string _functionName,
             int _line);
  void Print(YAML::Exception& _exception,
             int _verbosity,
             std::string _functionName,
             int _line);
  void Print(std::string _message,
             int _verbosity,
             std::string _functionName,
             int _line);
  
  int verbosity_;

  std::string filename_;
  time_t prevModTime_;

  YAML::Node* docRoot_;
};
}

