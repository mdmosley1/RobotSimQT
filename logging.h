#ifndef LOGGING_H
#define LOGGING_H

// include log4cxx header files.
#include "log4cxx/logger.h"
#include "log4cxx/basicconfigurator.h"
#include "log4cxx/propertyconfigurator.h"
#include "log4cxx/helpers/exception.h"

#include "throttledLogging.hh" // LOG4CXX_DEBUG_THROTTLE

//extern LoggerPtr stereoLog
extern log4cxx::LoggerPtr logger;

#define WHEREARG            __FILE__ <<":"<<  __LINE__ <<": "<<

#define TRACE(msg) LOG4CXX_TRACE(logger, WHEREARG msg)
#define DEBUG(msg) LOG4CXX_DEBUG(logger, WHEREARG msg)
#define INFO(msg) LOG4CXX_INFO(logger, WHEREARG msg)
#define WARN(msg) LOG4CXX_WARN(logger, WHEREARG msg )
#define ERROR(msg) LOG4CXX_ERROR(logger, WHEREARG msg ) // red

const double THROTTLE_TIME = 5.0;    

// throttled macros
#define DEBUG_THROTTLE(msg) LOG4CXX_DEBUG_THROTTLE(THROTTLE_TIME, logger, WHEREARG msg )
#define INFO_THROTTLE(msg) LOG4CXX_INFO_THROTTLE(THROTTLE_TIME, logger, WHEREARG msg )
#define WARN_THROTTLE(msg) LOG4CXX_WARN_THROTTLE(THROTTLE_TIME, logger, WHEREARG msg)
#define ERROR_THROTTLE(msg) LOG4CXX_ERROR_THROTTLE(THROTTLE_TIME, logger, WHEREARG msg )

#endif /* LOGGING_H */
