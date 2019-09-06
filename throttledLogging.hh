/**
   @file throttledLogging.hh
   @date December 01, 2016
   @brief Macros for throttled log4cxx logging
   @author Anne Schneider 
 
   @license
   Government Purpose Rights
   Contract No.: W15QKN-14-9-1002, Project Agreement 69-201529
   Contractor Name: Robotic Research, LLC
   Contractor Address: 555 Quince Orchard Rd., Suite 300, Gaithersburg, MD, 20878
   Expiration Date: 1 October 2025
    
   The Government's rights to use, modify, reproduce, release, perform, display, or disclose these
   technical data are restricted by paragraph(b)(2) of the Rights in Technical Data—Noncommercial
   Items clause contained in the above identified contract. No restrictions apply after
   the expiration date shown above. Any reproduction of technical data or portions thereof marked
   with this legend must also reproduce the markings.
 
   DISTRIBUTION STATEMENT C: Distribution authorized to U. S. Government agencies and
   their contractors (Administrative or Operational Use, Export Control)
   (Date of determination 28 March 2016). Other requests for this document shall be referred
   to AMSTA-CSS-S, 6501 E. 11 Mile Road, MS 105, Warren, MI 48397-5000, COM
   (586) 282-6262 or DSN 786-6262
 
   WARNING - This document contains technical data whose export is restricted by the Arms Export
   Control Act (Title 22, U.S.C., Sec 2751, et seq.) or the Export Administration Act of 1979
   (Title 50, U.S.C., App. 2401 et seq.), as amended. Violations of these export laws are subject
   to severe criminal penalties. Disseminate in accordance with provisions of DoD Directive 5230.25.
 
   @copyright
   Data delivered under this Agreement contains unpublished copyrighted material. Use,
   duplication, or disclosure is subject to the restrictions stated in the Agreement between
   the U.S. Government and the National Advanced Mobility Consortium, Agreement No. W15QKN-14-9-1002,
   Project Agreement 69-201529 and Task Assignment T01 with Robotic Research, LLC.
 
   Use or disclosure of the software files specifically identified and marked
   as Begin Government Purpose Rights and End Government Purpose Rights are subject to
   the Government Purpose Rights restriction in the header section of this software
*/
/* Begin Government Purpose Rights */
#define LOG4CXX_ERROR_THROTTLE(rate, log, arg) \
  {                                           \
    static double last_time_printed = 0.0;    \
    timespec currentTime;                       \
    clock_gettime(CLOCK_MONOTONIC, &currentTime);                       \
    double time = (currentTime.tv_sec + (currentTime.tv_nsec / 1e9)); \
    if (LOG4CXX_UNLIKELY((time-last_time_printed) >= rate))      \
    {                                                                   \
      last_time_printed = time;                                  \
      LOG4CXX_ERROR(log, arg);                                   \
    }                                                            \
  }

#define LOG4CXX_WARN_THROTTLE(rate, log, arg) \
  {                                           \
    static double last_time_printed = 0.0;    \
    timespec currentTime;                       \
    clock_gettime(CLOCK_MONOTONIC, &currentTime);                       \
    double time = (currentTime.tv_sec + (currentTime.tv_nsec / 1e9)); \
    if (LOG4CXX_UNLIKELY((time-last_time_printed) >= rate))      \
    {                                                                   \
      last_time_printed = time;                                  \
      LOG4CXX_WARN(log, arg);                                   \
    }                                                            \
  }

#define LOG4CXX_DEBUG_THROTTLE(rate, log, arg) \
  {                                           \
    static double last_time_printed = 0.0;    \
    timespec currentTime;                       \
    clock_gettime(CLOCK_MONOTONIC, &currentTime);                       \
    double time = (currentTime.tv_sec + (currentTime.tv_nsec / 1e9)); \
    if (LOG4CXX_UNLIKELY((time-last_time_printed) >= rate))      \
    {                                                                   \
      last_time_printed = time;                                  \
      LOG4CXX_DEBUG(log, arg);                                   \
    }                                                            \
  }

#define LOG4CXX_TRACE_THROTTLE(rate, log, arg) \
  {                                           \
    static double last_time_printed = 0.0;    \
    timespec currentTime;                       \
    clock_gettime(CLOCK_MONOTONIC, &currentTime);                       \
    double time = (currentTime.tv_sec + (currentTime.tv_nsec / 1e9)); \
    if (LOG4CXX_UNLIKELY((time-last_time_printed) >= rate))      \
    {                                                                   \
      last_time_printed = time;                                  \
      LOG4CXX_TRACE(log, arg);                                   \
    }                                                            \
  }

#define LOG4CXX_INFO_THROTTLE(rate, log, arg) \
  {                                           \
    static double last_time_printed = 0.0;    \
    timespec currentTime;                       \
    clock_gettime(CLOCK_MONOTONIC, &currentTime);                       \
    double time = (currentTime.tv_sec + (currentTime.tv_nsec / 1e9)); \
    if (LOG4CXX_UNLIKELY((time-last_time_printed) >= rate))      \
    {                                                                   \
      last_time_printed = time;                                  \
      LOG4CXX_INFO(log, arg);                                   \
    }                                                            \
  }

/* End Government Purpose Rights */
