/** @file
 *	@brief MAVLink comm protocol.
 *	@see http://pixhawk.ethz.ch/software/mavlink
 *	 Generated on Tuesday, February 15 2011, 15:58 UTC
 */
//#ifndef MIKROKOPTER_H
#define MIKROKOPTER_H

#ifdef __cplusplus
extern "C" {
#endif


#include "../protocol.h"

#define MAVLINK_ENABLED_MIKROKOPTER


#include "../common/common.h"
// MAVLINK VERSION

#ifndef MAVLINK_VERSION
#define MAVLINK_VERSION 0
#endif

#if (MAVLINK_VERSION == 0)
#undef MAVLINK_VERSION
#define MAVLINK_VERSION 0
#endif

// ENUM DEFINITIONS


// MESSAGE DEFINITIONS

#include "./mavlink_msg_request_debug.h"
#include "./mavlink_msg_send_debug.h"

#ifdef __cplusplus
}
#endif
//#endif
