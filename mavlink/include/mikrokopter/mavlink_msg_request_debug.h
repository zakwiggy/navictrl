// MESSAGE REQUEST_RC_CHANNELS PACKING

#define MAVLINK_MSG_ID_REQUEST_DEBUG 221

typedef struct __mavlink_request_debug_t 
{
	uint8_t auto_send_interval; 
} mavlink_request_debug_t;



/**
 * @brief Pack a request_debug message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param auto_send_interval- Auto Send Interval in ms
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_request_debug_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t auto_send_interval)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_REQUEST_DEBUG;

	i += put_uint8_t_by_index(auto_send_interval, i, msg->payload); 
	return mavlink_finalize_message(msg, system_id, component_id, i);
}

/**
 * @brief Encode a request_rc_channels struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param request_rc_channels C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_request_debug_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_request_debug_t* request_debug)
{
	return mavlink_msg_request_debug_pack(system_id, component_id, msg, request_debug->auto_send_interval);
}

// MESSAGE REQUEST_DEBUG UNPACKING

/**
 * @brief Get field enabled from request_rc_channels message
 *
 * @return True: start sending data; False: stop sending data
 */
static inline uint8_t mavlink_msg_request_debug_get_auto_send_interval(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload)[0];
}

/**
 * @brief Decode a request_debug message into a struct
 *
 * @param msg The message to decode
 * @param request_rc_channels C-struct to decode the message contents into
 */
static inline void mavlink_msg_request_debug_decode(const mavlink_message_t* msg, mavlink_request_debug_t* request_debug)
{
	request_debug->auto_send_interval = mavlink_msg_request_debug_get_auto_send_interval(msg);
}
