
// MESSAGE SEND_DEBUG

#define MAVLINK_MSG_ID_SEND_DEBUG 179


typedef struct __mavlink_debug_out_t
{
	uint8_t Status[2];
	int16_t Analog[32];

} mavlink_debug_out_t;

#define MAVLINK_MSG_SEND_DEBUG_FIELD_STATUS_LEN 2
#define MAVLINK_MSG_SEND_DEBUG_FIELD_ANALOG_LEN 32

/**
 * @brief Pack a send_debug message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param seqnr sequence number (starting with 0 on every transmission)
 * @param data image data bytes
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_send_debug_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const uint8_t* data)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_SEND_DEBUG;

	i += put_array_by_index((const int8_t*)data,  sizeof(uint8_t)*2 + sizeof(int16_t)*32, i, msg->payload);


	return mavlink_finalize_message(msg, system_id, component_id, i);
}

/**
 * @brief Pack a send_debug message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param seqnr sequence number (starting with 0 on every transmission)
 * @param data image data bytes
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_send_debug_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const uint8_t* data)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_SEND_DEBUG;

	i += put_array_by_index((const int8_t*)data,  sizeof(uint8_t)*2 + sizeof(int16_t)*32, i, msg->payload);

	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, i);
}

/**
 * @brief Encode a send_debug struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param send_debug C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_send_debug_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const uint8_t* data)
{
        return mavlink_msg_send_debug_pack(system_id, component_id, msg, data);
}

/**
 * @brief Send a send_debug message
 * @param chan MAVLink channel to send the message
 *
 * @param seqnr sequence number (starting with 0 on every transmission)
 * @param data image data bytes
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_send_debug_send(mavlink_channel_t chan, const uint8_t* data)
{
	mavlink_message_t msg;
	mavlink_msg_send_debug_pack_chan(mavlink_system.sysid, mavlink_system.compid, chan, &msg, data);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE SEND_DEBUG UNPACKING

/**
 * @brief Get field data from send_debug message
 *
 * @return image data bytes
 */
static inline uint16_t mavlink_msg_send_debug_get_debug_out(const mavlink_message_t* msg,  mavlink_debug_out_t* r_data)
{

	memcpy(r_data, msg->payload, sizeof(uint8_t)*2 + sizeof(int16_t)*32);
	return sizeof(uint8_t)*2 + sizeof(int16_t)*32;
}

/**
 * @brief Decode a send_debug message into a struct
 *
 * @param msg The message to decode
 * @param send_debug C-struct to decode the message contents into
 */
static inline void mavlink_msg_send_debug_decode(const mavlink_message_t* msg, mavlink_debug_out_t* debug_out)
{
        mavlink_msg_send_debug_get_debug_out(msg, debug_out);
}
