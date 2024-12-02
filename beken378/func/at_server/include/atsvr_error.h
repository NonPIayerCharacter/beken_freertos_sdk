#ifndef __ATSRV_ERRROR_H__
#define  __ATSRV_ERRROR_H__



typedef enum{
ATSVR_RET_SUCCESS = 0, // Successful return

ATSVR_ERR_FAILURE = -1001, //Generic failure return
ATSVR_ERR_INVAL= -1002, // Invalid parameter
ATSVR_ERR_DEV_INFO = -1003, // Fail to get device info
ATSVR_ERR_MALLOC =-1004,  //Fail to malloc memory
ATSVR_ERR_HTTP_CLOSED = -3, // HTTP server close the connection
ATSVR_ERR_HTTP = -4,  // HTTP unknown error
ATSVR_ERR_HTTP_PRTCL = -5, // HTTP protocol error
ATSVR_ERR_HTTP_UNRESOLVED_DNS = -6, // HTTP DNS resolve failed
ATSVR_ERR_HTTP_PARSE = -7, // HTTP URL parse failed
ATSVR_ERR_HTTPCONN = -8, // HTTP connect failed
ATSVR_ERR_HTTP_AUTH = -9, // HTTP auth failed
ATSVR_ERR_HTTP_NOT_FOUND = -10, // ��TTP 404
ATSVR_ERR_HTTP_TIMEOUT = -11, // HTTP timeout

ATSVR_RET_MQTT_RECONNECTED=1,// Reconnected with MQTT server successfully
ATSVR_RET_MQTT_MANUALLY_DISCONNECTED = 2,// Manually disconnected with MQTT server
ATSVR_RET_MQTT_CONNACK_CONNECTION_ACCEPTED = 3, //MQTT connection accepted by server
ATSVR_RET_MQTT_ALREADY_CONNECTED =4,   // Already connected with MOTT server
ATSVR_ERR_MQTT_PUSH_TO_LIST_FAILED = -102, // Fail to push node to MQTT waiting list
ATSVR_ERR_MQTT_NO_CONN = -103, // Not connected with MQTT server
ATSVR_ERR_MQTT_UNKNOINN = -104, // MQTT unknown error
ATSVR_ERR_MQTT_ATTEMPTING_RECONNECT= -105,// Reconnecting with MQTT server
ATSVR_ERR_MQTT_RECONNECT_TIMEOUT = -106, // MQTT reconnect timeout
ATSVR_ERR_MQTT_MAX_SUBSCRIPTIONS = -107, // MQTT topic subscription out of range
ATSVR_ERR_MQTT_SUB= -108, // MQTT topic subscription fail
ATSVR_ERR_MQTT_NOTHING_TO_READ = -109, // MQTT nothing to read
ATSVR_ERR_MQTT_PACKET_READ = -110, //Something wrong when reading MQTT packet
ATSVR_ERR_MQTT_REQUEST_TIMEOUT = -111, // MQTT request timeout
ATSVR_ERR_MQTT_CONNACK_UNKNOWN= -112, // MQTT connection refused: unknown error
ATSVR_ERR_MQTT_CONNACK_UNACCEPTABLE_PROTOCOL_VERSION = -113,// MQTT connection refused: protocol version invalid
ATSVR_ERR_MQTT_CONNACK_IDENTIFIER_REJECTED = -114,// MQTT connection refused: identifier rejected
ATSVR_ERR_MQTT_CONNACK_SERVER_UNAVAILABLE= -115,// MQTT connection refused: service not available
ATSVR_ERR_MQTT_CONNACK_BAD_USERDATA= -116, // MQTT connection refused: bad user name or password

ATSVR_ERR_MQTT_CONNACK_NOT_AUTHORIZED=-117, //MQTT connection rerusea: not authorized
ATSVR_ERR_RX_MESSAGE_INVAL = -118, // MQTT received invalid msg
ATSVR_ERR_BUF_TOO_SHORT= -119, // MQTT recv buffer not enough
ATSVR_ERR_MQTT_QOS_NOT_SUPPORT = -120, // MQTT QoS level not supported
ATSVR_ERR_MQTT_UNSUB_FAIL = -121, // MQTT unsubscribe failed

ATSVR_ERRMAX_APPENDING_REQUEST = -137, // appending request out of range
ATSVR_ERR_MAX_TOPIC_LENGTH = -138, // Topic length oversize
ATSVR_ERRCOAP_NULL = -150, // COAP null pointer
ATSVR_ERRCOAP_DATA_SIZE = -151, // COAP data size out of range
ATSVR_ERRCOAP_INTERNAL = -152, // COAP interval erron
ATSVR_ERRCOAP_BADMSG = -153, // COAP bad msg

ATSVR_ERR_DTLS_PEER_CLOSE_NOTIFY = -160, // DTLS connection is closed
ATSVR_ERR_PROPERTY_EXIST = -201, // property already exist
ATSVR_ERR_NOT_PROPERTY_EXIST = -202, // property not exist
ATSVR_ERR_REPORT_TIMEOUT = -203, // update timeout
ATSVR_ERR_REPORT_REJECTED =-204, // update rejected by serven
ATSVR_ERR_GET_TIMEOUT =-205, // get timeout
ATSVR_ERR_GET_REJECTED = -206, // get rejected by server
ATSVR_ERRACTIONEXIST = -210,  // acion already exist
ATSVR_ERR_NOT_ACTION_EXIST = -211, // acion not exist
ATSVR_ERR_GATEWAY_CREATE_SESSIONFAIL = -221, //Gateway fail to create sub-device session
ATSVR_ERR_GATENAY_SESSION_NO_EXIST=-222, // Gateway sub-device session not exist
ATSVR_ERR_GATEWAY_SESSION_TIMEOUT=-223,// Gateway sub-device session timeout
ATSVRERR_GATEWAY_SUBDEV_ONLINE=-224,// Gateway sub-device online
ATSVR_ERR_GATEWAY_SUBDEV_OFFLINE=-225,// Gateway sub-device offline

ATSVR_ERR_TCP_SOCKET_FAILED= -601,// TLS TCP socket connect fail
ATSVR_ERR_TCP_UNKNOWN_HOST =-602,// TCP unknown host (DNS fail)

ATSVR_ERR_TCP_CONNECT = -603,     // TCP/UDP socket connect fail
ATSVR_ERR_TCP_READ_TIMEOUT =-684, // TCP read timeout
ATSVR_ERR_TCP_WRITE_TIMEOUT = -605, // TCP write timeout
ATSVR_ERR_TCP_READ_FAIL = -606,    //TCP read error
ATSVR_ERR_TCP_WRITE_FAIL = -607,   //TCP write error
ATSVR_ERR_TCP_PEER_SHUTDOWN =-608,  //TCP server close connection

ATSVR_ERR_TCP_NOTHING_TO_READ = -609, // TCP socket nothing to read
ATSVR_ERR_SSL_INIT = -701,          // TLS/SSL init fail
ATSVR_ERR_SSL_CERT =-702,           // TLS/SSL certificate issue
ATSVR_ERR_SSL_CONNECT=-703,         // TLS/SSL connect fail
ATSVR_ERR_SSL_CONNECT_TIMEOUT = -784, // TLS/SSL connect timeout
ATSVR_ERR_SSL_WRITE_TIMEOUT=-705,     //TLS/SSL write timeout
ATSVR_ERR_SSL_WRITE= -706,           //TLS/SSL write error
ATSVR_ERR_SSL_READ_TIMEOUT =-707, //TLS/SSL read timeout
ATSVR_ERR_SSL_READ =-708,         // TLS/SSL read error
ATSVR_ERR_SSL_NOTHING_TO_READ = -709, //TLS/SSL nothing to read
ATSVR_ERR_BIND_ARA_ERR = -801,       // bind sub device param error
ATSVR_ERR_BIND_SUBDEV_ERR =-802,     // sub device not exist or illegal
ATSVR_ERR_BIND_SIGNERR =-803,        // signature check err
ATSVR_ERR_BIND_SIGN_METHOD_RRR = -804, //signmethod not supporte
ATSVR_ERR_BIND_SIGN_EXPIRED =-805,    // signature expired
ATSVR_ERR_BIND_BEEN_BINDED = -806,    // sub device has been binded by other gat
ATSVR_ERR_BIND_SUBDEV_FORBID =-807,   // sub device not allow to bind
ATSVR_ERR_BIND_OP_FORBID =-808,       // operation not permit
ATSVR_ERR_BIND_REPEATED_REQ =-889,     // repeated bind request,has been binded

} ATSVR_Return_Code;

#endif
