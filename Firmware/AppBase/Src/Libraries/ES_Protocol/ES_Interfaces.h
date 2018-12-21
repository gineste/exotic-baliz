/**
 * @file ES_Interfaces.h
 * @brief
 * @author Easymov Robotics
 * @date 2017-03
 */

/* Multi-include protection --------------------------------------------------*/
#ifndef __ES_INTERFACES_H__
#define __ES_INTERFACES_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>

/* Exported constants --------------------------------------------------------*/

#define ESI_LIBRARY_VERSION     "00.07.00"

typedef enum _ESI_RETURNCODE_{
    ESI_Failed          = 0x00u,
    ESI_Succeed         = 0x01u,
    ESI_Overflow        = 0xFEu,
    ESI_BadFrame        = 0xFFu,
}ESI_ReturnCode_t;

typedef enum _ESI_LOG_LEVEL_{
    ESI_LOG_NODEBUG     = 0x00u,
    ESI_LOG_ERROR       = 0x01u,
    ESI_LOG_WARNING     = 0x02u,
    ESI_LOG_INFO        = 0x04u,
    ESI_LOG_VERBOSE     = 0xFFu,
}ESI_LogLevel_t;

/* Exported macro ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

typedef uint8_t 	(*ESI_WriteBuffer_f) (uint8_t buffer[], uint16_t len);
typedef uint8_t   (*ESI_ReadBuffer_f) (uint8_t * buffer, uint16_t * len);
typedef uint8_t 	(*ESI_ReleaseBuffer_f) (void);

typedef void 		  (*ESI_HandleFrame_f)(uint8_t op_code,
                                       uint32_t expdt,
                                       uint32_t recpt,
                                       uint8_t * payload, uint16_t len);

typedef uint8_t	  (*ESI_SendFrame_f)(uint8_t * data, uint16_t len);

typedef struct _ESI_PACKET_ {
    uint8_t   * data;
    uint16_t  len;
    uint16_t  size;
} ESI_Packet_t;

typedef struct _ESI_BOX_ {
    ESI_ReadBuffer_f     read;
    ESI_WriteBuffer_f    write;
    ESI_ReleaseBuffer_f  on_read_finish;
    ESI_ReleaseBuffer_f  on_write_finish;
    ESI_Packet_t         buffer_read;
    ESI_Packet_t         buffer_write;
} ESI_FrameBox_t;

typedef struct _ESI_HANDLER_ {
    ESI_HandleFrame_f    handle;
} ESI_FrameHandler_t;

typedef struct _ESI_SENDER_ {
    ESI_SendFrame_f      send;
} ESI_FrameSender_t;

typedef struct _ESI_INTERFACE_ {
    ESI_FrameBox_t *     inbox;
    ESI_FrameBox_t *     outbox;
    ESI_FrameHandler_t * handler;
    ESI_FrameSender_t *  sender;
} ESI_Interface_t;

/* Exported variables --------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

ESI_ReturnCode_t ESI_Init(ESI_Interface_t * interface, ESI_FrameBox_t * inbox, ESI_FrameBox_t * outbox, ESI_FrameHandler_t * handler, ESI_FrameSender_t * sender);

ESI_ReturnCode_t ESI_NewOutgoingFrame(ESI_Interface_t * interface, uint8_t opCode, uint32_t expdt, uint32_t recpt, const uint8_t payload[], uint16_t len);
ESI_ReturnCode_t ESI_ProcessOutgoingFrame(ESI_Interface_t * interface);

ESI_ReturnCode_t ESI_NewIncomingFrame(ESI_Interface_t * interface, uint8_t * data, uint16_t len);
ESI_ReturnCode_t ESI_ProcessIncomingFrame(ESI_Interface_t * interface);

#ifdef __cplusplus
}
#endif

#endif /* __ES_INTERFACES_H__ */
