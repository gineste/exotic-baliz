/**
 * @file ES_Interfaces.c
 * @brief
 * @author Easymov Robotics
 * @date 2017-03
 */

/* Multi-include protection --------------------------------------------------*/
#ifndef __ES_INTERFACES_C__
#define __ES_INTERFACES_C__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "ES_Protocol.h"
#include "ES_Interfaces.h"


/* Private constants ---------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* Public functions ----------------------------------------------------------*/

ESI_ReturnCode_t ESI_Init(ESI_Interface_t * interface, ESI_FrameBox_t * inbox, ESI_FrameBox_t * outbox, ESI_FrameHandler_t * handler, ESI_FrameSender_t * sender)
{
  ESI_ReturnCode_t retVal_ = ESI_Failed;

  if (inbox && inbox->write && inbox->on_write_finish && inbox->read && inbox->on_read_finish)
  {
    interface->inbox = inbox;
    retVal_ = ESI_Succeed;
  }

  if (outbox && outbox->write && outbox->on_write_finish && outbox->read && outbox->on_read_finish)
  {
    interface->outbox = outbox;
    retVal_ = ESI_Succeed;
  }

  if (handler && handler->handle)
  {
    interface->handler = handler;
    retVal_ = ESI_Succeed;
  }

  if (sender && sender->send)
  {
    interface->sender = sender;
    retVal_ = ESI_Succeed;
  }

  return retVal_;
}

/**
 * TODO: store data in a better way
 */
ESI_ReturnCode_t ESI_NewOutgoingFrame(ESI_Interface_t * interface, uint8_t opCode, uint32_t expdt, uint32_t recpt, const uint8_t payload[], uint16_t len)
{
  ESI_ReturnCode_t retVal_ = ESI_Failed;

  if (interface->outbox && interface->outbox->write && interface->outbox->on_write_finish && interface->outbox->buffer_write.data)
  {
    interface->outbox->buffer_write.len = ESP_BuildCommand(interface->outbox->buffer_write.data, expdt, recpt, opCode, payload, len);
    retVal_ = (ESI_ReturnCode_t) interface->outbox->write(interface->outbox->buffer_write.data, interface->outbox->buffer_write.len);
    interface->outbox->on_write_finish();
  }

  return retVal_;
}

ESI_ReturnCode_t ESI_ProcessOutgoingFrame(ESI_Interface_t * interface)
{
  ESI_ReturnCode_t retVal_ = ESI_Failed;

  if (interface->outbox && interface->outbox->read && interface->outbox->on_read_finish && interface->sender->send)
  {
    interface->outbox->read(interface->outbox->buffer_read.data, &interface->outbox->buffer_read.len);

    if(interface->outbox->buffer_read.len > 0u)
    {
      interface->sender->send(interface->outbox->buffer_read.data, interface->outbox->buffer_read.len);
      retVal_ = ESI_Succeed;
    }

    interface->outbox->on_read_finish();
  }

  return retVal_;
}

ESI_ReturnCode_t ESI_NewIncomingFrame(ESI_Interface_t * interface, uint8_t * data, uint16_t len)
{
  ESI_ReturnCode_t retVal_ = ESI_Failed;
  ESP_ReturnCode_t flag_ = ESP_Receiving;

  if (interface->inbox && interface->inbox->write && interface->inbox->on_write_finish)
  {
    flag_ = ESP_CheckForValidFrame(&data, len);

    switch(flag_)
    {
    case ESP_Receiving:
      break;

    case ESP_ValidFrame:
      retVal_ = (ESI_ReturnCode_t) interface->inbox->write(data, *((uint16_t *)(data + 4u)));
      interface->inbox->on_write_finish();
      retVal_ = ESI_Succeed;
      break;

    case ESP_Failed:
      break;

    default:
      break;
    }
  }

  return retVal_;
}

ESI_ReturnCode_t ESI_ProcessIncomingFrame(ESI_Interface_t * interface)
{
  ESI_ReturnCode_t retVal_ = ESI_Failed;

  if(interface->inbox && interface->inbox->read && interface->inbox->on_read_finish)
  {
    interface->inbox->read(interface->inbox->buffer_read.data, &interface->inbox->buffer_read.len);

    if(interface->inbox->buffer_read.len > 0u && interface->handler && interface->handler->handle)
    {
      interface->handler->handle(
                               ESP_GetCommand(interface->inbox->buffer_read.data),
                               ESP_GetExpeditor(interface->inbox->buffer_read.data),
                               ESP_GetRecipient(interface->inbox->buffer_read.data),
                               ESP_GetPayload(interface->inbox->buffer_read.data),
                               ESP_GetPayloadLength(interface->inbox->buffer_read.data)
                                 );

      retVal_ = ESI_Succeed;
    }

    interface->inbox->on_read_finish();
  }

  return retVal_;
}

#ifdef __cplusplus
}
#endif

#endif /* __ES_INTERFACES_C__ */
