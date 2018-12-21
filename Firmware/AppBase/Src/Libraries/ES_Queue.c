/* 
 *    ____  _  _   __   ____   __    ___    ____  _  _  ____  ____  ____  _  _  ____
 *   (  __)( \/ ) /  \ (_  _) (  )  / __)  / ___)( \/ )/ ___)(_  _)(  __)( \/ )/ ___)
 *    ) _)  )  ( (  O )  )(    )(  ( (__   \___ \ )  / \___ \  )(   ) _) / \/ \\___ \     
 *   (____)(_/\_) \__/  (__)  (__)  \___)  (____/(__/  (____/ (__) (____)\_)(_/(____/
 *
 * Copyright (c) 2018 EXOTIC SYSTEMS. All Rights Reserved.
 *
 * Licensees are granted free, non-transferable use of the information. NO WARRANTY 
 * of ANY KIND is provided. This heading must NOT be removed from the file.
 *
 * Date:          xx/03/2017 (dd MM YYYY)
 * Author:        
 * Description:   Exotic Queue
 *
 */
 
/****************************************************************************************
 * Include Files
 ****************************************************************************************/
#include <string.h>
#include <stdio.h>

#include "ES_Queue.h"

/****************************************************************************************
 * Defines
 ****************************************************************************************/

/****************************************************************************************
 * Private type declarations
 ****************************************************************************************/
 
/****************************************************************************************
 * Private function declarations
 ****************************************************************************************/
static ES_Queue_ReturnCode_t ES_Queue_Next(ES_Queue_t * queue, uint16_t * index);
static ES_Queue_ReturnCode_t ES_Queue_PreventOverlap(ES_Queue_t * queue, uint16_t * index_write, uint16_t * index_read);

/****************************************************************************************
 * Variable declarations
 ****************************************************************************************/

/****************************************************************************************
 * Public functions
 ****************************************************************************************/ 
/**
 *
 */
ES_Queue_ReturnCode_t ES_Queue_Init(ES_Queue_t * queue, ES_Queue_Buffer_t * queue_buffer, ES_Queue_Semaphore_t * semaphore)
{
	queue->index_write = 0u;
	queue->index_read = 0u;
	queue->element_counter = 0u;
	queue->overlap_counter = 0u;
	queue->buffer = queue_buffer;

	if (semaphore) queue->semaphore = semaphore;

	return ES_Queue_Succeed;
}

/**
 *
 */
ES_Queue_ReturnCode_t ES_Queue_Write(ES_Queue_t * queue, uint8_t * data, uint16_t len)
{
	ES_Queue_ReturnCode_t ret_val_ = ES_Queue_Failed;
	uint8_t * queue_data_ = (uint8_t*) 0u;
	uint8_t * len_ = (uint8_t *)(&len);
	uint16_t index_write_ = queue->index_write;
	uint16_t index_read_ = queue->index_read;

	queue_data_ = queue->buffer->data;

	if (len > 0u)
	{
		if (queue->semaphore) queue->semaphore->lock();

		queue_data_[index_write_] = len_[0];
		ret_val_ |= ES_Queue_Next(queue, &index_write_);
		ret_val_ |= ES_Queue_PreventOverlap(queue, &index_write_, &index_read_);

		queue_data_[index_write_] = len_[1];
		ret_val_ |= ES_Queue_Next(queue, &index_write_);
		ret_val_ |= ES_Queue_PreventOverlap(queue, &index_write_, &index_read_);

		for (uint16_t i = 0u ; i < len ; ++i)
		{
			queue_data_[index_write_] = data[i];
			ret_val_ |= ES_Queue_Next(queue, &index_write_);
			ret_val_ |= ES_Queue_PreventOverlap(queue, &index_write_, &index_read_);
		}

		queue->index_write = index_write_;
		queue->index_read = index_read_;
		++queue->element_counter;

		if (queue->semaphore) queue->semaphore->unlock();
	}

	return ret_val_;
}

/**
 *
 */
ES_Queue_ReturnCode_t ES_Queue_Read(ES_Queue_t * queue, uint8_t * data, uint16_t * len)
{
	ES_Queue_ReturnCode_t ret_val_ = ES_Queue_Failed;
	uint8_t * queue_data_ = (uint8_t*) 0u;
	uint16_t index_read_ = queue->index_read;
	uint16_t frame_len_ = 0u;
	uint8_t len_[2u];

	if(queue->index_read != queue->index_write)
	{
		if (queue->semaphore) queue->semaphore->lock();

		queue_data_ = queue->buffer->data;

		len_[0] = queue_data_[index_read_];
		ret_val_ |= ES_Queue_Next(queue, &index_read_);

		len_[1] = queue_data_[index_read_];
		ret_val_ |= ES_Queue_Next(queue, &index_read_);

		frame_len_ = *((uint16_t*)(len_));
		*len = frame_len_;

		for (uint16_t i = 0u ; i < frame_len_ ; ++i)
		{
			data[i] = queue_data_[index_read_];
			ret_val_ |= ES_Queue_Next(queue, &index_read_);
		}

		queue->index_read = index_read_;
		--queue->element_counter;

		if (queue->semaphore) queue->semaphore->unlock();
	}
  else
  {
    *len = 0u;
  }

	return ret_val_;
}

/**
 *
 */
uint32_t ES_Queue_GetElementCounter(ES_Queue_t * queue)
{
	return queue->element_counter;
}

/**
 *
 */
uint32_t ES_Queue_GetOverlapCounter(ES_Queue_t * queue)
{
	return queue->overlap_counter;
}
/**
 *
 */
ES_Queue_ReturnCode_t ES_Queue_ClearAll(ES_Queue_t * queue)
{
    memset(queue->buffer->data, 0, queue->buffer->size);
    queue->index_read = 0u;
    queue->index_write = 0u;
    queue->element_counter = 0u;
    queue->overlap_counter = 0u;
   
   return ES_Queue_Succeed;
}

/****************************************************************************************
 * Private functions
 ****************************************************************************************/
static ES_Queue_ReturnCode_t ES_Queue_Next(ES_Queue_t * queue, uint16_t * index)
{
	if ((*index + 1u) < queue->buffer->size)
	{
		++(*index);
		return ES_Queue_Succeed;
	}
	else
	{
		(*index) = 0u;
		return ES_Queue_Cycle;
	}
}

static ES_Queue_ReturnCode_t ES_Queue_PreventOverlap(ES_Queue_t * queue, uint16_t * index_write, uint16_t * index_read)
{
	uint8_t len_[2];
	uint16_t frame_len_ = 0u;

	if (*index_write == *index_read)
	{
		len_[0] = queue->buffer->data[*index_read];
		ES_Queue_Next(queue, index_read);

		len_[1] = queue->buffer->data[*index_read];
		ES_Queue_Next(queue, index_read);

		frame_len_ = *((uint16_t*)(len_));

		for (uint16_t i = 0u ; i < frame_len_ ; ++i)
		{
			ES_Queue_Next(queue, index_read);
		}

		--queue->element_counter;
		++queue->overlap_counter;

		return ES_Queue_Overlap;
	}
	else
	{
		return ES_Queue_Succeed;
	}
}

/****************************************************************************************
 * End Of File
 ****************************************************************************************/



