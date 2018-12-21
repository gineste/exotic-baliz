/******************** (C) COPYRIGHT 2015 exoTIC Systems ************************
* File Name          : es_protocol.h
* Description        : EXOTIC SYSTEMS Protocol for communication.
********************************************************************************/

/* Multi-include protection --------------------------------------------------*/
#ifndef __ES_PROTOCOL_H__
#define __ES_PROTOCOL_H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported constants --------------------------------------------------------*/
#define ESP_LIBRARY_VERSION     "00.05.00"
#define ESP_COMMAND_HEADER     	0x55AAu

 /*#define ESP_USE_ADDR32*/
 /*#define ESP_USE_ADDR16*/
 /*#define ESP_USE_ADDR8*/

/* Exported macro ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef enum _ESP_RETURNCODE_{
    ESP_Receiving      = 0x00,
    ESP_ValidFrame     = 0x01,
    ESP_Failed         = 0xFF,
}ESP_ReturnCode_t;

/* Exported variables --------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/*!
 * Fonction qui analyse le buffer reçu.
 * \param byte la commande recue
 * \return ESP_Receiving if receiving
 * \return ESP_Failed if failure
 * \return ESP_ValidFrame if valid frame received
 */
ESP_ReturnCode_t ESP_CheckForValidFrame(uint8_t *buffer[], uint16_t len);

/*!
 * Fonction qui construit une trame.
 * \param pBuffer le buffer de données à remplir
 * \param expdt l'expediteur
 * \param recpt le destinataire
 * \param opCode le code de la commande
 * \param pPayload le pointeur sur la payload
 * \return la longueur de la trame construite
 */
uint16_t ESP_BuildCommand(uint8_t *pBuffer, uint32_t expdt, uint32_t recpt,
                         uint8_t opCode, const uint8_t payload[], uint16_t len);

/*!
 * Fonction qui renvoie l'opCode recu.
 * \param pBuffer les données recues
 */
uint8_t ESP_GetCommand(const uint8_t buffer[]);

/*!
 * Fonction qui renvoie la longueur de la commande recue.
 * \param pBuffer les données recues
 */
uint16_t ESP_GetFrameLen(const uint8_t buffer[]);

/*!
 * Fonction qui renvoie un pointeur sur la payload.
 * \param pBuffer les données
 * \return la payload
 */
uint8_t* ESP_GetPayload(uint8_t buffer[]);

/*!
 * Fonction qui renvoie la longueur de la payload
 * \param pBuffer les données
 * \return la longueur de la payload
 */
uint16_t ESP_GetPayloadLength(const uint8_t buffer[]);

/*!
 * Fonction qui renvoie l'expediteur d'une trame
 * \param pBuffer les données
 * \return le destinataire
 */
uint32_t ESP_GetExpeditor(const uint8_t buffer[]);

/*!
 * Fonction qui renvoie le destinataire d'une trame
 * \param pBuffer les données
 * \return le destinataire
 */
uint32_t ESP_GetRecipient(const uint8_t buffer[]);

#ifdef __cplusplus
}
#endif

#endif /* __ES_MICHELIN_PROTOCOL_H__ */

