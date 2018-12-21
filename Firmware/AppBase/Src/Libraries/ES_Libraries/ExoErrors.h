/* =============================================================================
 ____  _  _   __   ____   __    ___    ____  _  _  ____  ____  ____  _  _  ____
(  __)( \/ ) /  \ (_  _) (  )  / __)  / ___)( \/ )/ ___)(_  _)(  __)( \/ )/ ___)
 ) _)  )  ( (  O )  )(    )(  ( (__   \___ \ )  / \___ \  )(   ) _) / \/ \\___ \
(____)(_/\_) \__/  (__)  (__)  \___)  (____/(__/  (____/ (__) (____)\_)(_/(____/

  Copyright (c) 2017 EXOTIC SYSTEMS. All Rights Reserved.

  Licensees are granted free, non-transferable use of the information. NO
  WARRANTY of ANY KIND is provided. This heading must NOT be removed from
  the file.

  File name:    ExoErrors.h
  Date:         13 12 2017
  Author:       Emeric L.
  Description:  Exotic systems errors enumeration. - Header file.
============================================================================= */
#ifndef EXOERRORS__H
  #define EXOERRORS__H

  /* ===========================================================================
                                 DEBUG Section
  =========================================================================== */

  /* ===========================================================================
                                 Include Files
  =========================================================================== */

  /* ===========================================================================
                        Public constants and variables
  =========================================================================== */
  typedef enum _EXOERRORS_ {
    EXOERRORS_NO = 0u,
    EXOERRORS_PARAM,
    EXOERRORS_FSM,
    EXOERRORS_DATA_NO,
    EXOERRORS_WRONG_CRC,
    EXOERRORS_VALIDATE_FAILED,
    EXOERRORS_NOT_IMPLEMENTED,
    EXOERRORS_BUSY,
    EXOERRORS_TIMEOUT,
    EXOERRORS_NOT_PERMITTED,
    EXOERRORS_CALLBACK_MISSING,
    EXOERRORS_NOT_FOUND,
  } e_ExoErrors_t;

  /* ===========================================================================
                         Public functions declarations
  =========================================================================== */

#endif /* EXOERRORS__H */
