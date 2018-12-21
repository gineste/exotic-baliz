/* =============================================================================
 ____  _  _   __   ____   __    ___    ____  _  _  ____  ____  ____  _  _  ____
(  __)( \/ ) /  \ (_  _) (  )  / __)  / ___)( \/ )/ ___)(_  _)(  __)( \/ )/ ___)
 ) _)  )  ( (  O )  )(    )(  ( (__   \___ \ )  / \___ \  )(   ) _) / \/ \\___ \
(____)(_/\_) \__/  (__)  (__)  \___)  (____/(__/  (____/ (__) (____)\_)(_/(____/

  Copyright (c) 2017 EXOTIC SYSTEMS. All Rights Reserved.

  Licensees are granted free, non-transferable use of the information. NO
  WARRANTY of ANY KIND is provided. This heading must NOT be removed from
  the file.

  File name:    ExoTypes.h
  Date:         02 01 2018
  Author:       Emeric L.
  Description:  EXOTYPES - Header file.
============================================================================= */
#ifndef EXOTYPES__H
  #define EXOTYPES__H

  /* ===========================================================================
                                   DEBUG Section
  =========================================================================== */

  /* ===========================================================================
                            Public defines and typedefs
  =========================================================================== */
  typedef enum _EXOTYPES_BIT_ {
    EXOBIT_ZERO = 0u,
    EXOBIT_ONE,
    EXOBIT_HIZ
  } e_ExoBit_t;

  typedef enum _EXOTYPES_BOOL_ {
    EXOBOOL_FALSE = 0u,
    EXOBOOL_TRUE
  } e_ExoBool_t;

  /* ===========================================================================
                          Public constants and variables
  =========================================================================== */


  /* ===========================================================================
                          Public function declarations
  =========================================================================== */

#endif /* EXOTYPES__H */
