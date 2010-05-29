/***************************************************************************
 *     Copyright (c) 1999-2009, Broadcom Corporation
 *     All Rights Reserved
 *     Confidential Property of Broadcom Corporation
 *
 *
 * THIS SOFTWARE MAY ONLY BE USED SUBJECT TO AN EXECUTED SOFTWARE LICENSE
 * AGREEMENT  BETWEEN THE USER AND BROADCOM.  YOU HAVE NO RIGHT TO USE OR
 * EXPLOIT THIS MATERIAL EXCEPT SUBJECT TO THE TERMS OF SUCH AN AGREEMENT.
 *
 * $brcm_Workfile: bchp_gio.h $
 * $brcm_Revision: Hydra_Software_Devel/1 $
 * $brcm_Date: 7/17/09 8:07p $
 *
 * Module Description:
 *                     DO NOT EDIT THIS FILE DIRECTLY
 *
 * This module was generated magically with RDB from a source description
 * file. You must edit the source file for changes to be made to this file.
 *
 *
 * Date:           Generated on         Fri Jul 17 19:42:13 2009
 *                 MD5 Checksum         2914699efc3fb3edefca5cb4f4f38b34
 *
 * Compiled with:  RDB Utility          combo_header.pl
 *                 RDB Parser           3.0
 *                 unknown              unknown
 *                 Perl Interpreter     5.008008
 *                 Operating System     linux
 *
 * Revision History:
 *
 * $brcm_Log: /magnum/basemodules/chp/70015/rdb/a0/bchp_gio.h $
 * 
 * Hydra_Software_Devel/1   7/17/09 8:07p albertl
 * PR56880: Initial revision.
 *
 ***************************************************************************/

#ifndef BCHP_GIO_H__
#define BCHP_GIO_H__

/***************************************************************************
 *GIO - GPIO
 ***************************************************************************/
#define BCHP_GIO_ODEN_LO                         0x00406000 /* GENERAL PURPOSE I/O OPEN DRAIN ENABLE [31:0] */
#define BCHP_GIO_DATA_LO                         0x00406004 /* GENERAL PURPOSE I/O DATA [31:0] */
#define BCHP_GIO_IODIR_LO                        0x00406008 /* GENERAL PURPOSE I/O DIRECTION [31:0] */
#define BCHP_GIO_EC_LO                           0x0040600c /* GENERAL PURPOSE I/O EDGE CONFIGURATION [31:0] */
#define BCHP_GIO_EI_LO                           0x00406010 /* GENERAL PURPOSE I/O EDGE INSENSITIVE [31:0] */
#define BCHP_GIO_MASK_LO                         0x00406014 /* GENERAL PURPOSE I/O INTERRUPT MASK [31:0] */
#define BCHP_GIO_LEVEL_LO                        0x00406018 /* GENERAL PURPOSE I/O INTERRUPT TYPE [31:0] */
#define BCHP_GIO_STAT_LO                         0x0040601c /* GENERAL PURPOSE I/O INTERRUPT STATUS [31:0] */

/***************************************************************************
 *ODEN_LO - GENERAL PURPOSE I/O OPEN DRAIN ENABLE [31:0]
 ***************************************************************************/
/* GIO :: ODEN_LO :: reserved0 [31:12] */
#define BCHP_GIO_ODEN_LO_reserved0_MASK                            0xfffff000
#define BCHP_GIO_ODEN_LO_reserved0_SHIFT                           12

/* GIO :: ODEN_LO :: oden [11:00] */
#define BCHP_GIO_ODEN_LO_oden_MASK                                 0x00000fff
#define BCHP_GIO_ODEN_LO_oden_SHIFT                                0

/***************************************************************************
 *DATA_LO - GENERAL PURPOSE I/O DATA [31:0]
 ***************************************************************************/
/* GIO :: DATA_LO :: reserved0 [31:12] */
#define BCHP_GIO_DATA_LO_reserved0_MASK                            0xfffff000
#define BCHP_GIO_DATA_LO_reserved0_SHIFT                           12

/* GIO :: DATA_LO :: data [11:00] */
#define BCHP_GIO_DATA_LO_data_MASK                                 0x00000fff
#define BCHP_GIO_DATA_LO_data_SHIFT                                0

/***************************************************************************
 *IODIR_LO - GENERAL PURPOSE I/O DIRECTION [31:0]
 ***************************************************************************/
/* GIO :: IODIR_LO :: reserved0 [31:12] */
#define BCHP_GIO_IODIR_LO_reserved0_MASK                           0xfffff000
#define BCHP_GIO_IODIR_LO_reserved0_SHIFT                          12

/* GIO :: IODIR_LO :: iodir [11:00] */
#define BCHP_GIO_IODIR_LO_iodir_MASK                               0x00000fff
#define BCHP_GIO_IODIR_LO_iodir_SHIFT                              0

/***************************************************************************
 *EC_LO - GENERAL PURPOSE I/O EDGE CONFIGURATION [31:0]
 ***************************************************************************/
/* GIO :: EC_LO :: reserved0 [31:12] */
#define BCHP_GIO_EC_LO_reserved0_MASK                              0xfffff000
#define BCHP_GIO_EC_LO_reserved0_SHIFT                             12

/* GIO :: EC_LO :: edge_config [11:00] */
#define BCHP_GIO_EC_LO_edge_config_MASK                            0x00000fff
#define BCHP_GIO_EC_LO_edge_config_SHIFT                           0

/***************************************************************************
 *EI_LO - GENERAL PURPOSE I/O EDGE INSENSITIVE [31:0]
 ***************************************************************************/
/* GIO :: EI_LO :: reserved0 [31:12] */
#define BCHP_GIO_EI_LO_reserved0_MASK                              0xfffff000
#define BCHP_GIO_EI_LO_reserved0_SHIFT                             12

/* GIO :: EI_LO :: edge_insensitive [11:00] */
#define BCHP_GIO_EI_LO_edge_insensitive_MASK                       0x00000fff
#define BCHP_GIO_EI_LO_edge_insensitive_SHIFT                      0

/***************************************************************************
 *MASK_LO - GENERAL PURPOSE I/O INTERRUPT MASK [31:0]
 ***************************************************************************/
/* GIO :: MASK_LO :: reserved0 [31:12] */
#define BCHP_GIO_MASK_LO_reserved0_MASK                            0xfffff000
#define BCHP_GIO_MASK_LO_reserved0_SHIFT                           12

/* GIO :: MASK_LO :: irq_mask [11:00] */
#define BCHP_GIO_MASK_LO_irq_mask_MASK                             0x00000fff
#define BCHP_GIO_MASK_LO_irq_mask_SHIFT                            0

/***************************************************************************
 *LEVEL_LO - GENERAL PURPOSE I/O INTERRUPT TYPE [31:0]
 ***************************************************************************/
/* GIO :: LEVEL_LO :: reserved0 [31:12] */
#define BCHP_GIO_LEVEL_LO_reserved0_MASK                           0xfffff000
#define BCHP_GIO_LEVEL_LO_reserved0_SHIFT                          12

/* GIO :: LEVEL_LO :: level [11:00] */
#define BCHP_GIO_LEVEL_LO_level_MASK                               0x00000fff
#define BCHP_GIO_LEVEL_LO_level_SHIFT                              0

/***************************************************************************
 *STAT_LO - GENERAL PURPOSE I/O INTERRUPT STATUS [31:0]
 ***************************************************************************/
/* GIO :: STAT_LO :: reserved0 [31:12] */
#define BCHP_GIO_STAT_LO_reserved0_MASK                            0xfffff000
#define BCHP_GIO_STAT_LO_reserved0_SHIFT                           12

/* GIO :: STAT_LO :: irq_status [11:00] */
#define BCHP_GIO_STAT_LO_irq_status_MASK                           0x00000fff
#define BCHP_GIO_STAT_LO_irq_status_SHIFT                          0

#endif /* #ifndef BCHP_GIO_H__ */

/* End of File */