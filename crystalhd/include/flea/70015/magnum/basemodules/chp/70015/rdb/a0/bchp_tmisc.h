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
 * $brcm_Workfile: bchp_tmisc.h $
 * $brcm_Revision: Hydra_Software_Devel/1 $
 * $brcm_Date: 7/17/09 8:21p $
 *
 * Module Description:
 *                     DO NOT EDIT THIS FILE DIRECTLY
 *
 * This module was generated magically with RDB from a source description
 * file. You must edit the source file for changes to be made to this file.
 *
 *
 * Date:           Generated on         Fri Jul 17 19:43:17 2009
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
 * $brcm_Log: /magnum/basemodules/chp/70015/rdb/a0/bchp_tmisc.h $
 * 
 * Hydra_Software_Devel/1   7/17/09 8:21p albertl
 * PR56880: Initial revision.
 *
 ***************************************************************************/

#ifndef BCHP_TMISC_H__
#define BCHP_TMISC_H__

/***************************************************************************
 *TMISC - BVN Top Control Registers
 ***************************************************************************/
#define BCHP_TMISC_SOFT_RESET                    0x00541400 /* BVN TOP Soft Reset */
#define BCHP_TMISC_TEST_PORT_DATA                0x00541404 /* BVN TOP Test Port Status */
#define BCHP_TMISC_TEST_PORT_CTRL                0x00541408 /* BVN TOP Test Port Control */
#define BCHP_TMISC_BVNT_MBIST_TM_CTRL_MFD        0x00541410 /* BVN TOP MBIST TM Control for MFD */
#define BCHP_TMISC_BVNT_MBIST_TM_CTRL_DNR        0x00541414 /* BVN TOP MBIST TM Control for DNR */
#define BCHP_TMISC_BVNT_MBIST_TM_CTRL_SCL        0x00541418 /* BVN TOP MBIST TM Control for SCL */
#define BCHP_TMISC_SCRATCH_0                     0x0054143c /* Scratch Register */

/***************************************************************************
 *SOFT_RESET - BVN TOP Soft Reset
 ***************************************************************************/
/* TMISC :: SOFT_RESET :: reserved0 [31:04] */
#define BCHP_TMISC_SOFT_RESET_reserved0_MASK                       0xfffffff0
#define BCHP_TMISC_SOFT_RESET_reserved0_SHIFT                      4

/* TMISC :: SOFT_RESET :: CSC [03:03] */
#define BCHP_TMISC_SOFT_RESET_CSC_MASK                             0x00000008
#define BCHP_TMISC_SOFT_RESET_CSC_SHIFT                            3

/* TMISC :: SOFT_RESET :: SCL [02:02] */
#define BCHP_TMISC_SOFT_RESET_SCL_MASK                             0x00000004
#define BCHP_TMISC_SOFT_RESET_SCL_SHIFT                            2

/* TMISC :: SOFT_RESET :: DNR [01:01] */
#define BCHP_TMISC_SOFT_RESET_DNR_MASK                             0x00000002
#define BCHP_TMISC_SOFT_RESET_DNR_SHIFT                            1

/* TMISC :: SOFT_RESET :: MFD [00:00] */
#define BCHP_TMISC_SOFT_RESET_MFD_MASK                             0x00000001
#define BCHP_TMISC_SOFT_RESET_MFD_SHIFT                            0

/***************************************************************************
 *TEST_PORT_DATA - BVN TOP Test Port Status
 ***************************************************************************/
/* TMISC :: TEST_PORT_DATA :: TEST_PORT_DATA [31:00] */
#define BCHP_TMISC_TEST_PORT_DATA_TEST_PORT_DATA_MASK              0xffffffff
#define BCHP_TMISC_TEST_PORT_DATA_TEST_PORT_DATA_SHIFT             0

/***************************************************************************
 *TEST_PORT_CTRL - BVN TOP Test Port Control
 ***************************************************************************/
/* TMISC :: TEST_PORT_CTRL :: reserved0 [31:24] */
#define BCHP_TMISC_TEST_PORT_CTRL_reserved0_MASK                   0xff000000
#define BCHP_TMISC_TEST_PORT_CTRL_reserved0_SHIFT                  24

/* TMISC :: TEST_PORT_CTRL :: TM_CTRL [23:00] */
#define BCHP_TMISC_TEST_PORT_CTRL_TM_CTRL_MASK                     0x00ffffff
#define BCHP_TMISC_TEST_PORT_CTRL_TM_CTRL_SHIFT                    0

/***************************************************************************
 *BVNT_MBIST_TM_CTRL_MFD - BVN TOP MBIST TM Control for MFD
 ***************************************************************************/
/* TMISC :: BVNT_MBIST_TM_CTRL_MFD :: reserved0 [31:24] */
#define BCHP_TMISC_BVNT_MBIST_TM_CTRL_MFD_reserved0_MASK           0xff000000
#define BCHP_TMISC_BVNT_MBIST_TM_CTRL_MFD_reserved0_SHIFT          24

/* TMISC :: BVNT_MBIST_TM_CTRL_MFD :: TM_CTRL [23:00] */
#define BCHP_TMISC_BVNT_MBIST_TM_CTRL_MFD_TM_CTRL_MASK             0x00ffffff
#define BCHP_TMISC_BVNT_MBIST_TM_CTRL_MFD_TM_CTRL_SHIFT            0

/***************************************************************************
 *BVNT_MBIST_TM_CTRL_DNR - BVN TOP MBIST TM Control for DNR
 ***************************************************************************/
/* TMISC :: BVNT_MBIST_TM_CTRL_DNR :: reserved0 [31:24] */
#define BCHP_TMISC_BVNT_MBIST_TM_CTRL_DNR_reserved0_MASK           0xff000000
#define BCHP_TMISC_BVNT_MBIST_TM_CTRL_DNR_reserved0_SHIFT          24

/* TMISC :: BVNT_MBIST_TM_CTRL_DNR :: TM_CTRL [23:00] */
#define BCHP_TMISC_BVNT_MBIST_TM_CTRL_DNR_TM_CTRL_MASK             0x00ffffff
#define BCHP_TMISC_BVNT_MBIST_TM_CTRL_DNR_TM_CTRL_SHIFT            0

/***************************************************************************
 *BVNT_MBIST_TM_CTRL_SCL - BVN TOP MBIST TM Control for SCL
 ***************************************************************************/
/* TMISC :: BVNT_MBIST_TM_CTRL_SCL :: reserved0 [31:24] */
#define BCHP_TMISC_BVNT_MBIST_TM_CTRL_SCL_reserved0_MASK           0xff000000
#define BCHP_TMISC_BVNT_MBIST_TM_CTRL_SCL_reserved0_SHIFT          24

/* TMISC :: BVNT_MBIST_TM_CTRL_SCL :: TM_CTRL [23:00] */
#define BCHP_TMISC_BVNT_MBIST_TM_CTRL_SCL_TM_CTRL_MASK             0x00ffffff
#define BCHP_TMISC_BVNT_MBIST_TM_CTRL_SCL_TM_CTRL_SHIFT            0

/***************************************************************************
 *SCRATCH_0 - Scratch Register
 ***************************************************************************/
/* TMISC :: SCRATCH_0 :: VALUE [31:00] */
#define BCHP_TMISC_SCRATCH_0_VALUE_MASK                            0xffffffff
#define BCHP_TMISC_SCRATCH_0_VALUE_SHIFT                           0

#endif /* #ifndef BCHP_TMISC_H__ */

/* End of File */