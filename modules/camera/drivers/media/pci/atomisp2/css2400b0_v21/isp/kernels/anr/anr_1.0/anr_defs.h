/*
 * INTEL CONFIDENTIAL
 *
 * Copyright (C) 2010 - 2013 Intel Corporation.
 * All Rights Reserved.
 *
 * The source code contained or described herein and all documents
 * related to the source code ("Material") are owned by Intel Corporation
 * or licensors. Title to the Material remains with Intel
 * Corporation or its licensors. The Material contains trade
 * secrets and proprietary and confidential information of Intel or its
 * licensors. The Material is protected by worldwide copyright
 * and trade secret laws and treaty provisions. No part of the Material may
 * be used, copied, reproduced, modified, published, uploaded, posted,
 * transmitted, distributed, or disclosed in any way without Intel's prior
 * express written permission.
 *
 * No License under any patent, copyright, trade secret or other intellectual
 * property right is granted to or conferred upon you by disclosure or
 * delivery of the Materials, either expressly, by implication, inducement,
 * estoppel or otherwise. Any license under such intellectual property rights
 * must be express and approved by Intel in writing.
 */

#include <sh_css_defs.h>

/* define the shift for the multiplications, which equals the width of a vector element
  (the result of a multiplication should always be the MSB half and the LSB half should be discarded) */
#define MUL_SHIFT    14

/* maximum positive value of the input range */
#define MAX_RANGE  8191

#define MAX_ANR_FRAME_SIMDWIDTH   CEIL_DIV(ISP_MAX_INPUT_WIDTH, ISP_VEC_NELEMS)


/* Algorithmic parameters */  
#define BLOCK_NUM                 4     /* Maximal number of blocks in a 3-D group.                  */
                                        /* Greater values increase both the computational complexity */
                                        /* and the denoising quality. Restricted to 4                */
#define BLOCK_DIM                 4     /* Block size is set to 4x4 pixels                           */
#define LOG2_BLOCK_DIM            2     /* Equal to LOG2 (BLOCK_DIM)                                 */
#define SA_X                      8     /* Search Area X dimension [pixels]                          */
#define SA_Y                      8     /* Search Area Y dimension [pixels]                          */
#define SLIDING_STEP              2     /* Distance between two adjacent ref blocks [pixels]         */        



#define MAX10b                    1024
#define SAD_SHIFT_CONST           3     /* Used to reduce the precision of the SAD value             */

/* Reference block's position related parameters */
#define ROW_REF                  (SA_Y-BLOCK_DIM)/2       /* Position in the search area */
#define COL_REF                  (SA_X-BLOCK_DIM)/2       /* Position in the search area */

/* Application specific DMA settings  */
#define ANR_BPP                   10                        
#define ANR_ELEMENT_BITS        ((CEIL_DIV(ANR_BPP, 8))*8) 

/* DMA settings for IN/OUT channel   */
#define DMA_ANR_IO_CURR_CHANNEL   0    /* height: SA_Y; first iteration */
#define DMA_ANR_IO_NEXT_CHANNEL   1    /* height: SLIDING_STEP; next iteration */

/* ANR_IO channel ISP side */ 
#define DMA_ANR_IO_CURR_BLOCK_HEIGHT_A       SA_Y
#define DMA_ANR_IO_CURR_ELEMS_A              ISP_VEC_NELEMS
#define DMA_ANR_IO_CURR_BLOCK_WIDTH_A        VECTORS_PER_LINE
#define DMA_ANR_IO_CURR_STRIDE_A             MAX_ANR_FRAME_SIMDWIDTH*ISP_VEC_ALIGN
#define DMA_ANR_IO_CURR_CROPPING_A           0

/* ANR_IO channel XMEM side */ 
#define DMA_ANR_IO_CURR_ELEMS_B              CEIL_DIV(XMEM_WIDTH_BITS,ANR_ELEMENT_BITS)
#define DMA_ANR_IO_CURR_BLOCK_WIDTH_B        CEIL_DIV(DMA_ANR_IO_CURR_BLOCK_WIDTH_A * DMA_ANR_IO_CURR_ELEMS_A, DMA_ANR_IO_CURR_ELEMS_B)
#define DMA_ANR_IO_CURR_STRIDE_B            (DMA_ANR_IO_CURR_BLOCK_WIDTH_B * HIVE_ISP_DDR_WORD_BYTES) 
#define DMA_ANR_IO_CURR_CROPPING_B           0


/* ANR_IN channel ISP side */ 
#define DMA_ANR_IO_NEXT_BLOCK_HEIGHT_A       SLIDING_STEP
#define DMA_ANR_IO_NEXT_ELEMS_A              ISP_VEC_NELEMS
#define DMA_ANR_IO_NEXT_BLOCK_WIDTH_A        VECTORS_PER_LINE
#define DMA_ANR_IO_NEXT_STRIDE_A             MAX_ANR_FRAME_SIMDWIDTH*ISP_VEC_ALIGN
#define DMA_ANR_IO_NEXT_CROPPING_A           0

/* ANR_IN channel XMEM side */
#define DMA_ANR_IO_NEXT_ELEMS_B              CEIL_DIV(XMEM_WIDTH_BITS,ANR_ELEMENT_BITS)
#define DMA_ANR_IO_NEXT_BLOCK_WIDTH_B        CEIL_DIV(DMA_ANR_IO_NEXT_BLOCK_WIDTH_A * DMA_ANR_IO_NEXT_ELEMS_A, DMA_ANR_IO_NEXT_ELEMS_B)
#define DMA_ANR_IO_NEXT_STRIDE_B            (DMA_ANR_IO_NEXT_BLOCK_WIDTH_B * HIVE_ISP_DDR_WORD_BYTES)
#define DMA_ANR_IO_NEXT_CROPPING_B           0


#define SRC_BUF  0
#define DST_BUF  1
#define BM_BUF   2
#define SA_BUF   3
