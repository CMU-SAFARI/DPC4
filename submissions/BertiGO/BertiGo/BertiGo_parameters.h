#ifndef _BERTI_PARAMETERS_H_
#define _BERTI_PARAMETERS_H_

/*****************************************************************************
 *                              SIZES                                        *
 *****************************************************************************/
#define BERTI_TABLE_SIZE (64)
#define BERTI_TABLE_DELTA_SIZE (16)
#define PC_PATH_HISTORY_SIZE (4)

// (Sizes summarized above)
#define HISTORY_TABLE_SETS (48)
#define HISTORY_TABLE_WAYS (32)

// Hash Function
// #define HASH_FN
// # define HASH_ORIGINAL
// # define THOMAS_WANG_HASH_1
// # define THOMAS_WANG_HASH_2
// # define THOMAS_WANG_HASH_3
// # define THOMAS_WANG_HASH_4
// # define THOMAS_WANG_HASH_5
// # define THOMAS_WANG_HASH_6
// # define THOMAS_WANG_HASH_7
// # define THOMAS_WANG_NEW_HASH
// # define THOMAS_WANG_HASH_HALF_AVALANCHE
// # define THOMAS_WANG_HASH_FULL_AVALANCHE
// # define THOMAS_WANG_HASH_INT_1
// # define THOMAS_WANG_HASH_INT_2
#define ENTANGLING_HASH
// # define FOLD_HASH

/*****************************************************************************
 *                              MASKS                                        *
 *****************************************************************************/
#define SIZE_IP_MASK (64)
#define IP_MASK (0xFFFF)
#define TIME_MASK (0xFFFF)
#define LAT_MASK (0xFFF)
#define ADDR_MASK (0xFFFFFF)
#define DELTA_MASK (12)
#define TABLE_SET_MASK (0x7)

/*****************************************************************************
 *                      CONFIDENCE VALUES                                    *
 *****************************************************************************/
#define CONFIDENCE_MAX (16) // 6 bits
#define CONFIDENCE_INC (1)  // 6 bits
#define CONFIDENCE_INIT (1) // 6 bits

// TODO: chekc of limit
#define CONFIDENCE_L1 (10) // 6 bits - 10
#define CONFIDENCE_L2 (6)  // 6 bits - 8
#define CONFIDENCE_L2R (4) // 6 bits - 6

#define CONFIDENCE_MIDDLE_L1 (14) // 6 bits
#define CONFIDENCE_MIDDLE_L2 (12) // 6 bits
#define LAUNCH_MIDDLE_CONF (8)

/*****************************************************************************
 *                              LIMITS                                       *
 *****************************************************************************/
#define MSHR_LIMIT (70)

/*****************************************************************************
 *                              CONSTANT PARAMETERS                          *
 *****************************************************************************/
#define BERTI_R (0x0)
#define BERTI_L1 (0x1)
#define BERTI_L2 (0x2)
#define BERTI_L2R (0x3)
#endif
