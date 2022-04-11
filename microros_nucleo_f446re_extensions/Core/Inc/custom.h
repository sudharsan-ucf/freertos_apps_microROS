/*
 * custom.h
 *
 *  Created on: Mar 23, 2022
 *      Author: Sudharsan Vaidhun
 *      Email: sudharsan.vaidhun@knights.ucf.edu
 */

#ifndef INC_CUSTOM_H_
#define INC_CUSTOM_H_

typedef struct {
  TickType_t budget;    /// Execution Time
  TickType_t period;    /// Period
  TickType_t deadline;  /// Relative Deadline
  const uint8_t *message;   /// Message
} customParameters_t;


#endif /* INC_CUSTOM_H_ */
