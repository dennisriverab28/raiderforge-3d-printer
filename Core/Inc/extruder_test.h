/*
 * extruder_test.h
 *
 * UART dry-run test harness for Motor E.
 */

#ifndef INC_EXTRUDER_TEST_H_
#define INC_EXTRUDER_TEST_H_

void ExtruderTest_Task(void *argument);
void ExtruderTest_ProcessLine(const char *line);

#endif /* INC_EXTRUDER_TEST_H_ */
