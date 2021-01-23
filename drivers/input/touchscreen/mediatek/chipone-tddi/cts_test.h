#ifndef CTS_TEST_H
#define CTS_TEST_H

struct cts_device;

extern int cts_short_test(struct cts_device *cts_dev, u16 threshold);
extern int cts_open_test(struct cts_device *cts_dev, u16 threshold);

#endif /* CTS_TEST_H */

