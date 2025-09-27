// The minimal amount of assert functionality necessary when building the nRF 802.15.4 radio driver.

#ifndef __ASSERT_H__
#define __ASSERT_H__

#define NULL ((void*)0)

// TODO: Implement proper assert handling
#define assert(condition) { if (!(condition)) { for(;;); } }

#endif // __ASSERT_H__
