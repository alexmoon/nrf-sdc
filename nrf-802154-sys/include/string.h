// The minimal set of string functions necessary when building the nRF 802.15.4 radio driver.
//
// NOTE:
// User is responsible for providing the implementation of these functions at runtime.
// One option is to just depend on the `tinyrlibc` crate as in e.g. `use tinyrlibc as _;`

#ifndef __STRING_H__
#define __STRING_H__

typedef unsigned size_t;

void* memset(void* dest, int ch, size_t count);
void* memcpy(void* dest, const void* src, size_t count);
void* memmove(void* dest, const void* src, size_t count);
int memcmp(const void* lhs, const void* rhs, size_t count);

#endif // __STRING_H__
