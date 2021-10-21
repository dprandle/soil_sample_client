#pragma once

#define ISR(param) __attribute__((interrupt(param)))
typedef unsigned char u8;
typedef char i8;
typedef unsigned char uchar;
typedef unsigned int u16;
typedef int i16;
typedef unsigned long int u32;
typedef long int i32;
typedef unsigned long long int u64;
typedef long long int i64;