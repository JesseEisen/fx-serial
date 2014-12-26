fx-serial
=========

三菱FX系列PLC通信库

Feature
========

1. 使用线程和队列缓存命令， 因此支持批量操作, ``fx_register_set```和```fx_register_get``` is nonblocking

Example
========

```
#include <stdio.h>
#include "fx-serial.h"

int main(int argc, char *argv[]) 
{
	int data;
	struct fx_serial *ss = fx_serial_start();

	fx_register_set(ss, 120, 100);
	fx_register_get(ss, 120, &data);

	// then the data should be 100
	printf("D[%d] register data is :%d\n", 120, data);

	fx_serial_stop(ss);
	return 0;
}
```

Limit
======

1. 目前的版本只支持D寄存器(区间为 0 <= D <= 255)， 由于三菱不同FX系列D寄存器个数配置不一样可以根据通信手册扩展和修改

2. 在FX1S上测试通过
