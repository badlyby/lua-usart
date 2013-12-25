linux: lusartlib.c
	gcc -o usart.so -shared lusartlib.c -fPIC

windows: lusartlib.c
	gcc -shared -o usart.dll lusartlib.c -L./libwin -llua -DLUA_BUILD_AS_DLL -s

arm: lusartlib.c
	arm-linux-gcc -o arm-usart.so -shared lusartlib.c -fPIC