local usart = require("usart")
local port = assert(usart.open("/dev/ttyUSB0",115200,8,1,"n"))
port:write("Hello world!")
usart.msleep(100)
print(port:read())
port:close()

