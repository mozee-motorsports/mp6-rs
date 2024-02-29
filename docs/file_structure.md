
## some important files
- `Core/Src/main.c`: entry point for the HCB application. Calls all of the CubeMX generated code that initializes peripherals and such. This calls `MX_ThreadX_Init()`, which in turn calls `tx_kernel_enter()`
- `AZURE_RTOS/App/app_azure_rtos.c`: Contains `tx_application_define()`
- `Core/Src/app_threadx.c`: contains App_ThreadX_Init(), which according to [this](https://www.youtube.com/watch?v=DSbXqWrOnn0), is where we're supposed to define our user threads. 

The call order is:
```
MX_ThreadX_init() -> tx_kernel_enter() -> tx_application_define() -> App_ThreadX_Init()
```
