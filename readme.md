To be uploaded to Github
# RTOS icviet 19.8.2017 note
tích hơp source code RTOS vào con mcu mình muốn.

open-source.

download trên mạng -> portable : ARM_CM7

trong file FreeRTOSConfig.h: Thay đổi SystemCoreClock

configTICK_RATE_HZ: tần số xảy ra 1 switch task: 1 giây xr 1000 lần

configTOTAL_HEAP_SIZE: 75K RAM của con ST để cho nó hoạt động


define lại vPortSVCHandle SVC_Handler -> stm32f4xx_it: __weak

start….RTOS -> nhảy vào hàm trong ARM_CM4F: thay đổi các file trong folder này, port cho 1 dòng ARM Cortex -> coi folder ARM_CM4F như là folder để users viết thay đỏi, để có thể tùy biến theo chip. Còn lại ví dụ như các file trong Foder FreeRTOS khác như tasks.c timers.c là ko phải của users, ko đụng vào

FreeRTOSConfig.h 1 con chip ST

---
lấy timer để FreeRTOS switch task: BaseType_t xPortStartScheduler(void) in port.c

sau khi gọi System_Config >> mỗi ms…

Vào hàm xử lý ngắt mỗi ms… để switch task​

phân tích cơ chế tại mỗi block state: backup, restore >>

Với lần đầu tiên chưa có bảng: R0->R15 của mỗi Task, nên ta sẽ Initilize

---
vào block state hay kết thúc 1 tick để vào function trong .asm 

vPortSVCHandler:

vPortStartFirstTask: ( trước khi vào First Task phải khởi tạo các giá trị R0-R15: như PC ,… cho mỗi Task như )​

initialize: 

StackType_t * pxPortInitialiseStack()…
