target remote localhost:3333
monitor reset halt
load
break usbd_cdc_vcp.c:170
