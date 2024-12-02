/**
 ******************************************************************************
 * @file    helloworld.c
 * @author
 * @version V1.0.0
 * @date
 * @brief   First Bk72xx application to say hello world!
 ******************************************************************************
 *
 *  The MIT License
 *  Copyright (c) 2017 Beken Inc.
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is furnished
 *  to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in
 *  all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 *  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR
 *  IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 ******************************************************************************
 */
#include "helloworld.h"
#include "include.h"

#if HELLOWORLD_DEMO
#include "include.h"
#include "rtos_pub.h"
#include "error.h"
#include "uart_pub.h"
#include "drv_model_pub.h"

void helloworld_thread( beken_thread_arg_t arg )
{
    os_printf( "\r\n\r\nhello world testing...................\r\n\r\n" );

    while(1)
    {
        os_printf( "hello world!\r\n" );
        rtos_delay_milliseconds(1000);
    }

    os_printf( "hello world test over\r\n\r\n" );
}

int demo_start(void)
{
    OSStatus err = kNoErr;

    /* Start hello world application thread*/
    err = rtos_create_thread( NULL, BEKEN_APPLICATION_PRIORITY,
                              "helloworld_thread",
                              helloworld_thread,
                              0x400,
                              0);

    if(err != kNoErr)
    {
        os_printf("helloworld thread failed\r\n");
    }

    return err;
}

#endif // HELLOWORLD_DEMO
// eof

