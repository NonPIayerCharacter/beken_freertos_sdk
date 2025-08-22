#include "components/webclient.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <float.h>
#include <limits.h>
#include <ctype.h>
#include <os/mem.h>
#include <components/netif.h>
#include <components/event.h>
#include "lwip/errno.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include <os/str.h>
#include <os/mem.h>
#include <os/mem.h>
#define RT_NULL NULL

#define TAG  "http"

#if 1
#define RCV_BUF_SIZE                1024*2
#define SEND_HEADER_SIZE              1024
int test_http_post_case1(void)
{
    char buf_url[] = "http://www.rt-thread.com/service/echo";
    char post_data[] = "devicesCode=120002000814&userKey=6b6d71e72c70d136179177ddd231af7d&devicesName=bk7256&random=ZWVUTR&token=2c70426994";
    char *uri;
    uri = os_strdup(buf_url);
    size_t data_len;
    char rep_data[2*1024] = {0};
    data_len = sizeof(post_data);
    struct webclient_session* session = RT_NULL;
    unsigned char *buffer = RT_NULL;
    int  ret = 0;
    int bytes_read, resp_status;

    buffer = (unsigned char *) web_malloc(RCV_BUF_SIZE);
    if (buffer == RT_NULL)
    {
        BK_LOGE(TAG,"no memory for receive response buffer.\n");
        ret = -5;
        goto __exit;
    }

    /* create webclient session and set header response size */
    session = webclient_session_create(SEND_HEADER_SIZE);
    if (session == RT_NULL)
    {
        ret = -5;
        goto __exit;
    }
    char version_buf[32] ="7256000";

    /* build header for upload */
    webclient_header_fields_add(session, "Content-Length: %d\r\n", strlen(post_data));
    webclient_header_fields_add(session, "Content-Type: application/x-www-form-urlencoded\r\n");
    webclient_header_fields_add(session, "%s\r\n",version_buf);

    /* send POST request by default header */
    if ((resp_status = webclient_post(session, uri, post_data, data_len)) != 200)
    {
        BK_LOGE(TAG,"webclient POST request failed, response(%d) error.\n", resp_status);
        ret = -1;
        goto __exit;
    }

    BK_LOGI(TAG,"webclient post response data: \n");
    do
    {
        bytes_read = webclient_read(session, buffer, RCV_BUF_SIZE);
        if (bytes_read <= 0)
        {
            break;
        }
        strncat(rep_data,(char*)buffer,bytes_read);
    } while (1);

    BK_LOGI(TAG,"rep_data %s.\n", rep_data);

__exit:
    if (session)
    {
        webclient_close(session);
    }

    if (buffer)
    {
        web_free(buffer);
    }

    return ret;

}

#define GET_HEADER_BUFSZ               1024
#define GET_RESP_BUFSZ                 1024


/* send HTTP GET request by common request interface, it used to receive longer data */
static int webclient_get_comm(const char *uri)
{
    struct webclient_session* session = RT_NULL;
    unsigned char *buffer = RT_NULL;
    int index, ret = 0;
    int bytes_read, resp_status;
    int content_length = -1;

    buffer = (unsigned char *) web_malloc(GET_RESP_BUFSZ);
    if (buffer == RT_NULL)
    {
        BK_LOGE(TAG,"no memory for receive buffer.\n");
        ret = BK_ERR_NO_MEM;
        goto __exit;

    }

    /* create webclient session and set header response size */
    session = webclient_session_create(GET_HEADER_BUFSZ);
    if (session == RT_NULL)
    {
        ret = BK_ERR_NO_MEM;
        goto __exit;
    }

    /* send GET request by default header */
    if ((resp_status = webclient_get(session, uri)) != 200)
    {
        BK_LOGE(TAG,"webclient GET request failed, response(%d) error.\n", resp_status);
        ret = BK_ERR_STATE;
        goto __exit;
    }

    BK_LOGI(TAG,"webclient get response data: \n");

    content_length = webclient_content_length_get(session);
    if (content_length < 0)
    {
        BK_LOGI(TAG,"webclient GET request type is chunked.\n");
        do
        {
            bytes_read = webclient_read(session, (void *)buffer, GET_RESP_BUFSZ);
            if (bytes_read <= 0)
            {
                break;
            }

            for (index = 0; index < bytes_read; index++)
            {
                BK_LOG_RAW("%c", buffer[index]);
            }
        } while (1);

        BK_LOGI(TAG,"\n");
    }
    else
    {
        int content_pos = 0;

        do
        {
            bytes_read = webclient_read(session, (void *)buffer,
                                        content_length - content_pos > GET_RESP_BUFSZ ?
                                        GET_RESP_BUFSZ : content_length - content_pos);
            if (bytes_read <= 0)
            {
                break;
            }

            for (index = 0; index < bytes_read; index++)
            {
                BK_LOG_RAW("%c", buffer[index]);
            }

            content_pos += bytes_read;
        } while (content_pos < content_length);

        BK_LOGI(TAG,"\n");
    }

__exit:
    if (session)
    {
        webclient_close(session);
    }

    if (buffer)
    {
        web_free(buffer);
    }

    return ret;
}

#define GET_LOCAL_URI                  "http://www.rt-thread.com/service/rt-thread.txt"

int test_http_get_case1(void)
{
    char *uri = RT_NULL;


    uri = web_strdup(GET_LOCAL_URI);
    if(uri == RT_NULL)
    {
        BK_LOGE(TAG,"no memory for create get request uri buffer.\n");
        return BK_ERR_NO_MEM;
    }

    webclient_get_comm(uri);


    if (uri)
    {
        web_free(uri);
    }

    return 0;
}

/* send HTTP GET request by simplify request interface, it used to received shorter data */
static int webclient_get_smpl(const char *uri)
{
    char *response = RT_NULL;
    size_t resp_len = 0;
    int index;

    if (webclient_request(uri, RT_NULL, RT_NULL, 0, (void **)&response, &resp_len) < 0)
    {
        BK_LOGE(TAG,"webclient send get request failed.\n");
        return BK_FAIL;
    }

    BK_LOGI(TAG,"webclient send get request by simplify request interface.\n");
    if (response)
    {
        BK_LOGI(TAG,"webclient get response data: \n");
        for (index = 0; index < os_strlen(response); index++)
        {
            BK_LOG_RAW("%c", response[index]);
        }
        BK_LOGI(TAG,"\n");

        web_free(response);
    } else {
        BK_LOGI(TAG,"webclient get response data: NULL.\n");
    }

    return 0;
}

int test_http_get_case2(void)
{
    char *uri = RT_NULL;


    uri = web_strdup(GET_LOCAL_URI);
    if(uri == RT_NULL)
    {
        BK_LOGE(TAG,"no memory for create get request uri buffer.\n");
        return BK_ERR_NO_MEM;
    }

    webclient_get_smpl(uri);


    if (uri)
    {
        web_free(uri);
    }

    return 0;
}

int test_http(void) {
    int ret = 0;

    // ret = test_http_post_case1();
    // BK_LOGI(TAG,"http test post case1 result(%d).\n", ret);
    ret |= test_http_get_case1();
    BK_LOGI(TAG,"http test get case1 result(%d).\n", ret);
    ret |= test_http_get_case2();
    BK_LOGI(TAG,"http test get case2 result(%d).\n", ret);
    return ret;
}

#endif