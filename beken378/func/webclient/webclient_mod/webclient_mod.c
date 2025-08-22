#include "components/webclient.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <float.h>
#include <limits.h>
#include <ctype.h>
#include <os/mem.h>
#include <os/str.h>
#include <os/os.h>

#if CONFIG_DYNAMIC_LOAD
#include "udynlink.h"
#include "udynlink_externals.h"
// #include "webclient_module_data.h"
#include "udynlink_utils.h"
#include <stdio.h>
#include <string.h>

#include "bk_posix.h"
#include "driver/flash_partition.h"
#include <vendor_flash_partition.h>

#define UDYNLINK_GET_SYMBOL()                                           \
    udynlink_sym_t sym;                                                 \
    if (!s_mod_webclient_is_inited)                                     \
    {                                                                   \
        webclient_mod_init();                                           \
    }                                                                   \
    if (!s_mod_webclient_is_inited)                                     \
    {                                                                   \
        bk_printf("webcliend module init failed.\n");                   \
        return 0;                                                       \
    }                                                                   \
    if (bk_udynlink_lookup_symbol(&mod_webclient, __func__, &sym) == NULL) \
    {                                                                   \
        bk_printf("'%s' symbol not found.\n", __func__);                \
        return 0;                                                       \
    }

udynlink_module_t mod_webclient = {0};
uint8_t s_mod_webclient_is_inited = false;



static char *webclient_file_name = "/webclient_mod.bin";
static char *webclient_mount_point = "/";

void webclient_mod_deinit(void)
{
    if (s_mod_webclient_is_inited)
    {
        bk_udynlink_unload_module(&mod_webclient);
        s_mod_webclient_is_inited = false;
    }
}

int webclient_mod_init(void)
{
    int res = 0;
    uint8 *webclient_psram_addr = NULL;

    if (s_mod_webclient_is_inited)
    {
        bk_printf("webclient module is already inited.\n");
        return 0;
    }

    /* read matter dynamic module image from vfs */
    if (get_dynamic_module_image(webclient_file_name, webclient_mount_point, &webclient_psram_addr) <= 0) {
        bk_printf("%s file not exist.\n", webclient_file_name);
        return 0;
    }

    do
    {
        if (bk_udynlink_load_module(&mod_webclient, webclient_psram_addr, NULL, 0, UDYNLINK_LOAD_MODE_COPY_ALL))
        {
            bk_printf("webclient module load failed.\n");
            return 0;
        }

        CHECK_RAM_SIZE(&mod_webclient, 0);
        // if (!check_exported_symbols(&mod_webclient, exported_syms)) {
        //     bk_printf("webclient module check exported symbols failed.\n");
        //     res = 1;
        //     break;
        // }
        // if (!check_extern_symbols(&mod_webclient, extern_syms)) {
        //     bk_printf("webclient module check extern symbols failed.\n");
        //     res = 2;
        //     break;
        // }
    } while (0);

exit:
    if (res != 0)
    {
        webclient_mod_deinit();
    }
    else
    {
        s_mod_webclient_is_inited = true;
    }

    return res;
}

/* create webclient session and set header response size */
typedef struct webclient_session *(*pf_webclient_session_create)(size_t header_sz);
struct webclient_session *webclient_session_create(size_t header_sz)
{
    UDYNLINK_GET_SYMBOL();
    pf_webclient_session_create p_func = (pf_webclient_session_create)sym.val;

    UDYNLINK_SET_BASE_R9(mod_webclient.p_ram);
    struct webclient_session *session = p_func(header_sz);
    UDYNLINK_RESTROE_R9();
    return session;
}

/* send HTTP GET request */
typedef int (*pf_webclient_get)(struct webclient_session *session, const char *URI);
int webclient_get(struct webclient_session *session, const char *URI)
{
    UDYNLINK_GET_SYMBOL();
    pf_webclient_get p_func = (pf_webclient_get)sym.val;

    UDYNLINK_SET_BASE_R9(mod_webclient.p_ram);
    int ret = p_func(session, URI);
    UDYNLINK_RESTROE_R9();
    return ret;
}

typedef int (*pf_webclient_get_position)(struct webclient_session *session, const char *URI, int position);
int webclient_get_position(struct webclient_session *session, const char *URI, int position)
{
    UDYNLINK_GET_SYMBOL();
    pf_webclient_get_position p_func = (pf_webclient_get_position)sym.val;

    UDYNLINK_SET_BASE_R9(mod_webclient.p_ram);
    int ret = p_func(session, URI, position);
    UDYNLINK_RESTROE_R9();
    return ret;
}

/* send HTTP POST request */
typedef int (*pf_webclient_post)(struct webclient_session *session, const char *URI, const void *post_data, size_t data_len);
int webclient_post(struct webclient_session *session, const char *URI, const void *post_data, size_t data_len)
{
    UDYNLINK_GET_SYMBOL();
    pf_webclient_post p_func = (pf_webclient_post)sym.val;

    UDYNLINK_SET_BASE_R9(mod_webclient.p_ram);
    int ret = p_func(session, URI, post_data, data_len);
    UDYNLINK_RESTROE_R9();
    return ret;
}

/* close and release wenclient session */
typedef int (*pf_webclient_close)(struct webclient_session *session);
int webclient_close(struct webclient_session *session)
{
    UDYNLINK_GET_SYMBOL();
    pf_webclient_close p_func = (pf_webclient_close)sym.val;

    UDYNLINK_SET_BASE_R9(mod_webclient.p_ram);
    int ret = p_func(session);
    UDYNLINK_RESTROE_R9();
    return ret;
}

typedef int (*pf_webclient_set_timeout)(struct webclient_session *session, int millisecond);
int webclient_set_timeout(struct webclient_session *session, int millisecond)
{
    UDYNLINK_GET_SYMBOL();
    pf_webclient_set_timeout p_func = (pf_webclient_set_timeout)sym.val;

    UDYNLINK_SET_BASE_R9(mod_webclient.p_ram);
    int ret = p_func(session, millisecond);
    UDYNLINK_RESTROE_R9();
    return ret;
}

/* send or receive data from server */
typedef int (*pf_webclient_read)(struct webclient_session *session, void *buffer, size_t size);
int webclient_read(struct webclient_session *session, void *buffer, size_t size)
{
    UDYNLINK_GET_SYMBOL();
    pf_webclient_read p_func = (pf_webclient_read)sym.val;

    UDYNLINK_SET_BASE_R9(mod_webclient.p_ram);
    int ret = p_func(session, buffer, size);
    UDYNLINK_RESTROE_R9();
    return ret;
}

typedef int (*pf_webclient_write)(struct webclient_session *session, const void *buffer, size_t size);
int webclient_write(struct webclient_session *session, const void *buffer, size_t size)
{
    UDYNLINK_GET_SYMBOL();
    pf_webclient_write p_func = (pf_webclient_write)sym.val;

    UDYNLINK_SET_BASE_R9(mod_webclient.p_ram);
    int ret = p_func(session, buffer, size);
    UDYNLINK_RESTROE_R9();
    return ret;
}

/* webclient GET/POST header buffer operate by the header fields */
typedef int (*pf_webclient_header_fields_add)(struct webclient_session *session, const char *fmt, ...);
int webclient_header_fields_add(struct webclient_session *session, const char *fmt, ...)
{
    int ret = 0;
    UDYNLINK_GET_SYMBOL();
    pf_webclient_header_fields_add p_func = (pf_webclient_header_fields_add)sym.val;

    UDYNLINK_SET_BASE_R9(mod_webclient.p_ram);
    va_list args;
    va_start(args, fmt);
    ret = p_func(session, fmt, args);
    va_end(args);
    UDYNLINK_RESTROE_R9();

    return ret;
}

typedef const char *(*pf_webclient_header_fields_get)(struct webclient_session *session, const char *fields);
const char *webclient_header_fields_get(struct webclient_session *session, const char *fields)
{
    UDYNLINK_GET_SYMBOL();
    pf_webclient_header_fields_get p_func = (pf_webclient_header_fields_get)sym.val;

    UDYNLINK_SET_BASE_R9(mod_webclient.p_ram);
    const char *ret = p_func(session, fields);
    UDYNLINK_RESTROE_R9();
    return ret;
}

/* send HTTP POST/GET request, and get response data */
typedef int (*pf_webclient_response)(struct webclient_session *session, void **response, size_t *resp_len);
int webclient_response(struct webclient_session *session, void **response, size_t *resp_len)
{
    UDYNLINK_GET_SYMBOL();
    pf_webclient_response p_func = (pf_webclient_response)sym.val;
    bk_printf("[udynlink]:%s: (%d), session(0x%08x)\n", __func__, __LINE__, session);

    UDYNLINK_SET_BASE_R9(mod_webclient.p_ram);
    int ret = p_func(session, response, resp_len);
    UDYNLINK_RESTROE_R9();
    return ret;
}

typedef int (*pf_webclient_request)(const char *URI, const char *header, const void *post_data, size_t data_len, void **response, size_t *resp_len);
int webclient_request(const char *URI, const char *header, const void *post_data, size_t data_len, void **response, size_t *resp_len)
{
    UDYNLINK_GET_SYMBOL();
    pf_webclient_request p_func = (pf_webclient_request)sym.val;
    bk_printf("webclient_request(%d), URI(%s), header(0x%08x), post_data(0x%08x), data_len(%d), response(0x%08x), resp_len(0x%08x)!\r\n", __LINE__,
              URI, header, post_data, data_len, response, resp_len);

    UDYNLINK_SET_BASE_R9(mod_webclient.p_ram);
    int ret = p_func(URI, header, post_data, data_len, response, resp_len);
    UDYNLINK_RESTROE_R9();
    return ret;
}
typedef int (*pf_webclient_request_header_add)(char **request_header, const char *fmt, ...);
int webclient_request_header_add(char **request_header, const char *fmt, ...)
{
    int ret = 0;
    UDYNLINK_GET_SYMBOL();

    pf_webclient_request_header_add p_func = (pf_webclient_request_header_add)sym.val;

    UDYNLINK_SET_BASE_R9(mod_webclient.p_ram);
    va_list args;
    va_start(args, fmt);
    ret = p_func(request_header, fmt, args);
    va_end(args);
    UDYNLINK_RESTROE_R9();

    return ret;
}

typedef int (*pf_webclient_resp_status_get)(struct webclient_session *session);
int webclient_resp_status_get(struct webclient_session *session)
{
    UDYNLINK_GET_SYMBOL();
    pf_webclient_resp_status_get p_func = (pf_webclient_resp_status_get)sym.val;
    return p_func(session);
}

typedef int (*pf_webclient_content_length_get)(struct webclient_session *session);
int webclient_content_length_get(struct webclient_session *session)
{
    UDYNLINK_GET_SYMBOL();
    pf_webclient_content_length_get p_func = (pf_webclient_content_length_get)sym.val;
    bk_printf("[udynlink]:%s: (%d), session(0x%08x)\n", __func__, __LINE__, session);
    return p_func(session);
}

typedef int (*pf_bk_webclient_ota_get_comm)(bk_webclient_input_t *input);
int bk_webclient_ota_get_comm(bk_webclient_input_t *input)
{
    UDYNLINK_GET_SYMBOL();
    pf_bk_webclient_ota_get_comm p_func = (pf_bk_webclient_ota_get_comm)sym.val;
    bk_printf("[udynlink]:%s: (%d), input(0x%08x)\n", __func__, __LINE__, input);

    UDYNLINK_SET_BASE_R9(mod_webclient.p_ram);
    int ret = p_func(input);
    UDYNLINK_RESTROE_R9();
    return ret;
}

typedef int (*pf_bk_webclient_get)(bk_webclient_input_t *input);
int bk_webclient_get(bk_webclient_input_t *input)
{
    UDYNLINK_GET_SYMBOL();
    pf_bk_webclient_get p_func = (pf_bk_webclient_get)sym.val;
    bk_printf("[udynlink]:%s: (%d), input(0x%08x)\n", __func__, __LINE__, input);

    UDYNLINK_SET_BASE_R9(mod_webclient.p_ram);
    int ret = p_func(input);
    UDYNLINK_RESTROE_R9();
    return ret;
}

typedef int (*pf_bk_webclient_post)(bk_webclient_input_t *input);
int bk_webclient_post(bk_webclient_input_t *input)
{
    UDYNLINK_GET_SYMBOL();
    pf_bk_webclient_get p_func = (pf_bk_webclient_get)sym.val;
    bk_printf("[udynlink]:%s: (%d), input(0x%08x)\n", __func__, __LINE__, input);

    UDYNLINK_SET_BASE_R9(mod_webclient.p_ram);
    int ret = p_func(input);
    UDYNLINK_RESTROE_R9();
    return ret;
}
#endif