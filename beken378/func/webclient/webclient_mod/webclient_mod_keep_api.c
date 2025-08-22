
#include <common/bk_include.h>
#include <sys/types.h>
#include <components/webclient.h>
#include <string.h>

#define __UDYNLINK_EXPORT_SECTION__  __attribute__((used, section(".udynlink_export")))

typedef struct {
    const char *symbol_name;
    unsigned int symbol_addr;
} udynlink_export_symbol_t;


extern void __init_array(void) __UDYNLINK_EXPORT_SECTION__;
extern struct webclient_session *webclient_session_create(size_t header_sz);
int bk_webclient_ota_get_comm(bk_webclient_input_t *input);
int bk_webclient_get(bk_webclient_input_t *input);
int bk_webclient_post(bk_webclient_input_t *input);

/* create webclient session and set header response size */
struct webclient_session *webclient_session_create(size_t header_sz);

/* send HTTP GET request */
int webclient_get(struct webclient_session *session, const char *URI);
int webclient_get_position(struct webclient_session *session, const char *URI, int position);

/* send HTTP POST request */
int webclient_post(struct webclient_session *session, const char *URI, const void *post_data, size_t data_len);

/* close and release wenclient session */
int webclient_close(struct webclient_session *session);

int webclient_set_timeout(struct webclient_session *session, int millisecond);

/* send or receive data from server */
int webclient_read(struct webclient_session *session, void *buffer, size_t size);
int webclient_write(struct webclient_session *session, const void *buffer, size_t size);

/* webclient GET/POST header buffer operate by the header fields */
int webclient_header_fields_add(struct webclient_session *session, const char *fmt, ...);
const char *webclient_header_fields_get(struct webclient_session *session, const char *fields);

/* send HTTP POST/GET request, and get response data */
int webclient_response(struct webclient_session *session, void **response, size_t *resp_len);
int webclient_request(const char *URI, const char *header, const void *post_data, size_t data_len, void **response, size_t *resp_len);
int webclient_request_header_add(char **request_header, const char *fmt, ...);
int webclient_resp_status_get(struct webclient_session *session);
int webclient_content_length_get(struct webclient_session *session);


const static udynlink_export_symbol_t s_export_symbol_table[] = {
    /**********************common****************************/
    {"NA", (unsigned int)&webclient_session_create},
    {"NA", (unsigned int)&bk_webclient_get},
    {"NA", (unsigned int)&bk_webclient_post},
    {"NA", (unsigned int)&webclient_get},
    {"NA", (unsigned int)&webclient_get_position},
    {"NA", (unsigned int)&webclient_post},
    {"NA", (unsigned int)&webclient_close},
    {"NA", (unsigned int)&webclient_read},
    {"NA", (unsigned int)&webclient_write},
    {"NA", (unsigned int)&webclient_header_fields_add},
    {"NA", (unsigned int)&webclient_header_fields_get},
    {"NA", (unsigned int)&webclient_response},
    {"NA", (unsigned int)&webclient_request},
    {"NA", (unsigned int)&webclient_request_header_add},
    {"NA", (unsigned int)&webclient_resp_status_get},
    {"NA", (unsigned int)&webclient_content_length_get},

    // {"__init_array", (unsigned int)&__init_array}, // C++ needed
};


__UDYNLINK_EXPORT_SECTION__ unsigned int udynlink_keep_api(const char *name) {
    int count = (int)sizeof(s_export_symbol_table)/sizeof(s_export_symbol_table[0]);

    for(int i = 0; i < count; i++) {
        const char *temp_name = s_export_symbol_table[i].symbol_name;
        if (NULL != temp_name && 0 == strcmp(name, temp_name)) {
            return s_export_symbol_table[i].symbol_addr;
        }
    }
    return 0;
}