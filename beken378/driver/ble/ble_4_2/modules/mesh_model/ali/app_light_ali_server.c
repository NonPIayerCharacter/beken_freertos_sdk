#include "m_api.h"
#include "mesh_api_msg.h"
#include "mal.h"
#include "m_fnd_int.h"
#include "m_fnd_Scenes.h"

#include "mm_gens_int.h"
#include "mm_lights_int.h"

#include "app_light_ali_server.h"
#include "application.h"
#include "mm_vendors.h"
#include "m_fnd_BLOB_Transfer.h"
#include "m_fnd_Fw_Update.h"
#include "ble_uart.h"

/**
 * @brief initialize alibaba lightness light
 */

static uint16_t scene_num[14];
static uint16_t cur_scene_num;

int32_t light_scene_server_data(const m_fnd_model_env_p pmodel_info, uint32_t type,
                                void *pargs)
{
    MESH_APP_PRINT_INFO("%s type = 0x%x\r\n", __func__, type);
    switch (type)
    {
        case M_FND_SCENE_SERVER_GET_PRESENT:
        {
            m_fnd_scene_server_get_present_t *pdata = pargs;
            pdata->scene_number = cur_scene_num;
        } break;

        case M_FND_SCENE_SERVER_RECALL:
        {
            m_fnd_scene_server_set_t *pdata = pargs;
            if (0 == pdata->remaining_time.num_steps)
            {
                cur_scene_num = pdata->scene_number;
            }
        } break;

        case M_FND_SCENE_SERVER_STORE:
        {
            m_fnd_scenes_store_t *pdata = pargs;

            cur_scene_num = pdata->scene_number;
            {
                for (int i =0; i < M_FND_SCENES_STORE_MAX; i++)
                {
                    if ((scene_num[i] == 0 ) || (scene_num[i] == pdata->scene_number))
                    {
                        scene_num[i]    = pdata->scene_number;
                        break;
                    }
                }
            }

        } break;

        case M_FND_SCENE_SERVER_DELETE:
        {
            MESH_APP_PRINT_INFO("M_FND_SCENE_SERVER_DELETE\r\n");
            m_fnd_scenes_delete_t *pdata = pargs;
            for (int i =0; i < M_FND_SCENES_STORE_MAX; i++)
            {
                if (scene_num[i] == pdata->scene_number)
                {
                    scene_num[i] = 0;
                    break;
                }
            }
        } break;

        case M_FND_SCENE_SERVER_GET_REGISTER:
        {
            MESH_APP_PRINT_INFO("M_FND_SCENE_SERVER_GET_REGISTER\r\n");
            m_fnd_scene_server_get_register_t *pdata = pargs;
            uint8_t j = 0;
            for (int i =0; i < M_FND_SCENES_STORE_MAX; i++)
            {
                if (scene_num[i] != 0)
                {
                    pdata->scenes[j++] =  scene_num[i];
                }
            }
        } break;
        default:break;
    }

    return 0;
}

m_lid_t g_vdr_lid, g_ln_mdl_lid, g_ctl_ln_mdl_lid;
m_lid_t g_hsl_ln_mdl_lid, g_ctl_mdl_lid;
m_lid_t g_ctlt_mdl_lid, g_oo_mdl_lid;
m_lid_t g_hsl_mdl_lid, g_hslh_mdl_lid, g_hslsat_mdl_lid;

void app_ai_lights_models_init(uint8_t elmt_idx)
{
    uint16_t status = MESH_ERR_MDL_ALREADY_REGISTERED;

    if ((mm_tb_state_get_lid(elmt_idx, MM_ID_LIGHTS_HSL) == MESH_INVALID_LID)
            && (mm_tb_state_get_lid(elmt_idx, MM_ID_GENS_OO) == MESH_INVALID_LID)
            && (mm_tb_state_get_lid(elmt_idx, MM_ID_LIGHTS_LN) == MESH_INVALID_LID)
            && (mm_tb_state_get_lid(elmt_idx + 1, MM_ID_LIGHTS_LN) == MESH_INVALID_LID)
            && (mm_tb_state_get_lid(elmt_idx + 2, MM_ID_LIGHTS_LN) == MESH_INVALID_LID)
            && (mm_tb_state_get_lid(elmt_idx, MM_ID_LIGHTS_CTL) == MESH_INVALID_LID)
            && (mm_tb_state_get_lid(elmt_idx, MM_ID_VENDORS) == MESH_INVALID_LID))
    {
        do
        {
            mm_vendors_register(elmt_idx, &g_vdr_lid);
            m_fnd_scenes_init(light_scene_server_data);

            // Register Light Lightness Server model and associated models
            status = mm_gens_oo_register(elmt_idx, &g_oo_mdl_lid);
            if (status != MESH_ERR_NO_ERROR)
            {
                break;
            }
            // Register Light Lightness Server model and associated models
            status = mm_lights_ln_register(elmt_idx, &g_ln_mdl_lid);
            if (status != MESH_ERR_NO_ERROR)
            {
                break;
            }
            status = mm_lights_ln_register(elmt_idx + 1, &g_ctl_ln_mdl_lid);
            if (status != MESH_ERR_NO_ERROR)
            {
                break;
            }

            status = mm_lights_ctl_register(elmt_idx, &g_ctl_mdl_lid, &g_ctlt_mdl_lid);
            if (status != MESH_ERR_NO_ERROR)
            {
                break;
            }

            if (status == MESH_ERR_NO_ERROR)
            {
                // Group local index
                m_lid_t grp_lid;

                // Create group and set Light Lightness Server model as main model
                mm_tb_bind_add_group(1, elmt_idx, &grp_lid, g_ln_mdl_lid,
                                     mm_lights_ln_cb_grp_event, mm_lights_ln_cb_trans_req);

                // Add Generic OnOff Server model to the group
                mm_api_grp_add_local(elmt_idx, MM_ID_GENS_OO, grp_lid);
            }

            if (status == MESH_ERR_NO_ERROR)
            {
                // Group local index
                m_lid_t grp_lid;

                // Create group and set Light CTL Server model as main model
                mm_tb_bind_add_group(1, elmt_idx, &grp_lid, g_ctl_mdl_lid,
                                     mm_lights_ctl_cb_grp_event, mm_lights_ctl_cb_trans_req);


                // Add Light Lightness Server model to the group
                mm_tb_bind_group_add_mdl(grp_lid, g_ctl_ln_mdl_lid, MM_ID_LIGHTS_LN,
                                         mm_lights_ln_cb_grp_event, mm_lights_ln_cb_set_state);


                // Create group and set Light CTL Temperature Server model as main model
                mm_tb_bind_add_group(0, elmt_idx + 1, &grp_lid, g_ctlt_mdl_lid,
                                     mm_lights_ctl_cb_grp_event_temp, mm_lights_ctl_cb_trans_req_temp);

            }
        }
        while (0);
    }
}



